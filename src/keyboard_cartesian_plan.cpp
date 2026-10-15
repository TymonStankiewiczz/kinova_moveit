// keyboard_cartesian_plan
// =========================
// Sterowanie kiścią Kinova Gen3 w przestrzeni kartezjańskiej z klawiatury.
// PLANUJE każdy mały ruch: czyta aktualną pozycję kiści przez TF, liczy
// nowy cel (mała translacja albo rotacja), prosi /compute_cartesian_path
// o pełną, sprawdzoną trajektorię do tego celu, i dopiero wtedy ją
// wykonuje.
//
// Jeśli cel jest nieosiągalny (blisko granicy zasięgu, osobliwości,
// kolizja) - serwis zwraca informację "osiągnąłem tylko X% ścieżki", więc
// ten węzeł ODRZUCA ruch zamiast robić coś nieprzewidywalnego.
//
// WYMAGA działającego launch/demo.launch.py (move_group + ros2_control).
//
// Sterowanie:
//   w/s - przód/tył   a/d - lewo/prawo   q/e - góra/dół
//   m       - przełącz tryb: translate (przesuwanie) <-> rotate (obrót)
//   [ / ]   - zmniejsz / zwiększ krok
//   ?       - pokaż pomoc ponownie
//   Ctrl+C  - wyjście
//
// Użycie:
//   ros2 run kinova_moveit keyboard_cartesian_plan

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "kinova_moveit/terminal_input.hpp"

using namespace std::chrono_literals;

namespace kinova_moveit
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GetCartesianPath = moveit_msgs::srv::GetCartesianPath;

enum class Mode { Translate, Rotate };

struct AxisKey
{
  int axis;     // 0=X, 1=Y, 2=Z ramy base_frame
  double sign;
};

// Efekt (przód/tył/lewo/prawo/góra/dół) jest stały niezależnie od trybu -
// zmienia się tylko to, czy dana wartość idzie do translacji czy rotacji.
const std::map<char, AxisKey> kAxisKeys = {
  {'w', {1, -1.0}}, {'s', {1, +1.0}},
  {'a', {0, +1.0}}, {'d', {0, -1.0}},
  {'q', {2, +1.0}}, {'e', {2, -1.0}},
};

constexpr char kHelpText[] = R"(Sterowanie:
  w/s - przód/tył   a/d - lewo/prawo   q/e - góra/dół
  m       - przełącz tryb: translate (przesuwanie) <-> rotate (obrót)
  [ / ]   - zmniejsz / zwiększ krok
  ?       - pokaż pomoc ponownie
  Ctrl+C  - wyjście
)";

// Kinova Gen3 ma 7 DOF (nadmiarowe względem 6 wymaganych do pełnego
// sterowania pozycją+orientacją), więc czysty ruch translacyjny przy
// stałej orientacji powinien niemal zawsze się udawać wprost. Ten fallback
// zostaje jako zabezpieczenie blisko rzeczywistych osobliwości/granic
// zasięgu - próbuje kolejno z coraz większym dopuszczalnym dryfem yaw.
constexpr int kYawAxis = 2;
const std::vector<double> kYawDriftAttempts = {0.0, 0.05, -0.05, 0.1, -0.1, 0.2, -0.2};

class KeyboardCartesianPlan : public rclcpp::Node
{
public:
  KeyboardCartesianPlan()
  : rclcpp::Node("keyboard_cartesian_plan")
  {
    group_name_ = declare_parameter<std::string>("group_name", "arm");
    ee_link_ = declare_parameter<std::string>("ee_link", "end_effector_link");
    base_frame_ = declare_parameter<std::string>("base_frame", "world");
    const auto controller_name = declare_parameter<std::string>("controller_name", "arm_controller");
    // Małe kroki celowo - narzut na cykl (serwis + akcja + monitorowanie
    // postępu kontrolera) jest z grubsza stały niezależnie od prędkości
    // jointów, więc żeby ruch wyglądał płynnie, same kroki muszą być
    // drobne (patrz README).
    step_linear_ = declare_parameter<double>("step_linear", 0.003);
    step_angular_ = declare_parameter<double>("step_angular", 0.03);
    max_step_ = declare_parameter<double>("max_step", 0.002);
    avoid_collisions_ = declare_parameter<bool>("avoid_collisions", true);
    min_fraction_ = declare_parameter<double>("min_fraction", 0.95);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cartesian_path_client_ = create_client<GetCartesianPath>("/compute_cartesian_path");
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/" + controller_name + "/follow_joint_trajectory");
  }

  bool wait_for_dependencies(double timeout_sec)
  {
    const auto timeout = std::chrono::duration<double>(timeout_sec);
    const bool ok_service = cartesian_path_client_->wait_for_service(timeout);
    const bool ok_action = action_client_->wait_for_action_server(timeout);
    return ok_service && ok_action;
  }

  const std::string & base_frame() const {return base_frame_;}
  const std::string & ee_link() const {return ee_link_;}
  Mode mode() const {return mode_;}
  double step_linear() const {return step_linear_;}
  double step_angular() const {return step_angular_;}

  void toggle_mode()
  {
    mode_ = (mode_ == Mode::Translate) ? Mode::Rotate : Mode::Translate;
  }

  void change_step(double delta_linear, double delta_angular)
  {
    step_linear_ = std::clamp(step_linear_ + delta_linear, 0.001, 0.05);
    step_angular_ = std::clamp(step_angular_ + delta_angular, 0.01, 0.3);
  }

  // Czyta TF aktualnej pozycji kiści. Aktywnie spinuje node w pętli, bo
  // TransformListener potrzebuje regularnego spinowania, żeby w ogóle
  // odebrać /tf i /tf_static.
  std::optional<geometry_msgs::msg::Pose> get_current_pose(double timeout_sec = 3.0)
  {
    const auto deadline = now() + rclcpp::Duration::from_seconds(timeout_sec);
    std::string last_error;
    while (rclcpp::ok() && now() < deadline) {
      try {
        const auto tf = tf_buffer_->lookupTransform(base_frame_, ee_link_, tf2::TimePointZero);
        geometry_msgs::msg::Pose pose;
        pose.position.x = tf.transform.translation.x;
        pose.position.y = tf.transform.translation.y;
        pose.position.z = tf.transform.translation.z;
        pose.orientation = tf.transform.rotation;
        return pose;
      } catch (const tf2::TransformException & ex) {
        last_error = ex.what();
        rclcpp::spin_some(get_node_base_interface());
        std::this_thread::sleep_for(100ms);
      }
    }
    RCLCPP_WARN(
      get_logger(), "Nie mogę odczytać TF %s->%s: %s",
      base_frame_.c_str(), ee_link_.c_str(), last_error.c_str());
    return std::nullopt;
  }

  void jog(int axis, double sign)
  {
    const auto current_opt = get_current_pose();
    if (!current_opt) {
      std::cout << "  [błąd] brak TF - czy demo.launch.py działa?\n";
      return;
    }
    const auto & current = *current_opt;

    if (mode_ == Mode::Rotate) {
      tf2::Quaternion current_q;
      tf2::fromMsg(current.orientation, current_q);
      tf2::Vector3 axis_vec(axis == 0 ? 1 : 0, axis == 1 ? 1 : 0, axis == 2 ? 1 : 0);
      tf2::Quaternion delta_q(axis_vec, sign * step_angular_);
      tf2::Quaternion new_q = (delta_q * current_q).normalized();

      geometry_msgs::msg::Pose target = current;
      target.orientation = tf2::toMsg(new_q);
      plan_and_execute(target, false);
      return;
    }

    // translate: najpierw próba z dokładnie tą samą orientacją; jeśli się
    // nie uda, kolejne próby z coraz większym dopuszczalnym dryfem yaw.
    double delta[3] = {0.0, 0.0, 0.0};
    delta[axis] = sign * step_linear_;
    tf2::Quaternion current_q;
    tf2::fromMsg(current.orientation, current_q);

    for (std::size_t i = 0; i < kYawDriftAttempts.size(); ++i) {
      const double yaw = kYawDriftAttempts[i];
      tf2::Quaternion new_q = current_q;
      if (yaw != 0.0) {
        tf2::Quaternion delta_q(tf2::Vector3(0, 0, 1), yaw);
        new_q = (delta_q * current_q).normalized();
      }

      geometry_msgs::msg::Pose target;
      target.position.x = current.position.x + delta[0];
      target.position.y = current.position.y + delta[1];
      target.position.z = current.position.z + delta[2];
      target.orientation = tf2::toMsg(new_q);

      const bool is_last = (i + 1 == kYawDriftAttempts.size());
      const auto fraction = plan_and_execute(target, !is_last);
      if (fraction && *fraction >= min_fraction_) {
        if (yaw != 0.0) {
          std::cout << "  (dopuszczono dryf orientacji yaw=" << yaw
                     << " rad, żeby ruch był wykonalny)\n";
        }
        return;
      }
    }
  }

private:
  // Zwraca osiągniętą frakcję ścieżki albo nullopt przy błędzie
  // komunikacji. quiet_reject=true wycisza komunikat o odrzuceniu -
  // używane przy próbach pośrednich w jog().
  std::optional<double> plan_and_execute(const geometry_msgs::msg::Pose & target_pose, bool quiet_reject)
  {
    if (!cartesian_path_client_->service_is_ready()) {
      std::cout << "  [błąd] /compute_cartesian_path niedostępny - czy move_group działa?\n";
      return std::nullopt;
    }

    auto request = std::make_shared<GetCartesianPath::Request>();
    request->header.frame_id = base_frame_;
    request->group_name = group_name_;
    request->link_name = ee_link_;
    request->waypoints = {target_pose};
    request->max_step = max_step_;
    request->jump_threshold = 0.0;
    request->avoid_collisions = avoid_collisions_;
    request->max_velocity_scaling_factor = 1.0;
    request->max_acceleration_scaling_factor = 1.0;

    auto future = cartesian_path_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "  [błąd] brak odpowiedzi z /compute_cartesian_path (timeout)\n";
      return std::nullopt;
    }
    const auto result = future.get();

    if (result->fraction < min_fraction_) {
      if (!quiet_reject) {
        std::cout << "  [odrzucono] osiągalne tylko " << (result->fraction * 100.0)
                   << "% ruchu (blisko granicy zasięgu/osobliwości/kolizji) - nie wykonuję\n";
      }
      return result->fraction;
    }

    const auto & traj = result->solution.joint_trajectory;
    if (traj.points.empty()) {
      std::cout << "  [błąd] pusta trajektoria mimo fraction>=min_fraction\n";
      return result->fraction;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = traj;
    auto send_goal_future = action_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), send_goal_future, 2s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "  [błąd] nie udało się wysłać trajektorii (timeout)\n";
      return result->fraction;
    }
    const auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      std::cout << "  [błąd] kontroler odrzucił trajektorię\n";
      return result->fraction;
    }

    // Czekamy na pełne WYKONANIE, nie tylko przyjęcie celu - inaczej przy
    // trzymaniu klawisza (auto-repeat) nowy cel przerywałby poprzedni
    // zanim się wykona (preemption), dając ciszę aż do puszczenia klawisza.
    auto result_future = action_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "  [błąd] brak wyniku wykonania trajektorii (timeout)\n";
      return result->fraction;
    }
    const auto exec_result = result_future.get();
    if (exec_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      std::cout << "  [błąd] kontroler zwrócił code=" << static_cast<int>(exec_result.code) << "\n";
      return result->fraction;
    }

    std::cout << "  OK (" << (result->fraction * 100.0) << "% ścieżki, "
              << traj.points.size() << " punktów)\n";
    return result->fraction;
  }

  std::string group_name_;
  std::string ee_link_;
  std::string base_frame_;
  double step_linear_;
  double step_angular_;
  double max_step_;
  bool avoid_collisions_;
  double min_fraction_;
  Mode mode_ = Mode::Translate;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<GetCartesianPath>::SharedPtr cartesian_path_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
};

}  // namespace kinova_moveit

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kinova_moveit::KeyboardCartesianPlan>();
  std::cout << kinova_moveit::kHelpText;

  std::unique_ptr<kinova_moveit::TerminalInput> terminal;
  try {
    terminal = std::make_unique<kinova_moveit::TerminalInput>();
  } catch (const std::exception & e) {
    std::cout << "  [błąd] " << e.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }

  std::cout << "Łączę się z /compute_cartesian_path i kontrolerem...\n";
  if (!node->wait_for_dependencies(10.0)) {
    std::cout << "  [błąd] nie widzę /compute_cartesian_path i/lub akcji kontrolera - "
                 "czy demo.launch.py na pewno działa?\n";
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "Połączono.\n";

  std::cout << "Czekam na pierwszy odczyt TF...\n";
  if (!node->get_current_pose(5.0)) {
    std::cout << "  [uwaga] nie udało się jeszcze odczytać TF - pierwsze naciśnięcie "
                 "klawisza spróbuje ponownie, ale jeśli to się powtarza, sprawdź czy '"
              << node->base_frame() << "' i '" << node->ee_link()
              << "' istnieją (ros2 topic echo /tf_static --once)\n";
  }

  while (rclcpp::ok()) {
    const char key = terminal->get_key(0.1);
    if (key == '\0') {
      continue;
    }

    const auto it = kinova_moveit::kAxisKeys.find(key);
    if (it != kinova_moveit::kAxisKeys.end()) {
      // Planowanie+wykonanie trwa chwilę - w tym czasie terminal buforuje
      // kolejne wciśnięcia (auto-repeat), więc po zakończeniu ruchu
      // odsączamy zaległe powtórzenia TEGO SAMEGO klawisza zamiast je
      // kolejkować (inaczej po puszczeniu przycisku ramię "doganiałoby"
      // zaległe ruchy).
      node->jog(it->second.axis, it->second.sign);
      int drained = 0;
      while (true) {
        const char peek = terminal->get_key(0.0);
        if (peek == key) {
          ++drained;
          continue;
        }
        if (peek != '\0') {
          std::cout << "  [uwaga] pomijam zbuforowany inny klawisz '" << peek << "'\n";
        }
        break;
      }
      if (drained > 0) {
        std::cout << "  (pominięto " << drained << " zbuforowanych powtórzeń '" << key << "')\n";
      }
    } else if (key == 'm') {
      node->toggle_mode();
      std::cout << ">>> TRYB: " << (node->mode() == kinova_moveit::Mode::Translate ? "translate" : "rotate")
                << " <<<\n";
    } else if (key == '[') {
      node->change_step(-0.001, -0.01);
      std::cout << "krok: lin=" << node->step_linear() << " kąt=" << node->step_angular() << "\n";
    } else if (key == ']') {
      node->change_step(+0.001, +0.01);
      std::cout << "krok: lin=" << node->step_linear() << " kąt=" << node->step_angular() << "\n";
    } else if (key == '?') {
      std::cout << kinova_moveit::kHelpText;
    } else if (key == '\x03') {
      break;
    }
  }

  terminal.reset();
  rclcpp::shutdown();
  return 0;
}
