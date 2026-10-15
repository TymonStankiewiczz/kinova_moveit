// keyboard_joy_teleop
// ====================
// "Symulowany pad" dla Kinova Gen3 - sterowanie pojedynczymi jointami w
// czasie rzeczywistym z klawiatury. Startuje od aktualnej pozycji ramienia
// (czyta ją raz z /joint_states), więc nie ma szarpnięcia przy starcie.
//
// Sterowanie:
//   1..7   - wybierz joint (joint_1..joint_7)
//   w / s  - zwiększ / zmniejsz wybrany joint o krok
//   [ / ]  - zmniejsz / zwiększ krok
//   h      - preset "home"
//   v      - preset "vertical" (wszystkie jointy = 0)
//   ?      - pokaż tę pomoc ponownie
//   q      - wyjście (Ctrl+C też działa)
//
// Użycie:
//   ros2 run kinova_moveit keyboard_joy_teleop

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "kinova_moveit/arm_trajectory_client.hpp"
#include "kinova_moveit/robot_model.hpp"
#include "kinova_moveit/terminal_input.hpp"

namespace kinova_moveit
{

constexpr char kHelpText[] = R"(Sterowanie:
  1..7   - wybierz joint (joint_1..joint_7)
  w / s  - zwiększ / zmniejsz wybrany joint o krok
  [ / ]  - zmniejsz / zwiększ krok
  h      - preset "home"
  v      - preset "vertical" (wszystkie jointy = 0)
  ?      - pokaż tę pomoc ponownie
  q      - wyjście (Ctrl+C też działa)
)";

constexpr double kJogMoveDuration = 0.15;

// Czyta /joint_states aż pozna pozycje wszystkich 7 jointów albo upłynie
// timeout. Zwraca false (i zostawia current_arm bez zmian), jeśli się nie
// udało w porę.
bool sync_with_current_state(
  const std::shared_ptr<rclcpp::Node> & node, std::vector<double> & current_arm,
  double timeout_sec = 5.0)
{
  std::map<std::string, double> found;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [&found](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        found[msg->name[i]] = msg->position[i];
      }
    });

  const auto deadline = node->now() + rclcpp::Duration::from_seconds(timeout_sec);
  auto has_all = [&found]() {
      return std::all_of(
        kArmJoints.begin(), kArmJoints.end(),
        [&found](const std::string & name) {return found.count(name) > 0;});
    };
  while (rclcpp::ok() && node->now() < deadline && !has_all()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (!has_all()) {
    return false;
  }
  for (std::size_t i = 0; i < kArmJoints.size(); ++i) {
    current_arm[i] = found.at(kArmJoints[i]);
  }
  return true;
}

}  // namespace kinova_moveit

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kinova_moveit::ArmTrajectoryClient>("keyboard_joy_teleop");

  std::cout << "Łączę się z arm_controller...\n";
  if (!node->wait_for_controller(10.0)) {
    std::cout << "  [błąd] nie widzę kontrolera - czy demo.launch.py na pewno działa?\n";
    rclcpp::shutdown();
    return 1;
  }

  std::vector<double> current_arm = kinova_moveit::kPresetHome;
  if (kinova_moveit::sync_with_current_state(node, current_arm)) {
    std::cout << "Zsynchronizowano z aktualną pozycją ramienia.\n";
  } else {
    std::cout << "  [uwaga] nie udało się odczytać /joint_states w porę - zaczynam od 'home'.\n";
  }
  std::cout << kinova_moveit::kHelpText;

  std::unique_ptr<kinova_moveit::TerminalInput> terminal;
  try {
    terminal = std::make_unique<kinova_moveit::TerminalInput>();
  } catch (const std::exception & e) {
    std::cout << "  [błąd] " << e.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }

  std::size_t selected = 0;
  double step = 0.05;
  std::cout << "Wybrany joint: " << kinova_moveit::kArmJoints[selected]
            << " | krok: " << step << " rad\n";

  while (rclcpp::ok()) {
    const char key = terminal->get_key(0.1);
    if (key == '\0') {
      continue;
    }

    if (key >= '1' && key <= '7') {
      selected = static_cast<std::size_t>(key - '1');
      std::cout << "Wybrany joint: " << kinova_moveit::kArmJoints[selected] << "\n";
    } else if (key == 'w' || key == 's') {
      const double delta = (key == 'w') ? step : -step;
      const auto limit = kinova_moveit::kJointLimits[selected];
      current_arm[selected] = std::max(limit.lower, std::min(limit.upper, current_arm[selected] + delta));
      node->move_arm(current_arm, kinova_moveit::kJogMoveDuration);
    } else if (key == '[') {
      step = std::max(0.01, step - 0.01);
      std::cout << "krok: " << step << " rad\n";
    } else if (key == ']') {
      step = std::min(0.5, step + 0.01);
      std::cout << "krok: " << step << " rad\n";
    } else if (key == 'h') {
      current_arm = kinova_moveit::kPresetHome;
      node->move_arm(current_arm, 2.0);
      std::cout << "-> home\n";
    } else if (key == 'v') {
      current_arm = kinova_moveit::kPresetVertical;
      node->move_arm(current_arm, 2.0);
      std::cout << "-> vertical\n";
    } else if (key == '?') {
      std::cout << kinova_moveit::kHelpText;
    } else if (key == 'q' || key == '\x03') {
      break;
    }
  }

  terminal.reset();
  rclcpp::shutdown();
  return 0;
}
