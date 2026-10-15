#pragma once
// Wspólny klient akcji FollowJointTrajectory dla arm_controller - używany
// przez joint_teleop i keyboard_joy_teleop (oba sterują ramieniem
// bezpośrednio w przestrzeni jointów, bez planowania przez MoveIt).

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <iostream>
#include <string>
#include <vector>

#include "kinova_moveit/robot_model.hpp"

namespace kinova_moveit
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class ArmTrajectoryClient : public rclcpp::Node
{
public:
  explicit ArmTrajectoryClient(const std::string & node_name)
  : rclcpp::Node(node_name)
  {
    declare_parameter<std::string>("arm_controller_name", "arm_controller");
    const auto controller_name = get_parameter("arm_controller_name").as_string();
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/" + controller_name + "/follow_joint_trajectory");
  }

  bool wait_for_controller(double timeout_sec)
  {
    return action_client_->wait_for_action_server(std::chrono::duration<double>(timeout_sec));
  }

  // Przycina do limitów, wysyła i czeka na PEŁNE wykonanie (nie tylko
  // przyjęcie celu) - inaczej przy szybkich, kolejnych wywołaniach (np.
  // trzymanym klawiszu) nowy cel przerywałby poprzedni zanim się wykona.
  bool move_arm(std::vector<double> positions, double duration_sec)
  {
    for (std::size_t i = 0; i < positions.size(); ++i) {
      positions[i] = clamp_joint(i, positions[i]);
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = kArmJoints;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    goal.trajectory.points = {point};

    auto send_goal_future = action_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), send_goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "  [błąd] nie udało się wysłać celu (timeout)\n";
      return false;
    }
    const auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      std::cout << "  [błąd] kontroler odrzucił cel trajektorii\n";
      return false;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "  [błąd] brak wyniku wykonania trajektorii (timeout)\n";
      return false;
    }
    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      std::cout << "  [błąd] trajektoria nie powiodła się (code="
                 << static_cast<int>(wrapped_result.code) << ")\n";
      return false;
    }
    return true;
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
};

}  // namespace kinova_moveit
