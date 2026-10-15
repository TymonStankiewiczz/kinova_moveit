"""
Pełna symulacja Kinova Gen3: robot_state_publisher + ros2_control (mock
hardware) + move_group (MoveIt2) + RViz z pluginem MotionPlanning.

Użycie:
    ros2 launch kinova_moveit demo.launch.py
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = "kinova_moveit"


def load_file(file_path):
    package_path = get_package_share_directory(PKG)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(file_path):
    package_path = get_package_share_directory(PKG)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)

    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "kinova_gen3.urdf.xacro"])
    initial_positions_file = os.path.join(pkg_share, "config", "initial_positions.yaml")
    robot_description_content = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " use_mock_hardware:=true",
                " initial_positions_file:=",
                initial_positions_file,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic = {
        "robot_description_semantic": load_file("config/kinova_gen3.srdf")
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml("config/kinematics.yaml")
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml("config/joint_limits.yaml")
    }

    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": load_yaml("config/ompl_planning.yaml"),
    }

    moveit_controllers = load_yaml("config/moveit_controllers.yaml")

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    ros2_controllers_yaml = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "moveit.rviz")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_yaml],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
        ],
    )

    return LaunchDescription(
        [
            # Locale z przecinkiem dziesiętnym (np. pl_PL) potrafi psuć
            # parsowanie parametrów typu double w RViz/MoveIt.
            SetEnvironmentVariable("LC_NUMERIC", "en_US.UTF-8"),
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_controller_spawner],
                )
            ),
            TimerAction(period=3.0, actions=[move_group_node]),
            TimerAction(period=4.0, actions=[rviz_node]),
        ]
    )
