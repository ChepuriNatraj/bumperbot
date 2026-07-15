import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            bumperbot_description_dir, "urdf", "robot.urdf.xacro"
        ),
        description="Absolute path to robot urdf file",
    )

    # New: rviz toggle. Defaults to "false" - pass rviz:=true to launch it.
    rviz_arg = DeclareLaunchArgument(
        name="rviz",
        default_value="false",
        description="Whether to launch rviz2 (true/false)",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bumperbot_description_dir, "rviz", "1display.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])