import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bumperbot_description = get_package_share_directory(
        "bumperbot_description"
    )

    wheel_imu_localization = get_package_share_directory(
        "wheel_imu_localization"
    )

    rplidar_ros = get_package_share_directory(
        "rplidar_ros"
    )

    slam_toolbox = get_package_share_directory(
        "slam_toolbox"
    )

    # -----------------------------
    # Robot Description + RViz
    # -----------------------------

    robot_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bumperbot_description,
                "launch",
                "robot_display.launch.py"
            )
        )
    )

    # -----------------------------
    # IMU
    # -----------------------------

    imu_node = Node(
        package="robot_firmware",
        executable="imu_serial",
        output="screen"
    )

    # -----------------------------
    # Motor Serial
    # -----------------------------

    motor_node = Node(
        package="robot_firmware",
        executable="motor_serial",
        output="screen"
    )

    # -----------------------------
    # LiDAR
    # -----------------------------

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rplidar_ros,
                "launch",
                "rplidar_a2m12_launch.py"
            )
        ),
        launch_arguments={
            "frame_id": "laser_link"
        }.items()
    )

    # -----------------------------
    # Wheel Odometry
    # -----------------------------

    odom_node = Node(
        package="wheel_odometry",
        executable="odom_nodef",
        output="screen"
    )

    # -----------------------------
    # EKF
    # -----------------------------

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                wheel_imu_localization,
                "launch",
                "ekf.launch.py"
            )
        )
    )

    # -----------------------------
    # Teleop
    # -----------------------------

    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        prefix="xterm -e",
        output="screen"
    )

    # -----------------------------
    # SLAM Toolbox
    # -----------------------------

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox,
                "launch",
                "online_async_launch.py"
            )
        )
    )

    return LaunchDescription([

        robot_display,

        imu_node,

        motor_node,

        lidar,

        odom_node,

        ekf,

        teleop,

        slam,

    ])