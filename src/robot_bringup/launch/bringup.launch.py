from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bumperbot = get_package_share_directory("bumperbot_description")
    wheel_localization = get_package_share_directory("wheel_imu_localization")
    rplidar = get_package_share_directory("rplidar_ros")
    nav2 = get_package_share_directory("nav2_bringup")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bumperbot,
                "launch",
                "robot_display.launch.py"
            )
        )
    )

    imu = Node(
        package="robot_firmware",
        executable="imu_serial",
        output="screen"
    )

    motor = Node(
        package="robot_firmware",
        executable="motor_serial",
        output="screen"
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rplidar,
                "launch",
                "rplidar_a2m12_launch.py"
            )
        ),
        launch_arguments={
            "frame_id": "laser_link"
        }.items()
    )

    odom = Node(
        package="wheel_odometry",
        executable="odom_nodef",
        output="screen"
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                wheel_localization,
                "launch",
                "ekf.launch.py"
            )
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2,
                "launch",
                "localization_launch.py"
            )
        ),
        launch_arguments={
            "map": "/home/botforgelabs2/Desktop/bumperbot/maps/my_map.yaml",
            "use_sim_time": "false",
            "params_file": "/home/botforgelabs2/Desktop/bumperbot/src/robot_bringup/config/nav2_params_default.yaml"
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2,
                "launch",
                "navigation_launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": "/home/botforgelabs2/Desktop/bumperbot/src/robot_bringup/config/nav2_params_default.yaml"
        }.items()
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            "/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"
        ],
        output="screen"
    )

    return LaunchDescription([

        # 0 sec
        robot,

        # 2 sec
        TimerAction(
            period=2.0,
            actions=[imu]
        ),

        # 4 sec
        TimerAction(
            period=4.0,
            actions=[motor]
        ),

        # 7 sec
        TimerAction(
            period=7.0,
            actions=[lidar]
        ),

        # 11 sec
        TimerAction(
            period=11.0,
            actions=[odom]
        ),

        # 14 sec
        TimerAction(
            period=14.0,
            actions=[ekf]
        ),

        # 18 sec
        TimerAction(
            period=18.0,
            actions=[localization]
        ),

        # 24 sec
        TimerAction(
            period=24.0,
            actions=[navigation]
        ),

        # 30 sec
        TimerAction(
            period=30.0,
            actions=[rviz]
        ),
    ])