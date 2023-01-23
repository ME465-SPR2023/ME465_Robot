import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription


def generate_launch_description():
    usb_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="camera",
    )
    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        parameters=[{
            "tag_threads": 3,
        }],
        remappings=[
            ("image_rect", "/image_raw"),
        ],
    )
    kobuki_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("kobuki_node"),
                "launch/kobuki_node-launch.py"
            )
        )
    )
    stats_node = Node(
        package="ME465_Robot",
        executable="stats_node",
    )
    enable_robot = ExecuteProcess(
        cmd=["ros2", "topic", "pub", "-r", "0.5", "kobuki_ros_interfaces/msg/MotorPower", "state: 1"],
    )
    return LaunchDescription([
        usb_cam,
        apriltag,
        kobuki_include,
        stats_node,
        enable_robot,
    ])
