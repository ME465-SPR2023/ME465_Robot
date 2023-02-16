import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

import yaml


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
    kobuki_node_share = get_package_share_directory("kobuki_node")
    params_file = os.path.join(kobuki_node_share, "config", "kobuki_node_params.yaml")
    with open(params_file) as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    kobuki_node = Node(
        package="kobuki_node",
        executable="kobuki_ros_node",
        parameters=[params],
    )
    stats_node = Node(
        package="ME465_Robot",
        executable="stats_node",
    )
    enable_robot = ExecuteProcess(
        cmd=["ros2", "topic", "pub", "-r", "0.5", "/commands/motor_power", "kobuki_ros_interfaces/msg/MotorPower", "state: 1"],
    )
    kobuki_safety_share = get_package_share_directory("kobuki_safety_controller")
    params_file = os.path.join(kobuki_safety_share, "config", "safety_controller_params.yaml")
    with open(params_file) as f:
        params = yaml.safe_load(f)["kobuki_safety_controller_node"]["ros__parameters"]
    safety_node = Node(
        package="kobuki_safety_controller",
        executable="kobuki_safety_controller_node",
        parameters=[params],
        remappings=[
            ("/cmd_vel", "/safety_vel"),
        ],
    )
    share = get_package_share_directory("ME465_Robot")
    with open(os.path.join(share, "robot.yaml")) as f:
        robot_params = yaml.safe_load(f)
    mux_node = Node(
        package="cmd_vel_mux",
        executable="cmd_vel_mux_node",
        parameters=[
            robot_params["cmd_vel_mux"]["ros__parameters"],
        ],
    )
    smoother_node = Node(
        package="kobuki_velocity_smoother",
        executable="velocity_smoother",
        remappings=[
            ("~/input", "/cmd_vel"),
            ("~/smoothed", "/commands/velocity"),
        ],
        parameters=[
            robot_params["velocity_smoother"]["ros__parameters"],
        ],
    )
    camera_info_node = Node(
        package="ME465_Robot",
        executable="camera_info_node",
    )
    return LaunchDescription([
        usb_cam,
        apriltag,
        kobuki_node,
        #stats_node,
        enable_robot,
        safety_node,
        mux_node,
        smoother_node,
        camera_info_node,
    ])
