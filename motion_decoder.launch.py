from os import name, path, environ, getcwd

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory


def generate_launch_description():
    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        parameters=[{}],
        remappings=[
            ("image_rect", "/usb_cam/image_raw"),
            ("camera_info", "/usb_cam/camera_info"),
        ],
    )

    image_converter = Node(
        package="motion_decoder",
        executable="image_converter",
        parameters=[{}],
        remappings=[("image_rect", "/usb_cam/image_raw")],
    )

    trajectory_server = Node(
        package="trajectory_server",
        executable="trajectory_server",
        parameters=[
            {
                "target_frame_name": "tag36h11:0",
                "source_frame_name": "usb_cam",
            }
        ],
        remappings=[],
    )

    # To stop a node from running, simply remove it here.
    return LaunchDescription([apriltag, image_converter, trajectory_server])
