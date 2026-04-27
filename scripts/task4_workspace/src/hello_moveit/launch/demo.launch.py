from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("smmg", package_name="smmg").to_moveit_configs())

    return LaunchDescription([
        Node(
            package="hello_moveit",
            executable = "hello_moveit",
            parameters=[
                moveit_config.to_dict(),
            ]
        )
    ])
    
    return LaunchDescription()
