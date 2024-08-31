from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    runmanipulator = Node(
        package = "gesture_robot",
        executable = "gesture_move",
        output = "screen",
        parameters = [
            moveit_config,
        ],
    )

    return LaunchDescription([runmanipulator])