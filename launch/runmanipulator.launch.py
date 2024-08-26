from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    pick_place_demo = Node(
        package = "gesture_robot",
        executable = "mtc_tutorial",
        output = "screen",
        parameters = [
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])