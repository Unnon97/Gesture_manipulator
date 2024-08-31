# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# import os

# def generate_launch_description():
#     ld = LaunchDescription()

#     package_1 = 'py_handgesture'
#     launch_file_1 = os.path.join(get_package_share_directory(package_1, 'launch', 'handsign.launch.py'))
#     ld.add_action(IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(launch_file_1),
#     ))

#     package_2 = 'moveit2_tutorials'
#     launch_file_2 = os.path.join(get_package_share_directory(package_2), 'launch', 'mtc_demo.launch.py')
#     ld.add_action(IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(launch_file_2),
#     ))

#     return ld
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit2_tutorials") + "/launch/mtc.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )
    handgesture_node = Node(
        package='py_handgesture',
        namespace='signrecogniser',
        executable='recogniser',
        output='screen',
    )

    ros2_webcam_node = Node(
        package='ros2_webcam',
        namespace='webcampublisher',
        executable='webcampub',
        output = 'screen'
    )
    handreader_node = Node(
        package="gesture_robot",
        namespace="handreader",
        executable="gesture_subscriber.py",
        output = 'screen'
    )
    moveitmover_node = Node(
        package="gesture_robot",
        namespace="gesture_move",
        executable="gesture_move",
        output = 'screen',
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]
    

    return LaunchDescription([
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        handgesture_node,
        ros2_webcam_node,
        handreader_node,
        moveitmover_node
    ])