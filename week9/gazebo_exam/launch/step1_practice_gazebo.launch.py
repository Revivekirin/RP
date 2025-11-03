import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import LogInfo
from launch.event_handlers import OnProcessIO


import os
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* Windows Error where available
        return None

def spawn_entity(context):
    package = FindPackageShare('gazebo_exam')
    models = {
        "box1": "box_red.xacro",
        "box2": "box_blue.xacro",
    }
    models_entity ={
        "box1": {
            "xacro_args": {
                "box_size_x": "0.05",
                "box_size_y": "0.05",
                "box_size_z": "0.06",
                "box_offset_x": "0.4",
                "box_offset_y": "0.1",
                "box_offset_z": "0.03",
            }
        },
        "box2": {
            "xacro_args": {
                "box_size_x": "0.05",
                "box_size_y": "0.05",
                "box_size_z": "0.06",
                "box_offset_x": "0.4",
                "box_offset_y": "-0.1",
                "box_offset_z": "0.03",
            }
        },
    }

    spawn_nodes = []
    for i, (model_name, xacro_file) in enumerate(models.items()):

        entity_data = models_entity.get(model_name, {})
        xacro_args_dict = entity_data.get("xacro_args", {})
        xacro_args_list=[]
        for arg_name, arg_value in xacro_args_dict.items():
            xacro_args_list.append(f" {arg_name}:={arg_value}")
        print(xacro_args_list)

        xacro_path = os.path.join(package.perform(context), 'models', xacro_file)
        robot_description_content = Command(
            [
                "xacro ",
                PathJoinSubstitution([package,"models",xacro_file]),
                *xacro_args_list,
            ]
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=f"{model_name}_rsp", # 노드 이름 중복 방지
            output="screen",
            parameters=[{"use_sim_time": True},
                        {"robot_description": robot_description_content}],
            remappings=[("/robot_description", f"/{model_name}/robot_description")],
        )

        spawn_entity_node = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name=f"spawn_{model_name}", # 노드 이름 중복 방지
            arguments=[
                "-entity", model_name,
                "-topic", f"/{model_name}/robot_description",
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.0",],
            output="screen",
        )

        delay_spawn_node = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=robot_state_publisher_node,
                on_start=[spawn_entity_node]
            )
        )

        spawn_nodes.append(robot_state_publisher_node)
        spawn_nodes.append(delay_spawn_node)

    return spawn_nodes

def launch_setup(context, *args, **kwargs):

    package_name = "gazebo_exam"
    ee_id = 'franka_hand'
    arm_id = 'fr3'

    package = FindPackageShare(package_name)

    # Initialize Arguments
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_joint_controllers = PathJoinSubstitution(
        [package, "controller", "fr3_controller_gripper.yaml"]
    )
    robot_description_content = Command(
        [
            # PathJoinSubstitution([FindExecutable(name="xacro")]),
            "xacro ",
            PathJoinSubstitution([package,"urdf","robots",arm_id,arm_id+".urdf.xacro"]),
            " ",
            "sim_gazebo:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    #############################################################################
    #TODO Gazebo
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, initial_joint_controllers],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    manipulator_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        # arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        arguments=["manipulator_controller", "-c", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        # arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )
    
    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_fr3",
        arguments=["-entity", "fr3", "-topic", "robot_description"],
        output="screen",
    )

    # Delay start joint_state_broadcaster
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Delay start of robot_controller
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[manipulator_controller_spawner, gripper_controller_spawner],
        )
    )

    spawn_entities = spawn_entity(context)

    delay_entities_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[*spawn_entities]
        )
    )

    nodes_to_start = [
                gazebo,
                gazebo_spawn_robot,

                robot_state_publisher_node,
                ros2_control_node,

                delay_joint_state_broadcaster_spawner,
                delay_robot_controller_spawner,
                delay_entities_spawner,
                ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="fr3"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
            If changed than also joint names in the controllers configuration have to be updated."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "cmd",
            default_value="1",
            description="Command value for MoveIt demo (0:exit, 1:move, 2:rotation, 3:gripper, 4:waypoint)"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
