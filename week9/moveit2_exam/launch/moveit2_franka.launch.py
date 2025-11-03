# import os
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

def launch_setup(context, *args, **kwargs):

    package_name = "moveit2_exam"
    ee_id = 'franka_hand'
    arm_id = 'fr3'

    package = FindPackageShare(package_name)

    # Initialize Arguments
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
    use_sim_time = LaunchConfiguration("use_sim_time")

    symphony_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [package, "/launch", "/gazebo_franka.launch.py"]
        ),
        launch_arguments={
            "name": name,
            "prefix": prefix,
            "launch_rviz": "false",
        }.items(),
    )

    robot_description_content = Command(
        [
            # PathJoinSubstitution([FindExecutable(name="xacro")]),
            "xacro ",
            PathJoinSubstitution([package,"urdf","robots",arm_id,arm_id+".urdf.xacro"]),
            " ",
            "sim_gazebo:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            # PathJoinSubstitution([FindExecutable(name="xacro")]),
            "xacro ",
            PathJoinSubstitution([package,"srdf",arm_id+".srdf.xacro"])
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [package, "moveit_config", "kinematics.yaml"]
    )


    joint_limit_yaml = load_yaml(package_name, "moveit_config/joint_limits_6dof_gripper.yaml")
    robot_description_planning = {
        "robot_description_planning": joint_limit_yaml
    }

    # ompl_planning_pipeline_config = {
    #     "move_group": {
    #         "planning_plugins": ["ompl_interface/OMPLPlanner"],
    #         "request_adapters": [
    #             "default_planning_request_adapters/ResolveConstraintFrames",
    #             "default_planning_request_adapters/ValidateWorkspaceBounds",
    #             "default_planning_request_adapters/CheckStartStateBounds",
    #             "default_planning_request_adapters/CheckStartStateCollision",
    #         ],
    #         "response_adapters": [
    #             "default_planning_response_adapters/AddTimeOptimalParameterization",
    #             "default_planning_response_adapters/ValidateSolution",
    #             "default_planning_response_adapters/DisplayMotionPath",
    #         ],
    #     }
    # }
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.5,
        }
    }
    ompl_planning_yaml = load_yaml(package_name, "moveit_config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml(package_name, "moveit_config/controllers_6dof_gripper.yaml")

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.5,
        "trajectory_execution.trajectory_duration_monitoring": False
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description":True, 
        "publish_robot_description_semantic":True
    }
    
    # Start the actual move_group node/action server  
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [package, "rviz", "moveit_franka.rviz"]
    )

    rviz_node = Node(
        condition=IfCondition(launch_rviz_moveit),
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
        ],
    )

    nodes_to_start = [
                symphony_gazebo_launch,
                move_group_node,
                rviz_node]

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
            default_value="true",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz_moveit", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
