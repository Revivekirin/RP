from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    package_name = "gazebo_exam"
    ee_id = 'franka_hand'
    arm_id = 'fr3'

    package = FindPackageShare(package_name)    


    # Initialize Arguments
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")

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

    rviz_config_file = PathJoinSubstitution(
        [package, "rviz", "indy.rviz"]
    )

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

    # joint_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    # )

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

    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
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

    # # Delay rviz
    # delay_rviz2_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=manipulator_controller_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    nodes_to_start = [
        gazebo,
        gazebo_spawn_robot,

        robot_state_publisher_node,
        ros2_control_node,

        # joint_state_broadcaster_spawner,
        # joint_controller_spawner,

        delay_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        # delay_rviz2_spawner,
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
            "launch_rviz", 
            default_value="true", 
            description="Launch RViz?"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
    