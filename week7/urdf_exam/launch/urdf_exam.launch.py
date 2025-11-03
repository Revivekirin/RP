from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution,FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    package_name = 'urdf_exam'

    # === Launch Argument: urdf_file ===
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='my_robot.urdf',
        description='URDF file name inside urdf/ folder'
    )
    
    joint_gui_arg = DeclareLaunchArgument(
        'joint_gui',
        default_value='false',
    )


    urdf_file = LaunchConfiguration('urdf_file')
    joint_gui = LaunchConfiguration('joint_gui')

    
    pkg_share = FindPackageShare(package=package_name)
    
    urdf_path = PathJoinSubstitution([
      pkg_share, 'urdf', urdf_file
    ])
    
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    #robot_description_content = ParameterValue(
    #    Command([FindExecutable(name='xacro'), urdf_path]),
    #    value_type=str
    #)
    
    # Node definitions
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(joint_gui)
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(joint_gui)
    )
        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(
            get_package_share_directory(package_name), 'rviz','urdf.rviz')]
    )
    
    return LaunchDescription([
        urdf_file_arg,
        joint_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui,
        rviz_node
    ])
