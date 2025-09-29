from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def make_group(ns: str, method: int):
    return GroupAction([
        PushRosNamespace(ns),
        Node(
            package='service_exam2',
            executable='service_exam_server2',
            name='service_exam_server2',
            parameters=[{'calculation_method': method}],
            output='screen'
        )
    ])

def generate_launch_description():
    plus_group  = make_group('plus', 1)
    minus_group = make_group('minus', 2)
    mul_group   = make_group('multiplication', 3)
    div_group   = make_group('division', 4)

    return LaunchDescription([
        plus_group,
        minus_group,
        mul_group,
        div_group,
    ])

