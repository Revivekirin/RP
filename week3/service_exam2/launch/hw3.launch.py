from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

# PLUS=1, MINUS=2, MULTIPLICATION=3, DIVISION=4
OPS = [
    ("plus",           "plus_server",           1),
    ("minus",          "minus_server",          2),
    ("multiplication", "multiplication_server", 3),
    ("division",       "division_server",       4),
]

def make_group(namespace: str, node_name: str, method: int):
    return GroupAction([
        PushRosNamespace(namespace),
        Node(
            package="service_exam2",
            executable="service_exam_server2",  
            name=node_name,                     
            parameters=[{"calculation_method": method}],
            output="screen",
        ),
    ])

def generate_launch_description():
    groups = [make_group(ns, nn, m) for (ns, nn, m) in OPS]
    return LaunchDescription(groups)