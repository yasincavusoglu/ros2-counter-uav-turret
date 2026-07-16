"""Closed-loop counter-UAS turret simulation launch.

Brings up the full node graph:
    scenario_sim -> detector -> tracker -> estimator -> lead_solver
                 -> gimbal_control -> turret_plant -> (feedback) ; engagement_fsm ; hmi_telemetry

Usage:
    ros2 launch counter_uav_turret sim.launch.py
    ros2 launch counter_uav_turret sim.launch.py record:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'counter_uav_turret'
NODES = [
    'scenario_sim', 'detector', 'tracker', 'estimator', 'lead_solver',
    'gimbal_control', 'turret_plant', 'engagement_fsm',
]


def generate_launch_description():
    record = LaunchConfiguration('record')
    ld = LaunchDescription([
        DeclareLaunchArgument('record', default_value='false',
                              description='Record an MP4 demo via hmi_telemetry.'),
    ])
    for exe in NODES:
        ld.add_action(Node(package=PKG, executable=exe, name=exe, output='screen'))
    ld.add_action(Node(package=PKG, executable='hmi_telemetry', name='hmi_telemetry',
                       output='screen', parameters=[{'record': record}]))
    return ld
