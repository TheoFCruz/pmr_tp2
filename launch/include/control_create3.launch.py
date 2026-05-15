# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 with diffdrive controller in Gazebo and optionally also in RViz.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Resolve namespace before delayed controller events run outside the launch context.
    namespace = LaunchConfiguration('namespace').perform(context)
    pmr_tp2_pkg = FindPackageShare('pmr_tp2')

    control_params_file = PathJoinSubstitution(
        [pmr_tp2_pkg, 'config', 'control.yaml'])

    diffdrive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        parameters=[control_params_file],
        arguments=[
            'diffdrive_controller',
            '-c',
            'controller_manager',
            '--controller-manager-timeout',
            '30',
            '--param-file', control_params_file # OVERRIDES DEFAULT FILE TO NOT PUBLISH DRIFTING TF
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            'joint_state_broadcaster',
            '-c',
            'controller_manager',
            '--controller-manager-timeout',
            '30',
            '--param-file', control_params_file
        ],
        output='screen',
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffdrive_controller_node],
        )
    )

    return [
        joint_state_broadcaster_spawner,
        diffdrive_controller_callback
    ]


def generate_launch_description():
    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
