from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Packages
    pmr_tp2_pkg = FindPackageShare('pmr_tp2')

    # Filepaths
    default_map_name = 'bug_map.yaml'
    default_rviz_config_path = PathJoinSubstitution([pmr_tp2_pkg, 'config', 'config.rviz'])

    # Arguments
    map_name = LaunchConfiguration('map_name')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value=default_map_name,
        description='Map YAML file name'
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        'rviz_config_path',
        default_value=default_rviz_config_path,
        description='Path to the RViz configuration file'
    )

    map_path = PathJoinSubstitution([pmr_tp2_pkg, 'maps', map_name])

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_path,
            'use_sim_time': True
        }]
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    # Static Transform Publisher (map -> odom)
    # This allows RViz to show the map even without AMCL or SLAM
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_map_name)
    ld.add_action(declare_rviz_config_path)

    # Add nodes
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(rviz)
    ld.add_action(static_tf)

    return ld
