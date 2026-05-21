from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Packages
    pmr_tp2_pkg = FindPackageShare('pmr_tp2')

    # Filepaths
    default_rviz_config_path = PathJoinSubstitution([pmr_tp2_pkg, 'rviz', 'astar.rviz'])

    # Arguments
    world_name = LaunchConfiguration('world')
    map_name = LaunchConfiguration('map_name')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world name'
    )

    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='small_maze.yaml',
        description='Map YAML file name'
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        'rviz_config_path',
        default_value=default_rviz_config_path,
        description='Path to the RViz configuration file'
    )

    # Includes
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pmr_tp2_pkg, 'launch', 'include', 'sim_create3.launch.py'])
        ),
        launch_arguments={'world': world_name}.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pmr_tp2_pkg, 'launch', 'include', 'rviz_map.launch.py'])
        ),
        launch_arguments={
            'map_name': map_name,
            'rviz_config_path': rviz_config_path
        }.items()
    )

    # A* Planner Node
    astar_node = Node(
        package='pmr_tp2',
        executable='astar',
        name='a_star',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_world)
    ld.add_action(declare_map_name)
    ld.add_action(declare_rviz_config_path)

    # Add actions
    ld.add_action(sim_launch)
    ld.add_action(rviz_launch)
    ld.add_action(astar_node)

    return ld
