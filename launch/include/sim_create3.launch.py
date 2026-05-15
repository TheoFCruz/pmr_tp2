from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # arguments
    world_name = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')

    declare_world = DeclareLaunchArgument('world', default_value='empty.sdf', description='Gazebo world name')
    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='create3',
        description='Gazebo robot name'
    )
    
    # packages
    pmr_tp2_pkg = FindPackageShare('pmr_tp2')
    gazebo_pkg = FindPackageShare('ros_gz_sim')

    # filepaths
    world_path = PathJoinSubstitution([pmr_tp2_pkg, 'worlds', world_name])
    bridge_file = PathJoinSubstitution([pmr_tp2_pkg, 'config/bridge.yaml'])

    # Start gazebo and spawn robot
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_pkg, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true'
        }.items(),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pmr_tp2_pkg, 'launch', 'include', 'spawn_create3.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'robot_name': robot_name
        }.items()
    )

    # Global ros-gz bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_file,
            'use_sim_time': True
        }],
        output='screen'
    )

    ld = LaunchDescription()

    # Arguments
    ld.add_action(declare_world)
    ld.add_action(declare_namespace)
    ld.add_action(declare_robot_name)

    # Nodes and launches
    ld.add_action(gazebo)
    ld.add_action(bridge)
    ld.add_action(spawn)

    return ld
