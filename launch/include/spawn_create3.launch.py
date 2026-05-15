from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # arguments
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    frame_prefix = PythonExpression(["'", namespace, "/' if '", namespace, "' else ''"])
    x_spawn = LaunchConfiguration('x')
    y_spawn = LaunchConfiguration('y')
    z_spawn = LaunchConfiguration('z')
    yaw_spawn = LaunchConfiguration('yaw')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='create3',
        description='Gazebo robot name'
    )
    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0', description='Spawn x')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0', description='Spawn y')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.1', description='Spawn z')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw')

    # packages
    pmr_tp2_pkg = FindPackageShare('pmr_tp2')

    # filepaths
    xacro_file = PathJoinSubstitution([pmr_tp2_pkg, 'description/create3/create3.urdf.xacro'])
    control_launch_file = PathJoinSubstitution([pmr_tp2_pkg, 'launch/include/control_create3.launch.py'])

    # topic names
    robot_description_topic = PathJoinSubstitution([namespace, 'robot_description'])
    gz_scan_topic = PathJoinSubstitution([namespace, 'scan'])
    gz_odom_topic = PathJoinSubstitution([namespace, 'odom'])

    # robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': Command([
                'xacro ',
                xacro_file,
                ' gazebo:=ignition',
                ' namespace:=', namespace
            ]),
            'frame_prefix': frame_prefix,
            'use_sim_time': True
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': True}],
    )

    # Robot-specific ros-gz bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        name='bridge',
        namespace=namespace,
        parameters=[{
            'bridge_names': ['scan', 'odom'],
            'bridges': {
                'scan': {
                    'ros_topic_name': 'scan',
                    'gz_topic_name': gz_scan_topic,
                    'ros_type_name': 'sensor_msgs/msg/LaserScan',
                    'gz_type_name': 'gz.msgs.LaserScan',
                    'direction': 'GZ_TO_ROS',
                    'qos_profile': 'SENSOR_DATA',
                },
                'odom': {
                    'ros_topic_name': 'odom',
                    'gz_topic_name': gz_odom_topic,
                    'ros_type_name': 'nav_msgs/msg/Odometry',
                    'gz_type_name': 'gz.msgs.Odometry',
                    'direction': 'GZ_TO_ROS',
                },
            },
            'use_sim_time': True
        }],
        output='screen'
    )


    # spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', robot_description_topic,
            '-x', x_spawn, 
            '-y', y_spawn,
            '-z', z_spawn, 
            '-Y', yaw_spawn
        ],
        output='screen'
    )

    # controller setup
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch_file),
        launch_arguments={'namespace': namespace}.items()
    )

    # stamper to correctly expose cmd_vel
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        namespace=namespace,
        parameters=[{'use_sim_time': True}],
        remappings=[('cmd_vel_in','cmd_vel'),
                    ('cmd_vel_out','diffdrive_controller/cmd_vel')]
    )


    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_namespace)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)

    # Add nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)
    ld.add_action(controller)
    ld.add_action(twist_stamper)

    return ld
