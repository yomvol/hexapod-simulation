from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable

from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('hexapod_bringup')
    pkg_project_description = get_package_share_directory('hexapod_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = PathJoinSubstitution([
        pkg_project_description,
        'models',
        'phantomx',
        'urdf',
        'phantomx.xacro'
    ])
    # xacro_file  =  os.path.join(pkg_project_description, 'models', 'phantomx', 'urdf', 'phantomx.xacro')
    generated_urdf = Command([FindExecutable(name='xacro'), ' ', xacro_file])
    # with open(generated_urdf, 'r') as infp:
    #     robot_desc = infp.read()
    robot_desc = {'robot_description': generated_urdf}


    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments=[
            ('gz_args', [PathJoinSubstitution([pkg_project_bringup, 'worlds', 'world.sdf']),
                         ' -v 1', ' -r'])] # minimal verbosity level is 1, adjust up to 4 included on need
                         
        # launch_arguments={'gz_args': PathJoinSubstitution([
        #     pkg_project_bringup,
        #     'worlds',
        #     'world.sdf',
        # ]),}.items()
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[robot_desc],
        output='screen'
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            robot_desc,
        ]
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', PathJoinSubstitution([pkg_project_bringup, 'config', 'phantomx.rviz'])],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.3'
        ],
        output='screen'
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     parameters=[{
    #         'config_file': PathJoinSubstitution([pkg_project_bringup, 'config', 'ros_gz_bridge.yaml']),
    #         'qos_overrides./tf_static.publisher.durability': 'transient_local',
    #     }],
    #     output='screen'
    # )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawn leg controllers
    leg1_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg1_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    leg2_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg2_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    leg3_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg3_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    leg4_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg4_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    leg5_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg5_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    leg6_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg6_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        robot_state_publisher,
        spawn_entity,
        joint_state_publisher_gui,
        rviz,
        joint_state_broadcaster_spawner,
        leg1_controller_spawner,
        leg2_controller_spawner,
        leg3_controller_spawner,
        leg4_controller_spawner,
        leg5_controller_spawner,
        leg6_controller_spawner
    ])