from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'fake_ur_config'
    use_sim_time = LaunchConfiguration('use_sim_time', default=False) 

    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
         PathJoinSubstitution([FindPackageShare(pkg_name), 'config', 'fake_ur_config.urdf.xacro']), ' ', 'use_gazebo:=false'])
    robot_description = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time} 
    
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(pkg_name), 'config', 'ros2_controllers.yaml'])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name), 'config', 'view_robot.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    controller_manager_spawner = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers,
            {'grtp_path': PathJoinSubstitution([FindPackageShare(pkg_name), 'config', 'grtp.yaml'])},
        ],
        output="both",
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers,
            ],
    )
    
    jtc_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'trajectory_controller',
            '--param-file',
            robot_controllers,
            ],
    )
    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'effort_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    cia402_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'cia402_controller',
            '--param-file',
            robot_controllers,
            ],
    )


    delayed_jtc_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[
                TimerAction(
                    period=20.0,
                    actions=[jtc_controller_spawner, effort_controller_spawner]
                )
            ]
        )
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='If true, use simulated clock. Set to false for RViz viewing.'),
        
        node_robot_state_publisher,
        rviz_node,
        
        controller_manager_spawner,
        jsb_spawner,
        cia402_spawner,
        delayed_jtc_spawner, 
    ])