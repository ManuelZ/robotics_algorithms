# Standard-Library imports
import os

# External imports
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir  = get_package_share_directory('robotics_algorithms')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world        = LaunchConfiguration('world', default='webots_world_file.wbt')
    rviz_config_path = os.path.join(package_dir, 'resource', 'config.rviz')
    
    # The WebotsLauncher is a Webots custom action that allows you to start a Webots simulation instance.
    # It searches for the Webots installation in the path specified by the `WEBOTS_HOME` environment variable and default installation paths.
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        # Apparently,the supervisor node is the one that allows to 
        # have an automatic broadcast of tf frames for all the elements in the 
        # robot's PROTO
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'webots_robot_description.urdf')
    ros2_control_params_path = os.path.join(package_dir, 'resource', 'ros2control.yaml')

    epuck_driver = WebotsController(
        robot_name='e-puck',
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True
            },
            ros2_control_params_path
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/diffdrive_controller/odom', '/odom')
        ],
        respawn=True
    )

    # Often we want to publish robot transforms, so we use the `robot_state_publisher` node for that.
    # If robot model is not specified in the URDF file then Webots can help us with the URDF exportation feature.
    # Since the exportation feature is available only once the simulation has started and the `robot_state_publisher`
    # node requires a `robot_description` parameter before we have to specify a dummy robot.
    robot_state_publisher = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # The base_footprint is the representation of the robot position on the floor.
    # https://www.ros.org/reps/rep-0120.html#base-footprint
    footprint_publisher = Node(
        package    = 'tf2_ros',
        executable = 'static_transform_publisher',
        output     = 'screen',
        arguments  = [
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_footprint'
        ],
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix  = 'python.exe' if os.name == 'nt' else ''

    diffdrive_controller_spawner = Node(
        package    = 'controller_manager',
        executable = 'spawner',
        output     = 'screen',
        prefix     = controller_manager_prefix,
        arguments  = ['diffdrive_controller'] + controller_manager_timeout,
        parameters = [
            {'use_sim_time': use_sim_time},
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package    = 'controller_manager',
        executable = 'spawner',
        output     = 'screen',
        prefix     = controller_manager_prefix,
        arguments  = ['joint_state_broadcaster'] + controller_manager_timeout,
        parameters = [
            {'use_sim_time': use_sim_time},
        ]
    )

    nav_rviz = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rviz_config_path],
        parameters = [{
            'use_sim_time': use_sim_time
        }]
    )

    # Wait for the simulation to be ready to start the tools and/or spawners
    waiting_control_nodes = WaitForControllerConnection(
        target_driver = epuck_driver,
        nodes_to_start = [nav_rviz, diffdrive_controller_spawner, joint_state_broadcaster_spawner]
    )


    occupancy_grid = Node(
        package    = 'robotics_algorithms',
        executable = 'occupancy_grid',
        output     = 'screen',
        parameters = [
            {'use_sim_time': use_sim_time},
        ],
    )




    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='webots_world_file.wbt'
        ),

        webots,
        webots._supervisor,

        robot_state_publisher,
        footprint_publisher,

        epuck_driver,
        occupancy_grid,
        waiting_control_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
