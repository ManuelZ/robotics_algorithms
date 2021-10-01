import os
import pathlib
import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_core.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir         = get_package_share_directory('slam_algorithms')
    use_sim_time        = LaunchConfiguration('use_sim_time', default=True)
    world               = LaunchConfiguration('world', default='webots_world_file.wbt')
    robot_description   = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_robot_description.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')
    rviz_config         = os.path.join(package_dir, 'resource', 'config.rviz')
    

    # The WebotsLauncher is a Webots custom action that allows you to start a Webots simulation instance.
    # It searches for the Webots installation in the path specified by the `WEBOTS_HOME` environment variable and default installation paths.
    # The accepted arguments are:
    # - `world` (str): Path to the world to launch.
    # - `gui` (bool): Whether to display GUI or not.
    # - `mode` (str): Can be `pause`, `realtime`, or `fast`.
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world])
    )


    # The node which interacts with a robot in the Webots simulation is located in the `webots_ros2_driver` package under name `driver`.
    # It is necessary to run such a node for each robot in the simulation.
    # Typically, we provide it the `robot_description` parameters from a URDF file and `ros2_control_params` from the `ros2_control` configuration file.
    webots_robot_driver = Node(
        package    = 'webots_ros2_driver',
        executable = 'driver',
        output     = 'screen',
        remappings = [
            ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
        ],
        parameters = [
            {
                'robot_description': robot_description,
                'use_sim_time'     : use_sim_time
            },
            ros2_control_params
        ],
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
        # x y z qx qy qz qw frame_id child_frame_id 
        arguments  = ['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50'] if os.name == 'nt' else []
    controller_manager_prefix  = 'python.exe' if os.name == 'nt' else "bash -c 'sleep 10; $0 $@' "

    diffdrive_controller_spawner = Node(
        package    = 'controller_manager',
        executable = 'spawner.py',
        output     = 'screen',
        prefix     = controller_manager_prefix,
        arguments  = ['diffdrive_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package    = 'controller_manager',
        executable = 'spawner.py',
        output     = 'screen',
        prefix     = controller_manager_prefix,
        arguments  = ['joint_state_broadcaster'] + controller_manager_timeout,
    )

    occupancy_grid = Node(
        package    = 'slam_algorithms',
        executable = 'occupancy_grid',
        output     = 'screen',
        parameters = [
            {'use_sim_time': use_sim_time},
        ],
    )

    nav_rviz = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rviz_config],
        parameters = [{
            'use_sim_time': use_sim_time
        }]
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([

        # Start the Webots node
        webots,

        joint_state_broadcaster_spawner,
        
        diffdrive_controller_spawner,

        webots_robot_driver,

        robot_state_publisher,

        footprint_publisher,

        occupancy_grid,

        nav_rviz,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action = webots,
                on_exit       = [
                    launch.actions.EmitEvent(event = launch.events.Shutdown())
                ],
            )
        )
    ])
