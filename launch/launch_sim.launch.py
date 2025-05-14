import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME



    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge.yaml'
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[bridge_params]
    )

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
        )    
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    
    world = LaunchConfiguration('world')

    # Include the Gazebo launch file, provided by the ros_gz_sim package

    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )
    
    gazebo = IncludeLaunchDescription(
                gazebo_pkg_launch,
        launch_arguments={
            'gz_args': [f'-r -v 4 ', world],
            'on_exit_shutdown': 'true'
        }.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot'],
                        output='screen')



    # Launch them all!
    return LaunchDescription([
        world_arg,
        rsp,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
    ])