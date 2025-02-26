import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():

    package_name_description='arcs_cohort_description' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name_description),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    package_name_gazebo_sim='arcs_cohort_gazebo_sim'
    default_world=os.path.join(
        get_package_share_directory(package_name_gazebo_sim),
        'worlds',
        'empty.world'
        )
    
    world=LaunchConfiguration('world')

    world_arg=DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

  
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.1'],
                        output='screen')

    bridge_params = os.path.join(
        get_package_share_directory(package_name_gazebo_sim),
        'config',
        'gazebo_bridge.yaml'
    )

    start_gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    ros_gz_image_bridge_left = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/left_camera_image_raw"]
    )

    ros_gz_image_bridge_right = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/right_camera_image_raw"]
    )

    return LaunchDescription([
        rsp,
        world_arg,
        gazebo,
        spawn_entity,
        start_gazebo_ros_bridge,
        ros_gz_image_bridge_left,
        ros_gz_image_bridge_right
    ])
