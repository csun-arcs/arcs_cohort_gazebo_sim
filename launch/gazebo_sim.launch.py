import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
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
    # Package names
    pkg_gazebo_sim = 'arcs_cohort_gazebo_sim'
    pkg_description = 'arcs_cohort_description'

    # Paths
    default_world_path = os.path.join(
        get_package_share_directory(pkg_gazebo_sim),
        'worlds',
        'empty.world'
    )
    default_model_path = 'description/robot.urdf.xacro'

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Path to the world file to load'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    declare_model_package_cmd = DeclareLaunchArgument(
        'model_package',
        default_value=pkg_description,
        description='Package containing the robot model'
    )
    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        default_value=default_model_path,
        description='Relative path to the robot model file'
    )
    declare_use_rsp_cmd = DeclareLaunchArgument(
        'use_rsp',
        default_value='true',
        description='Launch robot_state_publisher'
    )
    declare_use_jsp_cmd = DeclareLaunchArgument(
        'use_jsp',
        default_value='false',
        description='Launch joint_state_publisher'
    )
    declare_use_jsp_gui_cmd = DeclareLaunchArgument(
        'use_jsp_gui',
        default_value='false',
        description='Launch joint_state_publisher_gui'
    )

    # Launch configurations
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model_package = LaunchConfiguration('model_package')
    model_file = LaunchConfiguration('model_file')
    use_rsp = LaunchConfiguration('use_rsp')
    use_jsp = LaunchConfiguration('use_jsp')
    use_jsp_gui = LaunchConfiguration('use_jsp_gui')

    # Robot description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare(model_package),
            model_file
        ]),
        ' use_sim_time:=', use_sim_time
    ])
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(use_rsp),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Joint State Publisher node
    jsp_node = Node(
        condition=IfCondition(
            PythonExpression(
                ["'", use_jsp, "' == 'true' and '", use_jsp_gui, "' != 'true'"]
            )
        ),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Joint State Publisher GUI node
    jsp_gui_node = Node(
        condition=IfCondition(use_jsp_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world],
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # Spawn robot into Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'cohort',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Gazebo bridge parameters
    bridge_params = os.path.join(
        get_package_share_directory(pkg_gazebo_sim),
        'config',
        'gazebo_bridge.yaml'
    )

    # Start the Gazebo ROS bridge
    start_gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params}],
        output='screen'
    )

    # Image bridges for cameras
    ros_gz_image_bridge_left = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/left_camera_image_raw'],
        output='screen'
    )

    ros_gz_image_bridge_right = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/right_camera_image_raw'],
        output='screen'
    )

    return LaunchDescription([
        declare_world_cmd,
        declare_use_sim_time_cmd,
        declare_model_package_cmd,
        declare_model_file_cmd,
        declare_use_rsp_cmd,
        declare_use_jsp_cmd,
        declare_use_jsp_gui_cmd,
        rsp_node,
        jsp_node,
        jsp_gui_node,
        gazebo_launch,
        spawn_entity,
        start_gazebo_ros_bridge,
        ros_gz_image_bridge_left,
        ros_gz_image_bridge_right,
    ])
