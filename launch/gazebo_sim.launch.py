import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

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
    pkg_gazebo_sim = "arcs_cohort_gazebo_sim"
    pkg_description = "arcs_cohort_description"
    pkg_nav = "arcs_cohort_navigation"

    # Defaults
    default_world_path = os.path.join(
        get_package_share_directory(pkg_gazebo_sim),
        "worlds",
        "test_obstacles_world_1.world",
    )
    default_model_path = "description/robot.urdf.xacro"
    default_joystick_params_path = os.path.join(
        get_package_share_directory(pkg_gazebo_sim),
        "config",
        "gazebo_joystick_teleop.yaml",
    )
    default_twist_mux_params_path = os.path.join(
        get_package_share_directory(pkg_nav), "config", "twist_mux.yaml"
    )
    default_log_level = "INFO"

    # Declare launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace under which to bring up nodes, topics, etc.",
    )
    declare_prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description=(
            "A prefix for the names of joints, links, etc. in the robot model). "
            "E.g. 'base_link' will become 'cohort1_base_link' if prefix "
            "is set to 'cohort1'."
        ),
    )
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="Path to the world file to load",
    )
    declare_model_package_arg = DeclareLaunchArgument(
        "model_package",
        default_value=pkg_description,
        description="Package containing the robot model",
    )
    declare_model_file_arg = DeclareLaunchArgument(
        "model_file",
        default_value=default_model_path,
        description="Relative path to the robot model file",
    )
    declare_camera_resolution_arg = DeclareLaunchArgument(
        "camera_resolution",
        default_value="VGA",
        description=(
            "Resolution profile of the simulated Stereolabs Zed camera."
            'Options: "HD2K" (2208x1242), "HD1080" (1920x1080), '
            '"HD720" (1280x720) or "VGA" (672x376).'
        ),
    )
    declare_lidar_update_rate_arg = DeclareLaunchArgument(
        "lidar_update_rate",
        default_value="30",
        description="Set the update rate of the LiDAR sensor.",
    )
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=default_log_level,
        description="Set the log level for nodes.",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    declare_use_rsp_arg = DeclareLaunchArgument(
        "use_rsp", default_value="true", description="Launch robot_state_publisher"
    )
    declare_use_jsp_arg = DeclareLaunchArgument(
        "use_jsp", default_value="false", description="Launch joint_state_publisher"
    )
    declare_use_jsp_gui_arg = DeclareLaunchArgument(
        "use_jsp_gui",
        default_value="false",
        description="Launch joint_state_publisher_gui",
    )
    declare_use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="false",
        description="If true, include the lidar in the robot description",
    )
    declare_use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="false",
        description="Use ROS2 Control for the robot",
    )
    declare_use_joystick_arg = DeclareLaunchArgument(
        "use_joystick",
        default_value="false",
        description="Use joystick to control the robot",
    )
    declare_use_keyboard_arg = DeclareLaunchArgument(
        "use_keyboard",
        default_value="false",
        description="Use keyboard to control the robot",
    )
    declare_use_navigation_arg = DeclareLaunchArgument(
        "use_navigation",
        default_value="false",
        description="Set the argument to true if you want to launch twist mux",
    )

    # Launch configurations
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    world = LaunchConfiguration("world")
    model_package = LaunchConfiguration("model_package")
    model_file = LaunchConfiguration("model_file")
    camera_resolution = LaunchConfiguration("camera_resolution")
    lidar_update_rate = LaunchConfiguration("lidar_update_rate")
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rsp = LaunchConfiguration("use_rsp")
    use_jsp = LaunchConfiguration("use_jsp")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    use_lidar = LaunchConfiguration("use_lidar")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_joystick = LaunchConfiguration("use_joystick")
    use_keyboard = LaunchConfiguration("use_keyboard")
    use_navigation = LaunchConfiguration("use_navigation")

    # Robot description from Xacro, including the conditional robot name prefix.
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution([FindPackageShare(model_package), model_file]),
            " prefix:=",
            prefix,
            " camera_resolution:=",
            camera_resolution,
            " use_lidar:=",
            use_lidar,
            " lidar_update_rate:=",
            lidar_update_rate,
            " use_ros2_control:=",
            use_ros2_control,
            " use_joystick:=",
            use_joystick,
            " use_keyboard:=",
            use_keyboard,
        ]
    )

    # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(use_rsp),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Joint State Publisher node
    jsp_node = Node(
        condition=IfCondition(
            PythonExpression(
                ["'", use_jsp, "' == 'true' and '", use_jsp_gui, "' != 'true'"]
            )
        ),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Joint State Publisher GUI node
    jsp_gui_node = Node(
        condition=IfCondition(use_jsp_gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Jostick Node
    joy_node = Node(
        condition=IfCondition(PythonExpression(["'", use_joystick, "' == 'true'"])),
        package="joy",
        executable="joy_node",
        parameters=[default_joystick_params_path],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Teleop with joystick
    teleop_joy_node = Node(
        condition=IfCondition(PythonExpression(["'", use_joystick, "' == 'true'"])),
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[default_joystick_params_path, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "diff_cont/cmd_vel_joy_unstamped")],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Twist stamper for joystick teleop node
    teleop_joy_stamper_node = Node(
        condition=IfCondition(PythonExpression(["'", use_joystick, "' == 'true'"])),
        package="twist_stamper",
        executable="twist_stamper",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel_in", "diff_cont/cmd_vel_joy_unstamped"),
            ("cmd_vel_out", "diff_cont/cmd_vel"),
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Teleop with keyboard
    teleop_keyboard_node = Node(
        condition=IfCondition(PythonExpression(["'", use_keyboard, "' == 'true'"])),
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        prefix="xterm -e",
        parameters=[{"stamped": True}],
        remappings=[("cmd_vel", "diff_cont/cmd_vel")],
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn robot into Gazebo
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "cohort",
            "-z",
            "0.1",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        output="screen",
    )

    # Differential drive controller spawner
    diff_drive_spawner_node = Node(
        condition=IfCondition(use_ros2_control),
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--ros-args", "--log-level", log_level],
    )

    # Joint state broadcaster spawner
    joint_broad_spawner_node = Node(
        condition=IfCondition(use_ros2_control),
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--ros-args", "--log-level", log_level],
    )

    # Gazebo bridge parameters
    bridge_params = os.path.join(
        get_package_share_directory(pkg_gazebo_sim), "config", "gazebo_bridge.yaml"
    )

    # Start the Gazebo ROS bridge
    start_gazebo_ros_bridge_node = Node(
        name="parameter_bridge",
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_params}],
        output="screen",
        arguments=["--ros-args", "-p", "expand_gz_topic_names:=true",
                   "--ros-args", "--log-level", log_level],
        remappings=[
            ("/tf", "tf"),
        ],
    )

    # Image bridge for left camera image
    ros_gz_image_bridge_left_node = Node(
        name="left_camera_image_bridge",
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[[namespace, "/camera/left_camera/image"], "--ros-args", "--log-level", log_level],
        output="screen",
        remappings=[
            (
                [namespace, "/camera/left_camera/image"],
                "camera/left_camera/image",
            ),
            (
                [namespace, "/camera/left_camera/image/compressed"],
                "camera/left_camera/image/compressed",
            ),
            (
                [namespace, "/camera/left_camera/image/compressedDepth"],
                "camera/left_camera/image/compressedDepth",
            ),
            (
                [namespace, "/camera/left_camera/image/theora"],
                "camera/left_camera/image/theora"
            ),
            (
                [namespace, "/camera/left_camera/image/zstd"],
                "camera/left_camera/image/zstd"
            ),
        ],
    )

    # Image bridge for right camera image
    ros_gz_image_bridge_right_node = Node(
        name="right_camera_image_bridge",
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[[namespace, "/camera/right_camera/image"], "--ros-args", "--log-level", log_level],
        output="screen",
        remappings=[
            (
                [namespace, "/camera/right_camera/image"],
                "camera/right_camera/image",
            ),
            (
                [namespace, "/camera/right_camera/image/compressed"],
                "camera/right_camera/image/compressed",
            ),
            (
                [namespace, "/camera/right_camera/image/compressedDepth"],
                "camera/right_camera/image/compressedDepth",
            ),
            (
                [namespace, "/camera/right_camera/image/theora"],
                "camera/right_camera/image/theora"
            ),
            (
                [namespace, "/camera/right_camera/image/zstd"],
                "camera/right_camera/image/zstd"
            ),
        ],
    )

    # Image bridge for left camera depth image
    ros_gz_image_bridge_depth_node = Node(
        name="left_camera_depth_image_bridge",
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[[namespace, "/camera/left_camera/depth_image"], "--ros-args", "--log-level", log_level],
        output="screen",
        remappings=[
            (
                [namespace, "/camera/left_camera/depth_image"],
                "camera/left_camera/depth_image"),
            (
                [namespace, "/camera/left_camera/depth_image/compressed"],
                "camera/left_camera/depth_image/compressed",
            ),
            (
                [namespace, "/camera/left_camera/depth_image/compressedDepth"],
                "camera/left_camera/depth_image/compressedDepth",
            ),
            (
                [namespace, "/camera/left_camera/depth_image/theora"],
                "camera/left_camera/depth_image/theora",
            ),
            (
                [namespace, "/camera/left_camera/depth_image/zstd"],
                "camera/left_camera/depth_image/zstd",
            ),
        ],
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            declare_namespace_arg,
            declare_prefix_arg,
            declare_world_arg,
            declare_model_package_arg,
            declare_model_file_arg,
            declare_camera_resolution_arg,
            declare_lidar_update_rate_arg,
            declare_log_level_arg,
            declare_use_sim_time_arg,
            declare_use_rsp_arg,
            declare_use_jsp_arg,
            declare_use_jsp_gui_arg,
            declare_use_lidar_arg,
            declare_use_ros2_control_arg,
            declare_use_joystick_arg,
            declare_use_keyboard_arg,
            declare_use_navigation_arg,
            # Launchers
            gazebo_launch,
            # Nodes
            spawn_entity_node,
            ros_gz_image_bridge_left_node,
            ros_gz_image_bridge_right_node,
            ros_gz_image_bridge_depth_node,
            rsp_node,
            jsp_node,
            jsp_gui_node,
            teleop_keyboard_node,
            joy_node,
            teleop_joy_node,
            teleop_joy_stamper_node,
            diff_drive_spawner_node,
            joint_broad_spawner_node,
            start_gazebo_ros_bridge_node,
        ]
    )
