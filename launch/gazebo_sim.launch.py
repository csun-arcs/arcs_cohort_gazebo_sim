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

    # Paths
    default_world_path = os.path.join(
        get_package_share_directory(pkg_gazebo_sim),
        "worlds",
        "test_obstacles_world_1.world",
    )
    default_model_path = "description/robot.urdf.xacro"
    default_joystick_params = os.path.join(
        get_package_share_directory(pkg_gazebo_sim),
        "config",
        "gazebo_joystick_teleop.yaml",
    )

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="Path to the world file to load",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    declare_model_package_cmd = DeclareLaunchArgument(
        "model_package",
        default_value=pkg_description,
        description="Package containing the robot model",
    )
    declare_model_file_cmd = DeclareLaunchArgument(
        "model_file",
        default_value=default_model_path,
        description="Relative path to the robot model file",
    )
    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description=(
            "Name of the robot (specifying this will add the "
            "robot name prefix to joints, links, etc. in the robot model)."
        ),
    )
    declare_camera_resolution_cmd = DeclareLaunchArgument(
        "camera_resolution",
        default_value="VGA",
        description=(
            "Resolution profile of the simulated Stereolabs Zed camera."
            'Options: "HD2K" (2208x1242), "HD1080" (1920x1080), '
            '"HD720" (1280x720) or "VGA" (672x376).'
        ),
    )
    # NOTE: Removing the namespace parameter from the Gazebo launcher for now
    # due to double-namespacing issue when launcher is called from an upstream
    # launcher.
    #
    # declare_namespace_cmd = DeclareLaunchArgument(
    #     "namespace",
    #     default_value="",
    #     description="Namespace under which to bring up camera image bridges."
    # )
    declare_use_rsp_cmd = DeclareLaunchArgument(
        "use_rsp", default_value="true", description="Launch robot_state_publisher"
    )
    declare_use_jsp_cmd = DeclareLaunchArgument(
        "use_jsp", default_value="false", description="Launch joint_state_publisher"
    )
    declare_use_jsp_gui_cmd = DeclareLaunchArgument(
        "use_jsp_gui",
        default_value="false",
        description="Launch joint_state_publisher_gui",
    )
    declare_use_lidar_cmd = DeclareLaunchArgument(
        "use_lidar",
        default_value="false",
        description="If true, include the lidar in the robot description",
    )
    declare_lidar_update_rate_cmd = DeclareLaunchArgument(
        "lidar_update_rate",
        default_value="30",
        description="Set the update rate of the LiDAR sensor.",
    )
    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="false",
        description="Use ROS2 Control for the robot",
    )
    declare_use_joystick_cmd = DeclareLaunchArgument(
        "use_joystick",
        default_value="false",
        description="Use joystick to control the robot",
    )

    # Launch configurations
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    model_package = LaunchConfiguration("model_package")
    model_file = LaunchConfiguration("model_file")
    robot_name = LaunchConfiguration("robot_name")
    camera_resolution = LaunchConfiguration("camera_resolution")
    # namespace = LaunchConfiguration("namespace")
    use_rsp = LaunchConfiguration("use_rsp")
    use_jsp = LaunchConfiguration("use_jsp")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    use_lidar = LaunchConfiguration("use_lidar")
    lidar_update_rate = LaunchConfiguration("lidar_update_rate")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_joystick = LaunchConfiguration("use_joystick")

    # Compute the robot prefix only if a robot name is provided
    # This expression will evaluate to, for example, "cohort_" if
    # robot_name is "cohort", or to an empty string if robot_name is empty.
    robot_prefix = PythonExpression(
        ["'", robot_name, "_' if '", robot_name, "' else ''"]
    )
    # Compute the prefix argument only if a robot_name/robot_prefix is provided.
    # This expression will evaluate to, for example, "prefix:=cohort_" if
    # robot_prefix is "cohort_", or to an empty string if robot_prefix is empty.
    robot_prefix_arg = PythonExpression(
        ["('prefix:=' + '", robot_prefix, "') if '", robot_prefix, "' else ''"]
    )

    # Robot description from Xacro, including the conditional robot name prefix.
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution([FindPackageShare(model_package), model_file]),
            " ",
            robot_prefix_arg,
            " ",
            "camera_resolution:=",
            camera_resolution,
            " use_lidar:=",
            use_lidar,
            " lidar_update_rate:=",
            lidar_update_rate,
            " use_ros2_control:=",
            use_ros2_control,
            " use_joystick:=",
            use_joystick,
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
    )

    # Joint State Publisher GUI node
    jsp_gui_node = Node(
        condition=IfCondition(use_jsp_gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Jostick Node
    joy_node = Node(
        condition=IfCondition(use_joystick),
        package="joy",
        executable="joy_node",
        parameters=[default_joystick_params],
    )

    # Teleop with joystick
    teleop_joy = Node(
        condition=IfCondition(PythonExpression(["'", use_joystick, "' == 'true'"])),
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[default_joystick_params, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel", "/diff_cont/cmd_vel_unstamped")],
    )

    # Twist stamper for joystick teleop node when ros2 control plugin is enabled
    teleop_joy_stamper = Node(
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_ros2_control,
                    "' == 'true' and '",
                    use_joystick,
                    "' == 'true'",
                ]
            )
        ),
        package="twist_stamper",
        executable="twist_stamper",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/cmd_vel_in", "/diff_cont/cmd_vel_unstamped"),
            ("/cmd_vel_out", "/diff_cont/cmd_vel"),
        ],
    )

    # Teleop with keyboard
    teleop_keyboard = Node(
        condition=IfCondition(PythonExpression(["'", use_joystick, "' == 'false'"])),
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        prefix="xterm -e",
        parameters=[
            {"stamped": PythonExpression(["'", use_ros2_control, "' == 'true'"])}
        ],
        remappings=[
            (
                "/cmd_vel",
                PythonExpression(
                    [
                        "'/diff_cont/cmd_vel' if '",
                        use_ros2_control,
                        "' == 'true' else '/diff_cont/cmd_vel_unstamped'",
                    ]
                ),
            )
        ],
    )

    # Twist unstamper for keyboard to pass into twist mux for Nav2 waypoint navigation
    teleop_keyboard_unstamper = Node(
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_ros2_control,
                    "' == 'true' and '",
                    use_joystick,
                    "' == 'false'",
                ]
            )
        ),
        package="twist_stamper",
        executable="twist_unstamper",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/cmd_vel_in", "/diff_cont/cmd_vel"),
            ("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped_keyboard"),
        ],
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
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "cohort", "-z", "0.1"],
        output="screen",
    )

    # Differential drive controller spawner
    diff_drive_spawner = Node(
        condition=IfCondition(use_ros2_control),
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # Joint state broadcaster spawner
    joint_broad_spawner = Node(
        condition=IfCondition(use_ros2_control),
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Gazebo bridge parameters
    bridge_params = os.path.join(
        get_package_share_directory(pkg_gazebo_sim), "config", "gazebo_bridge.yaml"
    )

    # Start the Gazebo ROS bridge
    start_gazebo_ros_bridge = Node(
        name="parameter_bridge",
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_params}],
        output="screen",
    )

    # Image bridge for left camera image
    ros_gz_image_bridge_left = Node(
        name="left_camera_image_bridge",
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["camera/left_camera/image"],
        output="screen",
        remappings=[
            ("camera/left_camera/image", "camera/left_camera/image"),
            (
                "camera/left_camera/image/compressed",
                "camera/left_camera/image/compressed",
            ),
            (
                "camera/left_camera/image/compressedDepth",
                "camera/left_camera/image/compressedDepth",
            ),
            ("camera/left_camera/image/theora", "camera/left_camera/image/theora"),
            ("camera/left_camera/image/zstd", "camera/left_camera/image/zstd"),
        ],
    )

    # Image bridge for right camera image
    ros_gz_image_bridge_right = Node(
        name="right_camera_image_bridge",
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["camera/right_camera/image"],
        output="screen",
        remappings=[
            ("camera/right_camera/image", "camera/right_camera/image"),
            (
                "camera/right_camera/image/compressed",
                "camera/right_camera/image/compressed",
            ),
            (
                "camera/right_camera/image/compressedDepth",
                "camera/right_camera/image/compressedDepth",
            ),
            ("camera/right_camera/image/theora", "camera/right_camera/image/theora"),
            ("camera/right_camera/image/zstd", "camera/right_camera/image/zstd"),
        ],
    )

    # Image bridge for left camera depth image
    ros_gz_image_bridge_depth = Node(
        name="left_camera_depth_image_bridge",
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["camera/left_camera/depth_image"],
        output="screen",
        remappings=[
            ("camera/left_camera/depth_image", "camera/left_camera/depth_image"),
            (
                "camera/left_camera/depth_image/compressed",
                "camera/left_camera/depth_image/compressed",
            ),
            (
                "camera/left_camera/depth_image/compressedDepth",
                "camera/left_camera/depth_image/compressedDepth",
            ),
            (
                "camera/left_camera/depth_image/theora",
                "camera/left_camera/depth_image/theora",
            ),
            (
                "camera/left_camera/depth_image/zstd",
                "camera/left_camera/depth_image/zstd",
            ),
        ],
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            declare_world_cmd,
            declare_use_sim_time_cmd,
            declare_model_package_cmd,
            declare_model_file_cmd,
            declare_robot_name_cmd,
            declare_camera_resolution_cmd,
            # declare_namespace_cmd,
            declare_use_rsp_cmd,
            declare_use_jsp_cmd,
            declare_use_jsp_gui_cmd,
            declare_use_lidar_cmd,
            declare_lidar_update_rate_cmd,
            declare_use_ros2_control_cmd,
            declare_use_joystick_cmd,
            # Nodes
            rsp_node,
            jsp_node,
            jsp_gui_node,
            teleop_keyboard,
            joy_node,
            teleop_joy,
            teleop_joy_stamper,
            teleop_keyboard_unstamper,
            gazebo_launch,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
            start_gazebo_ros_bridge,
            ros_gz_image_bridge_left,
            ros_gz_image_bridge_right,
            ros_gz_image_bridge_depth,
        ]
    )
