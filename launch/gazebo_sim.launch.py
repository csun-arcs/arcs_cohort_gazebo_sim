import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    LogInfo,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package names
    pkg_gazebo_sim = "arcs_cohort_gazebo_sim"
    pkg_description = "arcs_cohort_description"
    pkg_nav = "arcs_cohort_navigation"

    # Defaults
    default_ros2_control_params_file_template = os.path.join(
        get_package_share_directory(pkg_gazebo_sim), "config", "gazebo_ros2_control_params.yaml.template"
    )
    default_ekf_params_file = os.path.join(
        get_package_share_directory(pkg_gazebo_sim), "config", "gazebo_ros2_control_params.yaml"
    )
    default_world_path = os.path.join(
        get_package_share_directory(pkg_gazebo_sim),
        "worlds",
        "test_obstacles_world_1.world",
    )
    default_model_path = "description/robot.gazebo.xacro"
    default_joystick_params_path = os.path.join(
        get_package_share_directory(pkg_gazebo_sim),
        "config",
        "gazebo_joystick_teleop.yaml",
    )
    default_log_level = "INFO"
    default_model_resource_path = os.path.join(
        get_package_share_directory(pkg_gazebo_sim), "models"
    )
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=default_model_resource_path
    )
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
    declare_ros2_control_params_template_arg = DeclareLaunchArgument(
        "ros2_control_params_template",
        default_value=default_ros2_control_params_file_template,
        description="Path to the params file template from which to generate the params file for ros2_control.",
    )
    declare_ros2_control_params_arg = DeclareLaunchArgument(
        "ros2_control_params",
        default_value=default_ekf_params_file,
        description="Path to the params file for ros2_control.",
    )
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="Path to the world file to load",
    )
    declare_model_package_arg = DeclareLaunchArgument(
        "model_package",
        default_value=pkg_gazebo_sim,
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
        default_value="10",
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
    declare_use_ros2_control_params_template_arg = DeclareLaunchArgument(
        "use_ros2_control_params_template",
        default_value="true",
        description="If true, generate the ros2_control params from the specified ros2_control params template.",
    )

    # Launch configurations
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    ros2_control_params_template = LaunchConfiguration("ros2_control_params_template")
    ros2_control_params = LaunchConfiguration("ros2_control_params")
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
    use_ros2_control_params_template = LaunchConfiguration("use_ros2_control_params_template")

    # Log info
    log_info = LogInfo(msg=['Gazebo bringup launching with namespace: ', namespace, ', prefix: ', prefix])

    # Use PushRosNamespace to apply the namespace to all nodes below
    push_namespace = PushRosNamespace(namespace=namespace)

    # Build the prefix with underscore.
    # This expression will evaluate to, for example, "cohort1_" if
    # the prefix is "cohort1", or to an empty string if prefix is empty.
    prefix_ = PythonExpression(
        ["'", prefix, "_' if '", prefix, "' else ''"]
    )

    # Build the namespace with slash
    # This expression will evaluate to, for example, "cohort1/" if
    # the namespace is "cohort1", or to an empty string if namespace is empty.
    namespace_ = PythonExpression(
        ["'", namespace, "/' if '", namespace, "' else ''"]
    )

    # Generate ros2_control params from template.
    # The prefix will be substituted into the template in
    # place of the ARCS_COHORT_PREFIX variable and the namespace will be
    # substituted in place of ARCS_COHORT_NAMESPACE.
    ros2_control_params_generator = ExecuteProcess(
        condition=IfCondition(use_ros2_control_params_template),
        cmd=[
            [
                "ARCS_COHORT_PREFIX='",
                prefix_,
                "' ",
                "ARCS_COHORT_NAMESPACE='",
                namespace_,
                "' ",
                "envsubst < ",
                ros2_control_params_template,
                " > ",
                ros2_control_params,
            ]
        ],
        shell=True,
        output="screen",
    )

    # Robot description from Xacro, including the conditional robot name prefix.
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution([FindPackageShare(model_package), model_file]),
            " namespace:=",
            namespace,
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
            ("/tf_static", "tf_static"),
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
            # Set gz sim resource path environment variable
            set_gz_sim_resource_path,
            # Declare launch arguments
            declare_namespace_arg,
            declare_prefix_arg,
            declare_ros2_control_params_template_arg,
            declare_ros2_control_params_arg,
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
            declare_use_ros2_control_params_template_arg,
            # Log info
            log_info,
            # Param file generators
            ros2_control_params_generator,
            # Launchers
            gazebo_launch,
            # Nodes
            GroupAction([
                push_namespace,
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
            ])
        ]
    )
