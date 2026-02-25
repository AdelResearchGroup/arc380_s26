# abb_irb120_gazebo/launch/simulation.launch.py

import os
import shlex

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    TextSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _split_args(arg_string: str):
    arg_string = (arg_string or "").strip()
    if not arg_string:
        return []
    return shlex.split(arg_string, posix=(os.name != "nt"))


def _start_gz(context, *args, **kwargs):
    """
    Cross-platform Gazebo Sim startup:

    - Windows: server-only required (add -s). Optionally launch GUI separately (gz sim -g).
    - macOS/Linux: default gz sim launches GUI+server; avoid launching a second GUI client.
    """
    world_path = LaunchConfiguration("world").perform(context)
    gz_args_str = LaunchConfiguration("gz_args").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes", "on")

    extra = _split_args(gz_args_str)

    if os.name == "nt":
        # Windows requires -s (server-only) when launched this way
        if "-s" not in extra and "--server" not in extra:
            extra = ["-s"] + extra
    else:
        # macOS/Linux: only add -s if user explicitly requests headless
        if headless and "-s" not in extra and "--server" not in extra:
            extra = ["-s"] + extra

    cmd = [FindExecutable(name="gz").perform(context), "sim"] + extra + [world_path]
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    gazebo_share = get_package_share_directory("abb_irb120_gazebo")
    desc_share = get_package_share_directory("abb_irb120_description")

    default_xacro = os.path.join(desc_share, "urdf", "irb120_3_60.xacro")
    default_world = os.path.join(gazebo_share, "worlds", "empty.sdf")

    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_file = LaunchConfiguration("xacro_file")
    robot_name = LaunchConfiguration("robot_name")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    gz_args = LaunchConfiguration("gz_args")
    start_gui = LaunchConfiguration("start_gui")
    gui_delay = LaunchConfiguration("gui_delay")
    headless = LaunchConfiguration("headless")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "xacro_file",
            default_value=TextSubstitution(text=default_xacro),
            description="Absolute path to the IRB120 xacro file",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=TextSubstitution(text=default_world),
            description="Absolute path to the SDF world file",
        ),
        DeclareLaunchArgument("robot_name", default_value="irb120"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.0"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "gz_args",
            default_value=TextSubstitution(text="-r"),
            description="Extra args passed to `gz sim` (excluding world path). Example: '-r -v 4'",
        ),
        DeclareLaunchArgument(
            "headless",
            # Windows effectively headless in this workflow; mac/linux default is GUI unless set true.
            default_value=("true" if os.name == "nt" else "false"),
            description="Run gz sim headless/server-only (-s).",
        ),
        DeclareLaunchArgument(
            "start_gui",
            # Only meaningful on Windows (starts separate GUI client process).
            default_value=("true" if os.name == "nt" else "false"),
            description="(Windows) Start a separate `gz sim -g` GUI client.",
        ),
        DeclareLaunchArgument(
            "gui_delay",
            default_value="2.0",
            description="Seconds to wait before starting GUI client (Windows).",
        ),
    ]

    # -------------------------
    # Gazebo resource paths for model:// resolution
    # model://<pkg>/... requires GZ_SIM_RESOURCE_PATH contain <install_prefix>/share
    # -------------------------
    gazebo_prefix_share = os.path.join(get_package_prefix("abb_irb120_gazebo"), "share")
    desc_prefix_share = os.path.join(get_package_prefix("abb_irb120_description"), "share")

    worlds_dir = os.path.join(gazebo_share, "worlds")
    models_dir = os.path.join(gazebo_share, "models")

    resource_paths = os.pathsep.join(
        [
            p
            for p in [
                gazebo_prefix_share,
                desc_prefix_share,
                worlds_dir,
                models_dir,
                gazebo_share,
                desc_share,
            ]
            if os.path.exists(p)
        ]
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            TextSubstitution(text=resource_paths),
            TextSubstitution(text=os.pathsep),
            TextSubstitution(text=os.environ.get("GZ_SIM_RESOURCE_PATH", "")),
        ],
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            TextSubstitution(text=resource_paths),
            TextSubstitution(text=os.pathsep),
            TextSubstitution(text=os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")),
        ],
    )

    # Put gz logs somewhere writable across platforms
    gz_log_dir = os.path.join(os.path.expanduser("~"), ".ros", "gz_logs")
    set_gz_log_path = SetEnvironmentVariable(name="GZ_SIM_LOG_PATH", value=gz_log_dir)
    set_gz_log_path_legacy = SetEnvironmentVariable(name="GZ_LOG_PATH", value=gz_log_dir)

    # -------------------------
    # Robot description + RSP
    # -------------------------
    robot_description_cmd = Command(
        [FindExecutable(name="xacro"), TextSubstitution(text=" "), xacro_file]
    )
    robot_description = ParameterValue(robot_description_cmd, value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, {"robot_description": robot_description}],
    )

    # -------------------------
    # Start gz (single process on mac/linux; server-only on Windows)
    # -------------------------
    gz = OpaqueFunction(function=_start_gz)

    # -------------------------
    # (Windows only) Start GUI client in a second process if requested
    # -------------------------
    is_windows_and_start_gui = IfCondition(
        PythonExpression(["'", "true" if os.name == "nt" else "false", "' == 'true' and ", start_gui])
    )

    gz_gui = TimerAction(
        period=gui_delay,
        actions=[
            ExecuteProcess(
                cmd=[FindExecutable(name="gz"), "sim", "-g"],
                output="screen",
            )
        ],
        condition=is_windows_and_start_gui,
    )

    # -------------------------
    # Spawn robot from robot_description
    # -------------------------
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-R",
            roll,
            "-P",
            pitch,
            "-Y",
            yaw,
        ],
    )

    # -------------------------
    # Bridge /clock
    # -------------------------
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    return LaunchDescription(
        declare_args
        + [
            set_gz_resource_path,
            set_ign_resource_path,
            set_gz_log_path,
            set_gz_log_path_legacy,
            gz,
            gz_gui,
            robot_state_publisher,
            clock_bridge,
            spawn,
        ]
    )