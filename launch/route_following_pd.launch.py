import os as _os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Absolute path to metrics directory within source package
_METRICS_ROOT = "/home/misael/ros2_workspaces/diplomado_ws/src/vrx_experiment_benchmark/metrics"


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("vrx_experiment_benchmark")

    route_file_value = LaunchConfiguration("route_file").perform(context).strip()
    route_name_value = LaunchConfiguration("route_name").perform(context).strip()

    # Resolve absolute route file path
    if route_file_value:
        resolved_route = route_file_value
    else:
        resolved_route = PathJoinSubstitution([
            pkg_share,
            "config",
            "routes",
            route_name_value,
        ])

    # Include state estimation subsystem
    state_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share,
                "launch",
                "state_estimation.launch.py",
            ])
        )
    )

    # Route management and waypoint tracking
    route_manager_node = Node(
        package="vrx_experiment_benchmark",
        executable="route_manager",
        name="route_manager",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "route_manager_config.yaml"]),
            {
                "route_file": resolved_route,
                "origin_topic": "/wamv/navigation/local_origin",
                "state_topic": "/wamv/navigation/state2d",
                "route_local_topic": "/wamv/navigation/route_local",
                "active_waypoint_topic": "/wamv/navigation/active_waypoint",
                "active_waypoint_meta_topic": "/wamv/navigation/active_waypoint_meta",
                "route_status_topic": "/wamv/navigation/route_status",
                "frame_id": "map_local",
            }
        ],
    )

    # Line-of-sight guidance
    guidance_los_node = Node(
        package="vrx_experiment_benchmark",
        executable="guidance_los",
        name="guidance_los",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "los_config.yaml"]),
            {
                "state_topic": "/wamv/navigation/state2d",
                "active_waypoint_meta_topic": "/wamv/navigation/active_waypoint_meta",
                "guidance_topic": "/wamv/control/guidance_cmd",
            }
        ],
    )

    # Proportional-Derivative heading controller
    controller_pd_node = Node(
        package="vrx_experiment_benchmark",
        executable="controller_pd",
        name="controller_pd",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "pd_config.yaml"]),
            {
                "state_topic": "/wamv/navigation/state2d",
                "guidance_topic": "/wamv/control/guidance_cmd",
                "thruster_cmd_topic": "/wamv/control/thruster_cmd",
                "controller_debug_topic": "/wamv/control/controller_debug",
            }
        ],
    )

    # Thruster allocation and limiting
    thruster_commander_node = Node(
        package="vrx_experiment_benchmark",
        executable="thruster_commander",
        name="thruster_commander",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "controller_limits.yaml"]),
            {
                "thruster_cmd_topic": "/wamv/control/thruster_cmd",
                "left_thrust_topic": "/wamv/thrusters/left/thrust",
                "right_thrust_topic": "/wamv/thrusters/right/thrust",
                "left_pos_topic": "/wamv/thrusters/left/pos",
                "right_pos_topic": "/wamv/thrusters/right/pos",
            }
        ],
    )

    # Data logging and metrics calculation
    metrics_logger_node = Node(
        package="vrx_experiment_benchmark",
        executable="metrics_logger",
        name="metrics_logger",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "metrics_config.yaml"]),
            {
                "state_topic": "/wamv/navigation/state2d",
                "route_status_topic": "/wamv/navigation/route_status",
                "active_waypoint_meta_topic": "/wamv/navigation/active_waypoint_meta",
                "guidance_topic": "/wamv/control/guidance_cmd",
                "controller_debug_topic": "/wamv/control/controller_debug",
                "thruster_cmd_topic": "/wamv/control/thruster_cmd",
                "run_tag": route_name_value,
                "output_root": _METRICS_ROOT,
            }
        ],
    )

    return [
        state_estimation_launch,
        route_manager_node,
        guidance_los_node,
        controller_pd_node,
        thruster_commander_node,
        metrics_logger_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "route_name",
            default_value="route_curves.yaml",
            description="YAML route name in config/routes (e.g. route_zigzag.yaml)",
        ),
        DeclareLaunchArgument(
            "route_file",
            default_value="",
            description="Optional absolute route YAML path (overrides route_name)",
        ),
        DeclareLaunchArgument(
            "guidance_mode",
            default_value="los",
            description="Guidance mode ('los' or 'mppi')",
        ),
        OpaqueFunction(function=launch_setup),
    ])