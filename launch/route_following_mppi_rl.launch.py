from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Absolute paths to source tree for metrics and dataset workflow
_METRICS_ROOT = "/home/misael/ros2_workspaces/diplomado_ws/src/vrx_experiment_benchmark/metrics"
_RL_DATASET_ROOT = "/home/misael/ros2_workspaces/diplomado_ws/src/vrx_experiment_benchmark/metrics/rl_dataset"
# Source path where train_rl_residual saves the runtime model
_SRC_RL_MODELS = "/home/misael/ros2_workspaces/diplomado_ws/src/vrx_experiment_benchmark/config/rl_models"
_RL_MODEL_NAME = "mppi_rl_residual_policy.ts"


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("vrx_experiment_benchmark")

    route_file_value = LaunchConfiguration("route_file").perform(context).strip()
    route_name_value = LaunchConfiguration("route_name").perform(context).strip()
    use_learned_policy_value = LaunchConfiguration("use_learned_policy").perform(context).strip().lower()
    shadow_mode_value = LaunchConfiguration("shadow_mode").perform(context).strip().lower()
    record_dataset_value = LaunchConfiguration("record_dataset").perform(context).strip().lower()

    use_learned_policy = use_learned_policy_value in ("true", "1", "yes")
    shadow_mode = shadow_mode_value in ("true", "1", "yes")
    record_dataset = record_dataset_value in ("true", "1", "yes")

    # Resolve model path: prefer src/ (saved by training script) falling back to install/
    import os as _os
    _src_model = f"{_SRC_RL_MODELS}/{_RL_MODEL_NAME}"
    _pkg_share_str = FindPackageShare("vrx_experiment_benchmark").find("vrx_experiment_benchmark")
    _install_model = f"{_pkg_share_str}/config/rl_models/{_RL_MODEL_NAME}"
    policy_path_value = _src_model if _os.path.isfile(_src_model) else _install_model

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

    # MPPI base predictive controller
    controller_mppi_node = Node(
        package="vrx_experiment_benchmark",
        executable="controller_mppi",
        name="controller_mppi",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "mppi_config.yaml"]),
            {
                "state_topic": "/wamv/navigation/state2d",
                "guidance_topic": "/wamv/control/guidance_cmd",
                "active_waypoint_meta_topic": "/wamv/navigation/active_waypoint_meta",
                "thruster_cmd_topic": "/wamv/control/thruster_cmd_raw",
                "controller_debug_topic": "/wamv/control/controller_debug_raw",
            }
        ],
    )

    # RL residual controller
    controller_rl_residual_node = Node(
        package="vrx_experiment_benchmark",
        executable="controller_rl_residual",
        name="controller_rl_residual",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "rl_residual_config.yaml"]),
            {
                "state_topic": "/wamv/navigation/state2d",
                "guidance_topic": "/wamv/control/guidance_cmd",
                "active_waypoint_meta_topic": "/wamv/navigation/active_waypoint_meta",
                "base_thruster_cmd_topic": "/wamv/control/thruster_cmd_raw",
                "base_controller_debug_topic": "/wamv/control/controller_debug_raw",
                "thruster_cmd_topic": "/wamv/control/thruster_cmd",
                "controller_debug_topic": "/wamv/control/controller_debug",
                "dataset_output_root": _RL_DATASET_ROOT,
                "dataset_tag": route_name_value,
                "use_learned_policy": use_learned_policy,
                "shadow_mode": shadow_mode,
                "record_dataset": record_dataset,
                "policy_path": policy_path_value,
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
        controller_mppi_node,
        controller_rl_residual_node,
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
            "use_learned_policy",
            default_value="false",
            description="true to load trained policy, false to use heuristic residual",
        ),
        DeclareLaunchArgument(
            "shadow_mode",
            default_value="false",
            description="true to record dataset without applying residual, false for active residual",
        ),
        DeclareLaunchArgument(
            "record_dataset",
            default_value="true",
            description="true to save residual dataset to metrics/rl_dataset",
        ),
        DeclareLaunchArgument(
            "guidance_mode",
            default_value="los",
            description="Guidance mode ('los' only, MPPI acts as base controller)",
        ),
        OpaqueFunction(function=launch_setup),
    ])