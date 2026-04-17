from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("vrx_experiment_benchmark")
    params_file = os.path.join(pkg_share, "config", "state_estimator_2d.yaml")

    # Only pass YAML params if the file exists in the install directory, else use defaults
    node_params = [params_file] if os.path.isfile(params_file) else []

    # State estimator node
    state_estimator = Node(
        package="vrx_experiment_benchmark",
        executable="state_estimator_2d",
        name="state_estimator_2d",
        output="screen",
        parameters=node_params,
    )

    return LaunchDescription([
        state_estimator,
    ])