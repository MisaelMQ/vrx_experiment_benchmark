from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('vrx_experiment_benchmark')
    vrx_gz_share = get_package_share_directory('vrx_gz')

    env_name = LaunchConfiguration('env').perform(context)
    launch_experiment_manager = LaunchConfiguration('launch_experiment_manager').perform(context).lower() in ('true', '1', 'yes')

    # Generate requested world from preset
    from vrx_experiment_benchmark.world_generator import generate_world
    generated_world = generate_world(env_name)

    generated_world_dir = os.path.dirname(generated_world)
    current_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_path = f"{generated_world_dir}:{pkg_share}:{current_gz_path}"

    world_name = os.path.splitext(os.path.basename(generated_world))[0]

    # Include VRX competition launch file
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_gz_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(vrx_gz_share, 'launch', 'competition.launch.py')
            ),
            launch_arguments={
                'world': world_name,
            }.items()
        ),
    ]

    # Optionally launch experiment manager
    if launch_experiment_manager:
        actions.append(
            Node(
                package='vrx_experiment_benchmark',
                executable='experiment_manager',
                name='experiment_manager',
                output='screen'
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'env',
            default_value='env_01_calm',
            description='Environmental preset to use'
        ),
        DeclareLaunchArgument(
            'launch_experiment_manager',
            default_value='false',
            description='Launch experiment_manager if available in the package'
        ),
        OpaqueFunction(function=launch_setup),
    ])