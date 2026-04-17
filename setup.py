from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vrx_experiment_benchmark'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.jinja')),
        (os.path.join('share', package_name, 'config', 'env'),
         glob('config/env/*.yaml')),
        (os.path.join('share', package_name, 'config', 'routes'),
         glob('config/routes/*.yaml')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
         (os.path.join('share', package_name, 'config', 'rl_models'),
         glob('config/rl_models/*')),
        # Metrics are written at runtime to an absolute path (see launch file).
        # They are not installed as data_files to avoid build failures.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='misael',
    maintainer_email='misael@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'world_generator = vrx_experiment_benchmark.world_generator:main',
            "state_estimator_2d = vrx_experiment_benchmark.state_estimator_2d:main",
            'route_manager = vrx_experiment_benchmark.route_manager:main',
            'guidance_los = vrx_experiment_benchmark.guidance_los:main',
            'controller_pd = vrx_experiment_benchmark.controller_pd:main',
            'thruster_commander = vrx_experiment_benchmark.thruster_commander:main',
            'controller_mppi = vrx_experiment_benchmark.controller_mppi:main',
            'controller_rl_residual = vrx_experiment_benchmark.controller_rl_residual:main',
            'metrics_logger = vrx_experiment_benchmark.metrics_logger:main',
            'experiment_manager = vrx_experiment_benchmark.experiment_manager:main',
            'train_rl_residual = vrx_experiment_benchmark.train_rl_residual:main',
        ],
    },
)
