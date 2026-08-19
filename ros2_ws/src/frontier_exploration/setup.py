from setuptools import find_packages, setup

package_name = 'frontier_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/frontier_detector.launch.py',
                'launch/frontier_autonomy.launch.py',
                'launch/nav2_sim_compat.launch.py',
                'launch/map_snapshot_recorder.launch.py',
                'launch/hospital_map_snapshot_recorder.launch.py',
                'launch/tb4_simulation_safe.launch.py',
                'launch/hospital_simulation.launch.py',
            ],
        ),
        (
            'share/' + package_name + '/config',
            [
                'config/frontier.yaml',
                'config/map_snapshot.yaml',
                'config/map_snapshot_hospital.yaml',
            ],
        ),
        (
            'share/' + package_name + '/worlds',
            ['worlds/hospital_aws.sdf'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vah103',
    maintainer_email='165911392+vah103@users.noreply.github.com',
    description='Frontier exploration baseline for TurtleBot4.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            (
                'frontier_map_preprocessor = '
                'frontier_exploration.frontier_map_preprocessor_resilient:main'
            ),
            (
                'frontier_detector = '
                'frontier_exploration.frontier_detector_resilient:main'
            ),
            (
                'exploration_manager = '
                'frontier_exploration.exploration_manager:main'
            ),
            (
                'sim_twist_adapter = '
                'frontier_exploration.sim_twist_adapter:main'
            ),
            (
                'map_snapshot_recorder = '
                'frontier_exploration.map_snapshot_recorder:main'
            ),
            (
                'compute_hospital_canvas = '
                'frontier_exploration.hospital_canvas:main'
            ),
        ],
    },
)
