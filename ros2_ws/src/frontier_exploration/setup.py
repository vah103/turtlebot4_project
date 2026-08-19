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
                'launch/hospital_nav2.launch.py',
                'launch/hospital_stack.launch.py',
            ],
        ),
        (
            'share/' + package_name + '/config',
            [
                'config/frontier.yaml',
                'config/map_snapshot.yaml',
                'config/map_snapshot_hospital.yaml',
                'config/nav2_hospital_override.yaml',
            ],
        ),
        (
            'share/' + package_name + '/rviz',
            ['rviz/hospital_exploration.rviz'],
        ),
        (
            'share/' + package_name + '/urdf',
            [
                'urdf/create3_hospital.urdf.xacro',
                'urdf/hospital_turtlebot4.urdf.xacro',
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
                'hospital_map_cloud_visualizer = '
                'frontier_exploration.hospital_map_cloud_visualizer:main'
            ),
            (
                'compute_hospital_canvas = '
                'frontier_exploration.hospital_canvas:main'
            ),
            (
                'prepare_lama_dataset = '
                'frontier_exploration.lama_dataset_preprocessor:main'
            ),
        ],
    },
)
