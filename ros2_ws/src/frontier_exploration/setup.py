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
            ],
        ),
        ('share/' + package_name + '/config', ['config/frontier.yaml']),
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
            'frontier_map_preprocessor = frontier_exploration.frontier_map_preprocessor:main',
            'frontier_detector = frontier_exploration.frontier_detector_retry:main',
            'exploration_manager = frontier_exploration.exploration_manager:main',
            'sim_twist_adapter = frontier_exploration.sim_twist_adapter:main',
        ],
    },
)
