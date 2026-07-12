from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nitish',
    maintainer_email='nitish@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'goal_navigator = navigation_manager.goal_navigator:main',
        'human_predictor = navigation_manager.human_predictor:main',
        'metrics_logger = navigation_manager.metrics_logger:main',
        'failure_monitor = navigation_manager.failure_monitor:main',
        'battery_simulator = navigation_manager.battery_simulator:main',
        ],
    },
)
