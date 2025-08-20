from setuptools import setup
import os
from glob import glob

package_name = 'unity_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('lib', package_name), ['unity_bridge/unity_bridge_node.py', 'unity_bridge/mock_unity_publisher.py', 'unity_bridge/goal_publisher.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ctx',
    maintainer_email='ctx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unity_bridge_node = unity_bridge.unity_bridge_node:main',
            'mock_unity_publisher = unity_bridge.mock_unity_publisher:main',
            'goal_publisher = unity_bridge.goal_publisher:main',
        ],
    },
) 