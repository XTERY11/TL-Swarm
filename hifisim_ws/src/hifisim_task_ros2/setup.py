from setuptools import find_packages, setup

package_name = 'hifisim_task_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', ['launch/npbtm_ros2_launch.py']),
        ('share/' + package_name + '/launch', ['launch/radio_comm_simulator_launch.py']),
        ('share/' + package_name + '/launch', ['launch/hifi_simulator_launch.py']),
        ('share/' + package_name + '/launch', ['launch/unity_component_launch.py']),
        ('share/' + package_name + '/launch', ['launch/ros2_component_launch.py']),
        ('share/' + package_name + '/launch', ['launch/view_mapping.launch.py']),
        # Install rviz config from package-relative path
        ('share/' + package_name + '/rviz', ['rviz/phase3_env1.rviz']),
    ],
    py_modules=[
        'hifisim_task_ros2.reset_testcase_server',
        'hifisim_task_ros2.reset_testcase_request',
        'hifisim_task_ros2.read_testcase',
        'hifisim_task_ros2.sim_flight',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wayne',
    maintainer_email='tslywh@nus.edu.sg',
    description='This is the ROS Component of HiFi_Simulator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'init_testcase_request = hifisim_task_ros2.init_testcase_request:main',
            'init_testcase_server = hifisim_task_ros2.init_testcase_server:main',
            'reset_testcase_request = hifisim_task_ros2.reset_testcase_request:main',
            'reset_testcase_server = hifisim_task_ros2.reset_testcase_server:main',
            'read_testcase = hifisim_task_ros2.read_testcase:main',
            'user_get_testcase = hifisim_task_ros2.user_get_testcase:main',
            'sim_flight = hifisim_task_ros2.sim_flight:main',
            'spawn_ooi_request = extra_code.spawn_ooi_request:main',
            'spawn_ooi_server = extra_code.spawn_ooi_server:main',
            'activate_ooi_request = extra_code.activate_ooi_request:main',
            'activate_ooi_server = extra_code.activate_ooi_server:main',
            'get_map_unity = hifisim_task_ros2.get_map_unity:main',
            'get_env_bounds_request = extra_code.get_env_bounds_request:main',
            'get_env_corners_request = extra_code.get_env_corners_request:main',
            'radio_comm_simulator = hifisim_task_ros2.radio_comm_simulator:main',
            'sim_agent = extra_code.sim_agent:main',
            'sim_reachable = extra_code.sim_reachable:main',
            'performance = hifisim_task_ros2.performance:main',
            'sim_gcs = extra_code.sim_gcs:main',
            'sim_hifisim = extra_code.sim_hifisim:main',
            'simulation_setup_service_request = hifisim_task_ros2.simulation_setup_service_request:main',
            'simulation_setup_service_server = extra_code.simulation_setup_service_server:main',
            'data_path = hifisim_task_ros2.data_path:main',
            'sub_nwu_pose = extra_code.sub_nwu_pose:main',
        ],
    },
)
