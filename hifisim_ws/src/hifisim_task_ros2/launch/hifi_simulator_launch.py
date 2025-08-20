import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    unity_executable = os.path.expanduser('~/Unity/build/hifi_simulator_unity.x86_64')
    tcp_endpoint_launch_file = os.path.join(
        get_package_share_directory('tcp_endpoint_ros2'),
        'launch',
        'endpoint.py'
    )

	# Launch args for ROS TCP endpoint configuration
	ros_ip_arg = DeclareLaunchArgument('ros_ip', default_value='127.0.0.1', description='IP for Unity to connect to (this host)')
	ros_tcp_port_arg = DeclareLaunchArgument('ros_tcp_port', default_value='10000', description='TCP port for ROS TCP Endpoint')

	ros_ip = LaunchConfiguration('ros_ip')
	ros_tcp_port = LaunchConfiguration('ros_tcp_port')
	ros_ip_with_port = [ros_ip, ':', ros_tcp_port]

    return LaunchDescription([
		# Args
		ros_ip_arg,
		ros_tcp_port_arg,

		# Export ROS_IP for both Unity binary (client) and endpoint (server launch reads it)
		SetEnvironmentVariable(name='ROS_IP', value=ros_ip_with_port),

		# Launch Unity executable with env (inherits ROS_IP)
        ExecuteProcess(
            cmd=[unity_executable],
			output='screen',
			env={'ROS_IP': ros_ip_with_port}
        ),

		# Launch TCP endpoint after delay to allow Unity to load the scene (inherits ROS_IP)
        TimerAction(
            period=6.0,  # Adjust the delay as needed
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(tcp_endpoint_launch_file)
                )
            ]
        ),

        # Run performance node
        ExecuteProcess(
            cmd=['ros2', 'run', 'hifisim_task_ros2', 'performance'],
            output='screen'
        ),

        # Run radio_comm_simulator after a 6-second delay
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'hifisim_task_ros2', 'radio_comm_simulator'],
                    output='screen'
                )
            ]
        )
    ])
