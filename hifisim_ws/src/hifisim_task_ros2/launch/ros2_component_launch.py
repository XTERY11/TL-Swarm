import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tcp_endpoint_launch_file = os.path.join(
        get_package_share_directory('tcp_endpoint_ros2'),
        'launch',
        'endpoint.py'
    )

    return LaunchDescription([
        # Launch TCP endpoint
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tcp_endpoint_launch_file)
        ),

        # Run performance node
        ExecuteProcess(
            cmd=['ros2', 'run', 'hifisim_task_ros2', 'performance'],
            output='screen'
        ),

        # Run radio_comm_simulator
        ExecuteProcess(
            cmd=['ros2', 'run', 'hifisim_task_ros2', 'radio_comm_simulator'],
            output='screen'
        )
    ])
