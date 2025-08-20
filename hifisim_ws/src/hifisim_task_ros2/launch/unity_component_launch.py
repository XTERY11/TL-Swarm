import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    unity_executable = os.path.expanduser('/home/hifisim/unity_build/hifi_simulator_unity.x86_64')

    return LaunchDescription([
        # Launch Unity executable
        ExecuteProcess(
            cmd=[unity_executable],
            output='screen'
        )
    ])
