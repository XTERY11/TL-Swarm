from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command

import time

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hifisim_task_ros2',
            executable='radio_comm_simulator',
            output='screen'
        ),
        Node(
            package='hifisim_task_ros2',
            executable='sim_reachable',
            output='screen'
        ),
        # Add a delay of 5 seconds before running sim_agent
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='hifisim_task_ros2',
                    executable='sim_agent',
                    output='screen',
                    arguments=['2'],  # agent_id
                    prefix='gnome-terminal --'  # Opens sim_agent in a new terminal
                )
            ]
        ),
        # Add a delay of 5 seconds before running 2nd sim_agent
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='hifisim_task_ros2',
                    executable='sim_agent',
                    output='screen',
                    arguments=['3'],  # agent_id
                    prefix='gnome-terminal --'  # Opens sim_agent in a new terminal
                )
            ]
        )
    ])
