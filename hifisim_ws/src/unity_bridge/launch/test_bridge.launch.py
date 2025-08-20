import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_drones = LaunchConfiguration('num_drones', default='2')

    num_drones_arg = DeclareLaunchArgument('num_drones', default_value='2', description='Number of drones in swarm')

    ld = LaunchDescription()
    ld.add_action(num_drones_arg)

    # Just test the bridge nodes for now
    for drone_id in [0, 1]:
        # ---- bridging node ----
        bridge_node = Node(
            package='unity_bridge',
            executable='unity_bridge_node',
            name=f'drone_{drone_id}_unity_bridge',
            parameters=[{
                'drone_id': drone_id,
                'odom_in_topic': f'/agent{drone_id+1:03d}/global/sim_nwu_pose',
                'cloud_in_topic': f'/agent{drone_id+1:03d}/lidar{drone_id+1:02d}'
            }],
            output='screen'
        )
        ld.add_action(bridge_node)

    return ld 