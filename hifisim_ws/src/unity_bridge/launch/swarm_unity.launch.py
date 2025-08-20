import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    num_drones = LaunchConfiguration('num_drones', default='2')

    num_drones_arg = DeclareLaunchArgument('num_drones', default_value='2', description='Number of drones in swarm')

    ld = LaunchDescription()
    ld.add_action(num_drones_arg)

    # Convert num_drones to python int at runtime via PythonExpression not ideal.
    # We'll just instantiate for 2 drones; users can edit file for more.
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

        # ---- planner include (advanced_param) ----
        # Note: ego_planner is actually plan_manage package
        adv_param_path = os.path.join(
            get_package_share_directory('ego_planner'),
            'launch',
            'advanced_param.launch.py'
        )

        planner_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(adv_param_path),
            launch_arguments={
                'drone_id': str(drone_id),
                # map/camera/waypoint params can be tuned here
                'point_num': '2',
                'point0_x': '-10.0',
                'point0_y': '0.0',
                'point0_z': '2.0',
                'point1_x': '10.0',
                'point1_y': '0.0',
                'point1_z': '2.0',
                'odometry_topic': 'visual_slam/odom',
                'use_distinctive_trajs': 'True',
                'max_vel': '2.0',
                'max_acc': '6.0'
            }.items()
        )
        ld.add_action(planner_include)

        # ---- trajectory server ----
        traj_server = Node(
            package='ego_planner',
            executable='traj_server',
            name=f'drone_{drone_id}_traj_server',
            output='screen',
            remappings=[
                ('position_cmd', f'drone_{drone_id}_planning/pos_cmd'),
                ('planning/bspline', f'drone_{drone_id}_planning/bspline')
            ],
            parameters=[{'traj_server/time_forward': 1.0}]
        )
        ld.add_action(traj_server)

    return ld 