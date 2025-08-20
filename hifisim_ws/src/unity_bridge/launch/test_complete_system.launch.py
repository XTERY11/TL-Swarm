#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():
    """启动完整的测试系统"""
    
    # 获取参数文件路径
    unity_bridge_dir = get_package_share_directory('unity_bridge')
    params_file = os.path.join(unity_bridge_dir, 'config', 'ego_planner.yaml')
    
    # 1. 模拟Unity数据发布
    mock_unity = Node(
        package='unity_bridge',
        executable='mock_unity_publisher',
        name='mock_unity_publisher',
        output='screen'
    )
    
    # 2. Unity数据桥接
    unity_bridge = Node(
        package='unity_bridge',
        executable='unity_bridge_node',
        name='unity_bridge_node',
        output='screen',
        parameters=[{
            'drone_id': 0,
        }]
    )
    
    # 3. EGO-Planner
    ego_planner = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        parameters=[
            params_file,
            {
                'manager/drone_id': 0,
                'manager/max_vel': 0.3,
                'manager/max_acc': 0.5,
                'manager/max_jerk': 0.5,
                'manager/feasibility_tolerance': 0.2,
                'manager/planning_horizon': 1.0,
                'manager/control_points_distance': 0.5,
                'manager/use_distinctive_trajs': False,
                'fsm/flight_type': 1,
                'fsm/thresh_replan_time': 2.0,
                'fsm/thresh_no_replan_meter': 1.0,
                'fsm/planning_horizen_time': 3.0,
                'fsm/emergency_time': 1.0,
                'fsm/realworld_experiment': False,
                'fsm/fail_safe': True,
                'fsm/single_drone': 1
            }
        ],
        remappings=[
            ('/odom_world', '/drone_0_visual_slam/odom'),
            ('/grid_map/cloud', '/drone_0_cloud'),
            ('/grid_map/odom', '/drone_0_visual_slam/odom')
        ]
    )
    
    # 4. 轨迹执行器
    traj_server = Node(
        package='ego_planner',
        executable='traj_server',
        name='traj_server',
        output='screen',
        parameters=[{
            'drone_id': 0
        }]
    )
    
    # 5. 目标点发布器
    goal_publisher = Node(
        package='unity_bridge',
        executable='goal_publisher',
        name='goal_publisher',
        output='screen'
    )
    
    # 按顺序启动节点
    return LaunchDescription([
        mock_unity,
        TimerAction(
            period=3.0,
            actions=[unity_bridge]
        ),
        TimerAction(
            period=6.0,
            actions=[ego_planner]
        ),
        TimerAction(
            period=9.0,
            actions=[traj_server]
        ),
        TimerAction(
            period=12.0,
            actions=[goal_publisher]
        )
    ]) 