import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	# LaunchConfigurations
	# 修改地图参数以匹配Unity env1的实际尺寸
	# env1实际范围：X: -70到50 (120m), Y: 0, Z: -30到50 (80m)
	# 设置地图中心在(0,0,*)覆盖整个env1区域

	# 地图尺寸参数
	map_size_x = LaunchConfiguration('map_size_x_', default='120.0')  # X轴覆盖120m
	map_size_y = LaunchConfiguration('map_size_y_', default='80.0')   # Y轴覆盖80m（Unity的Z）
	map_size_z = LaunchConfiguration('map_size_z_', default='40.0')   # 高度覆盖，适度降低计算量
	
	odometry_topic = LaunchConfiguration('odometry_topic', default='odom')
	camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')
	depth_topic = LaunchConfiguration('depth_topic', default='depth_image')
	cloud_topic = LaunchConfiguration('cloud_topic', default='cloud')
	
	cx = LaunchConfiguration('cx', default=321.04638671875)
	cy = LaunchConfiguration('cy', default=243.44969177246094)
	fx = LaunchConfiguration('fx', default=387.229248046875)
	fy = LaunchConfiguration('fy', default=387.229248046875)
	
	max_vel = LaunchConfiguration('max_vel', default=1.2)
	max_acc = LaunchConfiguration('max_acc', default=2.0)
	planning_horizon = LaunchConfiguration('planning_horizon', default=7.5)
	
	point_num = LaunchConfiguration('point_num', default=1)
	point0_x = LaunchConfiguration('point0_x', default=0.0)
	point0_y = LaunchConfiguration('point0_y', default=0.0)
	point0_z = LaunchConfiguration('point0_z', default=0.0)
	point1_x = LaunchConfiguration('point1_x', default=10.0)
	point1_y = LaunchConfiguration('point1_y', default=10.0)
	point1_z = LaunchConfiguration('point1_z', default=0.0)
	point2_x = LaunchConfiguration('point2_x', default=20.0)
	point2_y = LaunchConfiguration('point2_y', default=20.0)
	point2_z = LaunchConfiguration('point2_z', default=1.0)
	point3_x = LaunchConfiguration('point3_x', default=-10.0)
	point3_y = LaunchConfiguration('point3_y', default=-10.0)
	point3_z = LaunchConfiguration('point3_z', default=1.0)
	point4_x = LaunchConfiguration('point4_x', default=30.0)
	point4_y = LaunchConfiguration('point4_y', default=30.0)
	point4_z = LaunchConfiguration('point4_z', default=1.0)
	# 新增：grid_map关键参数（之前缺失会导致变量未定义）
	resolution = LaunchConfiguration('resolution_', default='0.2')
	local_update_range_x = LaunchConfiguration('local_update_range_x_', default='12.0')
	local_update_range_y = LaunchConfiguration('local_update_range_y_', default='12.0')
	local_update_range_z = LaunchConfiguration('local_update_range_z_', default='8.0')
	obstacles_inflation = LaunchConfiguration('obstacles_inflation_', default='1.5')
	ground_height = LaunchConfiguration('ground_height_', default='0.0')
	virtual_ceil_height = LaunchConfiguration('virtual_ceil_height_', default='35.0')
	
	flight_type = LaunchConfiguration('flight_type', default=2)
	use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs', default=True)
	
	obj_num_set = LaunchConfiguration('obj_num_set', default=10)
	
	drone_id = LaunchConfiguration('drone_id', default=0)

	# DeclareLaunchArguments
	map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X')
	map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y')
	map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z')
	# 新增：grid_map参数的声明加入到LD
	resolution_arg = DeclareLaunchArgument('resolution_', default_value=resolution, description='Grid map resolution (m)')
	local_update_range_x_arg = DeclareLaunchArgument('local_update_range_x_', default_value=local_update_range_x, description='Local update range X (m)')
	local_update_range_y_arg = DeclareLaunchArgument('local_update_range_y_', default_value=local_update_range_y, description='Local update range Y (m)')
	local_update_range_z_arg = DeclareLaunchArgument('local_update_range_z_', default_value=local_update_range_z, description='Local update range Z (m)')
	obstacles_inflation_arg = DeclareLaunchArgument('obstacles_inflation_', default_value=obstacles_inflation, description='Obstacle inflation radius (m)')
	ground_height_arg = DeclareLaunchArgument('ground_height_', default_value=ground_height, description='Ground height (m)')
	virtual_ceil_height_arg = DeclareLaunchArgument('virtual_ceil_height_', default_value=virtual_ceil_height, description='Virtual ceiling height (m)')
	odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
	camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic')
	depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth topic')
	cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud topic')
	cx_arg = DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx')
	cy_arg = DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy')
	fx_arg = DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx')
	fy_arg = DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy')
	max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')
	max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration')
	planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon')
	
	point_num_arg = DeclareLaunchArgument('point_num', default_value=point_num, description='Number of waypoints')
	point0_x_arg = DeclareLaunchArgument('point0_x', default_value=point0_x, description='Waypoint 0 X coordinate')
	point0_y_arg = DeclareLaunchArgument('point0_y', default_value=point0_y, description='Waypoint 0 Y coordinate')
	point0_z_arg = DeclareLaunchArgument('point0_z', default_value=point0_z, description='Waypoint 0 Z coordinate')
	point1_x_arg = DeclareLaunchArgument('point1_x', default_value=point1_x, description='Waypoint 1 X coordinate')
	point1_y_arg = DeclareLaunchArgument('point1_y', default_value=point1_y, description='Waypoint 1 Y coordinate')
	point1_z_arg = DeclareLaunchArgument('point1_z', default_value=point1_z, description='Waypoint 1 Z coordinate')
	point2_x_arg = DeclareLaunchArgument('point2_x', default_value=point2_x, description='Waypoint 2 X coordinate')
	point2_y_arg = DeclareLaunchArgument('point2_y', default_value=point2_y, description='Waypoint 2 Y coordinate')
	point2_z_arg = DeclareLaunchArgument('point2_z', default_value=point2_z, description='Waypoint 2 Z coordinate')
	point3_x_arg = DeclareLaunchArgument('point3_x', default_value=point3_x, description='Waypoint 3 X coordinate')
	point3_y_arg = DeclareLaunchArgument('point3_y', default_value=point3_y, description='Waypoint 3 Y coordinate')
	point3_z_arg = DeclareLaunchArgument('point3_z', default_value=point3_z, description='Waypoint 3 Z coordinate')
	point4_x_arg = DeclareLaunchArgument('point4_x', default_value=point4_x, description='Waypoint 4 X coordinate')
	point4_y_arg = DeclareLaunchArgument('point4_y', default_value=point4_y, description='Waypoint 4 Y coordinate')
	point4_z_arg = DeclareLaunchArgument('point4_z', default_value=point4_z, description='Waypoint 4 Z coordinate')
	
	flight_type_arg = DeclareLaunchArgument('flight_type', default_value=flight_type, description='flight_type')
	use_distinctive_trajs_arg = DeclareLaunchArgument('use_distinctive_trajs', default_value=use_distinctive_trajs, description='Use distinctive trajectories')
	obj_num_set_arg = DeclareLaunchArgument('obj_num_set', default_value=obj_num_set, description='Number of objects')
	drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')

	# Ego Planner Node
	ego_planner_node = Node(
		package='ego_planner',
		executable='ego_planner_node',
		name=['drone_', drone_id, '_ego_planner_node'],
		output='screen',
		remappings=[
			('odom_world', ['drone_', drone_id, '_', odometry_topic]),
			('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
			('planning/data_display', ['drone_', drone_id, '_planning/data_display']),
			('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
			('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
			('goal_point', ['drone_', drone_id, '_plan_vis/goal_point']),
			('global_list', ['drone_', drone_id, '_plan_vis/global_list']),
			('init_list', ['drone_', drone_id, '_plan_vis/init_list']),
			('optimal_list', ['drone_', drone_id, '_plan_vis/optimal_list']),
			('a_star_list', ['drone_', drone_id, '_plan_vis/a_star_list']),
			('grid_map/odom', ['drone_', drone_id, '_', odometry_topic]),
			('grid_map/cloud', ['drone_', drone_id, '_', cloud_topic]),
			('grid_map/pose', ['drone_', drone_id, '_', camera_pose_topic]),
			('grid_map/depth', ['drone_', drone_id, '_', depth_topic]),
			('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate'])
		],
		parameters=[
			{'fsm/flight_type': flight_type},
			{'fsm/thresh_replan_time': 1.0},
			{'fsm/thresh_no_replan_meter': 1.0},
			{'fsm/planning_horizon': planning_horizon},
			{'fsm/planning_horizen_time': 3.0},
			{'fsm/emergency_time': 1.0},
			{'fsm/realworld_experiment': False},
			{'fsm/fail_safe': True},
			{'fsm/waypoint_num': point_num},
			{'fsm/waypoint0_x': point0_x},
			{'fsm/waypoint0_y': point0_y},
			{'fsm/waypoint0_z': point0_z},
			{'fsm/waypoint1_x': point1_x},
			{'fsm/waypoint1_y': point1_y},
			{'fsm/waypoint1_z': point1_z},
			{'fsm/waypoint2_x': point2_x},
			{'fsm/waypoint2_y': point2_y},
			{'fsm/waypoint2_z': point2_z},
			{'fsm/waypoint3_x': point3_x},
			{'fsm/waypoint3_y': point3_y},
			{'fsm/waypoint3_z': point3_z},
			{'fsm/waypoint4_x': point4_x},
			{'fsm/waypoint4_y': point4_y},
			{'fsm/waypoint4_z': point4_z},
			# grid_map核心参数
			{'grid_map/resolution': resolution},
			{'grid_map/map_size_x': map_size_x},
			{'grid_map/map_size_y': map_size_y},
			{'grid_map/map_size_z': map_size_z},
			{'grid_map/local_update_range_x': local_update_range_x},
			{'grid_map/local_update_range_y': local_update_range_y},
			{'grid_map/local_update_range_z': local_update_range_z},
			{'grid_map/obstacles_inflation': obstacles_inflation},
			{'grid_map/local_map_margin': 10},
			{'grid_map/ground_height': ground_height},
			# 相机参数
			{'grid_map/cx': cx},
			{'grid_map/cy': cy},
			{'grid_map/fx': fx},
			{'grid_map/fy': fy},
			# 深度/点云过滤（关键）
			{'grid_map/use_depth_filter': False},
			{'grid_map/depth_filter_tolerance': 0.15},
			{'grid_map/depth_filter_maxdist': 35.0},
			{'grid_map/depth_filter_mindist': 1.0},
			{'grid_map/depth_filter_margin': 2},
			{'grid_map/k_depth_scaling_factor': 1000.0},
			{'grid_map/skip_pixel': 2},
			# 局部融合概率
			{'grid_map/p_hit': 0.65},
			{'grid_map/p_miss': 0.35},
			{'grid_map/p_min': 0.12},
			{'grid_map/p_max': 0.90},
			{'grid_map/p_occ': 0.80},
			# 光线截断（与maxdist一致，避免过短）
			{'grid_map/min_ray_length': 1.0},
			{'grid_map/max_ray_length': 35.0},
			# 天花板/可视化高度
			{'grid_map/virtual_ceil_height': virtual_ceil_height},
			{'grid_map/visualization_truncate_height': 30.0},
			{'grid_map/show_occ_time': False},
			{'grid_map/pose_type': 2},
			{'grid_map/frame_id': 'world'},
			# manager/optimization/bspline
			{'manager/max_vel': max_vel},
			{'manager/max_acc': max_acc},
			{'manager/max_jerk': 4.0},
			{'manager/control_points_distance': 0.4},
			{'manager/feasibility_tolerance': 0.05},
			{'manager/planning_horizon': planning_horizon},
			{'manager/use_distinctive_trajs': use_distinctive_trajs},
			{'manager/drone_id': drone_id},
			{'optimization/lambda_smooth': 1.0},
			{'optimization/lambda_collision': 80.0},
			{'optimization/lambda_feasibility': 0.1},
			{'optimization/lambda_fitness': 1.0},
			{'optimization/dist0': 1.5},
			{'optimization/swarm_clearance': 0.5},
			{'optimization/max_vel': max_vel},
			{'optimization/max_acc': max_acc},
			# 注意：bspline/* 参数在本实现中未声明，保留原样或依赖其他模块使用
			# {'bspline/limit_vel': max_vel},
			# {'bspline/limit_acc': max_acc},
			{'bspline/limit_ratio': 1.1},
			{'prediction/obj_num': obj_num_set},
			{'prediction/lambda': 1.0},
			{'prediction/predict_rate': 1.0}
		]
	)

	# Create LaunchDescription
	ld = LaunchDescription()

	# Add LaunchArguments
	ld.add_action(map_size_x_arg)
	ld.add_action(map_size_y_arg)
	ld.add_action(map_size_z_arg)
	# 新增：grid_map参数的声明加入到LD
	ld.add_action(resolution_arg)
	ld.add_action(local_update_range_x_arg)
	ld.add_action(local_update_range_y_arg)
	ld.add_action(local_update_range_z_arg)
	ld.add_action(obstacles_inflation_arg)
	ld.add_action(ground_height_arg)
	ld.add_action(virtual_ceil_height_arg)
	ld.add_action(odometry_topic_arg)
	ld.add_action(camera_pose_topic_arg)
	ld.add_action(depth_topic_arg)
	ld.add_action(cloud_topic_arg)
	ld.add_action(cx_arg)
	ld.add_action(cy_arg)
	ld.add_action(fx_arg)
	ld.add_action(fy_arg)
	ld.add_action(max_vel_arg)
	ld.add_action(max_acc_arg)
	ld.add_action(planning_horizon_arg)
	ld.add_action(point_num_arg)
	ld.add_action(point0_x_arg)
	ld.add_action(point0_y_arg)
	ld.add_action(point0_z_arg)
	ld.add_action(point1_x_arg)
	ld.add_action(point1_y_arg)
	ld.add_action(point1_z_arg)
	ld.add_action(point2_x_arg)
	ld.add_action(point2_y_arg)
	ld.add_action(point2_z_arg)
	ld.add_action(point3_x_arg)
	ld.add_action(point3_y_arg)
	ld.add_action(point3_z_arg)
	ld.add_action(point4_x_arg)
	ld.add_action(point4_y_arg)
	ld.add_action(point4_z_arg)
	ld.add_action(flight_type_arg)
	ld.add_action(use_distinctive_trajs_arg)
	ld.add_action(obj_num_set_arg)
	ld.add_action(drone_id_arg)

	
	# Add Node
	ld.add_action(ego_planner_node)

	return ld
