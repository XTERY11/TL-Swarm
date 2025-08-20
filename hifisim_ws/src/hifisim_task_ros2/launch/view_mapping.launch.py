from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
	config = LaunchConfiguration('config', default=os.path.expanduser('/home/ctx/hifisim_ws/rviz/phase3_env1.rviz'))
	return LaunchDescription([
		DeclareLaunchArgument('config', default_value=config,
								description='RViz config'),
		# static tf: world -> world_link (so TF tree exists)
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			name='world_tf',
			arguments=['0','0','0','0','0','0','world','world_link'],
			output='screen'
		),
		Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			arguments=['-d', config],
			output='screen'
		)
	]) 