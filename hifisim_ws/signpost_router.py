#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray, Pose
import argparse
import math

from hifisim_task_ros2.read_testcase import (
	read_testcase_struct,
	get_signposts_endpoint_nwu_pos_rot_env,
)


def compute_pre_pass_post(position, rotation_deg_yaw, pre_dist: float, post_dist: float, safe_z: float):
	yaw_rad = math.radians(rotation_deg_yaw)
	dir_x = math.cos(yaw_rad)
	dir_y = math.sin(yaw_rad)
	pre = (position.x - dir_x * pre_dist, position.y - dir_y * pre_dist, safe_z)
	pas = (position.x, position.y, safe_z)
	post = (position.x + dir_x * post_dist, position.y + dir_y * post_dist, safe_z)
	return [pre, pas, post]


class SignpostRouter(Node):
	def __init__(self, agent_id: int, testcase_id: int, safe_z: float, pre_dist: float, post_dist: float):
		super().__init__('signpost_router')
		self.agent_id = agent_id
		self.safe_z = safe_z
		self.pre_dist = pre_dist
		self.post_dist = post_dist

		# topic: /agentXXX/route/waypoints
		topic = f"/agent{agent_id:03d}/route/waypoints"
		qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
		self.pub = self.create_publisher(PoseArray, topic, qos)

		# Load testcase and signposts
		tc = read_testcase_struct(testcase_id)
		env_id = tc['Testcase']['EnvId']
		scenario_id = tc['Testcase']['EnvScenarioId']
		sp = get_signposts_endpoint_nwu_pos_rot_env(env_id, scenario_id)
		if not sp:
			raise RuntimeError('No signposts found in testcase')

		core = sp[:-1]
		endpoint = sp[-1]

		poses = []
		for data in core:
			pos = data['position']
			rot = data['rotation']
			yaw_deg = -rot.z
			for (x, y, z) in compute_pre_pass_post(pos, yaw_deg, self.pre_dist, self.post_dist, self.safe_z):
				p = Pose()
				p.position.x = float(x)
				p.position.y = float(y)
				p.position.z = float(z)
				p.orientation.w = 1.0
				poses.append(p)
		# endpoint
		endp = Pose()
		endp.position.x = float(endpoint['position'].x)
		endp.position.y = float(endpoint['position'].y)
		endp.position.z = float(self.safe_z)
		endp.orientation.w = 1.0
		poses.append(endp)

		pa = PoseArray()
		pa.header.frame_id = 'map'
		pa.poses = poses
		self.pub.publish(pa)
		self.get_logger().info(f"Published {len(poses)} waypoints on {topic} (env={env_id}, scenario={scenario_id})")


def main():
	parser = argparse.ArgumentParser(description='Publish PoseArray waypoints from TestCase signposts')
	parser.add_argument('--agent_id', type=int, required=True)
	parser.add_argument('--testcase_id', type=int, required=True)
	parser.add_argument('--safe_z', type=float, default=1.8)
	parser.add_argument('--pre_dist', type=float, default=1.5)
	parser.add_argument('--post_dist', type=float, default=1.5)
	args = parser.parse_args()

	rclpy.init()
	node = SignpostRouter(args.agent_id, args.testcase_id, args.safe_z, args.pre_dist, args.post_dist)
	try:
		# single-shot publish; keep node alive briefly to ensure delivery to late joiners
		rclpy.spin_once(node, timeout_sec=0.2)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main() 