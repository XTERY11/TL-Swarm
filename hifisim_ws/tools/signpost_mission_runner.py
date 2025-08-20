#!/usr/bin/env python3
import argparse
import json
import math
import os
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def unity_env_to_world(x_env: float, y_env: float, z_env: float) -> Tuple[float, float, float]:
	"""Convert Env JSON coords (x, y, z) to planner/world (x, y, z).
	Env1Scenario1.json uses EUN_RUF: x=east, y=up(height), z=north
	Planner world: x=east, y=north, z=up
	So: world_x = env.x, world_y = env.z, world_z = env.y
	"""
	return float(x_env), float(z_env), float(y_env)


def normalize(dx: float, dy: float) -> Tuple[float, float]:
	l = math.hypot(dx, dy)
	if l < 1e-6:
		return 1.0, 0.0
	return dx / l, dy / l


class SignpostMissionRunner(Node):
	def __init__(self, json_path: str, pose_topic: str, goal_topic: str, altitude: float, pre_offset: float, post_offset: float, pass_clear: float, max_dist_to_goal: float, signpost_regex: str, limit_num: int):
		super().__init__('signpost_mission_runner')
		self.altitude = float(altitude)
		self.pre_offset = float(pre_offset)
		self.post_offset = float(post_offset)
		self.pass_clear = float(pass_clear)
		self.max_dist_to_goal = float(max_dist_to_goal)
		self.limit_num = int(limit_num)
		self.targets: List[Tuple[float, float, float]] = []
		self.entries: List[Tuple[str, Tuple[float, float, float]]] = []  # raw signpost world coords
		self.target_idx: int = 0
		self.have_pose: bool = False
		self.curr_pose: Tuple[float, float, float] | None = None

		self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
		# Unity pose uses sensor-like QoS; subscribe with BEST_EFFORT to guarantee delivery
		sensor_qos = QoSProfile(
			depth=10,
			reliability=ReliabilityPolicy.BEST_EFFORT,
			durability=DurabilityPolicy.VOLATILE,
			history=HistoryPolicy.KEEP_LAST,
		)
		self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.cb_pose, sensor_qos)
		self._load_signposts(json_path, signpost_regex)
		self.get_logger().info(f"Loaded {len(self.entries)} signposts; will expand targets at runtime")

		self.timer = self.create_timer(0.2, self.tick)

	def _load_signposts(self, json_path: str, signpost_regex: str):
		with open(json_path, 'r') as f:
			data = json.load(f)
		signposts = data.get('Signposts', [])
		# Filter and keep in declared order
		entries: List[Tuple[str, Tuple[float,float,float]]] = []
		for sp in signposts:
			name = sp.get('ObjName', '')
			if not name:
				continue
			if signpost_regex and (signpost_regex not in name):
				continue
			tds = sp.get('TransformDatas', [])
			if not tds:
				continue
			pos = tds[0].get('Pos', {})
			x, y, z = pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0)
			# EUN(x_east,y_up,z_north) -> NWU(x_north,y_west,z_up) = (z, -x, y)
			wx, wy, wz = float(z), float(-x), float(y)
			entries.append((name, (wx, wy, self.altitude if self.altitude > 0 else wz)))
		# Store raw entries; expand lazily in tick()
		self.entries = entries[: self.limit_num if self.limit_num > 0 else None]

	def cb_pose(self, msg: PoseStamped):
		self.curr_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
		self.have_pose = True

	def publish_goal(self, xyz: Tuple[float, float, float]):
		msg = PoseStamped()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'world'
		msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
		msg.pose.orientation.w = 1.0
		self.goal_pub.publish(msg)
		self.get_logger().info(f"Goal[{self.target_idx}/{len(self.targets)}]: {xyz}")

	def tick(self):
		if not self.have_pose:
			# throttle log
			if (self.get_clock().now().nanoseconds // 1_000_000_000) % 3 == 0:
				self.get_logger().info('Waiting for pose...')
			return
		# Lazily expand first signpost goals using live pose vector
		if not self.targets and self.entries:
			# Build for first signpost using current pose → signpost vector
			xd, yd, zd = self.curr_pose
			name0, (x0, y0, z0) = self.entries[0]
			u_x0, u_y0 = normalize(x0 - xd, y0 - yd)
			pre0 = (x0 - u_x0 * self.pre_offset, y0 - u_y0 * self.pre_offset, z0)
			pas0 = (x0 - u_x0 * max(0.0, self.pass_clear), y0 - u_y0 * max(0.0, self.pass_clear), z0)
			pst0 = (x0 + u_x0 * self.post_offset, y0 + u_y0 * self.post_offset, z0)
			self.targets.extend([pre0, pas0, pst0])
			# Expand remaining signposts using prev->curr vector
			prev_xy = (x0, y0)
			for name, (x, y, z) in self.entries[1:]:
				u_x, u_y = normalize(x - prev_xy[0], y - prev_xy[1])
				pre = (x - u_x * self.pre_offset, y - u_y * self.pre_offset, z)
				pas = (x - u_x * max(0.0, self.pass_clear), y - u_y * max(0.0, self.pass_clear), z)
				pst = (x + u_x * self.post_offset, y + u_y * self.post_offset, z)
				self.targets.extend([pre, pas, pst])
				prev_xy = (x, y)
			self.get_logger().info(f"Expanded {len(self.targets)} waypoints from {len(self.entries)} signposts")
		if self.target_idx >= len(self.targets):
			return
		# if no active goal yet, publish first
		if self.target_idx == 0:
			self.publish_goal(self.targets[self.target_idx])
			self.target_idx += 1
			return
		# check distance to last published goal
		gx, gy, gz = self.targets[self.target_idx-1]
		x, y, z = self.curr_pose
		d = math.hypot(x - gx, y - gy)
		if d <= self.max_dist_to_goal:
			if self.target_idx < len(self.targets):
				self.publish_goal(self.targets[self.target_idx])
				self.target_idx += 1


def main():
	parser = argparse.ArgumentParser(description='Signpost mission runner (pre-pass-post per signpost)')
	parser.add_argument('--json', type=str, default='/home/ctx/Unity/build/hifi_simulator_unity_Data/StreamingAssets/AppData/EnvScenarios/Env1Scenario1.json', help='Env scenario JSON path')
	parser.add_argument('--pose', type=str, default='/agent001/global/sim_nwu_pose', help='Agent global pose topic (PoseStamped)')
	parser.add_argument('--goal', type=str, default='/move_base_simple/goal', help='Planner manual goal topic (PoseStamped)')
	parser.add_argument('--alt', type=float, default=2.0, help='Flight altitude for goals (m). If <=0, use JSON y→world z')
	parser.add_argument('--pre', type=float, default=2.0, help='Pre offset distance before signpost (m)')
	parser.add_argument('--post', type=float, default=2.0, help='Post offset distance beyond signpost (m)')
	parser.add_argument('--pass_clear', type=float, default=0.6, help='Keep pass waypoint this far in front of wall (m)')
	parser.add_argument('--reach', type=float, default=1.0, help='Distance threshold to switch to next goal (m)')
	parser.add_argument('--filter', type=str, default='Signpost_', help='Substring to select signposts')
	parser.add_argument('--limit', type=int, default=0, help='Limit number of signposts to use (0=all)')
	args = parser.parse_args()

	rclpy.init()
	runner = SignpostMissionRunner(args.json, args.pose, args.goal, args.alt, args.pre, args.post, args.pass_clear, args.reach, args.filter, args.limit)
	try:
		rclpy.spin(runner)
	except KeyboardInterrupt:
		pass
	finally:
		runner.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main() 