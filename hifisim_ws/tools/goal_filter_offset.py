#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import argparse

class GoalFilter(Node):
	def __init__(self, node_name: str, in_topic: str, out_topic: str, odom_topic: str, backoff_m: float, z_min: float, z_max: float, min_interval_s: float):
		super().__init__(node_name)
		self.in_topic = in_topic
		self.out_topic = out_topic
		self.odom_topic = odom_topic
		self.backoff = float(backoff_m)
		self.z_min = float(z_min)
		self.z_max = float(z_max)
		self.min_interval_s = float(min_interval_s)
		self.current_xyz = None
		self.last_pub_time = None
		self.create_subscription(PoseStamped, self.in_topic, self.cb_goal, 10)
		self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 10)
		self.publisher = self.create_publisher(PoseStamped, self.out_topic, 10)
		self.get_logger().info(f"GoalFilter[{node_name}] {self.in_topic} -> {self.out_topic}, odom={self.odom_topic}, backoff={self.backoff}m, z=[{self.z_min},{self.z_max}], min_interval={self.min_interval_s}s")

	def cb_odom(self, msg: Odometry):
		p = msg.pose.pose.position
		self.current_xyz = (p.x, p.y, p.z)

	def cb_goal(self, msg: PoseStamped):
		# Debounce: skip if publishing too frequently
		now = self.get_clock().now()
		if self.last_pub_time is not None:
			elapsed = (now.nanoseconds - self.last_pub_time.nanoseconds) / 1e9
			if elapsed < self.min_interval_s:
				self.get_logger().warn(f"Debounced goal: interval {elapsed:.3f}s < {self.min_interval_s:.3f}s")
				return
		if self.current_xyz is None:
			self.publisher.publish(msg)
			self.last_pub_time = now
			self.get_logger().info("Pass-through goal (no odom yet)")
			return
		sx, sy, sz = self.current_xyz
		g = msg.pose.position
		vx, vy, vz = g.x - sx, g.y - sy, g.z - sz
		n = (vx*vx + vy*vy + vz*vz) ** 0.5 or 1.0
		k = self.backoff / n
		gx2, gy2, gz2 = g.x - k*vx, g.y - k*vy, g.z - k*vz
		gz2 = min(self.z_max, max(self.z_min, gz2))
		out = PoseStamped()
		out.header = msg.header
		out.header.frame_id = 'world'
		out.pose.position.x = gx2
		out.pose.position.y = gy2
		out.pose.position.z = gz2
		out.pose.orientation.w = 1.0
		self.publisher.publish(out)
		self.last_pub_time = now
		self.get_logger().info(f"Published offset goal -> ({gx2:.2f},{gy2:.2f},{gz2:.2f})")


def main():
	parser = argparse.ArgumentParser(description='Goal offset filter to avoid targets sticking to walls')
	parser.add_argument('--node', dest='node_name', type=str, default='goal_filter_offset_v2')
	parser.add_argument('--in', dest='in_topic', type=str, default='/move_base_simple/goal_in')
	parser.add_argument('--out', dest='out_topic', type=str, default='/move_base_simple/goal')
	parser.add_argument('--odom', dest='odom_topic', type=str, default='/drone_0_odom')
	parser.add_argument('--backoff', type=float, default=2.0)
	parser.add_argument('--z_min', type=float, default=1.8)
	parser.add_argument('--z_max', type=float, default=8.0)
	parser.add_argument('--min_interval', type=float, default=0.30)
	args = parser.parse_args()

	rclpy.init()
	node = GoalFilter(args.node_name, args.in_topic, args.out_topic, args.odom_topic, args.backoff, args.z_min, args.z_max, args.min_interval)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main() 