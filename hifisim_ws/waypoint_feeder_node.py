#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
import math
import argparse
from typing import List, Optional


def dist3(a, b):
	return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


class WaypointFeederNode(Node):
	def __init__(self, agent_id: int, reach_thresh: float, resend_interval_s: float):
		super().__init__('waypoint_feeder_node')
		self.agent_id = agent_id
		self.reach_thresh = reach_thresh
		self.resend_interval_s = resend_interval_s

		self.waypoints: List[List[float]] = []
		self.idx: int = 0
		self.last_sent_idx: Optional[int] = None
		self.last_sent_time = self.get_clock().now()
		self.have_odom = False
		self.current_pos = (0.0, 0.0, 0.0)

		qos_lat = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
		qos_cmd = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)

		route_topic = f"/agent{agent_id:03d}/route/waypoints"
		odom_topic = f"/drone_{agent_id-1}_/drone_{agent_id-1}_odom"  # agent001->drone_0 映射

		self.sub_route = self.create_subscription(PoseArray, route_topic, self.cb_route, qos_cmd)
		self.sub_odom = self.create_subscription(Odometry, odom_topic, self.cb_odom, qos_lat)
		self.pub_trig = self.create_publisher(PoseStamped, '/traj_start_trigger', qos_cmd)
		self.pub_ctrl_target = self.create_publisher(PoseStamped, f"/agent{agent_id:03d}/ctrl_target", qos_cmd)

		self.timer = self.create_timer(0.1, self.tick)
		self.get_logger().info(f"Listening route:{route_topic}, odom:{odom_topic}")

	def cb_route(self, msg: PoseArray):
		self.waypoints = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
		self.idx = 0
		self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints; start from 0")

	def cb_odom(self, msg: Odometry):
		self.have_odom = True
		self.current_pos = (
			msg.pose.pose.position.x,
			msg.pose.pose.position.y,
			msg.pose.pose.position.z,
		)

	def tick(self):
		if not self.waypoints or not self.have_odom:
			return
		if self.idx >= len(self.waypoints):
			return
		wp = self.waypoints[self.idx]
		d = dist3(self.current_pos, wp)
		# 如果接近则推进
		if d <= self.reach_thresh:
			self.idx += 1
			if self.idx >= len(self.waypoints):
				self.get_logger().info("All waypoints reached")
				return
			wp = self.waypoints[self.idx]
			self.get_logger().info(f"Reached idx {self.idx-1}, advance to {self.idx}")

		# 节流发送触发（第一次或超时重发）
		now = self.get_clock().now()
		elapsed = (now - self.last_sent_time).nanoseconds / 1e9
		if self.last_sent_idx != self.idx or elapsed >= self.resend_interval_s:
			msg = PoseStamped()
			msg.header.frame_id = 'map'
			msg.header.stamp = now.to_msg()
			msg.pose.position.x = float(wp[0])
			msg.pose.position.y = float(wp[1])
			msg.pose.position.z = float(wp[2])
			msg.pose.orientation.w = 1.0
			self.pub_trig.publish(msg)
			# 额外：面向控制器适配的每-agent目标
			self.pub_ctrl_target.publish(msg)
			self.last_sent_idx = self.idx
			self.last_sent_time = now
			self.get_logger().info(f"Trigger idx {self.idx} at d={d:.2f}")


def main():
	parser = argparse.ArgumentParser(description='Waypoint Feeder Node that drives /traj_start_trigger from PoseArray route and odom')
	parser.add_argument('--agent_id', type=int, required=True)
	parser.add_argument('--reach_thresh', type=float, default=1.0)
	parser.add_argument('--resend', type=float, default=2.0)
	args = parser.parse_args()

	rclpy.init()
	node = WaypointFeederNode(args.agent_id, args.reach_thresh, args.resend)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main() 