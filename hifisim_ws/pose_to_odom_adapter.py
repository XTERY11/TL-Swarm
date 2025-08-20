#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import argparse
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class PoseToOdomAdapter(Node):
	def __init__(self, in_topic: str, out_topic: str, frame_id: str, child_frame_id: str):
		super().__init__('pose_to_odom_adapter')
		self.declare_parameter('in_topic', in_topic)
		self.declare_parameter('out_topic', out_topic)
		self.declare_parameter('frame_id', frame_id)
		self.declare_parameter('child_frame_id', child_frame_id)

		self.in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
		self.out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
		self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
		self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

		self.odom_pub = self.create_publisher(Odometry, self.out_topic, 10)
		# Sensor-data QoS to match Unity publishers (best-effort, volatile)
		sensor_qos = QoSProfile(
			depth=10,
			reliability=ReliabilityPolicy.BEST_EFFORT,
			durability=DurabilityPolicy.VOLATILE,
			history=HistoryPolicy.KEEP_LAST,
		)
		self.pose_sub = self.create_subscription(PoseStamped, self.in_topic, self.pose_cb, sensor_qos)
		self.get_logger().info(f"Subscribing: {self.in_topic} -> Publishing: {self.out_topic} (frame_id={self.frame_id}, child={self.child_frame_id})")

		self._last_odom = None
		# 20 Hz republish to keep consumers alive even if pose is static
		self._timer = self.create_timer(0.05, self._republish_last)

	def pose_cb(self, msg: PoseStamped):
		odom = Odometry()
		odom.header.stamp = msg.header.stamp
		# enforce frame id to target map frame for Ego grid_map
		odom.header.frame_id = self.frame_id
		odom.child_frame_id = self.child_frame_id
		odom.pose.pose = msg.pose
		# zero twists (Unity true pose used; no velocity from source)
		odom.twist.twist.linear.x = 0.0
		odom.twist.twist.linear.y = 0.0
		odom.twist.twist.linear.z = 0.0
		odom.twist.twist.angular.x = 0.0
		odom.twist.twist.angular.y = 0.0
		odom.twist.twist.angular.z = 0.0
		self._last_odom = odom
		self.odom_pub.publish(odom)

	def _republish_last(self):
		if self._last_odom is not None:
			# publish with current node clock to avoid stale time filters
			odom = Odometry()
			odom.header.stamp = self.get_clock().now().to_msg()
			odom.header.frame_id = self._last_odom.header.frame_id
			odom.child_frame_id = self._last_odom.child_frame_id
			odom.pose = self._last_odom.pose
			odom.twist = self._last_odom.twist
			self.odom_pub.publish(odom)


def main():
	parser = argparse.ArgumentParser(description='Convert PoseStamped to Odometry for Ego grid_map input')
	parser.add_argument('--in', dest='in_topic', required=True, help='Input PoseStamped topic (e.g. /agent001/global/nwu_pose)')
	parser.add_argument('--out', dest='out_topic', required=True, help='Output Odometry topic (e.g. /drone_0_grid/grid_map/odom)')
	parser.add_argument('--frame', dest='frame_id', default='map', help='Odometry header.frame_id (default: map)')
	parser.add_argument('--child', dest='child_frame_id', default='base_link', help='Odometry child_frame_id (default: base_link)')
	args = parser.parse_args()

	rclpy.init()
	node = PoseToOdomAdapter(args.in_topic, args.out_topic, args.frame_id, args.child_frame_id)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		try:
			node.destroy_node()
		except Exception:
			pass
		try:
			rclpy.shutdown()
		except Exception:
			pass


if __name__ == '__main__':
	main() 