#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import argparse

class OdomRelay(Node):
	def __init__(self, in_topic: str, out_topic: str):
		super().__init__('odom_relay')
		self.declare_parameter('in_topic', in_topic)
		self.declare_parameter('out_topic', out_topic)
		self.in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
		self.out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
		qos_sub = QoSProfile(
			reliability=ReliabilityPolicy.BEST_EFFORT,
			durability=DurabilityPolicy.VOLATILE,
			history=HistoryPolicy.KEEP_LAST,
			depth=10,
		)
		qos_pub = QoSProfile(
			reliability=ReliabilityPolicy.RELIABLE,
			durability=DurabilityPolicy.VOLATILE,
			history=HistoryPolicy.KEEP_LAST,
			depth=10,
		)
		self.pub = self.create_publisher(Odometry, self.out_topic, qos_pub)
		self.sub = self.create_subscription(Odometry, self.in_topic, self.cb, qos_sub)
		self.get_logger().info(f"Relaying: {self.in_topic} -> {self.out_topic}")

	def cb(self, msg: Odometry):
		self.pub.publish(msg)


def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('--in', dest='in_topic', required=True)
	parser.add_argument('--out', dest='out_topic', required=True)
	args = parser.parse_args()
	
	rclpy.init()
	node = OdomRelay(args.in_topic, args.out_topic)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main() 