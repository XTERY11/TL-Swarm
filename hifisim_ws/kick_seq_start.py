#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from traj_utils.msg import MultiBsplines
import argparse

class KickSeq(Node):
	def __init__(self, topic: str):
		super().__init__('kick_seq_start')
		qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
		self.pub = self.create_publisher(MultiBsplines, topic, qos)
		self.get_logger().info(f"Publishing empty MultiBsplines to {topic}")
		msg = MultiBsplines()
		self.pub.publish(msg)
		self.create_timer(0.2, self._shutdown)
	def _shutdown(self):
		rclpy.shutdown()


def main():
	parser = argparse.ArgumentParser(description='Publish one empty MultiBsplines to unlock SEQUENTIAL_START')
	parser.add_argument('--to', required=True, help='Topic to publish to, e.g. /drone_1_planning/swarm_trajs')
	args = parser.parse_args()
	
	rclpy.init()
	node = KickSeq(args.to)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass

if __name__ == '__main__':
	main() 