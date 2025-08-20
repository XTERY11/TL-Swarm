#!/usr/bin/env python3
import argparse
import csv
import math
import os
import signal
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
	"""Return yaw (rad) from quaternion (ENU/NWU yaw about Z)."""
	sin_yaw = 2.0 * (qw * qz + qx * qy)
	cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
	return math.atan2(sin_yaw, cos_yaw)


class TrajectoryRecorder(Node):
	def __init__(self, topic: str, out_csv: str, flush_every: int = 20):
		super().__init__('trajectory_recorder')
		self.topic = topic
		self.out_csv = out_csv
		self.flush_every = max(1, flush_every)
		self.writer: Optional[csv.writer] = None
		self.csv_file: Optional[csv.FileWriter] = None
		self.sample_count = 0
		self.last_written = None  # (x,y,z) to avoid spamming identical values

		# Ensure directory exists
		os.makedirs(os.path.dirname(out_csv) or '.', exist_ok=True)

		# Open CSV and write header if empty/new
		file_exists = os.path.exists(out_csv) and os.path.getsize(out_csv) > 0
		self.csv_file = open(out_csv, 'a', newline='')
		self.writer = csv.writer(self.csv_file)
		if not file_exists:
			self.writer.writerow(['sec', 'nanosec', 'x', 'y', 'z', 'yaw_rad'])
			self.csv_file.flush()

		self.sub = self.create_subscription(PoseStamped, topic, self.cb_pose, 20)
		self.get_logger().info(f"Recording PoseStamped from {topic} → {out_csv}")

	def cb_pose(self, msg: PoseStamped) -> None:
		pos = msg.pose.position
		ori = msg.pose.orientation
		yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
		xyz = (pos.x, pos.y, pos.z)
		if self.last_written is not None and all(
			abs(a - b) < 1e-6 for a, b in zip(xyz, self.last_written)
		):
			return
		self.last_written = xyz
		self.writer.writerow([msg.header.stamp.sec, msg.header.stamp.nanosec, pos.x, pos.y, pos.z, yaw])
		self.sample_count += 1
		if (self.sample_count % self.flush_every) == 0:
			self.csv_file.flush()

	def stop(self):
		self.get_logger().info(f"Stopping. Total samples written: {self.sample_count}")
		try:
			if self.csv_file:
				self.csv_file.flush()
				self.csv_file.close()
		except Exception:
			pass


def main():
	parser = argparse.ArgumentParser(description='Record agent trajectory (PoseStamped) into CSV')
	parser.add_argument('--topic', type=str, default='/agent001/global/sim_nwu_pose', help='PoseStamped topic to record')
	parser.add_argument('--out', type=str, default='/tmp/agent001_traj.csv', help='Output CSV path')
	parser.add_argument('--flush_every', type=int, default=20, help='Flush to disk every N samples')
	args = parser.parse_args()

	rclpy.init()
	recorder = TrajectoryRecorder(args.topic, args.out, args.flush_every)

	# Graceful shutdown on SIGINT/SIGTERM
	def handle_signal(signum, frame):
		recorder.get_logger().info(f"Signal {signum} received, shutting down...")
		recorder.stop()
		rclpy.shutdown()
		sys.exit(0)

	signal.signal(signal.SIGINT, handle_signal)
	signal.signal(signal.SIGTERM, handle_signal)

	try:
		rclpy.spin(recorder)
	except KeyboardInterrupt:
		pass
	finally:
		recorder.stop()
		rclpy.shutdown()


if __name__ == '__main__':
	main() 