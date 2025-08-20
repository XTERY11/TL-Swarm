#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
import argparse
import struct
from typing import List, Tuple


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> List[List[float]]:
	# Convert quaternion (x,y,z,w) to 3x3 rotation matrix
	x2, y2, z2 = x + x, y + y, z + z
	xx, yy, zz = x * x2, y * y2, z * z2
	xy, xz, yz = x * y2, x * z2, y * z2
	wx, wy, wz = w * x2, w * y2, w * z2
	return [
		[1.0 - (yy + zz), xy - wz, xz + wy],
		[xy + wz, 1.0 - (xx + zz), yz - wx],
		[xz - wy, yz + wx, 1.0 - (xx + yy)],
	]


def transform_point(p: Tuple[float, float, float], R: List[List[float]], t: Tuple[float, float, float]) -> Tuple[float, float, float]:
	x = R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2] + t[0]
	y = R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2] + t[1]
	z = R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2] + t[2]
	return (x, y, z)


class CloudRelay(Node):
	def __init__(self, in_topic: str, out_topic: str, pose_topic: str | None, assume_world: bool):
		super().__init__('cloud_relay')
		self.declare_parameter('in_topic', in_topic)
		self.declare_parameter('out_topic', out_topic)
		self.declare_parameter('pose_topic', pose_topic if pose_topic else '')
		self.declare_parameter('assume_world', bool(assume_world))

		self.in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
		self.out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
		self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
		self.assume_world = self.get_parameter('assume_world').get_parameter_value().bool_value

		self.pub = self.create_publisher(PointCloud2, self.out_topic, 10)
		self.sub = self.create_subscription(PointCloud2, self.in_topic, self.cb, 10)

		self.have_pose = False
		self.R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
		self.t = (0.0, 0.0, 0.0)
		if self.pose_topic and not self.assume_world:
			self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.cb_pose, 10)
			self.get_logger().info(f"Relaying: {self.in_topic} -> {self.out_topic} (transform to 'world' using pose:{self.pose_topic})")
		else:
			self.get_logger().info(f"Relaying: {self.in_topic} -> {self.out_topic} (assume input already in 'world')")

	def cb_pose(self, msg: PoseStamped):
		# Pose is NWU in 'world' frame; use directly
		self.t = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
		qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
		self.R = quaternion_to_rotation_matrix(qx, qy, qz, qw)
		self.have_pose = True

	def cb(self, msg: PointCloud2):
		if self.assume_world or not self.pose_topic:
			msg.header.frame_id = 'world'
		self.pub.publish(msg)
			return
		if not self.have_pose:
			return
		# Transform points to world and repack minimal XYZ fields; preserve stamp
		try:
			points = []
			fmt = None
			# Build struct format for reading XYZ float32
			offset_map = {f.name: f.offset for f in msg.fields}
			if not all(k in offset_map for k in ('x', 'y', 'z')):
				self.get_logger().warn('PointCloud2 missing x/y/z fields')
				return
			step = msg.point_step
			data = msg.data
			for i in range(0, len(data), step):
				x = struct.unpack_from('f', data, i + offset_map['x'])[0]
				y = struct.unpack_from('f', data, i + offset_map['y'])[0]
				z = struct.unpack_from('f', data, i + offset_map['z'])[0]
				px, py, pz = transform_point((x, y, z), self.R, self.t)
				points.append((px, py, pz))
			# Create new cloud
			out = PointCloud2()
			out.header.stamp = msg.header.stamp
			out.header.frame_id = 'world'
			out.height = 1
			out.width = len(points)
			out.is_bigendian = False
			out.is_dense = False
			out.fields = [
				PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
				PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
				PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
			]
			out.point_step = 12
			out.row_step = out.point_step * out.width
			buf = bytearray(out.row_step)
			for idx, (x, y, z) in enumerate(points):
				struct.pack_into('fff', buf, idx * 12, float(x), float(y), float(z))
			out.data = bytes(buf)
			self.pub.publish(out)
		except Exception as e:
			self.get_logger().warn(f"Cloud transform failed: {e}")


def main():
	parser = argparse.ArgumentParser(description='Relay PointCloud2 from one topic to another')
	parser.add_argument('--in', dest='in_topic', required=True, help='Input PointCloud2 topic')
	parser.add_argument('--out', dest='out_topic', required=True, help='Output PointCloud2 topic')
	parser.add_argument('--pose', dest='pose_topic', default='', help='Pose topic in world (PoseStamped) to transform cloud from sensor->world')
	parser.add_argument('--assume_world', dest='assume_world', action='store_true', help='Assume input cloud already in world frame; no transform')
	args = parser.parse_args()

	rclpy.init()
	node = CloudRelay(args.in_topic, args.out_topic, args.pose_topic if args.pose_topic else None, args.assume_world)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main() 