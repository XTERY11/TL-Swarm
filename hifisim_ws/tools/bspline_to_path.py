#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from traj_utils.msg import Bspline
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import argparse
from typing import List, Tuple


def clamp(v, lo, hi):
	return max(lo, min(hi, v))


def de_boor_point(order: int, knots: List[float], ctrl: List[Tuple[float, float, float]], u: float) -> Tuple[float, float, float]:
	p = int(order)
	n = len(ctrl) - 1
	m = len(knots) - 1
	umin = knots[p]
	umax = knots[m - p - 1]
	u = clamp(u, umin, umax)
	k = p
	for i in range(p, m - p):
		if u >= knots[i] and u < knots[i + 1]:
			k = i
			break
	else:
		k = m - p - 1
	dx = [0.0] * (p + 1)
	dy = [0.0] * (p + 1)
	dz = [0.0] * (p + 1)
	for j in range(0, p + 1):
		idx = k - p + j
		idx = int(clamp(idx, 0, n))
		dx[j], dy[j], dz[j] = ctrl[idx]
	for r in range(1, p + 1):
		for j in range(p, r - 1, -1):
			i = k - p + j
			den = knots[i + p - r + 1] - knots[i]
			alpha = 0.0 if den == 0.0 else (u - knots[i]) / den
			dx[j] = (1.0 - alpha) * dx[j - 1] + alpha * dx[j]
			dy[j] = (1.0 - alpha) * dy[j - 1] + alpha * dy[j]
			dz[j] = (1.0 - alpha) * dz[j - 1] + alpha * dz[j]
	return dx[p], dy[p], dz[p]


class BsplineToPath(Node):
	def __init__(self, drone_id: int, in_topic: str, out_topic: str, samples: int):
		super().__init__('bspline_to_path')
		self.samples = int(samples)
		self.in_topic = in_topic or f"/drone_{drone_id}_planning/bspline"
		self.out_topic = out_topic or f"/drone_{drone_id}_planning/path"
		qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
		self.sub = self.create_subscription(Bspline, self.in_topic, self.cb, qos)
		self.pub = self.create_publisher(Path, self.out_topic, qos)
		self.get_logger().info(f"Converting Bspline {self.in_topic} → Path {self.out_topic} (samples={self.samples})")

	def cb(self, msg: Bspline):
		ctrl = [(p.x, p.y, p.z) for p in msg.pos_pts]
		order = int(msg.order)
		knots = list(msg.knots)
		poses = []
		if not ctrl:
			return
		if len(knots) < (len(ctrl) + order + 1):
			samples = ctrl
		else:
			umin = knots[order]
			umax = knots[len(knots) - order - 1]
			N = max(20, min(1000, self.samples))
			samples = []
			for i in range(N):
				t = umin + (umax - umin) * (i / max(1, N - 1))
				x, y, z = de_boor_point(order, knots, ctrl, t)
				samples.append((x, y, z))
		path = Path()
		path.header.stamp = self.get_clock().now().to_msg()
		path.header.frame_id = 'world'
		for (x, y, z) in samples:
			ps = PoseStamped()
			ps.header = path.header
			ps.pose.position.x = float(x)
			ps.pose.position.y = float(y)
			ps.pose.position.z = float(z)
			ps.pose.orientation.w = 1.0
			poses.append(ps)
		path.poses = poses
		self.pub.publish(path)


def main():
	parser = argparse.ArgumentParser(description='Convert traj_utils/Bspline to nav_msgs/Path for RViz display')
	parser.add_argument('--drone_id', type=int, default=0)
	parser.add_argument('--in_topic', type=str, default=None)
	parser.add_argument('--out_topic', type=str, default=None)
	parser.add_argument('--samples', type=int, default=400)
	args = parser.parse_args()
	
	rclpy.init()
	node = BsplineToPath(args.drone_id, args.in_topic, args.out_topic, args.samples)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main() 