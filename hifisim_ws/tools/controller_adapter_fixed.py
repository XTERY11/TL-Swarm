#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from traj_utils.msg import Bspline
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import argparse
from typing import List, Tuple


def clamp(v, lo, hi):
	return max(lo, min(hi, v))


def de_boor_point(order: int, knots: List[float], ctrl: List[Tuple[float, float, float]], u: float) -> Tuple[float, float, float]:
	"""Evaluate B-spline at parameter u using De Boor's algorithm.
	order: spline order p (degree = p)
	knots: knot vector U of length m+1
	ctrl: control points P0..Pn (n+1 points)
	u: parameter value in [knots[p], knots[m-p-1]]
	"""
	p = int(order)
	n = len(ctrl) - 1
	m = len(knots) - 1
	# Clamp u into valid domain
	umin = knots[p]
	umax = knots[m - p - 1]
	u = clamp(u, umin, umax)
	# Find span k such that U[k] <= u < U[k+1]
	k = p
	for i in range(p, m - p):
		if u >= knots[i] and u < knots[i + 1]:
			k = i
			break
	else:
		k = m - p - 1  # u == umax
	# Initialize d[j] = P_{k-p+j}
	dx = [0.0] * (p + 1)
	dy = [0.0] * (p + 1)
	dz = [0.0] * (p + 1)
	for j in range(0, p + 1):
		idx = k - p + j
		idx = clamp(idx, 0, n)
		dx[j], dy[j], dz[j] = ctrl[idx]
	# De Boor recursion
	for r in range(1, p + 1):
		for j in range(p, r - 1, -1):
			i = k - p + j
			den = knots[i + p - r + 1] - knots[i]
			alpha = 0.0 if den == 0.0 else (u - knots[i]) / den
			dx[j] = (1.0 - alpha) * dx[j - 1] + alpha * dx[j]
			dy[j] = (1.0 - alpha) * dy[j - 1] + alpha * dy[j]
			dz[j] = (1.0 - alpha) * dz[j - 1] + alpha * dz[j]
	return dx[p], dy[p], dz[p]


class ControllerAdapter(Node):
	def __init__(self, agent_id: int, sample_hz: float, horizon_s: float, mission_code: int, z_mode: str, z_min: float, z_max: float, drive_mode: str, pose_topic: str):
		super().__init__('controller_adapter')
		self.agent_id = agent_id
		self.sample_dt = 1.0/float(sample_hz)
		self.horizon_s = float(horizon_s)
		self.mission_code = int(mission_code)
		self.z_mode = str(z_mode)
		self.z_min = float(z_min)
		self.z_max = float(z_max)
		self.drive_mode = str(drive_mode)
		self.pose_topic = str(pose_topic)
		self.last_bspline: Bspline | None = None
		self.have_new_bspline: bool = False
		self.last_published_stamp_sec: int | None = None
		self.last_published_stamp_nsec: int | None = None
		self.current_samples: list[tuple[float,float,float]] = []
		self.stream_idx: int = 0

		qos_sub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
		qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)

		self.sub_bspline = self.create_subscription(Bspline, f"/drone_{agent_id-1}_planning/bspline", self.cb_bspline, qos_sub)
		self.sub_hold = self.create_subscription(PoseStamped, f"/agent{self.agent_id:03d}/ctrl_target", self.cb_hold, qos_sub)

		self.pub_traj = self.create_publisher(JointTrajectory, f"/agent{self.agent_id:03d}/trajectory/points", qos_pub)
		self.pub_pose = self.create_publisher(PoseStamped, self.pose_topic, qos_pub)

		self.timer = self.create_timer(self.sample_dt, self.tick)
		self.get_logger().info(
			f"ControllerAdapter up: listen bspline:/drone_{agent_id-1}_planning/bspline, "
			f"target:/agent{agent_id:03d}/ctrl_target → publish /agent{agent_id:03d}/trajectory/points "
			f"(mission_code={self.mission_code}, z_mode={self.z_mode}, z_min={self.z_min}, z_max={self.z_max}, "
			f"drive_mode={self.drive_mode}, pose_topic={self.pose_topic})"
		)

	def cb_bspline(self, msg: Bspline):
		self.last_bspline = msg
		try:
			sec = int(msg.header.stamp.sec)
			nsec = int(msg.header.stamp.nanosec)
		except Exception:
			sec = -1
			nsec = -1
		if self.last_published_stamp_sec != sec or self.last_published_stamp_nsec != nsec:
			self.have_new_bspline = True

	def cb_hold(self, msg: PoseStamped):
		pass

	def tick(self):
		# If a new bspline arrived, resample and reset stream index
		if self.have_new_bspline and self.last_bspline is not None:
			bs = self.last_bspline
			ctrl = [(p.x, p.y, p.z) for p in bs.pos_pts]
			order = int(bs.order)
			knots = list(bs.knots)
			if not ctrl:
				self.current_samples = []
				self.have_new_bspline = False
				return
			if len(knots) < (len(ctrl) + order + 1):
				samples = ctrl
			else:
				umin = knots[order]
				umax = knots[len(knots) - order - 1]
				N = max(10, min(200, int(self.horizon_s / self.sample_dt)))
				samples = []
				for i in range(N):
					t = umin + (umax - umin) * (i / max(1, N - 1))
					x, y, z = de_boor_point(order, knots, ctrl, t)
					samples.append((x, y, z))
			# debug z-range
			try:
				zs = [p[2] for p in samples]
				if zs:
					zmin, zmax = min(zs), max(zs)
					self.get_logger().info(f"Bspline z-range before filter: [{zmin:.2f}, {zmax:.2f}]")
			except Exception:
				pass
			self.current_samples = samples
			self.stream_idx = 0
			try:
				self.last_published_stamp_sec = int(bs.header.stamp.sec)
				self.last_published_stamp_nsec = int(bs.header.stamp.nanosec)
			except Exception:
				self.last_published_stamp_sec = None
				self.last_published_stamp_nsec = None
			self.have_new_bspline = False
		# Stream one point per tick if available
		if self.stream_idx < len(self.current_samples):
			x, y, z = self.current_samples[self.stream_idx]
			if self.z_mode == 'clamp':
				z = clamp(z, self.z_min, self.z_max)
			if self.drive_mode == 'jt':
				out = JointTrajectory()
				out.header.stamp = self.get_clock().now().to_msg()
				out.header.frame_id = ''
				out.joint_names = [f'agent{self.agent_id:03d}']
				jtp = JointTrajectoryPoint()
				jtp.positions = [float(x), float(y), float(z)]
				jtp.time_from_start.sec = int(self.mission_code)
				jtp.time_from_start.nanosec = 0
				out.points.append(jtp)
				self.pub_traj.publish(out)
			else:
				pose = PoseStamped()
				pose.header.stamp = self.get_clock().now().to_msg()
				pose.header.frame_id = 'world'
				pose.pose.position.x = float(x)
				pose.pose.position.y = float(y)
				pose.pose.position.z = float(z)
				pose.pose.orientation.w = 1.0
				self.pub_pose.publish(pose)
			self.stream_idx += 1


def main():
	parser = argparse.ArgumentParser(description='Controller adapter: Bspline → Unity JointTrajectory/Pose')
	parser.add_argument('--agent_id', type=int, required=True)
	parser.add_argument('--hz', type=float, default=10.0)
	parser.add_argument('--horizon', type=float, default=2.0)
	parser.add_argument('--mission_code', type=int, default=3)
	parser.add_argument('--z_mode', type=str, default='clamp', choices=['clamp', 'raw'], help='How to handle z: clamp to [z_min,z_max] or pass-through')
	parser.add_argument('--z_min', type=float, default=1.5, help='Minimum allowed z when z_mode=clamp')
	parser.add_argument('--z_max', type=float, default=50.0, help='Maximum allowed z when z_mode=clamp')
	parser.add_argument('--drive_mode', type=str, default='jt', choices=['jt','pose'], help='Publish JT (to pbtm) or PoseStamped (direct drive)')
	parser.add_argument('--pose_topic', type=str, default=None, help='PoseStamped target topic when drive_mode=pose (defaults to /agentNNN/global/nwu_pose)')
	args = parser.parse_args()

	rclpy.init()
	pose_topic = args.pose_topic or f"/agent{args.agent_id:03d}/global/nwu_pose"
	node = ControllerAdapter(args.agent_id, args.hz, args.horizon, args.mission_code, args.z_mode, args.z_min, args.z_max, args.drive_mode, pose_topic)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main() 