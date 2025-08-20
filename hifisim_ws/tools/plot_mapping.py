#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from traj_utils.msg import Bspline
from sensor_msgs_py import point_cloud2 as pc2
import matplotlib.pyplot as plt
import threading

class MapPlotter(Node):
	def __init__(self):
		super().__init__('map_plotter')
		qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
		self.sub_occ = self.create_subscription(PointCloud2, '/drone_0_grid/grid_map/occupancy_inflate', self.cb_occ, qos)
		self.sub_bs = self.create_subscription(Bspline, '/drone_0_planning/bspline', self.cb_bs, qos)
		self.occ_xy = []
		self.bs_xy = []
		self.lock = threading.Lock()
		self.timer = self.create_timer(0.5, self.refresh)
		plt.ion()
		self.fig, self.ax = plt.subplots(figsize=(6,6))
		self.ax.set_title('Occupancy (inflated) and planned B-spline (XY)')
		self.ax.set_xlabel('X [m]')
		self.ax.set_ylabel('Y [m]')
		self.ax.axis('equal')

	def cb_occ(self, msg: PointCloud2):
		pts = []
		for p in pc2.read_points(msg, field_names=('x','y'), skip_nans=True):
			pts.append((float(p[0]), float(p[1])))
		with self.lock:
			self.occ_xy = pts

	def cb_bs(self, msg: Bspline):
		pts = [(float(p.x), float(p.y)) for p in msg.pos_pts]
		with self.lock:
			self.bs_xy = pts

	def refresh(self):
		with self.lock:
			occ = list(self.occ_xy)
			bs = list(self.bs_xy)
		self.ax.cla()
		self.ax.set_title('Occupancy (inflated) and planned B-spline (XY)')
		self.ax.set_xlabel('X [m]')
		self.ax.set_ylabel('Y [m]')
		self.ax.axis('equal')
		if occ:
			xo, yo = zip(*occ)
			self.ax.scatter(xo, yo, s=1, c='gray', alpha=0.5, label='occupancy')
		if bs:
			xb, yb = zip(*bs)
			self.ax.plot(xb, yb, 'r-', linewidth=2, label='bspline')
		self.ax.legend(loc='upper right')
		self.fig.canvas.draw()
		self.fig.canvas.flush_events()


def main():
	rclpy.init()
	node = MapPlotter()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main() 