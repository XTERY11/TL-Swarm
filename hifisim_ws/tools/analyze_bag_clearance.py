#!/usr/bin/env python3
import argparse
import os
import math
from typing import List, Tuple, Dict, Any

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from sensor_msgs_py import point_cloud2 as pc2

try:
	from scipy.spatial import cKDTree as KDTree  # Fast if available
	_HAS_KDTREE = True
except Exception:
	KDTree = None
	_HAS_KDTREE = False


def load_messages_with_ts(bag_path: str, topics: List[str]) -> Dict[str, List[Tuple[Any, int]]]:
	"""Return {topic: [(msg, timestamp_ns), ...]} preserving order."""
	storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
	converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
	reader = SequentialReader()
	reader.open(storage_options, converter_options)
	all_topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
	selected = {name: all_topics[name] for name in topics if name in all_topics}
	if not selected:
		raise RuntimeError(f"None of requested topics found. Available: {list(all_topics.keys())}")
	type_map = {name: get_message(typ) for name, typ in selected.items()}
	msgs: Dict[str, List[Tuple[Any, int]]] = {name: [] for name in topics}
	while reader.has_next():
		topic, data, t = reader.read_next()
		if topic not in selected:
			continue
		msg = deserialize_message(data, type_map[topic])
		msgs[topic].append((msg, int(t)))
	return msgs


def load_messages(bag_path: str, topics: List[str]):
	storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
	converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
	reader = SequentialReader()
	reader.open(storage_options, converter_options)
	all_topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
	selected = {name: all_topics[name] for name in topics if name in all_topics}
	if not selected:
		raise RuntimeError(f"None of requested topics found. Available: {list(all_topics.keys())}")
	# Prepare type map
	type_map = {name: get_message(typ) for name, typ in selected.items()}
	# Read sequentially and collect
	msgs = {name: [] for name in topics}
	while reader.has_next():
		topic, data, t = reader.read_next()
		if topic not in selected:
			continue
		msg = deserialize_message(data, type_map[topic])
		msgs[topic].append(msg)
	return msgs


def _clamp(v, lo, hi):
	return lo if v < lo else hi if v > hi else v


# ---- B-spline sampling ----

def _de_boor_point2(order: int, knots: List[float], ctrl: np.ndarray, u: float) -> Tuple[float, float]:
	# ctrl: (n,3) but we only need x,y here
	p = int(order)
	n = ctrl.shape[0] - 1
	m = len(knots) - 1
	umin = knots[p]
	umax = knots[m - p - 1]
	u = _clamp(u, umin, umax)
	# find span k
	k = p
	for i in range(p, m - p):
		if u >= knots[i] and u < knots[i + 1]:
			k = i
			break
	else:
		k = m - p - 1
	# de Boor
	dx = np.zeros(p + 1, dtype=np.float64)
	dy = np.zeros(p + 1, dtype=np.float64)
	for j in range(0, p + 1):
		idx = int(np.clip(k - p + j, 0, n))
		dx[j] = ctrl[idx, 0]
		dy[j] = ctrl[idx, 1]
	for r in range(1, p + 1):
		for j in range(p, r - 1, -1):
			i = k - p + j
			den = knots[i + p - r + 1] - knots[i]
			alpha = 0.0 if den == 0.0 else (u - knots[i]) / den
			dx[j] = (1.0 - alpha) * dx[j - 1] + alpha * dx[j]
			dy[j] = (1.0 - alpha) * dy[j - 1] + alpha * dy[j]
	return float(dx[p]), float(dy[p])


def _de_boor_point3(order: int, knots: List[float], ctrl: np.ndarray, u: float) -> Tuple[float, float, float]:
	# ctrl: (n,3), return x,y,z
	p = int(order)
	n = ctrl.shape[0] - 1
	m = len(knots) - 1
	umin = knots[p]
	umax = knots[m - p - 1]
	u = _clamp(u, umin, umax)
	# find span k
	k = p
	for i in range(p, m - p):
		if u >= knots[i] and u < knots[i + 1]:
			k = i
			break
	else:
		k = m - p - 1
	# de Boor
	dx = np.zeros(p + 1, dtype=np.float64)
	dy = np.zeros(p + 1, dtype=np.float64)
	dz = np.zeros(p + 1, dtype=np.float64)
	for j in range(0, p + 1):
		idx = int(np.clip(k - p + j, 0, n))
		dx[j] = ctrl[idx, 0]
		dy[j] = ctrl[idx, 1]
		dz[j] = ctrl[idx, 2]
	for r in range(1, p + 1):
		for j in range(p, r - 1, -1):
			i = k - p + j
			den = knots[i + p - r + 1] - knots[i]
			alpha = 0.0 if den == 0.0 else (u - knots[i]) / den
			dx[j] = (1.0 - alpha) * dx[j - 1] + alpha * dx[j]
			dy[j] = (1.0 - alpha) * dy[j - 1] + alpha * dy[j]
			dz[j] = (1.0 - alpha) * dz[j - 1] + alpha * dz[j]
	return float(dx[p]), float(dy[p]), float(dz[p])


def bspline_points_from_msgs(bs_msgs, last_n: int, stride: int, max_pts: int, samples_per_msg: int = 200, use_xyz: bool = False) -> np.ndarray:
	# bs_msgs is a list of messages (no timestamps)
	msgs = bs_msgs[-last_n:] if last_n > 0 else bs_msgs
	pts_out = []
	for m in msgs:
		ctrl = np.array([[p.x, p.y, p.z] for p in m.pos_pts], dtype=np.float64)
		if ctrl.shape[0] < 2:
			continue
		order = int(getattr(m, 'order', 3))
		knots = list(getattr(m, 'knots', []))
		# If knots invalid, fallback to control points polyline
		if len(knots) < (ctrl.shape[0] + order + 1):
			pts = ctrl[:, :3] if use_xyz else ctrl[:, :2]
		else:
			umin = knots[order]
			umax = knots[len(knots) - order - 1]
			N = max(20, min(800, samples_per_msg))
			pts = []
			for i in range(N):
				t = umin + (umax - umin) * (i / max(1, N - 1))
				if use_xyz:
					x, y, z = _de_boor_point3(order, knots, ctrl, t)
					pts.append((x, y, z))
				else:
					x, y = _de_boor_point2(order, knots, ctrl, t)
					pts.append((x, y))
			pts = np.array(pts, dtype=np.float32)
		pts_out.append(pts)
	if not pts_out:
		return np.zeros((0, 3 if use_xyz else 2), dtype=np.float32)
	arr = np.vstack(pts_out)
	arr = arr[::max(1, stride)]
	if max_pts > 0 and arr.shape[0] > max_pts:
		idx = np.linspace(0, arr.shape[0]-1, max_pts).astype(int)
		arr = arr[idx]
	return arr


def bspline_points_from_msgs_with_ts(bs_msgs_ts: List[Tuple[Any, int]], last_n: int, stride: int, max_pts: int, samples_per_msg: int = 200, use_xyz: bool = False) -> Tuple[np.ndarray, int]:
	"""Sample points from only the last bspline message and return (points, ts_ns)."""
	if not bs_msgs_ts:
		return np.zeros((0, 3 if use_xyz else 2), dtype=np.float32), -1
	# pick the last element or last_n-th from the end if provided
	chosen = bs_msgs_ts[-1]
	m, ts_ns = chosen
	pts = bspline_points_from_msgs([m], 1, stride, max_pts, samples_per_msg, use_xyz)
	return pts, ts_ns


# ---- Occupancy sampling ----

def occupancy_points_from_pc2_msgs(pc_msgs, last_n: int, stride: int, max_pts: int, use_xyz: bool = False) -> np.ndarray:
	msgs = pc_msgs[-last_n:] if last_n > 0 else pc_msgs
	pts = []
	for m in msgs:
		k = 0
		if use_xyz:
			fields = ('x','y','z')
		else:
			fields = ('x','y')
		for p in pc2.read_points(m, field_names=fields, skip_nans=True):
			k += 1
			if stride > 1 and (k % stride) != 0:
				continue
			if use_xyz:
				pts.append((float(p[0]), float(p[1]), float(p[2])))
			else:
				pts.append((float(p[0]), float(p[1])))
	if not pts:
		return np.zeros((0, 3 if use_xyz else 2), dtype=np.float32)
	arr = np.array(pts, dtype=np.float32)
	if max_pts > 0 and arr.shape[0] > max_pts:
		idx = np.linspace(0, arr.shape[0]-1, max_pts).astype(int)
		arr = arr[idx]
	return arr


def occupancy_points_from_pc2_msgs_with_time_window(pc_msgs_ts: List[Tuple[Any, int]], center_ts_ns: int, window_s: float, stride: int, max_pts: int, use_xyz: bool = False) -> np.ndarray:
	if not pc_msgs_ts or center_ts_ns < 0:
		return np.zeros((0, 3 if use_xyz else 2), dtype=np.float32)
	half_ns = int(max(0.0, window_s) * 0.5 * 1e9)
	lo = center_ts_ns - half_ns
	hi = center_ts_ns + half_ns
	selected_msgs = [m for (m, t) in pc_msgs_ts if (t >= lo and t <= hi)]
	if not selected_msgs:
		# fallback: use the closest one by timestamp
		closest = min(pc_msgs_ts, key=lambda mt: abs(mt[1] - center_ts_ns))[0]
		selected_msgs = [closest]
	return occupancy_points_from_pc2_msgs(selected_msgs, last_n=len(selected_msgs), stride=stride, max_pts=max_pts, use_xyz=use_xyz)


def min_clearance(bs_pts: np.ndarray, occ_pts: np.ndarray) -> float:
	if bs_pts.size == 0 or occ_pts.size == 0:
		return float('nan')
	if _HAS_KDTREE and occ_pts.shape[0] >= 10:
		tree = KDTree(occ_pts)
		dist, _ = tree.query(bs_pts, k=1)
		return float(np.min(dist))
	# Fallback: chunked brute-force
	min_val = float('inf')
	chunk = 50000
	for start in range(0, occ_pts.shape[0], chunk):
		sl = occ_pts[start:start+chunk]
		d = np.sqrt(np.min(np.sum((sl[:,None,:]-bs_pts[None,:,:])**2, axis=2), axis=0))
		min_val = min(min_val, float(np.min(d)))
	return min_val


def plot_save(bs_xy: np.ndarray, occ_xy: np.ndarray, out_png: str, title: str, clearance_m: float, used_xyz: bool):
	plt.figure(figsize=(6,5))
	if occ_xy.size:
		plt.scatter(occ_xy[:,0], occ_xy[:,1], s=2, c='0.5', label='occupancy')
	if bs_xy.size:
		plt.plot(bs_xy[:,0], bs_xy[:,1], 'r-', linewidth=2, label='bspline')
	plt.xlabel('X [m]'); plt.ylabel('Y [m]')
	mode = '3D' if used_xyz else '2D'
	plt.title(f"{title} ({mode})\nmin_clearance={clearance_m:.2f} m")
	plt.legend()
	plt.grid(True, alpha=0.2)
	plt.tight_layout()
	plt.savefig(out_png, dpi=150)
	print(f"Saved plot: {out_png}")


def main():
	parser = argparse.ArgumentParser(description='Analyze bspline vs occupancy clearance from rosbag2')
	parser.add_argument('--bag', required=True, help='Path to rosbag2 directory (contains metadata.yaml)')
	parser.add_argument('--png', required=True, help='Output PNG file')
	parser.add_argument('--bspline', default='/drone_0_planning/bspline')
	parser.add_argument('--occupancy', default='/drone_0_grid/grid_map/occupancy_inflate')
	# Performance controls
	parser.add_argument('--bs_last', type=int, default=1, help='Use last N bspline messages (default: 1)')
	parser.add_argument('--occ_last', type=int, default=3, help='Use last N occupancy messages (default: 3)')
	parser.add_argument('--bs_stride', type=int, default=1, help='Stride bspline samples (default: 1)')
	parser.add_argument('--occ_stride', type=int, default=10, help='Stride occupancy points (default: 10)')
	parser.add_argument('--max_bs', type=int, default=500, help='Max bspline samples after subsample (default: 500)')
	parser.add_argument('--max_occ', type=int, default=300000, help='Max occupancy points after subsample (default: 300k)')
	parser.add_argument('--samples_per_msg', type=int, default=200, help='De Boor samples per bspline message (default: 200)')
	# New options
	parser.add_argument('--use_xyz', action='store_true', help='Compute clearance in 3D (default: 2D XY)')
	parser.add_argument('--z_margin', type=float, default=0.0, help='If >0 and use_xyz, filter occupancy to bspline z-range +/- z_margin [m]')
	parser.add_argument('--time_align', action='store_true', help='Align occupancy to the last bspline by time window')
	parser.add_argument('--time_window', type=float, default=0.6, help='Time window [s] centered at last bspline for occupancy selection (requires --time_align)')
	args = parser.parse_args()

	msgs_ts = load_messages_with_ts(args.bag, [args.bspline, args.occupancy])

	# Debug: print frame ids if present
	bs_frame = None
	occ_frame = None
	for (m, _) in msgs_ts.get(args.bspline, [])[-1:]:
		bs_frame = getattr(getattr(m, 'header', None), 'frame_id', None)
	for (m, _) in msgs_ts.get(args.occupancy, [])[-1:]:
		occ_frame = getattr(getattr(m, 'header', None), 'frame_id', None)
	if bs_frame or occ_frame:
		print(f"frames: bspline='{bs_frame}', occupancy='{occ_frame}'")

	if args.time_align:
		bs_pts, ts_ns = bspline_points_from_msgs_with_ts(
			msgs_ts.get(args.bspline, []), args.bs_last, args.bs_stride, args.max_bs, args.samples_per_msg, use_xyz=args.use_xyz
		)
		occ_pts = occupancy_points_from_pc2_msgs_with_time_window(
			msgs_ts.get(args.occupancy, []), ts_ns, args.time_window, args.occ_stride, args.max_occ, use_xyz=args.use_xyz
		)
	else:
		# Legacy: mix last-N without time alignment
		bs_msgs_only = [m for (m, _) in msgs_ts.get(args.bspline, [])]
		occ_msgs_only = [m for (m, _) in msgs_ts.get(args.occupancy, [])]
		bs_pts = bspline_points_from_msgs(
			bs_msgs_only, args.bs_last, args.bs_stride, args.max_bs, args.samples_per_msg, use_xyz=args.use_xyz
		)
		occ_pts = occupancy_points_from_pc2_msgs(
			occ_msgs_only, args.occ_last, args.occ_stride, args.max_occ, use_xyz=args.use_xyz
		)

	# Optional z filtering to reduce ground/ceiling interference in 3D mode
	if args.use_xyz and args.z_margin > 0.0 and bs_pts.size and occ_pts.size and bs_pts.shape[1] == 3 and occ_pts.shape[1] == 3:
		bs_z_min = float(np.min(bs_pts[:,2]))
		bs_z_max = float(np.max(bs_pts[:,2]))
		z_lo = bs_z_min - args.z_margin
		z_hi = bs_z_max + args.z_margin
		mask = (occ_pts[:,2] >= z_lo) & (occ_pts[:,2] <= z_hi)
		occ_pts = occ_pts[mask]
		print(f"Z filter applied: bspline z∈[{bs_z_min:.2f},{bs_z_max:.2f}], keep occupancy z∈[{z_lo:.2f},{z_hi:.2f}] => {len(occ_pts)} pts")

	clr = min_clearance(bs_pts, occ_pts)
	print(f"bspline points: {len(bs_pts)}, occupancy points: {len(occ_pts)}, min_clearance(m): {clr}")

	# For plotting, project to XY if necessary
	bs_xy = bs_pts[:, :2] if bs_pts.size else np.zeros((0,2), dtype=np.float32)
	occ_xy = occ_pts[:, :2] if occ_pts.size else np.zeros((0,2), dtype=np.float32)
	plot_save(bs_xy, occ_xy, args.png, os.path.basename(args.bag), clr, used_xyz=args.use_xyz)

if __name__ == '__main__':
	main() 