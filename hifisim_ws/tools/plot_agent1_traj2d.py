#!/usr/bin/env python3
import argparse
import csv
import math
import os
from typing import List, Tuple

import matplotlib
matplotlib.use('Agg')  # headless
import matplotlib.pyplot as plt


def load_csv(path: str) -> List[Tuple[float, float]]:
	xs: List[float] = []
	ys: List[float] = []
	with open(path, 'r') as f:
		reader = csv.DictReader(f)
		for row in reader:
			xs.append(float(row['x']))
			ys.append(float(row['y']))
	return list(zip(xs, ys))


def polyline_turn_metrics(pts: List[Tuple[float, float]]):
	"""Compute simple curvature metrics: max turn angle (deg), total length, straightness score.
	Straightness score = chord_length / path_length (1 means straight line).
	"""
	if len(pts) < 3:
		return 0.0, 0.0, 1.0
	# path length
	plen = 0.0
	for i in range(1, len(pts)):
		(dx, dy) = (pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
		plen += math.hypot(dx, dy)
	# chord length
	chord = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
	# max turn angle
	max_turn = 0.0
	for i in range(1, len(pts)-1):
		a = (pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
		b = (pts[i+1][0] - pts[i][0], pts[i+1][1] - pts[i][1])
		la = math.hypot(a[0], a[1])
		lb = math.hypot(b[0], b[1])
		if la < 1e-6 or lb < 1e-6:
			continue
		cosang = max(-1.0, min(1.0, (a[0]*b[0] + a[1]*b[1]) / (la*lb)))
		ang = math.degrees(math.acos(cosang))
		if ang > max_turn:
			max_turn = ang
	straightness = (chord / plen) if plen > 1e-6 else 1.0
	return max_turn, plen, straightness


def main():
	parser = argparse.ArgumentParser(description='Plot full 2D XY trajectory from recorded CSV')
	parser.add_argument('--csv', type=str, default='/tmp/agent001_traj.csv', help='Input CSV path recorded by record_agent1_traj.py')
	parser.add_argument('--out', type=str, default='/tmp/agent001_traj_xy.png', help='Output PNG path')
	args = parser.parse_args()

	pts = load_csv(args.csv)
	if len(pts) < 2:
		raise SystemExit('Not enough points in CSV. Did the recorder run long enough?')
	max_turn, plen, straightness = polyline_turn_metrics(pts)

	xs = [p[0] for p in pts]
	ys = [p[1] for p in pts]

	plt.figure(figsize=(7, 6))
	plt.plot(xs, ys, '-', lw=2, color='tab:blue', label='XY path')
	plt.scatter([xs[0]], [ys[0]], c='green', s=60, label='start')
	plt.scatter([xs[-1]], [ys[-1]], c='red', s=60, label='end')
	plt.axis('equal')
	plt.grid(True, ls='--', alpha=0.4)
	plt.xlabel('X [m]')
	plt.ylabel('Y [m]')
	plt.title(f'Agent001 XY path\nmax_turn={max_turn:.1f} deg, length={plen:.1f} m, straightness={straightness:.3f}')
	plt.legend(loc='best')
	plt.tight_layout()
	os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
	plt.savefig(args.out, dpi=150)
	print(f'Saved 2D path image to {args.out}')


if __name__ == '__main__':
	main() 