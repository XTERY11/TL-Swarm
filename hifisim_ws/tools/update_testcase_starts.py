#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def main():
	p = argparse.ArgumentParser(description='Update first waypoint (start) of agents in TestCase AgentGroup JSON')
	p.add_argument('--file', type=str, required=True, help='Path to AgentGroupX.json (e.g., AgentGroup2.json)')
	p.add_argument('--base_x', type=float, default=30.0, help='Base X for Agent1 start (east)')
	p.add_argument('--base_y', type=float, default=12.0, help='Base Y for Agent1 start (north)')
	p.add_argument('--dz', type=float, default=2.0, help='Set Z (altitude above ground)')
	p.add_argument('--spacing', type=float, default=1.5, help='Spacing between agents along Y')
	p.add_argument('--count', type=int, default=6, help='Number of agents to update')
	args = p.parse_args()

	path = Path(args.file)
	data = json.loads(path.read_text())

	agents = data.get('Agents', [])
	changed = 0
	for i, ag in enumerate(agents):
		if changed >= args.count:
			break
		wps = ag.get('Waypoints', [])
		if not wps:
			continue
		start = wps[0]
		start['X'] = float(args.base_x)
		start['Y'] = float(args.base_y + changed * args.spacing)
		start['Z'] = float(args.dz)  # Always set Z to safe height
		changed += 1

	path.write_text(json.dumps(data, indent=2))
	print(f'Updated {changed} agent start(s) in {path}')
	print(f'New starts: X={args.base_x}, Y={args.base_y}~{args.base_y + (args.count-1)*args.spacing}, Z={args.dz}')


if __name__ == '__main__':
	main() 