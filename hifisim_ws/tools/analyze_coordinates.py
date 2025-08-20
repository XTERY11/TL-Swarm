#!/usr/bin/env python3
import json
import math
import sys
from typing import List, Tuple


def unity_env_to_world(x_env: float, y_env: float, z_env: float) -> Tuple[float, float, float]:
	"""Convert Env JSON coords (x, y, z) to planner/world (x, y, z).
	Env1Scenario1.json uses EUN_RUF: x=east, y=up(height), z=north
	Planner world: x=east, y=north, z=up
	So: world_x = env.x, world_y = env.z, world_z = env.y
	"""
	return float(x_env), float(z_env), float(y_env)


def distance_3d(p1: Tuple[float, float, float], p2: Tuple[float, float, float]) -> float:
	"""Calculate 3D Euclidean distance between two points."""
	return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)


def main():
	if len(sys.argv) != 4:
		print("Usage: python3 analyze_coordinates.py <drone_x> <drone_y> <drone_z>")
		print("Example: python3 analyze_coordinates.py 30.0 12.0 2.0")
		sys.exit(1)
	
	drone_x = float(sys.argv[1])
	drone_y = float(sys.argv[2]) 
	drone_z = float(sys.argv[3])
	drone_pos = (drone_x, drone_y, drone_z)
	
	print(f"Drone position (planner world): X={drone_x:.2f}, Y={drone_y:.2f}, Z={drone_z:.2f}")
	print()
	
	# Load signposts from Env1Scenario1.json
	json_path = "/home/ctx/Unity/build/hifi_simulator_unity_Data/StreamingAssets/AppData/EnvScenarios/Env1Scenario1.json"
	with open(json_path, 'r') as f:
		data = json.load(f)
	
	signposts = data.get('Signposts', [])
	distances = []
	
	print("Signpost coordinates analysis:")
	print("=" * 80)
	print(f"{'Name':<15} {'Env(EUN_RUF)':<25} {'Planner World':<25} {'Distance':<10}")
	print("-" * 80)
	
	for i, sp in enumerate(signposts):
		name = sp.get('ObjName', f'Signpost_{i+1}')
		pos = sp['TransformDatas'][0]['Pos']
		env_x, env_y, env_z = pos['x'], pos['y'], pos['z']
		
		# Convert to planner world coordinates
		world_x, world_y, world_z = unity_env_to_world(env_x, env_y, env_z)
		world_pos = (world_x, world_y, world_z)
		
		# Calculate distance
		dist = distance_3d(drone_pos, world_pos)
		distances.append((name, dist, world_pos, (env_x, env_y, env_z)))
		
		print(f"{name:<15} ({env_x:6.1f},{env_y:4.1f},{env_z:6.1f})  ({world_x:6.1f},{world_y:6.1f},{world_z:4.1f})  {dist:8.2f}m")
	
	# Sort by distance and show nearest
	distances.sort(key=lambda x: x[1])
	
	print()
	print("Nearest signposts to drone:")
	print("=" * 40)
	for i, (name, dist, world_pos, env_pos) in enumerate(distances[:3]):
		print(f"{i+1}. {name}: {dist:.2f}m at planner({world_pos[0]:.1f}, {world_pos[1]:.1f}, {world_pos[2]:.1f})")
	
	# Check if coordinates make sense
	nearest = distances[0]
	if nearest[1] > 50.0:  # If nearest is more than 50m away
		print()
		print("⚠️  WARNING: Nearest signpost is very far away!")
		print("   This suggests coordinate system mismatch.")
		print("   Possible issues:")
		print("   - Wrong coordinate mapping")
		print("   - X/Y/Z axis confusion")
		print("   - Different coordinate systems")
	
	print()
	print("Coordinate mapping used:")
	print(f"  Env1Scenario1.json (EUN_RUF): x=east, y=up, z=north")
	print(f"  Planner world: x=east, y=north, z=up")
	print(f"  Conversion: world_x=env.x, world_y=env.z, world_z=env.y")


if __name__ == '__main__':
	main() 