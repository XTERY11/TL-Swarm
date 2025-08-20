#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
import argparse
import math
import time

# import signpost utilities (ROS env is already sourced in this shell)
from hifisim_task_ros2.read_testcase import (
    read_testcase_struct,
    get_signposts_endpoint_nwu_pos_rot_env,
)


def compute_pre_pass_post(position, rotation_deg_yaw, pre_dist: float, post_dist: float, safe_z: float):
    # rotation.z in file is degrees; normal is facing direction
    yaw_rad = math.radians(rotation_deg_yaw)
    dir_x = math.cos(yaw_rad)
    dir_y = math.sin(yaw_rad)

    pre = (position.x - dir_x * pre_dist, position.y - dir_y * pre_dist, safe_z)
    pas = (position.x, position.y, safe_z)
    post = (position.x + dir_x * post_dist, position.y + dir_y * post_dist, safe_z)
    return [pre, pas, post]


class WaypointFeeder(Node):
    def __init__(self, drone_id: int, testcase_id: int, safe_z: float, pre_dist: float, post_dist: float, interval_s: float):
        super().__init__('waypoint_feeder')
        self.drone_id = drone_id
        self.safe_z = safe_z
        self.pre_dist = pre_dist
        self.post_dist = post_dist
        self.interval_s = interval_s

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(PoseStamped, '/traj_start_trigger', qos)

        # Load testcase → env/scenario → signposts
        tc = read_testcase_struct(testcase_id)
        env_id = tc['Testcase']['EnvId']
        scenario_id = tc['Testcase']['EnvScenarioId']
        sp = get_signposts_endpoint_nwu_pos_rot_env(env_id, scenario_id)
        if not sp:
            raise RuntimeError('No signposts found in testcase')
        # exclude last endpoint for routing core; we will append endpoint as final
        core = sp[:-1]
        endpoint = sp[-1]

        # Build waypoint list [pre, pass, post] for each signpost, then endpoint pass only
        waypoints = []
        for i, data in enumerate(core):
            pos = data['position']  # geometry_msgs/Point
            rot = data['rotation']  # rotation.z stores yaw(deg) with sign from conversion; we need facing yaw(deg)
            # In read_testcase.py, rotation.z = -rot_y (deg). Facing yaw(deg) = -rotation.z
            yaw_deg = -rot.z
            waypoints.extend(compute_pre_pass_post(pos, yaw_deg, self.pre_dist, self.post_dist, self.safe_z))
        # add endpoint as final target at safe_z
        end_pos = endpoint['position']
        waypoints.append((end_pos.x, end_pos.y, self.safe_z))

        self.waypoints = waypoints
        self.wp_idx = 0

        self.get_logger().info(f"Drone {self.drone_id}: loaded {len(self.waypoints)} targets from {env_id} scenario {scenario_id}")
        self.timer = self.create_timer(self.interval_s, self.publish_next)

    def publish_next(self):
        if self.wp_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints sent. Stopping timer.')
            self.timer.cancel()
            return
        x, y, z = self.waypoints[self.wp_idx]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.get_logger().info(f"Sent target {self.wp_idx+1}/{len(self.waypoints)}: ({x:.2f},{y:.2f},{z:.2f})")
        self.wp_idx += 1


def main():
    parser = argparse.ArgumentParser(description='Publish TestCase signpost waypoints to /traj_start_trigger for Ego-Planner')
    parser.add_argument('--drone_id', type=int, required=True)
    parser.add_argument('--testcase_id', type=int, required=True)
    parser.add_argument('--safe_z', type=float, default=1.8)
    parser.add_argument('--pre_dist', type=float, default=1.5)
    parser.add_argument('--post_dist', type=float, default=1.5)
    parser.add_argument('--interval', type=float, default=2.0, help='seconds between waypoints')
    args = parser.parse_args()

    rclpy.init()
    node = WaypointFeeder(args.drone_id, args.testcase_id, args.safe_z, args.pre_dist, args.post_dist, args.interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 