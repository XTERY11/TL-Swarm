#!/usr/bin/env python3

from __future__ import print_function

import sys
import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import GetEnvCornersService


class GetEnvCorners(Node):
    def __init__(self):
        super().__init__("get_env_corners_node")

    def get_env_corners(self, env_id):
        try:
            self.get_logger().info("Waiting for service 'get_env_corners'")
            client = self.create_client(GetEnvCornersService, 'get_env_corners')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again...')

            request = GetEnvCornersService.Request()
            request.env_id = env_id

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info("Service response: Success \n")
                    self.get_logger().info(f"Env {env_id} Relative Corner Positions with reference to origin:\n")
                    for i, corner in enumerate(response.boundary_corner):
                        self.get_logger().info(f"  Corner {i+1} NWU = ({corner.x: 6.2f}, {corner.y: 6.2f}, {corner.z: 6.2f})\n")
                else:
                    self.get_logger().info("Service response: Failure\n")
                return response
        except Exception as e:
            self.get_logger().error("Service call failed: %s" % e)


def main():
    rclpy.init(args=sys.argv)
    node = GetEnvCorners()
    if len(sys.argv) == 2:
        env_id = int(sys.argv[1])
    else:
        node.get_logger().info("Usage: get_env_corners_request.py [env_id]")
        sys.exit(1)

    node.get_env_corners(env_id)
    rclpy.shutdown()


if __name__ == "__main__":
    main()