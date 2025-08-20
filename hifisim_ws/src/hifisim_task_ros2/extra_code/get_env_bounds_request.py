#!/usr/bin/env python3

from __future__ import print_function

import sys
import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import GetEnvBoundsService
import time

class GetEnvBounds(Node):
    def __init__(self):
        super().__init__("get_env_bounds_node")

    def get_env_bounds(self):
        try:
            self.get_logger().info("Waiting for service 'get_env_bounds'")
            client = self.create_client(GetEnvBoundsService, 'get_env_bounds')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again...')

            request = GetEnvBoundsService.Request()
            t0 = time.time()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
    
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info("Service response: Success \n")
                    self.get_logger().info("Successfully got the environment bounds! [{} s]".format(time.time()-t0))
                    for i in response.box_infos:
                        name = i.instance_name
                        bounds = i.corners
                        height = i.height
                        # self.get_logger().info("Building Name: {}, Bounds: {}, {}, {}, {}, Height: {}".format(name, bounds[0], bounds[1], bounds[2], bounds[3], height))
                        print(f"Building Name: {name}\n" + \
                                f"Corner 1 NWU = ({bounds[0].x: 6.2f}, {bounds[0].y: 6.2f}, {bounds[0].z: 6.2f})\n" + \
                                f"Corner 2 NWU = ({bounds[1].x: 6.2f}, {bounds[1].y: 6.2f}, {bounds[1].z: 6.2f})\n" + \
                                f"Corner 3 NWU = ({bounds[2].x: 6.2f}, {bounds[2].y: 6.2f}, {bounds[2].z: 6.2f})\n" + \
                                f"Corner 4 NWU = ({bounds[3].x: 6.2f}, {bounds[3].y: 6.2f}, {bounds[3].z: 6.2f})\n" + \
                                f"Height = {height: 6.2f}\n")
                else:
                    self.get_logger().info("Service response: Failure\n")
                return response
        except Exception as e:
            self.get_logger().error("Service call failed: %s" % e)


def main():
    rclpy.init(args=sys.argv)
    node = GetEnvBounds()
    node.get_env_bounds()
    rclpy.shutdown()


if __name__ == "__main__":
    main()