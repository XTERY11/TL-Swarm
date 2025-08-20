#!/usr/bin/env python3

from __future__ import print_function

import sys
import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import GetMapService
from geometry_msgs.msg import Point
import time
import os


class GetMapClient(Node):
    def __init__(self):
        super().__init__("get_map_client_node")

    def get_global_point_cloud(self):
        try:
            self.get_logger().info("Waiting for service 'get_map_srv'")
            self.client = self.create_client(GetMapService, 'get_map_srv')
            self.client.wait_for_service()

            srv = GetMapService.Request()
            srv.range = Point(x=100.0, y=100.0, z=9.0)
            srv.resolution = 0.1
            srv.savepath = os.path.join(os.path.expanduser('~'), "catkin_ws")
            srv.filename = "map"

            self.get_logger().info("Saving to map....")
            t0 = time.time()
            future = self.client.call_async(srv)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()

            if res.success:
                self.get_logger().info("Successfully get map! [{} s]".format(time.time() - t0))
            else:
                self.get_logger().warn("Fail to get map!")
        except Exception as e:
            self.get_logger().error("Service call failed: %s" % e)


def main():
    rclpy.init(args=sys.argv)
    client = GetMapClient()
    client.get_global_point_cloud()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
