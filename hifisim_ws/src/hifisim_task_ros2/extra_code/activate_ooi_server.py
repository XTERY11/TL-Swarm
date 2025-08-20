#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import ActivateOoiService

def activate_ooi_handler(req, response):
    node.get_logger().info("Received an ActivateOoiService request.")
    # activate the current active ooi_id
    # activation code
    response.success = True
    return response

def activate_ooi_server():
    global node
    rclpy.init(args=None)
    node = Node("activate_ooi_server")
    node.get_logger().info("Initialized ActivateOoiService server.")
    srv = node.create_service(ActivateOoiService, "/activate_ooi", activate_ooi_handler)
    rclpy.spin(node)

def main():
    activate_ooi_server()

if __name__ == "__main__":
    main()
