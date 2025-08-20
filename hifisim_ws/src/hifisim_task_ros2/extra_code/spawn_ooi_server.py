#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import SpawnOoiService

def spawn_ooi_handler(req, response):
    response.success = True
    response.number_ooi_scenario = 3
    node.get_logger().info("Received a SpawnOoiService request with scenario ID: {}".format(req.ooi_scenario_id))
    # spawning an object of interest in the environment
    # return a success message once the action is completed
    return response

def spawn_ooi_server():
    global node
    rclpy.init(args=None)
    node = Node("spawn_ooi_server")
    node.get_logger().info("Initialized SpawnOoiService server.")
    srv = node.create_service(SpawnOoiService, "/spawn_ooi", spawn_ooi_handler)
    rclpy.spin(node)

def main():
    spawn_ooi_server()

if __name__ == "__main__":
    main()
