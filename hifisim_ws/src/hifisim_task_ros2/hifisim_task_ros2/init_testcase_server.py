#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import InitTestCaseService
from geometry_msgs.msg import Pose

class InitTestCaseServer(Node):
    def __init__(self):
        super().__init__('init_testcase_server')
        self.srv = self.create_service(InitTestCaseService, 'init_testcase', self.handle_init_testcase)
        self.get_logger().info("Ready to initialize test cases.")

    def handle_init_testcase(self, req, response):
        self.get_logger().info("Received request for initializing test case:")
        self.get_logger().info(f"Testcase ID: {req.testcase_id}")
        
        response.success = True
        response.agent_id = [1, 2]

        pose1 = Pose()
        pose1.position.x = 1.0
        pose1.position.y = 1.0
        pose1.position.z = 1.0
        pose1.orientation.x = 0.0
        pose1.orientation.y = 0.0
        pose1.orientation.z = 0.0
        pose1.orientation.w = 1.0

        pose2 = Pose()
        pose2.position.x = 2.0
        pose2.position.y = 2.0
        pose2.position.z = 2.0
        pose2.orientation.x = 0.0
        pose2.orientation.y = 0.0
        pose2.orientation.z = 0.0
        pose2.orientation.w = 1.0

        response.agent_feasible_pose = [pose1, pose2]
        return response

def main(args=None):
    rclpy.init(args=args)
    node = InitTestCaseServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()