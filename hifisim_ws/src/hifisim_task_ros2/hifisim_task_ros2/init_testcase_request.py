#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import InitTestCaseService
from geometry_msgs.msg import Pose

class InitTestcaseClient(Node):
    def __init__(self):
        super().__init__('init_testcase_client')

    def init_testcase_client(self, testcase_id):
        self.get_logger().info('... Waiting for init_testcase_service_server')
        client = self.create_client(InitTestCaseService, 'init_testcase')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = InitTestCaseService.Request()
        request.testcase_id = testcase_id

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info("Service response:")
            self.get_logger().info(f"Success: {response.success}")
            # Response arrays (agent_id and agent_feasible_pose) have matching indices
            for idx, agent_id in enumerate(response.agent_id):
                pose = response.agent_feasible_pose[idx]
                self.get_logger().info(f"Agent {agent_id} Feasible Pose:")
                self.get_logger().info(f"  Position: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
                self.get_logger().info(f"  Orientation: ({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})")
            return response
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")

def main(args=None):
    rclpy.init(args=args)
    node = InitTestcaseClient()
    if len(sys.argv) == 2:
        testcase_id = int(sys.argv[1])
    else:
        node.get_logger().info("Usage: init_testcase_request [testcase_id]")
        sys.exit(1)
    node.get_logger().info(f"Requesting init for test case {testcase_id}")
    node.init_testcase_client(testcase_id)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
