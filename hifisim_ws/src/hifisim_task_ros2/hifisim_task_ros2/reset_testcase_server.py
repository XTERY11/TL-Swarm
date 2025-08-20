#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import ResetTestCaseService
from hifisim_msg_ros2.msg import AgentGroup, Agent, AgentFeasiblePose, Waypoint

class ResetTestCaseServer(Node):

    def __init__(self):
        super().__init__('reset_testcase_server')
        self.srv = self.create_service(ResetTestCaseService, 'reset_testcase', self.handle_reset_testcase)
        self.get_logger().info("Ready to reset test cases.")

    def handle_reset_testcase(self, req, response):
        self.get_logger().info("Received request for resetting test case:")
        self.get_logger().info(f"Testcase ID: {req.testcase_id}")
        self.get_logger().info(f"Description: {req.description}")
        self.get_logger().info(f"Environment ID: {req.env_id}")
        self.get_logger().info(f"Env Scenario ID: {req.env_scenario_id}")
        self.get_logger().info(f"Radio Comm On: {req.radio_comm_on}")
        self.get_logger().info(f"Radio Comm Range: {req.radio_comm_range}")
        self.get_logger().info(f"Performance On: {req.performance_on}")
        self.get_logger().info(f"Sim Detection On: {req.sim_detection_on}")
        self.get_logger().info(f"Sim Detection Range: {req.sim_detection_range}")
        self.get_logger().info(f"Sim Detection Frequency: {req.sim_detection_frequency}")

        for i, group in enumerate(req.agent_groups):
            self.get_logger().info(f"Agent Group {i + 1}:")
            self.get_logger().info(f"  Coordinate System: {group.coordinate_system}")
            self.get_logger().info(f"  Agent Type: {group.agent_type}")
            self.get_logger().info(f"  Publish Frequency: {group.publish_frequency}")
            self.get_logger().info(f"  Image Size Scale: {group.image_size_scale}")
            self.get_logger().info(f"  Mono Camera Enabled: {group.is_mono_on}")
            self.get_logger().info(f"  Mono Camera Depth Enabled: {group.is_mono_depth_on}")
            self.get_logger().info(f"  Rotation X Mono: {group.rotation_x_mono}")
            self.get_logger().info(f"  Focal Length Mono: {group.focal_length_mono}")
            self.get_logger().info(f"  Clipping Mono: {group.clipping_mono}")
            self.get_logger().info(f"  Stereo Camera Enabled: {group.is_stereo_on}")
            self.get_logger().info(f"  Stereo Camera Depth Enabled: {group.is_stereo_depth_on}")
            self.get_logger().info(f"  Stereo Camera NoAgents Enabled: {group.is_stereo_no_agents_on}")
            self.get_logger().info(f"  Stereo Camera PointCloud Enabled: {group.is_stereo_point_cloud_on}")
            self.get_logger().info(f"  Rotation X Stereo: {group.rotation_x_stereo}")
            self.get_logger().info(f"  Focal Length Stereo: {group.focal_length_stereo}")
            self.get_logger().info(f"  Clipping Stereo: {group.clipping_stereo}")
            self.get_logger().info(f"  Fish Eye Enabled: {group.is_fisheye_on}")
            self.get_logger().info(f"  Eagle Eye Enabled: {group.is_eagleeye_on}")

            for j, agent in enumerate(group.agents):
                self.get_logger().info(f"  Agent {j + 1}:  Agent ID: {agent.agent_id}")
                self.get_logger().info(f"  Agent {j + 1} IP Address: {agent.ip_address}")
                self.get_logger().info(f"  Agent {j + 1} Waypoints:")
                for k, waypoint in enumerate(agent.waypoints):
                    self.get_logger().info(f"    Waypoint {k + 1}: Position ({waypoint.position.x}, {waypoint.position.y}, {waypoint.position.z}), Rotation ({waypoint.rotation.x}, {waypoint.rotation.y}, {waypoint.rotation.z})")

        response.success = True
        response.agent_feasible_pose = []

        for group in req.agent_groups:
            for agent in group.agents:
                feasible_pose = AgentFeasiblePose()
                feasible_pose.agent_id = agent.agent_id
                # Assuming the feasible pose comes from the agent's waypoints or current pose
                feasible_pose.pose = agent.initial_pose # Modify as needed based on the new structure
                response.agent_feasible_pose.append(feasible_pose)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ResetTestCaseServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
