#!/usr/bin/env python3
# Use Unity C# naming convention CamelCase because data read from Unity created json

import sys
import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import ResetTestCaseService
from hifisim_msg_ros2.msg import AgentGroup, Agent, AgentFeasiblePose, Waypoint
from transforms3d.euler import euler2quat
from geometry_msgs.msg import Pose, Point, Quaternion
from .read_testcase import read_testcase_struct

class ResetTestcaseClient(Node):

    def __init__(self):
        super().__init__('reset_testcase_client')

    def reset_testcase_client(self, testcase_id):
        self.get_logger().info('    ...... Waiting for reset_testcase_service_server')
        client = self.create_client(ResetTestCaseService, 'reset_testcase')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = ResetTestCaseService.Request()

        # Read the test case structure
        testcase_struct = read_testcase_struct(testcase_id)
        
        # Directly assign values from the nested structure
        request.testcase_id = testcase_struct['Testcase']['TestcaseId']
        request.description = testcase_struct['Testcase']['Description']
        request.env_id = testcase_struct['Testcase']['EnvId']
        request.env_scenario_id = testcase_struct['Testcase']['EnvScenarioId']
        request.radio_comm_on = testcase_struct['SimulatorSetting']['RadioCommOn']
        request.radio_comm_range = testcase_struct['SimulatorSetting']['RadioCommRange']
        request.performance_on = testcase_struct['SimulatorSetting']['PerformanceOn']
        request.sim_detection_on = testcase_struct['SimulatorSetting']['SimDetectionOn']
        request.sim_detection_range = testcase_struct['SimulatorSetting']['SimDetectionRange']
        request.sim_detection_frequency = testcase_struct['SimulatorSetting']['SimDetectionFrequency']

        for group in testcase_struct['AgentGroups']:
            agent_group = AgentGroup()
            agent_group.coordinate_system = group['CoordinateSystem']
            agent_group.agent_type = group['AgentType']
            agent_group.publish_frequency = group['PublishFrequency']
            agent_group.image_size_scale = group['ImageSizeScale']
            agent_group.is_mono_on = group['CameraConfig']['IsMonoOn']
            agent_group.is_mono_depth_on = group['CameraConfig']['IsMonoDepthOn']
            agent_group.rotation_x_mono = group['CameraConfig']['RotationXMono']
            agent_group.focal_length_mono = group['CameraConfig']['FocalLengthMono']
            agent_group.clipping_mono = group['CameraConfig']['ClippingMono']
            agent_group.is_stereo_on = group['CameraConfig']['IsStereoOn']
            agent_group.is_stereo_depth_on = group['CameraConfig']['IsStereoDepthOn']
            agent_group.is_stereo_no_agents_on = group['CameraConfig']['IsStereoNoAgentsOn']
            agent_group.is_stereo_point_cloud_on = group['CameraConfig']['IsStereoPointCloudOn']
            agent_group.rotation_x_stereo = group['CameraConfig']['RotationXStereo']
            agent_group.focal_length_stereo = group['CameraConfig']['FocalLengthStereo']
            agent_group.clipping_stereo = group['CameraConfig']['ClippingStereo']
            agent_group.is_fisheye_on = group['CameraConfig']['IsFishEyeOn']
            agent_group.is_eagleeye_on = group['CameraConfig']['IsEagleEyeOn']

            for agent in group['Agents']:
                agent_msg = Agent()
                agent_msg.agent_id = agent['AgentId']
                agent_msg.ip_address = agent['IpAddress']
                waypoint1 = agent['Waypoints'][0]

                # Define Euler angles (in radians), UAV does not roll and pitch at Initial Pose
                roll = 0.0
                pitch = 0.0
                yaw = waypoint1[5] * 3.14159265 / 180.0
                # Convert Euler angles to quaternion
                quaternion = euler2quat(roll, pitch, yaw)
                agent_msg.initial_pose = Pose(
                    position=Point(x=waypoint1[0], y=waypoint1[1], z=waypoint1[2]),
                    orientation=Quaternion(x=quaternion[1], y=quaternion[2], z=quaternion[3], w=quaternion[0])
                )

                # Add waypoints to agent_msg
                for waypoint in agent['Waypoints']:
                    waypoint_msg = Waypoint()
                    waypoint_msg.position.x = waypoint[0]
                    waypoint_msg.position.y = waypoint[1]
                    waypoint_msg.position.z = waypoint[2]
                    waypoint_msg.rotation.x = waypoint[3]
                    waypoint_msg.rotation.y = waypoint[4]
                    waypoint_msg.rotation.z = waypoint[5]
                    agent_msg.waypoints.append(waypoint_msg)

                agent_group.agents.append(agent_msg)
                self.get_logger().info(f"Agent ID: {agent['AgentId']}")
                self.get_logger().info(f"Agent IP Address: {agent['IpAddress']}")
                self.get_logger().info("Agent Waypoints:")
                for waypoint in agent['Waypoints']:
                    self.get_logger().info(f"  Waypoint: ({waypoint[0]}, {waypoint[1]}, {waypoint[2]}, {waypoint[3]}, {waypoint[4]}, {waypoint[5]})")

            request.agent_groups.append(agent_group)

        # Line sending the request
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()

            # Print the feasible poses in the response
            self.get_logger().info("Service response:")
            for i, pose in enumerate(response.agent_feasible_pose):
                self.get_logger().info(f"  Agent {pose.agent_id} Feasible Pose:")
                self.get_logger().info(f"  Position: ({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z})")
                self.get_logger().info(f"  Orientation: ({pose.pose.orientation.x}, {pose.pose.orientation.y}, {pose.pose.orientation.z}, {pose.pose.orientation.w})")

            return response
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")

def main(args=None):
    rclpy.init(args=args)
    node = ResetTestcaseClient()
    if len(sys.argv) == 2:
        testcase_id = int(sys.argv[1])
    else:
        node.get_logger().info("Usage: reset_testcase_request [testcase_id]")
        sys.exit(1)
    node.get_logger().info(f"Requesting reset for test case {testcase_id}")
    node.reset_testcase_client(testcase_id)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
