#!/usr/bin/env python3
# sim_flight.py does not send reset_testcase_request
# only to select flight tasks, read mission waypoints from json instead of param.

from __future__ import print_function
import rclpy
from rclpy.node import Node
import sys
#import math
#import threading
from datetime import datetime
#from sensor_msgs.msg import Image
#from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from hifisim_msg_ros2.msg import ImageCamInfoOoi
#from hifisim_msg_ros2.msg import OoiGroundTruth
from hifisim_msg_ros2.msg import OoiGroundTruthFrame
from hifisim_msg_ros2.srv import SpawnOoiService
from hifisim_msg_ros2.srv import ActivateOoiService
from .read_testcase import read_testcase_struct, read_agents_list
from pathlib import Path
import time
from rclpy.duration import Duration


class SimFlight(Node):

    class VehicleTask:
        kIdle = 0
        kTakeOff = 1
        kHover = 2
        kMission = 3
        kHome = 4
        kLand = 5
        kFollow = 6
        
    def __init__(self, testcase_id):
        super().__init__('sim_flight_node')
        testcase_struct = read_testcase_struct(testcase_id)
        self.ooi_scenario_id = testcase_struct['Testcase']['EnvId']
        self.agents_list = read_agents_list(testcase_struct)
        # print("\nAgents List:", self.agents_list)

        # Store the available agent IDs
        available_agent_ids = [agent['AgentId'] for agent in self.agents_list]
        print("Available Agent IDs:", available_agent_ids)

        # Get the number of available agent IDs
        num_agents = len(available_agent_ids)
        print(f"Number of available agent IDs: {num_agents}")
        
        self.n_uavs = num_agents
        # Publishers
        self._waypoint_pubs = {}
        for uav_id in range(1, self.n_uavs + 1):
            self._waypoint_pubs[uav_id] = self.create_publisher(JointTrajectory, f'/agent{uav_id:03}/trajectory/points', 1000)

    def get_user_input(self):
        # Use env_id for ooi_scenario_id 
        # ooi_scenario_id = int(input("Enter ooi_scenario_id & UAV waypoint file for mission (1-20): ")) 
        # self.ooi_scenario_id = ooi_scenario_id 
        print("\n    Select takeoff first (1 or 7)")
        while True:
            print("\nEnter task number for all UAVs ")
            uav_task = int(input("1:TakeOff, 2:Hover, 3:Mission, 4:Home, 5:Land, 6:Follow, 7:TakeOff & Spawn Ooi, 8:Mission, Act, Rec, 9:Exit : "))
            #print(f'uav_task no = {uav_task}')  
            if uav_task == 8: #(Mission, Act Ooi, Record)
                record_time = int(input("Enter recording time in seconds (10 recommended): "))  # time in second
                print("    Activate Ooi")
                activate_ooi_client = ActivateOoiClient()
                activate_ooi_client.activate_ooi()
                topics = {f'/agent{(i+1):03}/image_cam_info_ooi': ImageCamInfoOoi for i in range(self.n_uavs)}
                topics_gt = {f'/agent{(i+1):03}/ooi_ground_truth_frame': OoiGroundTruthFrame for i in range(self.n_uavs)}
                topics.update(topics_gt)
                now = datetime.now() # current date and time
                date_time = now.strftime("%Y-%m-%d-%H%M%S")
                bag_file = str(Path.home()) + f'/rosbag/bag-' + date_time
            for uav_id in range(1, self.n_uavs + 1):
                #print(f"Run UAV: {uav_id} task: {uav_task}")
                self.task_request(uav_id, uav_task)
            if uav_task == 7:  #(TakeOff & Spawn Ooi)
                print(f"    Spawn ooi_scenario_id: {self.ooi_scenario_id}")
                time.sleep(1)
                spawn_ooi_client = SpawnOoiClient()
                spawn_ooi_client.spawn_ooi(self.ooi_scenario_id)
            if uav_task == 8: #(Mission, Act Ooi, Record)
                try:
                    print("    .....Wait 10 seconds for agents mission to reach its highest point before recording")
                    time.sleep(10)  # Time to wait for mission to reach highest point
                    print(f"START ROSBAG RECORDING for {record_time} seconds ....")
                    recorder = RosbagRecorder(topics, bag_file, record_time)
                except rclpy.exceptions.ROSInterruptException:
                    pass  # when Exception run shutdown
            if uav_task < 1 or uav_task > 8:
                print(f'Exit uav_task no = {uav_task}')
                break
        print(f'After Break - uav_task no = {uav_task}')
        return
        
    def task_request(self, uav_id, uav_task):
        if uav_task == self.VehicleTask.kTakeOff:
            self.publish_traj(uav_id, uav_task)
        elif uav_task == self.VehicleTask.kHover:
            self.publish_traj(uav_id, uav_task)
        elif uav_task == self.VehicleTask.kMission:
            self.load_mission_json(uav_id, uav_task)
        elif uav_task == self.VehicleTask.kHome:
            self.publish_traj(uav_id, uav_task)
        elif uav_task == self.VehicleTask.kLand:
            self.publish_traj(uav_id, uav_task)
        elif uav_task == self.VehicleTask.kFollow:
            print("uav_task follow ", uav_task)
        elif uav_task == 7: #(TakeOff & Spawn Ooi)
            self.publish_traj(uav_id, self.VehicleTask.kTakeOff)
        elif uav_task == 8: #(Mission, Act Ooi, Record)
            self.load_mission_json(uav_id, self.VehicleTask.kMission) 
        elif uav_task == 9: #(Exit)
            print("Exiting...") 
        else:
            print(f"Invalid task number for UAV {uav_id}. Please enter a valid task number (1-8).")

    def load_mission_json(self, uav_id, uav_task):
        print(f"Load mission from json: UAV {uav_id}")
        # Initialize wp_list as an empty list
        wp_list = []
        # Iterate over each agent in self.agents_list
        for agent in self.agents_list:
            # Check if the current agent's ID matches the agent_id_to_find
            if agent['AgentId'] == uav_id:
                # If a match is found, extract specific waypoints and add them to wp_list
                for agent_wp in agent['Waypoints']:
                    wp = []
                    wp.append(agent_wp[0])  # X
                    wp.append(agent_wp[1])  # Y
                    wp.append(agent_wp[2])  # Z
                    wp.append(agent_wp[5])  # RZ
                    wp_list.append(wp)
                # Exit the loop early since the desired agent has been found
                break

        if len(wp_list) > 0:
            jt = JointTrajectory()
            jt.header.stamp = self.get_clock().now().to_msg()
            uav_str = f'{uav_id:03}'
            jt.joint_names.append("agent" + uav_str)

            for p in wp_list:
                jtp = JointTrajectoryPoint()
                jtp.positions = p[:3]
                jtp.time_from_start = Duration(seconds=uav_task).to_msg()
                jt.points.append(jtp)
                print(f"    wp_list= ({p[0]}, {p[1]}, {p[2]}, {p[3]})")

        if uav_id in self._waypoint_pubs:
            self._waypoint_pubs[uav_id].publish(jt)

            return True
        else:
            return False
        
    def follow(self, uav_id, uav_task):
        # ...
        pass

    def publish_traj(self, uav_id, uav_task):
        jt = JointTrajectory()
        jtp = JointTrajectoryPoint()

        jt.header.stamp = self.get_clock().now().to_msg()
        uav_str = f'{uav_id:03}'
        jt.joint_names.append(f"agent{uav_str}")
        jtp.positions.extend([0.0, 0.0, 0.0])
        jtp.time_from_start = Duration(seconds=uav_task).to_msg()
        jt.points.append(jtp)

        if uav_id in self._waypoint_pubs:
            self._waypoint_pubs[uav_id].publish(jt)
            
        print(f"    {jt.joint_names[0]} Send task: {jtp.time_from_start}  send point ({jtp.positions[0]}, {jtp.positions[1]}, {jtp.positions[2]})")

class SpawnOoiClient(Node):
    def __init__(self):
        super().__init__('spawn_ooi_client')

    def spawn_ooi(self, ooi_scenario_id):
        self.get_logger().info('Waiting for service /spawn_ooi')
        self.client = self.create_client(SpawnOoiService, '/spawn_ooi')
        self.client.wait_for_service()
        try:
            request = SpawnOoiService.Request()
            request.ooi_scenario_id = ooi_scenario_id
            t0 = time.time()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                res = future.result()
                if res.success:
                    self.get_logger().info(f"Successfully spawned OOI! [{time.time()-t0} s]")
                    print("    return number_ooi_scenario:", res.number_ooi_scenario)
                else:
                    self.get_logger().warn("Failed to spawn OOI!")
            else:
                self.get_logger().error(f"Service call failed: {future.exception()}")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

class ActivateOoiClient(Node):
    def __init__(self):
        super().__init__('activate_ooi_client')

    def activate_ooi(self):
        self.get_logger().info('Waiting for service /activate_ooi')
        self.client = self.create_client(ActivateOoiService, '/activate_ooi')
        self.client.wait_for_service()
        try:
            request = ActivateOoiService.Request()
            t0 = time.time()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                res = future.result()
                if res.success:
                    self.get_logger().info(f"Successfully activated OOI! [{time.time()-t0} s]")
                else:
                    self.get_logger().warn(f"Failed to activate OOI!")
            else:
                self.get_logger().error(f"Service call failed: {future.exception()}")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

class RosbagRecorder:
    def __init__(self, topics, bag_file, record_time):
        # Rosbag is not available in ROS2; this section needs a different approach
        pass

    def shutdown(self):
        pass

def main():
    try:
        rclpy.init(args=sys.argv)
        # Reading testcase
        testcase_id = int(input("Enter testcase id: "))
        sim_flight_node = SimFlight(testcase_id)
        sim_flight_node.get_user_input()
        rclpy.shutdown()
    except rclpy.exceptions.ROSInterruptException:
        print("sim_flight_node ended")

if __name__ == "__main__":
    main()
