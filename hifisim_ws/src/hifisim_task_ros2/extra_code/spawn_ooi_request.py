#!/usr/bin/env python3
import rclpy
import sys
from hifisim_task_ros2.sim_flight import SpawnOoiClient

def main():
    rclpy.init(args=sys.argv)
    spawn_ooi_client = SpawnOoiClient()
    ooi_scenario_id = int(input("Enter ooi_scenario_id(1-10): "))
    spawn_ooi_client.spawn_ooi(ooi_scenario_id)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()