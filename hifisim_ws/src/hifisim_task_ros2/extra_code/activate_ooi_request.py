#!/usr/bin/env python3
import rclpy
import sys
from hifisim_task_ros2.sim_flight import ActivateOoiClient

def main():
    rclpy.init(args=sys.argv)
    print("    Activate Ooi")
    activate_ooi_client = ActivateOoiClient()
    activate_ooi_client.activate_ooi()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()