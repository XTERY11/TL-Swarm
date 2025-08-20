import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.msg import MissionResult
from hifisim_msg_ros2.srv import SimulationSetupService
from geometry_msgs.msg import Point
import signal, sys
from hifisim_task_ros2.init_testcase_request import InitTestcaseClient

class GroundControlStation(Node):
    class State:
        SIM_UNINITIALIZED = 'SIM_UNINITIALIZED'
        SIM_INITIALIZED = 'SIM_INITIALIZED'
        SIM_START = 'SIM_START'
        SIM_END = 'SIM_END'

    def __init__(self):
        super().__init__('ground_control_station')
        self.publisher_ = self.create_publisher(MissionResult, 'GCS/mission_results', 10)
        self.timer = None  # Timer will be created after service response
        self.state = self.State.SIM_UNINITIALIZED

        signal.signal(signal.SIGINT, self.handle_shutdown)
        self.run_fsm()

    def run_fsm(self):
        if self.state == self.State.SIM_UNINITIALIZED:
            self.send_init_testcase_request()
            self.state = self.State.SIM_INITIALIZED
            self.get_logger().info(f'State changed to: {self.state}')

        if self.state == self.State.SIM_INITIALIZED:
            input('\nReady to send sim_started request to HiFi Simulator (press Enter to continue)...')
            response = self.send_sim_started()
            if response:
                self.get_logger().info('Simulation start approved.')
                self.state = self.State.SIM_START
                self.get_logger().info(f'State changed to: {self.state}')
            else:
                self.get_logger().info('Simulation start not approved.')

        if self.state == self.State.SIM_START:
            # Users should inform all agents the simulation has started
            self.get_logger().info('Informing all agents simulation has started........')
            #self.get_logger().info('Starting to publish mission_result...')
            self.publish_mission_result()
            pass

    def handle_shutdown(self, signum, frame):
        if self.state != self.State.SIM_END:
            mission_result = MissionResult()
            mission_result.result_type = 'completed'
            mission_result.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(mission_result)
            self.get_logger().info('Publishing final mission_result: completed')
            self.state = self.State.SIM_END
            self.get_logger().info(f'State changed to: {self.state}')
        self.destroy_node()
        rclpy.shutdown()

    def send_init_testcase_request(self):
        # Prompt user for testcase_id input
        testcase_input = input('Enter testcase_id: ')
        try:
            testcase_id = int(testcase_input)
        except ValueError:
            self.get_logger().error('Invalid testcase_id. Must be an integer.')
            return
        # Instantiate the init testcase client and call its service method
        init_client = InitTestcaseClient()
        self.get_logger().info(f"Requesting initialization for test case {testcase_id}")
        init_client.init_testcase_client(testcase_id)

    def send_sim_started(self):
        client = self.create_client(SimulationSetupService, 'simulation_setup_service')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = SimulationSetupService.Request()
        request.service_msg = 'sim_started'
        request.requester = 'GCS'
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'send_sim_started-Response: success={future.result().success}')
            return future.result().success
        else:
            self.get_logger().error('Service call failed')
            return False

    def publish_mission_result(self):  # Modified to add a quit option with exit
        if self.state != self.State.SIM_START:
            return

        while True:
            selection = input("\nSelect mission result type:\n"
                              " (1) 'ooi_found'\n"
                              " (2) 'mission completed for one agent'\n"
                              " (3) 'mission completed for all agents'\n"
                              " (4) 'Quit'\n"
                              "Enter selection [default 1]: ") or '1'

            match selection:
                case '1':
                    # ...existing ooi_found prompt...
                    ooi_id = input("Enter ooi_id [default 5]: ") or "5"
                    ooi_class_id = input("Enter ooi_class_id [default 105]: ") or "105"
                    x = input("Enter ooi_location x [default 1.0]: ") or "1.0"
                    y = input("Enter ooi_location y [default 2.0]: ") or "2.0"
                    z = input("Enter ooi_location z [default 3.0]: ") or "3.0"
                    mission_result = MissionResult()
                    mission_result.result_type = 'ooi_found'
                    mission_result.ooi_id = int(ooi_id)
                    mission_result.ooi_class_id = int(ooi_class_id)
                    mission_result.ooi_location = Point(x=float(x), y=float(y), z=float(z))
                    self.publisher_.publish(mission_result)
                    self.get_logger().info(f'Publishing mission_result: {mission_result}')
                    continue
                case '2':
                    # ...existing single-agent prompt...
                    agent_id = input("Enter agent_id for mission completion [default 1]: ") or "1"
                    mission_result = MissionResult()
                    mission_result.result_type = 'completed'
                    mission_result.agent_id = int(agent_id)
                    mission_result.stamp = self.get_clock().now().to_msg()
                    self.publisher_.publish(mission_result)
                    self.get_logger().info(f'Publishing mission_result: {mission_result}')
                    continue
                case '3':
                    # ...existing multi-agent prompt...
                    mission_result = MissionResult()
                    mission_result.result_type = 'completed'
                    mission_result.agent_id = 0
                    mission_result.stamp = self.get_clock().now().to_msg()
                    self.publisher_.publish(mission_result)
                    self.get_logger().info(f'Publishing mission_result: {mission_result}')
                    self.state = self.State.SIM_END
                    self.get_logger().info(f'State changed to: {self.state}')
                    continue
                case '4':
                    self.get_logger().info("Quitting... Shutting down node.")
                    self.destroy_node()
                    rclpy.shutdown()
                    sys.exit(0)
                case _:
                    self.get_logger().error("Invalid selection. Please try again.")
                    continue


def main(args=None):
    rclpy.init(args=args)
    ground_control_station = GroundControlStation()
    rclpy.spin(ground_control_station)
    ground_control_station.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
