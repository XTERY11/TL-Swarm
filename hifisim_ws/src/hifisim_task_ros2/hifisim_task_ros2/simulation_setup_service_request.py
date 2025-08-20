import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import SimulationSetupService

class SimulationSetupServiceClient(Node):

    def __init__(self):
        super().__init__('simulation_setup_service_client')
        self.client = self.create_client(SimulationSetupService, 'simulation_setup_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = SimulationSetupService.Request()

    def send_request(self, service_msg, requester):
        self.request.service_msg = service_msg
        self.request.requester = requester
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    node = SimulationSetupServiceClient()

    print("Select service message:")
    print("1: sim_started")
    print("2: sim_setup")
    print("3: request_start_sim")
    service_msg_option = input("Enter the number corresponding to the service message: ")

    service_msg_dict = {
        '1': 'sim_started',
        '2': 'sim_setup',
        '3': 'request_start_sim'
    }
    service_msg = service_msg_dict.get(service_msg_option, 'invalid')

    if service_msg == 'invalid':
        print("Invalid option selected.")
        return

    print("Select requester:")
    print("1: GCS")
    print("2: RadioComm")
    print("3: Performance")
    requester_option = input("Enter the number corresponding to the requester: ")

    requester_dict = {
        '1': 'GCS',
        '2': 'RadioComm',
        '3': 'Performance'
    }
    requester = requester_dict.get(requester_option, 'invalid')

    if requester == 'invalid':
        print("Invalid option selected.")
        return

    response = node.send_request(service_msg, requester)
    node.get_logger().info(f'Result: success={response.success}, testcase_id={response.testcase_id}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()