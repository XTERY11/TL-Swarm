import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import SimulationSetupService
import time

class SimulationSetupServiceServer(Node):

    def __init__(self):
        super().__init__('simulation_setup_service_server')
        self.srv = self.create_service(SimulationSetupService, 'simulation_setup_service', self.handle_service_request)

    def handle_service_request(self, request, response):
        self.get_logger().info(f'Received request: service_msg={request.service_msg}, requester={request.requester}')

        # Add a 2-second delay before responding
        time.sleep(1)

        if request.service_msg == 'sim_started':
            response.testcase_id = 13
            if request.requester == 'GCS':
                response.success = True
            else:
                response.success = False

        elif request.service_msg == 'sim_setup':
            response.testcase_id = 13
            if request.requester == 'Performance':
                response.success = True
            else:
                response.success = False

        elif request.service_msg == 'request_start_sim':
            response.testcase_id = 13
            if request.requester == 'Performance':
                response.success = True
            else:
                response.success = False

        else:
            response.success = False
            response.testcase_id = 0

        return response

def main(args=None):
    rclpy.init(args=args)
    node = SimulationSetupServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()