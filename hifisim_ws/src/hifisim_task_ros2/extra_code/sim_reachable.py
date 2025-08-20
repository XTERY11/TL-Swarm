import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.srv import ReachableAgentsService
from hifisim_msg_ros2.srv import GetTotalAgentsService

class ReachableAgents(Node):
    def __init__(self):
        super().__init__('reachable_agents')
        self.get_total_agents_service = self.create_service(GetTotalAgentsService, 'get_total_agents', self.handle_get_total_agents_request)
        self.reachable_agents_service = self.create_service(ReachableAgentsService, 'reachable_agents_request', self.handle_reachable_agents_request)
        self.get_logger().info('ReachableAgents node is up.')

    def handle_get_total_agents_request(self, request, response):
        total_agents = 10  # Hardcoded total number of agents for demonstration
        response.success = True
        response.total_agents = total_agents
        self.get_logger().info(f'Setting response: success={response.success}, total_agents={response.total_agents}')
        return response

    def handle_reachable_agents_request(self, request, response):
        self.get_logger().info(f'Handling reachable_agents_request for sent_agent: {request.sent_agent}')
        # Hardcoded list of reachable agents for demonstration
        self.get_logger().info('handling... reachable_agents_request')
        reachable_agents = [2, 6, 8]
        prob_success = [0.9] * len(reachable_agents)  # Hardcoded probability of success
        response.success = True if reachable_agents else False
        response.reachable_agents = list(reachable_agents)
        response.prob_success = list(prob_success)
        self.get_logger().info(f'Setting response: success={response.success}, reachable_agents={response.reachable_agents}, prob_success={response.prob_success}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ReachableAgents()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()