import rclpy
from rclpy.node import Node
import time
from hifisim_msg_ros2.srv import SimulationSetupService
from hifisim_msg_ros2.msg import RadioBroadcast
from hifisim_msg_ros2.srv import RadioBroadcastService, ReachableAgentsService
from .read_testcase import read_testcase_struct, read_agents_list

class RadioCommSimulator(Node):
    class State:
        SIM_UNINITIALIZED = 'SIM_UNINITIALIZED'
        SIM_INITIALIZED = 'SIM_INITIALIZED'
        SIM_START = 'SIM_START'
        SIM_END = 'SIM_END'

    def __init__(self):
        super().__init__('radio_comm_simulator')
        self.state = self.State.SIM_UNINITIALIZED
        self.total_agents = 0
        self.testcase_id = 0
        self.run_fsm()

    def run_fsm(self):
        if self.state == self.State.SIM_UNINITIALIZED:
            self.testcase_id = self.send_sim_setup_request()
            self.setup_broadcast_publisher()
            self.state = self.State.SIM_INITIALIZED
            self.get_logger().info(f'State changed to: {self.state}')

        if self.state == self.State.SIM_INITIALIZED:
            # Request_Start_Sim. Wait for response from hifi_simulator_unity before proceeding
            self.request_start_sim()
            self.start_broadcast_service()
            self.state = self.State.SIM_START
            self.get_logger().info(f'State changed to: {self.state}')

    def send_sim_setup_request(self):
        client = self.create_client(SimulationSetupService, 'simulation_setup_service')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = SimulationSetupService.Request()
        request.service_msg = 'sim_setup'
        request.requester = 'RadioComm'
        
        while True:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None and future.result().success:
                self.get_logger().info(f'sim_setup_request Response: success={future.result().success}, testcase_id={future.result().testcase_id}')
                return future.result().testcase_id
            else:
                self.get_logger().error('sim_setup_request failed or unsuccessful, retrying in 4 seconds...')
                time.sleep(4)
        
    def request_start_sim(self):
        client = self.create_client(SimulationSetupService, 'simulation_setup_service')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = SimulationSetupService.Request()
        request.service_msg = 'request_start_sim'
        request.requester = 'RadioComm'
        
        while True:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None and future.result().success:
                self.get_logger().info(f'request_start_sim Response: success={future.result().success}')
                break
            else:
                self.get_logger().error('request_start_sim failed or unsuccessful, retrying in 4 seconds...')
                time.sleep(4)

    def setup_broadcast_publisher(self):
        self.get_logger().info(f'Setting up broadcast_publisher of testcase_id: {self.testcase_id}')
        testcase_struct = read_testcase_struct(self.testcase_id)
        agents_list = read_agents_list(testcase_struct)

        # Store the available agent IDs
        available_agent_ids = [agent['AgentId'] for agent in agents_list]
        self.get_logger().info(f"<Radio Comm> All available Agent IDs: {available_agent_ids}")

        # Get the number of available agent IDs
        self.total_agents = len(available_agent_ids)
        self.get_logger().info(f"<Radio Comm> Total_Number of agents: {self.total_agents}")

        ## ----------- Create publishers for all agents
        self.agent_publishers = {}
        for agent_id in range(1, self.total_agents + 1): 
            topic_name = f'/agent{agent_id:03d}/radio_broadcast'
            self.agent_publishers[agent_id] = self.create_publisher(RadioBroadcast, topic_name, 10)
        self.get_logger().info('RadioCommSimulator node is up.')

    def start_broadcast_service(self):
        ## Setup broadcast server & reachable_agents request 
        self.radio_broadcast_service = self.create_service(RadioBroadcastService, 'radio_broadcast_request', self.handle_radio_broadcast_request)
        self.reachable_agents_client = self.create_client(ReachableAgentsService, 'reachable_agents_request')

    def handle_radio_broadcast_request(self, request, response):
        reachable_request = ReachableAgentsService.Request()
        reachable_request.sent_agent = request.radio_broadcast.sent_agent

        while not self.reachable_agents_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for reachable_agents service...')

        # Store current broadcast request to use in callback
        self.current_radio_broadcast_request = request.radio_broadcast

        # Call reachable_agents service and handle response asynchronously
        future = self.reachable_agents_client.call_async(reachable_request)
        future.add_done_callback(self.handle_reachable_agents_response)

        # Set response as True immediately to acknowledge request
        response.success = True
        return response

    def handle_reachable_agents_response(self, future):
        try:
            result = future.result()
            if result is not None and result.success:
                reachable_agents = result.reachable_agents
                prob_success = result.prob_success
                self.get_logger().info(f'Reachable agents obtained: {reachable_agents} with probabilities: {prob_success}')
                # Publishing to reachable agents
                self.publish_to_reachable_agents(reachable_agents, self.current_radio_broadcast_request)
            else:
                self.get_logger().error('Failed to get reachable agents or no agents are reachable.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def publish_to_reachable_agents(self, reachable_agents, radio_broadcast):
        for agent_id in reachable_agents:
            if agent_id in self.agent_publishers:
                msg = radio_broadcast
                self.agent_publishers[agent_id].publish(msg)
                self.get_logger().info(f'Radio broadcast message sent to agent {agent_id}')


def main(args=None):
    rclpy.init(args=args)
    radiocomm_sim_node = RadioCommSimulator()
    rclpy.spin(radiocomm_sim_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
