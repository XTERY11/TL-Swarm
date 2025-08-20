import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.msg import RadioBroadcast, ObjsMeasurement
from hifisim_msg_ros2.srv import RadioBroadcastService
import sys
import threading

class Agent(Node):
    def __init__(self, agent_id):
        super().__init__(f'agent_{agent_id:03d}')
        self.agent_id = agent_id
        self.radio_broadcast_client = self.create_client(RadioBroadcastService, 'radio_broadcast_request')
        self.subscription = self.create_subscription(
            RadioBroadcast,
            f'/agent{self.agent_id:03d}/radio_broadcast',
            self.receive_radio_broadcast_callback,
            10
        )
        self.objs_measurement_subscription = self.create_subscription(
            ObjsMeasurement,
            f'/agent{self.agent_id:03d}/camera01/sim_objs_measurement',
            self.objs_measurement_callback,
            10
        )
        self.get_logger().info(f'Agent {self.agent_id} node is up.')
        # Start user prompt in a separate thread
        threading.Thread(target=self.user_prompt_loop, daemon=True).start()
        
    def send_radio_broadcast_request(self, broadcast_msg):
        request = RadioBroadcastService.Request()
        request.radio_broadcast.sent_agent = self.agent_id
        request.radio_broadcast.broadcast_msg = broadcast_msg[:1500]  # Ensure no more than 1500 bytes

        while not self.radio_broadcast_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for radio_comm_simulator service...')

        future = self.radio_broadcast_client.call_async(request)
        future.add_done_callback(self.handle_radio_broadcast_response)

    def handle_radio_broadcast_response(self, future):
        try:
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info('Radio broadcast request sent successfully.')
            else:
                self.get_logger().error('Failed to send radio broadcast request.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def receive_radio_broadcast_callback(self, msg):
        self.get_logger().info(f'Received radio broadcast message from agent {msg.sent_agent}: {list(msg.broadcast_msg)}')

    def objs_measurement_callback(self, msg):
        self.get_logger().info(f'Received objs_measurement message:')
        self.get_logger().info(f'  Timestamp: {msg.stamp.sec}.{msg.stamp.nanosec}')
        self.get_logger().info(f'  Sensor ID: {msg.sensor_id}')
        self.get_logger().info(f'  Frame ID: {msg.frame_id}')
        self.get_logger().info(f'  Ground FOV Points:')
        for i, point in enumerate(msg.ground_fov):
            self.get_logger().info(f'    Point {i+1}: x={point.x}, y={point.y}, z={point.z}')
        self.get_logger().info(f'  Objects Measured:')
        for i, obj in enumerate(msg.objs_measured):
            self.get_logger().info(f'    Object {i+1}:')
            self.get_logger().info(f'      Class ID: {obj.class_id}')
            self.get_logger().info(f'      Class Str: {obj.class_str}')
            self.get_logger().info(f'      Confidence: {obj.confidence}')
            self.get_logger().info(f'      x_center: {obj.x_center}, y_center: {obj.y_center}, z_center: {obj.z_center}')
            self.get_logger().info(f'      Box Width: {obj.box_width}, Box Height: {obj.box_height}')

    def user_prompt_loop(self):
        while True:
            user_input = input('Do you want to send a radio broadcast request? (y/n): ')
            if user_input.lower() == 'y':
                self.send_radio_broadcast_request(bytearray([100, 200]))


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print('Usage: ros2 run hifisim_task_ros2 sim_agent <agent_id>')
        return
    agent_id = int(sys.argv[1])
    node = Agent(agent_id=agent_id)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
