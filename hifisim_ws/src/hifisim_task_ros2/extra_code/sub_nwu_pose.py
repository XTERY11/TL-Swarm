import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class NwuPoseSubscriber(Node):
    def __init__(self):
        super().__init__('nwu_pose_subscriber')
        agent_id = 1
        topic_name = f'/agent{agent_id:03d}/global/nwu_pose'
        self.create_subscription(
            PoseStamped,
            topic_name,
            self.pose_callback,
            20
        )

    def pose_callback(self, msg):
        current_position = msg.pose.position
        self.get_logger().info(
            f"{msg.header.stamp.sec} "
            f"Position = ({current_position.x:7.2f}, {current_position.y:7.2f}, {current_position.z:7.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = NwuPoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
