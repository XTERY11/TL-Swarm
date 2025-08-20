#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from builtin_interfaces.msg import Time
import numpy as np
from transforms3d.euler import euler2quat
from transforms3d.quaternions import qmult


class UnityBridgeNode(Node):
    """Republish Unity simulator topics to EGO-Swarm expected names."""

    def __init__(self):
        super().__init__('unity_bridge_node')
        # Parameters
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('odom_in_topic', '')
        self.declare_parameter('cloud_in_topic', '')

        self.drone_id: int = self.get_parameter('drone_id').get_parameter_value().integer_value
        odom_in = self.get_parameter('odom_in_topic').get_parameter_value().string_value
        cloud_in = self.get_parameter('cloud_in_topic').get_parameter_value().string_value

        # Fallback to default patterns if not provided
        if not odom_in:
            odom_in = f'/agent{self.drone_id:03d}/global/sim_nwu_pose'
        if not cloud_in:
            cloud_in = f'/agent{self.drone_id:03d}/lidar{self.drone_id+1:02d}'

        # Outgoing topics (match advanced_param remappings)
        odom_out_grid = f'drone_{self.drone_id}_visual_slam/odom'
        cloud_out = f'drone_{self.drone_id}_cloud'

        # Publishers
        self.odom_pub_grid = self.create_publisher(Odometry, odom_out_grid, 50)
        self.cloud_pub = self.create_publisher(PointCloud2, cloud_out, 10)

        # Subscribers
        self.create_subscription(PoseStamped, odom_in, self.pose_cb, 50)
        self.create_subscription(PointCloud2, cloud_in, self.cloud_cb, 10)

        # 记录上一次的位置和时间，用于计算速度
        self.last_pose = None
        self.last_time = None

        self.get_logger().info(
            f'Bridge for drone {self.drone_id}:\n'
            f'  Pose  {odom_in} -> {odom_out_grid}\n'
            f'  Cloud {cloud_in} -> {cloud_out}')

    def nwu_to_world(self, pose_msg: PoseStamped) -> PoseStamped:
        """将NWU坐标系转换为world坐标系"""
        # NWU到world的旋转：绕Z轴旋转-90度
        q_rot = euler2quat(0, 0, -np.pi/2)
        
        # 获取原始四元数
        q_orig = [
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w
        ]
        
        # 应用旋转
        q_new = qmult(q_rot, q_orig)
        
        # 创建新的消息
        new_msg = PoseStamped()
        new_msg.header = pose_msg.header
        new_msg.header.frame_id = 'world'
        
        # 转换位置
        new_msg.pose.position.x = pose_msg.pose.position.y  # North -> Y
        new_msg.pose.position.y = -pose_msg.pose.position.x  # West -> -X
        new_msg.pose.position.z = pose_msg.pose.position.z  # Up -> Z
        
        # 设置新的方向
        new_msg.pose.orientation.x = q_new[0]
        new_msg.pose.orientation.y = q_new[1]
        new_msg.pose.orientation.z = q_new[2]
        new_msg.pose.orientation.w = q_new[3]
        
        return new_msg

    def estimate_velocity(self, current_pose: PoseStamped, current_time: Time) -> (Vector3, Vector3):
        """估计线速度和角速度"""
        if self.last_pose is None or self.last_time is None:
            self.last_pose = current_pose
            self.last_time = current_time
            return Vector3(), Vector3()

        # 计算时间差
        dt = (current_time.sec - self.last_time.sec) + \
             (current_time.nanosec - self.last_time.nanosec) * 1e-9

        if dt <= 0:
            return Vector3(), Vector3()

        # 计算线速度
        linear = Vector3()
        linear.x = (current_pose.pose.position.x - self.last_pose.pose.position.x) / dt
        linear.y = (current_pose.pose.position.y - self.last_pose.pose.position.y) / dt
        linear.z = (current_pose.pose.position.z - self.last_pose.pose.position.z) / dt

        # 简单起见，角速度设为0
        angular = Vector3()

        # 更新上一次的状态
        self.last_pose = current_pose
        self.last_time = current_time

        return linear, angular

    def pose_cb(self, msg: PoseStamped):
        # 转换坐标系
        world_pose = self.nwu_to_world(msg)
        
        # 估计速度
        linear_vel, angular_vel = self.estimate_velocity(world_pose, msg.header.stamp)

        # Convert PoseStamped to Odometry
        odom_msg = Odometry()
        odom_msg.header = world_pose.header
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = f'drone_{self.drone_id}'
        odom_msg.pose.pose = world_pose.pose
        
        # 设置速度
        odom_msg.twist.twist.linear = linear_vel
        odom_msg.twist.twist.angular = angular_vel

        self.odom_pub_grid.publish(odom_msg)
        
        # 调试输出
        self.get_logger().debug(
            f'Published odometry:\n'
            f'  Position: ({odom_msg.pose.pose.position.x:.2f}, '
            f'{odom_msg.pose.pose.position.y:.2f}, '
            f'{odom_msg.pose.pose.position.z:.2f})\n'
            f'  Velocity: ({odom_msg.twist.twist.linear.x:.2f}, '
            f'{odom_msg.twist.twist.linear.y:.2f}, '
            f'{odom_msg.twist.twist.linear.z:.2f})'
        )

    def cloud_cb(self, msg: PointCloud2):
        # 确保frame_id正确
        msg.header.frame_id = 'world'
        self.cloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UnityBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main() 