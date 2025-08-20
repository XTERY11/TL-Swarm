#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from transforms3d.euler import euler2quat


class MockUnityPublisher(Node):
    """模拟Unity发布的传感器数据"""

    def __init__(self):
        super().__init__('mock_unity_publisher')

        # 创建发布器
        self.pose_pub = self.create_publisher(
            PoseStamped, '/agent000/global/sim_nwu_pose', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/agent000/lidar01', 10)

        # 创建定时器
        self.create_timer(0.1, self.timer_callback)  # 10Hz

        # 初始化模拟轨迹参数
        self.t = 0.0
        self.radius = 2.0  # 圆形轨迹半径
        self.height = 2.0  # 固定高度
        self.omega = 0.2   # 角速度 rad/s

        self.get_logger().info('Mock Unity Publisher started')
        self.get_logger().info('Publishing: /agent000/global/sim_nwu_pose, /agent000/lidar01')

    def create_point_cloud(self, center_x, center_y, center_z):
        """创建模拟的点云数据"""
        # 创建一个圆柱形点云
        num_points = 1000
        radius = 5.0
        height = 4.0

        # 生成随机点
        angles = np.random.uniform(0, 2*np.pi, num_points)
        rs = np.random.uniform(0, radius, num_points)
        zs = np.random.uniform(-height/2, height/2, num_points)

        # 计算点的坐标
        xs = center_x + rs * np.cos(angles)
        ys = center_y + rs * np.sin(angles)
        zs = center_z + zs

        # 创建点云消息
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        # 设置点云字段
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12  # 3 * float32
        msg.row_step = msg.point_step * num_points
        msg.height = 1
        msg.width = num_points
        msg.is_dense = True

        # 将点云数据打包为字节流
        points = np.zeros(num_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])
        points['x'] = xs
        points['y'] = ys
        points['z'] = zs
        msg.data = points.tobytes()

        return msg

    def timer_callback(self):
        """定时器回调函数，发布模拟数据"""
        now = self.get_clock().now()

        # 生成圆形轨迹上的位置
        x = self.radius * np.cos(self.omega * self.t)  # North
        y = self.radius * np.sin(self.omega * self.t)  # West
        z = self.height                                # Up

        # 计算切向方向作为偏航角
        yaw = self.omega * self.t + np.pi/2

        # 创建PoseStamped消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = 'world'

        # 设置位置
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # 设置姿态（使用偏航角）
        q = euler2quat(0, 0, yaw)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        # 发布位姿
        self.pose_pub.publish(pose_msg)

        # 创建并发布点云
        cloud_msg = self.create_point_cloud(x, y, z)
        self.cloud_pub.publish(cloud_msg)

        # 更新时间
        self.t += 0.1  # 10Hz的更新率

        # 调试输出
        self.get_logger().debug(
            f'Published pose at t={self.t:.1f}:\n'
            f'  Position: ({x:.2f}, {y:.2f}, {z:.2f})\n'
            f'  Yaw: {yaw:.2f} rad'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MockUnityPublisher()
    
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