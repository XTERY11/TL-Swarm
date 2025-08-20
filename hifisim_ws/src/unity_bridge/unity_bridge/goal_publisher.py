#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np


class GoalPublisher(Node):
    """发布目标点来测试ego_planner的轨迹规划功能"""
    
    def __init__(self):
        super().__init__('goal_publisher')
        
        # 发布到 ego_planner 期望的话题
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        
        # 定时器 - 每15秒发布一个目标点
        self.timer = self.create_timer(15.0, self.publish_goal)
        
        # 目标点序列 - 使用更简单的目标点
        self.goals = [
            [1.0, 0.0, 2.0],   # 前方1米
            [0.0, 1.0, 2.0],   # 右侧1米
            [-1.0, 0.0, 2.0],  # 后方1米
            [0.0, -1.0, 2.0],  # 左侧1米
            [0.0, 0.0, 2.0],   # 回到原点
        ]
        self.current_goal = 0
        
        self.get_logger().info('Goal Publisher started')
        self.get_logger().info('Publishing goals to /move_base_simple/goal every 15 seconds')
    
    def publish_goal(self):
        """发布目标点"""
        if self.current_goal >= len(self.goals):
            self.current_goal = 0  # 循环发布
        
        goal = self.goals[self.current_goal]
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        pose_msg.pose.position.x = goal[0]
        pose_msg.pose.position.y = goal[1]
        pose_msg.pose.position.z = goal[2]
        
        # 简单的四元数（水平飞行）
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        
        self.goal_pub.publish(pose_msg)
        
        self.get_logger().info(
            f'Published goal {self.current_goal + 1}: ({goal[0]:.1f}, {goal[1]:.1f}, {goal[2]:.1f})\n'
            f'  - frame_id: {pose_msg.header.frame_id}\n'
            f'  - stamp: {pose_msg.header.stamp.sec}.{pose_msg.header.stamp.nanosec:09d}'
        )
        
        self.current_goal += 1


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    
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