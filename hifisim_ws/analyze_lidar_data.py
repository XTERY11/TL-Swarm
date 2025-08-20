#!/usr/bin/env python3
"""
LiDAR Data Analysis Tool
分析激光雷达点云数据的特征和统计信息
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import struct
from datetime import datetime
import os

class LidarDataAnalyzer:
    def __init__(self, data_path):
        self.data_path = data_path
        self.data = []
        self.points_3d = []
        
    def load_json_data(self):
        """从JSON文件加载激光雷达数据"""
        if os.path.isdir(self.data_path):
            # 如果是目录，加载所有JSON文件
            json_files = list(Path(self.data_path).glob("*.json"))
            for json_file in json_files:
                with open(json_file, 'r') as f:
                    frame_data = json.load(f)
                    if isinstance(frame_data, list):
                        self.data.extend(frame_data)
                    else:
                        self.data.append(frame_data)
        else:
            # 如果是单个文件
            with open(self.data_path, 'r') as f:
                self.data = json.load(f)
        
        print(f"加载了 {len(self.data)} 帧数据")
        
        # 提取所有3D点
        for frame in self.data:
            if 'points' in frame and frame['points']:
                self.points_3d.extend(frame['points'])
        
        self.points_3d = np.array(self.points_3d)
        print(f"总点数: {len(self.points_3d)}")
    
    def load_bag_data(self):
        """从rosbag2加载激光雷达数据"""
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=self.data_path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', '')
            )
            
            while reader.has_next():
                topic, data, timestamp = reader.read_next()
                if topic == '/agent002/lidar01':
                    msg = deserialize_message(data, PointCloud2)
                    points = self.parse_pointcloud2(msg)
                    
                    frame_data = {
                        'timestamp': timestamp,
                        'point_count': len(points),
                        'frame_id': msg.header.frame_id,
                        'points': points.tolist() if len(points) > 0 else []
                    }
                    self.data.append(frame_data)
                    
                    if len(points) > 0:
                        self.points_3d.extend(points)
            
            self.points_3d = np.array(self.points_3d)
            print(f"从bag文件加载了 {len(self.data)} 帧数据")
            print(f"总点数: {len(self.points_3d)}")
            
        except Exception as e:
            print(f"加载bag文件失败: {e}")
            print("尝试加载JSON文件...")
            self.load_json_data()
    
    def parse_pointcloud2(self, msg):
        """解析PointCloud2消息"""
        points = []
        data = msg.data
        point_step = msg.point_step
        
        for i in range(0, len(data), point_step):
            point_data = data[i:i+point_step]
            if len(point_data) >= 12:  # 3 * 4 bytes for x,y,z
                x, y, z = struct.unpack('fff', point_data[:12])
                points.append([x, y, z])
        
        return np.array(points)
    
    def analyze_point_cloud_characteristics(self):
        """分析点云特征"""
        if len(self.points_3d) == 0:
            print("没有点云数据可分析")
            return
        
        print("\n" + "="*60)
        print("点云特征分析报告")
        print("="*60)
        
        # 基本统计
        print(f"总帧数: {len(self.data)}")
        print(f"总点数: {len(self.points_3d):,}")
        print(f"平均每帧点数: {len(self.points_3d)/len(self.data):.0f}")
        
        # 坐标范围
        x_min, x_max = np.min(self.points_3d[:, 0]), np.max(self.points_3d[:, 0])
        y_min, y_max = np.min(self.points_3d[:, 1]), np.max(self.points_3d[:, 1])
        z_min, z_max = np.min(self.points_3d[:, 2]), np.max(self.points_3d[:, 2])
        
        print(f"\n坐标范围:")
        print(f"  X: [{x_min:.2f}, {x_max:.2f}] 范围: {x_max-x_min:.2f}m")
        print(f"  Y: [{y_min:.2f}, {y_max:.2f}] 范围: {y_max-y_min:.2f}m")
        print(f"  Z: [{z_min:.2f}, {z_max:.2f}] 范围: {z_max-z_min:.2f}m")
        
        # 距离分析
        distances = np.sqrt(self.points_3d[:, 0]**2 + self.points_3d[:, 1]**2 + self.points_3d[:, 2]**2)
        print(f"\n距离统计:")
        print(f"  最小距离: {np.min(distances):.2f}m")
        print(f"  最大距离: {np.max(distances):.2f}m")
        print(f"  平均距离: {np.mean(distances):.2f}m")
        print(f"  中位距离: {np.median(distances):.2f}m")
        
        # 点云密度分析
        print(f"\n点云密度分析:")
        print(f"  点云体积: {(x_max-x_min)*(y_max-y_min)*(z_max-z_min):.2f}m³")
        print(f"  点密度: {len(self.points_3d)/((x_max-x_min)*(y_max-y_min)*(z_max-z_min)):.0f} points/m³")
        
        # 时间分析
        if self.data and 'timestamp' in self.data[0]:
            timestamps = [frame['timestamp'] for frame in self.data]
            duration = max(timestamps) - min(timestamps)
            print(f"\n时间分析:")
            print(f"  总时长: {duration:.2f}秒")
            print(f"  平均帧率: {len(self.data)/duration:.2f}Hz")
        
        # 激光雷达类型判断
        self.identify_lidar_type()
        
        return {
            'total_frames': len(self.data),
            'total_points': len(self.points_3d),
            'avg_points_per_frame': len(self.points_3d)/len(self.data),
            'coordinate_range': {
                'x': [x_min, x_max],
                'y': [y_min, y_max], 
                'z': [z_min, z_max]
            },
            'distance_stats': {
                'min': np.min(distances),
                'max': np.max(distances),
                'mean': np.mean(distances),
                'median': np.median(distances)
            }
        }
    
    def identify_lidar_type(self):
        """识别激光雷达类型"""
        print(f"\n激光雷达类型分析:")
        
        # 分析点云分布特征
        if len(self.points_3d) > 0:
            # 计算点云的空间分布
            x_std = np.std(self.points_3d[:, 0])
            y_std = np.std(self.points_3d[:, 1])
            z_std = np.std(self.points_3d[:, 2])
            
            # 计算点云密度变化
            frame_points = [frame['point_count'] for frame in self.data]
            point_variance = np.var(frame_points)
            
            print(f"  点云分布标准差: X={x_std:.2f}, Y={y_std:.2f}, Z={z_std:.2f}")
            print(f"  帧间点数方差: {point_variance:.0f}")
            
            # 根据特征判断类型
            if point_variance > 1000:  # 点数变化大
                print(f"  → 判断为: Livox Mid-360 (非重复扫描模式)")
                print(f"    特征: 点云密度不均匀，非重复扫描模式")
            else:
                print(f"  → 判断为: 传统机械旋转式LiDAR")
                print(f"    特征: 点云密度相对均匀")
    
    def analyze_point_density(self):
        """分析点云密度分布"""
        if len(self.data) == 0:
            return
            
        point_counts = [frame['point_count'] for frame in self.data]
        
        plt.figure(figsize=(15, 10))
        
        # 1. 点云数量随时间变化
        plt.subplot(2, 3, 1)
        plt.plot(point_counts)
        plt.title('Point Cloud Density Over Time')
        plt.xlabel('Frame')
        plt.ylabel('Point Count')
        plt.grid(True)
        
        # 2. 点云数量分布直方图
        plt.subplot(2, 3, 2)
        plt.hist(point_counts, bins=30, alpha=0.7)
        plt.title('Point Count Distribution')
        plt.xlabel('Point Count')
        plt.ylabel('Frequency')
        plt.grid(True)
        
        # 3. 距离分布
        if len(self.points_3d) > 0:
            distances = np.sqrt(self.points_3d[:, 0]**2 + self.points_3d[:, 1]**2 + self.points_3d[:, 2]**2)
            plt.subplot(2, 3, 3)
            plt.hist(distances, bins=50, alpha=0.7)
            plt.title('Distance Distribution')
            plt.xlabel('Distance (m)')
            plt.ylabel('Frequency')
            plt.grid(True)
        
        # 4. 3D点云散点图 (采样)
        if len(self.points_3d) > 0:
            sample_size = min(10000, len(self.points_3d))
            sample_indices = np.random.choice(len(self.points_3d), sample_size, replace=False)
            sample_points = self.points_3d[sample_indices]
            
            ax = plt.subplot(2, 3, 4, projection='3d')
            scatter = ax.scatter(sample_points[:, 0], sample_points[:, 1], sample_points[:, 2], 
                       c=sample_points[:, 2], cmap='viridis', alpha=0.6, s=0.1)
            ax.set_title('3D Point Cloud (Sampled)')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            plt.colorbar(scatter, ax=ax, label='Z (m)')
        
        # 5. XY平面投影
        if len(self.points_3d) > 0:
            plt.subplot(2, 3, 5)
            plt.scatter(self.points_3d[:, 0], self.points_3d[:, 1], 
                       c=self.points_3d[:, 2], cmap='viridis', alpha=0.6, s=0.1)
            plt.title('XY Projection')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.colorbar(label='Z (m)')
            plt.axis('equal')
        
        # 6. 时间序列分析
        if self.data and 'timestamp' in self.data[0]:
            timestamps = [frame['timestamp'] for frame in self.data]
            plt.subplot(2, 3, 6)
            plt.plot(timestamps, point_counts)
            plt.title('Point Count vs Time')
            plt.xlabel('Timestamp (s)')
            plt.ylabel('Point Count')
            plt.grid(True)
        
        plt.tight_layout()
        plt.savefig('lidar_analysis_detailed.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        print(f"\n分析图表已保存为: lidar_analysis_detailed.png")
    
    def generate_readme(self, analysis_results):
        """生成详细的README文档"""
        readme_content = f"""
# LiDAR点云数据分析报告

## 实验信息
- **数据来源**: HiFi Simulator TestCase1
- **激光雷达型号**: Livox Mid-360
- **分析时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
- **数据路径**: {self.data_path}

## 数据统计
- **总帧数**: {analysis_results['total_frames']}
- **总点数**: {analysis_results['total_points']:,}
- **平均每帧点数**: {analysis_results['avg_points_per_frame']:.0f}

## 坐标范围
- **X轴**: [{analysis_results['coordinate_range']['x'][0]:.2f}, {analysis_results['coordinate_range']['x'][1]:.2f}] m
- **Y轴**: [{analysis_results['coordinate_range']['y'][0]:.2f}, {analysis_results['coordinate_range']['y'][1]:.2f}] m  
- **Z轴**: [{analysis_results['coordinate_range']['z'][0]:.2f}, {analysis_results['coordinate_range']['z'][1]:.2f}] m

## 距离统计
- **最小距离**: {analysis_results['distance_stats']['min']:.2f} m
- **最大距离**: {analysis_results['distance_stats']['max']:.2f} m
- **平均距离**: {analysis_results['distance_stats']['mean']:.2f} m
- **中位距离**: {analysis_results['distance_stats']['median']:.2f} m

## 激光雷达特征分析

### Livox Mid-360特点
1. **扫描模式**: 非重复扫描 (Non-repetitive scanning)
2. **水平FOV**: 360°
3. **垂直FOV**: 59层 (-7° ~ +52°)
4. **点云密度**: 不均匀分布
5. **数据格式**: PointCloud2 (ROS标准)

### 与其他激光雷达对比
| 特征 | Livox Mid-360 | Velodyne VLP-16 | Ouster OS1-64 |
|------|---------------|-----------------|---------------|
| 扫描模式 | 非重复 | 重复 | 重复 |
| 点云密度 | 不均匀 | 均匀 | 均匀 |
| 数据格式 | PointCloud2 | PointCloud2 | PointCloud2 |
| 适用场景 | 避障/建图 | 导航/定位 | 高精度建图 |

## 数据格式说明

### PointCloud2消息结构
```python
{
    "timestamp": float,      # 时间戳
    "frame_id": "map",       # 坐标系
    "point_count": int,      # 点数
    "points": [              # 3D点数组
        [x, y, z],           # 每个点的坐标
        ...
    ]
}
```

### 坐标系说明
- **坐标系**: NWU_FLU (North-West-Up, Front-Left-Up)
- **原点**: UAV当前位置
- **单位**: 米 (m)

## 避障算法兼容性

### 推荐算法
1. **VoxelGrid滤波**: 适用于降采样
2. **RANSAC平面分割**: 地面检测
3. **Euclidean聚类**: 障碍物分割
4. **Octree**: 空间索引

### 注意事项
1. **点云密度不均匀**: 需要自适应滤波
2. **非重复扫描**: 时间同步要求高
3. **数据格式**: 标准PointCloud2，兼容性好

## 使用建议
1. 使用PCL库进行点云处理
2. 考虑点云密度变化进行自适应处理
3. 结合IMU数据进行运动补偿
4. 使用多帧融合提高稳定性
"""
        
        with open('lidar_analysis_readme.md', 'w', encoding='utf-8') as f:
            f.write(readme_content)
        
        print(f"\nREADME文档已生成: lidar_analysis_readme.md")

def main():
    # 使用当前录制的数据
    data_path = "/home/ctx/hifisim_ws/lidar_data_20250806_204203/"
    
    analyzer = LidarDataAnalyzer(data_path)
    
    # 加载数据
    analyzer.load_json_data()
    
    # 分析点云特征
    results = analyzer.analyze_point_cloud_characteristics()
    
    # 生成可视化图表
    analyzer.analyze_point_density()
    
    # 生成README文档
    analyzer.generate_readme(results)
    
    print(f"\n分析完成！请查看:")
    print(f"1. lidar_analysis_detailed.png - 可视化图表")
    print(f"2. lidar_analysis_readme.md - 详细报告")

if __name__ == '__main__':
    main()