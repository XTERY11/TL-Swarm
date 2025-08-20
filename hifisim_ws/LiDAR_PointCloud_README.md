# Unity默认激光雷达Livox Mid-360  
    LiDAR点云分析

## 实验概述


**激光雷达型号**: Livox Mid-360  
**数据格式**: ROS2 PointCloud2  
**坐标系**: NWU_FLU (North-West-Up, Front-Left-Up)  


## 激光雷达传感器详解

### Livox Mid-360 技术规格

| 参数 | 数值 | 说明 |
|------|------|------|
| **扫描模式** | 非重复扫描 | Non-repetitive scanning pattern |
| **水平FOV** | 360° | 全周扫描 |
| **垂直FOV** | 59层 | -7° ~ +52° |
| **最大测距** | 70m | 有效测量距离 |
| **水平分辨率** | 0.1° | 水平角度步长 |
| **点云分辨率** | 1.0m | 空间分辨率 |
| **近盲区** | 0.1m | 最小测量距离 |
| **安装位置** | 向上 | Upward mounting |
| **发布频率** | 5Hz | 数据更新频率 |

### 点云特征分析结果

**数据统计**:
- 总帧数: 158帧
- 总点数: 3,355,920个点
- 平均每帧点数: 21,240个点
- 总时长: 40.40秒
- 实际帧率: 3.91Hz

**空间范围**:
- X轴: [-70.18, 70.37] m (范围: 140.55m)
- Y轴: [-78.55, 89.89] m (范围: 168.44m)  
- Z轴: [-8.58, 64.40] m (范围: 72.98m)
    测试点云分布空间距离过大，后期处理点云加空间滤波
**距离统计**:
- 最小距离: 0.10m
- 最大距离: 92.26m
- 平均距离: 55.43m
- 中位距离: 61.80m

**点云密度**:
- 点云体积: 1,727,817.96 m³
- 点密度: 2 points/m³ 只是参考 跑的环境大部分是空地，稀疏分布且不均匀

## 激光雷达类型对比

| 特征 | Livox Mid-360 | Velodyne VLP-16 | Ouster OS1-64 | Hesai Pandar64 |
|------|---------------|-----------------|---------------|----------------|
| **扫描模式** | 非重复 | 重复 | 重复 | 重复 |
| **点云密度** | 不均匀 | 均匀 | 均匀 | 均匀 |
| **数据格式** | PointCloud2 | PointCloud2 | PointCloud2 | PointCloud2 |
| **水平FOV** | 360° | 360° | 360° | 360° |
| **垂直FOV** | 59层 | 16层 | 64层 | 64层 |
| **适用场景** | 避障/建图 | 导航/定位 | 高精度建图 | 自动驾驶 |
| **价格** | 中等 | 低 | 高 | 高 |

## 点云数据格式

### ROS2 PointCloud2消息结构

```python
# 消息头
header:
  stamp: {sec: 32, nanosec: 618156433}
  frame_id: "map"

# 点云属性
height: 1
width: 21240
fields:
  - name: "x", offset: 0, datatype: 7, count: 1
  - name: "y", offset: 4, datatype: 7, count: 1  
  - name: "z", offset: 8, datatype: 7, count: 1

# 数据属性
is_bigendian: false
point_step: 12  # 每个点12字节 (3个float32)
row_step: 254880
is_dense: true

# 点云数据 (二进制)
data: [76, 152, 138, 66, ...]
```

### JSON 取1个时间戳

```json
{
  "timestamp": 63.813606262,
  "frame_id": "map",
  "point_count": 21240,
  "points": [
    [69.29745483398438, 19.88652801513672, -8.53084659576416],
    [69.43575286865234, 19.88652801513672, -7.3169846534729],
    ...
  ]
}
```

## 推荐的点云处理库

### 1. PCL (Point Cloud Library) - 主要推荐

**安装**:
```bash
# Ubuntu/Debian
sudo apt-get install libpcl-dev pcl-tools

# ROS2集成
sudo apt-get install ros-humble-pcl-ros
```

**核心功能**:
```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

// 点云滤波
pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);

// 聚类分割
pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor;
cluster_extractor.setClusterTolerance(0.5);
cluster_extractor.setMinClusterSize(100);
```

**适用场景**:
- 点云降采样和滤波
- 地面检测和分割
- 障碍物聚类
- 特征提取

### 2. PCL Render - 可视化

**安装**:
```bash
sudo apt-get install libpcl-visualization-dev
```

**使用示例**:
```cpp
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer::Ptr viewer(
    new pcl::visualization::PCLVisualizer("3D Viewer"));
viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
viewer->spin();
```

### 3. Open3D - 现代点云处理

**安装**:
```bash
pip install open3d
```

**核心功能**:
```python
import open3d as o3d

# 读取点云
pcd = o3d.io.read_point_cloud("pointcloud
## 推荐的点云处理库

### 1. PCL (Point Cloud Library) - 主要推荐

**安装**:
```bash
# Ubuntu/Debian
sudo apt-get install libpcl-dev pcl-tools

# ROS2集成
sudo apt-get install ros-humble-pcl-ros
```

**核心功能**:
```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

// 点云滤波
pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);

// 聚类分割
pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor;
cluster_extractor.setClusterTolerance(0.5);
cluster_extractor.setMinClusterSize(100);
```

**适用场景**:
- 点云降采样和滤波
- 地面检测和分割
- 障碍物聚类
- 特征提取

### 2. PCL Render - 可视化

**安装**:
```bash
sudo apt-get install libpcl-visualization-dev
```

**使用示例**:
```cpp
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer::Ptr viewer(
    new pcl::visualization::PCLVisualizer("3D Viewer"));
viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
viewer->spin();
```

### 3. Open3D - 现代点云处理

**安装**:
```bash
pip install open3d.ply")

# 降采样
downsampled = pcd.voxel_down_sample(voxel_size=0.1)

# 平面分割
plane_model, inliers = pcd.segment_plane(
    distance_threshold=0.01, ransac_n=3, num_iterations=1000)
```

### 4. ROS2点云处理节点

**订阅点云话题**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')
        
        # 订阅激光雷达话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/agent002/lidar01',
            self.point_cloud_callback,
            10
        )
    
    def point_cloud_callback(self, msg):
        # 处理点云数据
        points = self.parse_pointcloud2(msg)
        # 进行滤波、分割等处理
        processed_points = self.process_points(points)
```


## 数据处理流程建议

### 1. 避障流程

```
原始点云 → 降采样 → 滤波 → 地面分割 → 障碍物检测 → 避障决策
```

### 2. 离线分析流程

```
JSON数据 → 数据加载 → 统计分析 → 可视化 → 报告生成
```

## 注意事项

### 1. Livox Mid-360特殊考虑

- **非重复扫描**: 点云密度不均匀，需要自适应处理
- **时间同步**: 需要与IMU数据同步
- **运动补偿**: 考虑UAV运动对点云的影响

### 2. 性能优化

- **降采样**: 使用VoxelGrid减少计算量
- **并行处理**: 利用多线程处理点云
- **内存管理**: 及时释放大型点云对象

### 3. 坐标系转换

- **NWU到FLU**: 注意坐标系转换
- **时间戳对齐**: 确保不同传感器数据时间同步

## 文件结构

```
hifisim_ws/
├── lidar_data_20250806_204203/     # 原始数据
│   ├── lidar_frames_100.json
│   └── lidar_frames_158.json
├── analyze_lidar_data.py           # 分析脚本
├── lidar_monitor.py               # 监听脚本
├── LiDAR_PointCloud_README.md     # 本文档
├── lidar_analysis_detailed.png    # 分析图表
└── lidar_analysis_readme.md       # 分析报告
```

## 快速开始

1. **安装依赖**:
```bash
sudo apt-get install libpcl-dev pcl-tools ros-humble-pcl-ros
pip install open3d numpy matplotlib
```

2. **运行分析**:
```bash
python3 analyze_lidar_data.py
```

3. **实时监听**:
```bash
python3 lidar_monitor.py
```

4. **查看结果**:
- 分析图表: `lidar_analysis_detailed.png`
- 详细报告: `lidar_analysis_readme.md`

---

**作者**: HiFi Simulator 研究团队  
**版本**: v1.0  
**更新时间**: 2024-08-06 