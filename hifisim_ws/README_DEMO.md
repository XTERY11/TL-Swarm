# HiFi Simulator 脚本使用说明

目录：

5. 系统停止 `stop_demo.sh`
6. 核心节点检测 `check_core_nodes.sh`
7. 数据录制 `record_hifisim_data.sh`
8. 图像导出 `picview_hifi_camera.py`

---

## 5 `stop_demo.sh`
安全停止所有 Unity/ROS2 相关进程。
```bash
./stop_demo.sh
```

---

## 6 `check_core_nodes.sh`
等待 /UnityEndpoint、/performance_node、/radio_comm_simulator 三个核心节点全部在线后才继续执行后续脚本或操作。

```bash
bash check_core_nodes.sh
```

脚本会循环查询 `ros2 node list`，打印上线信息；常用于其他脚本的前置检查。

---

## 7 `record_hifisim_data.sh`
自动根据 **在线 topic** 组装列表并录制 rosbag2，涵盖 UAV 姿态 / 碰撞 / 广播、全部相机 `Image` 话题与点云 `PointCloud2`，同时记录 service 交互。

| 位置参数 | 含义 | 默认值 |
|----------|------|--------|
| $1 | UAV 数量 | 2 |
| $2 | 输出目录 | `<时间戳>_hifisim_bag` |

脚本流程：  
1) 复用 `check_core_nodes.sh` 等待核心节点就绪  
2) 动态扫描 ROS graph 拼装 `TOPICS` 数组并去重  
3) 执行 `ros2 bag record --include-services --include-hidden-topics -o <DIR> ${TOPICS[@]}`  

示例：
```bash
# 2 架 UAV，输出目录自动按时间戳命名
bash record_hifisim_data.sh

# 4 架 UAV，指定目录 my_demo_bag
bash record_hifisim_data.sh 4 my_demo_bag
```

---

## 8 `picview_hifi_camera.py`
将 `rosbag2` 中的相机 `sensor_msgs/msg/Image` 话题批量导出为 PNG 图片，便于离线视觉算法评估。

关键参数：
* `bag_dir` — 待解析的 bag2 目录（脚本内硬编码，可按需修改）  
* `topics`  — 待导出的相机话题 → 输出文件夹映射  

使用方法：
```bash
cd ~/hifisim_ws
# 如需修改 bag 源目录或话题，请先编辑脚本顶部变量
python3 picview_hifi_camera.py
```

脚本将为每个相机话题创建独立文件夹（如 `imgs_agent002_left`），并保存为 `000000.png` 起递增编号。若 bag 中帧数较大，请确保磁盘空间充足。

<!-- ---

### 依赖提示
* **jq**    ：脚本用于解析 TestCase JSON（自动推断无人机数量 / 环境 / 无线电）。
  ```bash
  sudo apt-get install jq
  ```
* **expect**：`interactive_control.sh` 自动输入 TestCase ID；若未安装仍可手动键入。
* 运行脚本前请确认执行权限：`chmod +x *.sh`。 -->

