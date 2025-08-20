# 阶段1 · 文档第一部分

> 说明：本阶段以**终端命令 + 短名参数**为主，目标是建立起来 **Unity ↔ HiFiSim ↔ Ego Planner** 的**数据交互**；不依赖任何脚本。

---

## 1. 目标

### 1.1 最初目标

* Unity ↔ ROS2 **多机仿真**：按 TestCase 的 signpost 顺序飞到终点，完成**静态避障**与**多机不互撞**。
* 采用 **Ego-Planner-Swarm** 路线（局部占据栅格 + MINCO/B 样条）；Unity 提供**真值位姿/点云**；后续可加视觉与通信。

### 1.2 阶段1目标

* 每架机获取**真值位姿**与**点云**；Ego 侧能订阅到对应 `odom/cloud`；并支持 **N 架**扩展。

---

## 2. 阶段1完成内容

### 2.1 适配层（不改 Unity/Ego 内核，适配了两个映射文件，只做了bridge）

* `pose_to_odom_adapter.py`：`/agentXXX/global/sim_nwu_pose` → `/drone_i/odom`（`nav_msgs/Odometry`）

  * **增强**：`SensorData` QoS；收到**首帧后定频重发**（≈20Hz）
* `cloud_relay.py`：`/agentXXX/lidar01` → `/drone_i/cloud`（`sensor_msgs/PointCloud2`）
* **命名对齐**（统一命名空间风格）：

| Unity Agent | ROS 命名空间  |
| ----------- | --------- |
| `agent001`  | `drone_0` |
| `agent002`  | `drone_1` |
| …           | …         |

### 2.2 6 机验证通过（示例）

**sim pose 源检查**

```bash
./check_sim_pose.sh 6
# 期望输出：
# Summary: 6/6 agents have sim_nwu_pose
```

**适配后话题抽样**

```bash
ros2 topic echo --once /drone_0/odom     # 有 Odometry 样本
ros2 topic echo --once /drone_0/cloud    # 有 PointCloud2 样本（检查 fields x,y,z 和 width>0）
```

### 2.3 规划器启动（我使用了短名参数用于终端测试，没写脚本，后续同理）

**单机（drone\_0 命名空间）**，在另一个终端启动：

```bash
ros2 launch ego_planner advanced_param.launch.py \
  drone_id:=0 odometry_topic:=odom cloud_topic:=cloud odom_world:=odom
```

**节点订阅核对**（会自动加命名空间前缀）

```bash
ros2 node info /drone_0_ego_planner_node | sed -n '1,120p'
# 期望看到订阅：
#   ... /drone_0/odom : nav_msgs/msg/Odometry
#   ... /drone_0/cloud: sensor_msgs/msg/PointCloud2
```

> 注：传入**短名参数**（`odom/cloud/odom`）后，包内会在命名空间 `drone_0` 下解析为
> `/drone_0/odom`、`/drone_0/cloud`、`/drone_0/odom`，避免出现 `/drone_0_/drone_0_*` 的重复前缀问题。

---

## 3. 数据通路（阶段1重点）

### 3.1 源头（Unity 仿真）

* 发布**真值位姿**：`/agent001/global/sim_nwu_pose`（NWU 系，Pose/Transform）
* 发布**点云**：`/agent001/lidar01`（`sensor_msgs/PointCloud2`）

### 3.2 适配（HiFiSim 工作区内工具，最小转换）

* **位姿 → 里程计**：`pose_to_odom_adapter.py`

  * 输入：`/agent001/global/sim_nwu_pose`
  * 输出：`/drone_0/odom`（`nav_msgs/Odometry`，`header.frame_id` 建议 用参数`world`后续rviz可视化点云也是world坐标系下）
* **点云中转**：`cloud_relay.py`

  * 输入：`/agent001/lidar01`
  * 输出：`/drone_0/cloud`（`PointCloud2`，`header.frame_id` 与 `odom` 同一世界系，例如 `world`）
  * 可选：`--pose /agent001/global/sim_nwu_pose` 用于**时间与坐标对齐**

### 3.3 消费端（Ego Planner）

* 命名空间：`drone_0`
* 订阅：`/drone_0/odom`、`/drone_0/cloud`（通过短名参数映射）

### 3.4 QoS 与坐标帧

* **QoS**：`SensorData`（BestEffort / Volatile），减少初始无帧阻塞
* **RViz Fixed Frame**：与消息 `header.frame_id` 一致（推荐 `world` 或 `map`）
* **点云字段**：至少包含 `x,y,z`；点数 `width*height > 0` 才能看到

---

## 4. 复测

**终端 A：位姿 → 里程计**

```bash
python3 /home/ctx/hifisim_ws/pose_to_odom_adapter.py \
  --in /agent001/global/sim_nwu_pose \
  --out /drone_0/odom
```

**终端 B：点云中转**

```bash
python3 /home/ctx/hifisim_ws/cloud_relay.py \
  --in /agent001/lidar01 \
  --out /drone_0/cloud \
  --pose /agent001/global/sim_nwu_pose
```

**终端 C：Ego Planner（短名参数）**

```bash
ros2 launch ego_planner advanced_param.launch.py \
  drone_id:=0 odometry_topic:=odom cloud_topic:=cloud odom_world:=odom
```

---

## 5. RViz 最小配置

* **Fixed Frame** 设为 `world`（或与消息 `header.frame_id` 相同）
* 添加 **PointCloud2**：Topic 选 `/drone_0/cloud`；`Reliability = Best Effort`
* 添加 **Odometry**：Topic 选 `/drone_0/odom`

> 若「messages received 但看不到点」：
> ① 检查点云 `width/height > 0`；
> ② 字段包含 `x,y,z`；
> ③ 时间戳与 `frame` 一致。

---

## 6. 遇到的问题与修正

* **源话题识别错误**：`/agentXXX/global/nwu_pose` 为 Unity **订阅端**；发布端应为 `/agentXXX/global/sim_nwu_pose`（已切换）。
* **QoS/静态等待**：初始无帧导致 `echo/hz` 卡住；适配器改为 `SensorData` QoS，且**首帧缓存后定时重发**。
* **多机扩展**：部分 agent 未发 `sim_nwu_pose` → 需**触发仿真运行/起飞**或完善 TestCase 的 AgentGroup；用**批量检测脚本**定位。
* **remap 用法**：避免 `--ros-args -r ...` 造成 “unrecognized arguments”；统一用 **launch 短名参数**（`odom/cloud/odom`）。

---

## 7. 阶段1结论

* **Unity → Ego** 的**位姿/点云通道**已在**多机**下稳定工作；
* Ego Planner 能稳定订阅 `odom/cloud`；RViz 可稳定显示点云与轨迹基础信息。


## 8. 附录：检查清单（复测用）

* [ ] Unity 侧发布 `sim_nwu_pose` / `lidar01`。
* [ ] `pose_to_odom_adapter.py` 正常运行，`/drone_i/odom` 有样本。
* [ ] `cloud_relay.py` 正常运行，`/drone_i/cloud` 有样本（`x,y,z` & `width*height>0`）。
* [ ] `ego_planner` 以**短名参数**启动，无命名空间重复前缀。
* [ ] `ros2 node info` 可见订阅 `/drone_i/odom` 与 `/drone_i/cloud`。
* [ ] RViz：`Fixed Frame` 正确；PointCloud2 设为 `Best Effort`；可视化正常。
* [ ] 6 机示例通过 `./check_sim_pose.sh 6`。
