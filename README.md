### HiFiSim ↔ ROS 2 ↔ Ego-Planner 集成

> 目标：跑通单机任务
> note: 只用了终端最小命令和短名参数没有launch

---

### 1. QuickStart（依赖 → 编译 → 阶段1~3一页跑通）

#### 1.1 依赖
- **ROS 2**: 建议与本机一致的发行版（Ubuntu 22.04 + `ros2` humble ）
- **Python**: 3.10
- **CMake/colcon**: 工作区构建
- **RViz2**: 与 ROS 2 发行版匹配
- **Unity（仿真）**:
  - 包含 ROS TCP Connector/Endpoint 的构建，或使用提供的 Unity 可执行（如单独链接）
  - 输出话题：`/agent001/global/sim_nwu_pose`（PoseStamped）、`/agent001/lidar01`（PointCloud2）
  - EnvScenario JSON（示例）：`/home/ctx/Unity/build/hifi_simulator_unity_Data/StreamingAssets/AppData/EnvScenarios/Env1Scenario1.json`
- 建议：`rosdep`（自动安装依赖）、`traj_utils`、`tf2_ros` 等随本工作区一并构建

#### 1.2 获取与编译
```bash
cd /home/ctx/hifisim_ws
# 依赖安装（可选，但推荐）
rosdep install --from-paths src -y --ignore-src
# 构建
colcon build
# 环境
source install/setup.bash
```

#### 1.3 quicktest（单机，agent001 ↔ drone_0）
> 全程使用“短名参数”，RViz Fixed Frame 统一为 `world`。

- 终端A：位姿 → 里程计
```bash
python3 /home/ctx/hifisim_ws/pose_to_odom_adapter.py \
  --in /agent001/global/sim_nwu_pose \
  --out /drone_0/odom \
  --frame world --child base_link
```

- 终端B：点云中转（若 Unity 未直发 `/drone_0/cloud`）
```bash
python3 /home/ctx/hifisim_ws/cloud_relay.py \
  --in /agent001/lidar01 \
  --out /drone_0/cloud \
  --pose /agent001/global/sim_nwu_pose
```

- 终端C：Ego-Planner（短名参数）
```bash
ros2 launch ego_planner advanced_param.launch.py \
  drone_id:=0 odometry_topic:=odom cloud_topic:=cloud odom_world:=odom \
  flight_type:=1 planning_horizon:=5.0
```

- 终端D：发布一个测试目标（阶段2最小验证）
```bash
ros2 topic pub -1 /move_base_simple/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: world}, pose: {position: {x: 10.0, y: 5.0, z: 2.5}, orientation: {w: 1.0}}}"
```

- 阶段3（单机避障闭环，按顺序执行）
```bash
# 起飞控制（pbtm）
nohup ros2 run pbtm_ros2 pbtm_ros2_node --ros-args -r __ns:=/agent001 -r __node:=pbtm_node \
  -p agent_id:=agent001 -p takeoff_height:=5.0 -p send_command_rate:=20.0 \
  -p timeout:=5.0 -p height_range:='[0.5, 20.0]' >/tmp/pbtm.log 2>&1 & echo $!
sleep 1
ros2 topic info -v /agent001/trajectory/points | cat  # 期望：Sub=1

# 目标过滤/偏移（raw → filtered）
nohup python3 /home/ctx/hifisim_ws/tools/goal_filter_offset.py \
  --node goal_filter_offset_v2 \
  --in /move_base_simple/goal_raw --out /move_base_simple/goal \
  --odom /drone_0/odom --backoff 2.5 --z_min 1.8 --z_max 8.0 --min_interval 0.4 \
  >/tmp/goal_filter.log 2>&1 & echo $!

# 任务：Signpost→目标序列
nohup python3 /home/ctx/hifisim_ws/tools/signpost_mission_runner.py \
  --json /home/ctx/Unity/build/hifi_simulator_unity_Data/StreamingAssets/AppData/EnvScenarios/Env1Scenario1.json \
  --pose /agent001/global/sim_nwu_pose --goal /move_base_simple/goal_raw \
  --alt 2.0 --pre 2.0 --post 2.0 --reach 2.5 --filter Signpost_ --limit 3 \
  >/tmp/signpost_runner.log 2>&1 & echo $!

# 控制桥：Bspline → JointTrajectory（驱动 Unity）
nohup python3 /home/ctx/hifisim_ws/tools/controller_adapter_fixed.py \
  --agent_id 1 --hz 20 --horizon 4.0 --mission_code 3 \
  --z_mode clamp --z_min 1.8 --z_max 8.0 --drive_mode jt \
  >/tmp/ctrl_1.log 2>&1 & echo $!
```

- 监听
```bash
ros2 topic info -v /move_base_simple/goal_raw | cat   # Pub=1(signpost), Sub=1(goal_filter)
ros2 topic info -v /move_base_simple/goal | cat       # Pub=1(goal_filter), Sub=1(planner)
ros2 topic info -v /drone_0_planning/bspline | cat    # Pub=1(planner), Sub=1(controller_adapter)
ros2 topic hz /drone_0_planning/bspline | head -n 20 | cat
ros2 topic hz /agent001/trajectory/points | head -n 20 | cat  # ≈20Hz
```

- RViz2观测配置
  - **Fixed Frame**=`world`
  - 添加 PointCloud2：`/drone_0/cloud`（Reliability=Best Effort）
  - 添加 Odometry：`/drone_0/odom`
  - 可选：`ros2 launch hifisim_task_ros2 view_mapping.launch.py` 加载 `rviz/phase3_env1.rviz`

---



### 2. 建议阅读文档
- **阶段文档**：
  - 阶段1：`repo/phase_design/ex1.md`
  - 阶段2：`repo/phase_design/ex2.md`
  - 阶段3：`repo/phase_design/ex3.md`
- **工具/适配**：
  - 位姿→里程计：`pose_to_odom_adapter.py`
  - 点云中转：`cloud_relay.py`
  - 目标链路：`tools/goal_filter_offset.py`、`tools/signpost_mission_runner.py`
  - 控制桥：`tools/controller_adapter_fixed.py`
- **可视化**：
  - RViz 启动：`src/hifisim_task_ros2/launch/view_mapping.launch.py`
  - RViz 配置：`rviz/phase3_env1.rviz`
- **其他参考**：
  - LiDAR 说明：`LiDAR_PointCloud_README.md`
  - PBTM 适配说明：`pbtm_controller_adapter.md`

---

### HiFiSim ↔ ROS 2 ↔ Ego-Planner 集成（顶层 README｜第2部分）

> 本部分补充系统数据流/话题、排错清单、改动索引（按文件分组）、上传与复现、FAQ。

---

### 4. 系统数据流与话题（含 frame/QoS）
- **Unity → 适配层（阶段1）**
  - `/agent001/global/sim_nwu_pose` (PoseStamped, NWU, QoS≈SensorData)
    → `pose_to_odom_adapter.py` → `/drone_0/odom` (Odometry, header.frame_id=`world`, QoS=SensorData)
  - `/agent001/lidar01` (PointCloud2)
    → `cloud_relay.py`（可选 `--pose` 变换；否则强制 `frame_id=world`）→ `/drone_0/cloud` (PointCloud2, `world`, 推荐 QoS=SensorData)
- **目标输入（阶段2）**
  - Signpost/plane（JSON）→ 手工/工具生成 `PoseStamped` →
    - 原始目标：`/move_base_simple/goal_raw` (PoseStamped, frame=`world`)
    - 过滤/偏移：`tools/goal_filter_offset.py` → `/move_base_simple/goal` (PoseStamped, `world`)
  - Planner 订阅：`/move_base_simple/goal` + `/drone_0/odom` + `/drone_0/cloud`
  - Planner 输出：`/drone_0_planning/bspline` (traj_utils/Bspline)
- **执行（阶段3）**
  - `tools/controller_adapter_fixed.py`：`/drone_0_planning/bspline` → `/agent001/trajectory/points` (JointTrajectory, ≈20Hz)
  - pbtm 控制节点订阅 `/agent001/trajectory/points` 驱动 Unity
- **地图/可视化**
  - `/drone_0_grid/grid_map/occupancy_inflate`：宽高>0 表示在更新；RViz Fixed Frame=`world`

---

### 5. 推荐排错清单
- **有计数但点云不可见/占据为0**
  - 检查 `/drone_0/cloud`：`header.frame_id=world`、`fields` 含 `x,y,z`、`width*height>0`；RViz Reliability=Best Effort、Fixed Frame=`world`
  - 需要时在 `cloud_relay.py` 加 `--pose /agent001/global/sim_nwu_pose`
- **发布目标无轨迹**
  - 目标 `header.frame_id` 必须是 `world`；`ros2 topic info -v /move_base_simple/goal` 应有 Planner 订阅=1
  - Planner 启动必须用短名参数：`odometry_topic:=odom cloud_topic:=cloud odom_world:=odom`
- **命名空间重复（`/drone_0_/drone_0_*`）**
  - 只传短名（`odom/cloud/odom`），不要传全路径或 remap `--ros-args -r`
- **有 bspline 无运动**
  - 确认 `controller_adapter_fixed.py` 与 pbtm 在流：`/agent001/trajectory/points` ≈20Hz；订阅计数匹配
- **目标贴墙/抖动**
  - `goal_filter_offset.py`：增大 `--backoff`、设置 `--min_interval`；用 `--z_min/--z_max`
- **占据参数不合适**
  - `advanced_param.launch.py` 中调：`resolution_`、`local_update_range_*`、`obstacles_inflation_`、`virtual_ceil_height_`（已有默认）

---

### 6. 改动索引
- **适配层**
  - `pose_to_odom_adapter.py`：QoS=SensorData；首帧后 ~20Hz 重发；`--frame world --child base_link`
  - `cloud_relay.py`：点云中转；可选 `--pose` 变换；无变换时强制 `frame_id=world`
- **任务/目标工具（阶段3）**
  - `tools/goal_filter_offset.py`：`goal_raw`→`goal` 回退/去抖/高度夹紧
  - `tools/signpost_mission_runner.py`：解析 EnvScenario Signpost，生成 pre/pass/post 序列并推进
  - `tools/controller_adapter_fixed.py`：Bspline 采样→JointTrajectory（或 PoseStamped）
- **规划器启动（Ego-Planner）**
  - `src/planner/plan_manage/launch/advanced_param.launch.py`：统一短名 remap；补齐 grid_map 参数（`resolution_`、`local_update_range_*`、`obstacles_inflation_`、`virtual_ceil_height_`、`frame_id=world`）与地图尺寸等
  - 核心算法未改，仅 launch/参数与 remap 调整
- **仿真/可视化辅助**
  - `src/hifisim_task_ros2/launch/hifi_simulator_launch.py`：拉起 Unity 与 TCP endpoint（未改 Unity 内核）
  - `src/hifisim_task_ros2/launch/npbtm_ros2_launch.py`：可多实例 pbtm（单机按需）
  - `src/hifisim_task_ros2/launch/view_mapping.launch.py`：一键加载 `rviz/phase3_env1.rviz`
  - `src/hifisim_task_ros2/setup.py`：安装资源/rviz 配置
- **未改内核声明**
  - Ego-Planner 核心包未修改；Unity 内核未修改（只通过话题交互）


