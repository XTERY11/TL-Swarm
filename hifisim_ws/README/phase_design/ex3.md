
### 阶段3（单机避障闭环）

#### 目标与范围
- 目标：在单机条件下，打通 Unity→ROS 2→Ego-Planner→控制桥→Unity 的闭环链路，利用占据栅格进行静态避障，并在 RViz 中可视化点云、占据与规划轨迹。
- 范围：仅 1 架机（`agent001` ↔ `drone_0`）；沿用阶段1/2的命名与约定（`frame=world`、SensorData QoS、短名参数）。测试阶段全部使用终端最小命令。
- 在阶段2已完成“目标输入链路（/move_base_simple/goal → bspline）”的稳定验证后，开始引入“目标过滤/偏移 + 控制桥 + 起飞控制（pbtm）”，并强调观察占据栅格（inflate）与点云。

#### 系统与数据通路
- 数据链路（自上而下）
  - TestCase Signpost（JSON）→ `tools/signpost_mission_runner.py` 生成路点目标（`PoseStamped`）→ 目标过滤偏移 `tools/goal_filter_offset.py`（`/move_base_simple/goal_raw` → `/move_base_simple/goal`）
  - Ego-Planner：订阅 `goal`、`/drone_0/odom`、`/drone_0/cloud` → 输出 `/drone_0_planning/bspline`
  - 控制桥 `tools/controller_adapter_fixed.py`：订阅 `bspline` → 发布 `/agent001/trajectory/points` 给 pbtm 控制节点 → Unity 驱动
- 关键话题
  - 目标输入：`/move_base_simple/goal_raw`（未过滤），`/move_base_simple/goal`（过滤/偏移后，给 Planner）
  - 规划所需：`/drone_0/odom`（`nav_msgs/Odometry`，`frame=world`），`/drone_0/cloud`（`sensor_msgs/PointCloud2`，`frame=world`）
  - 规划输出：`/drone_0_planning/bspline`（`traj_utils/Bspline`）
  - 控制输出：`/agent001/trajectory/points`（`trajectory_msgs/JointTrajectory`）
  - 占据可视化：`/drone_0_grid/grid_map/occupancy_inflate`（宽高>0 才说明在更新）

#### 环境与版本
- 工作区：`/home/ctx/hifisim_ws`（分支 `main`）
- Unity/HiFiSim：`/agent001/global/sim_nwu_pose`（PoseStamped真值）、`/agent001/lidar01`（PointCloud2）
- ROS 2：终端均可运行 `ros2` CLI；RViz 已可用

#### 改动清单（与阶段3直接相关）
- 工具（永久保留，但按需手动启动）
  - `tools/goal_filter_offset.py`：对目标做距离回退（`--backoff`）、Z 夹紧（`--z_min/z_max`）、去抖（`--min_interval`），输出到 `/move_base_simple/goal`
  - `tools/signpost_mission_runner.py`：解析 EnvScenario JSON Signpost，生成 [pre, pass, post] 序列，按距离阈值推进，发布至 `--goal`
  - `tools/controller_adapter_fixed.py`：采样 `Bspline`，转 `JointTrajectory` 发布到 `/agent001/trajectory/points`（或 PoseStamped）
- 适配层（延用阶段1/2）
  - `pose_to_odom_adapter.py`：`/agent001/global/sim_nwu_pose` → `/drone_0/odom`，`frame=world`，SensorData QoS，20Hz 定时重发
  - `cloud_relay.py`：`/agent001/lidar01` → `/drone_0/cloud`，必要时 `--pose` 进行 world 对齐或 `--assume_world`
- 规划器启动方式（保持阶段2规范）
  - 仅用短名参数：`odometry_topic:=odom cloud_topic:=cloud odom_world:=odom`

#### 最小可复现实验（按顺序逐终端）
1) 清场
```bash
pkill -f goal_filter_offset.py || true
pkill -f controller_adapter_fixed.py || true
pkill -f controller_adapter.py || true
pkill -f signpost_mission_runner.py || true
pkill -f ego_planner_node || true
pkill -f cloud_relay.py || true
pkill -f pose_to_odom_adapter.py || true
```

2) 起飞控制（pbtm）
```bash
nohup ros2 run pbtm_ros2 pbtm_ros2_node --ros-args -r __ns:=/agent001 -r __node:=pbtm_node \
  -p agent_id:=agent001 -p takeoff_height:=5.0 -p send_command_rate:=20.0 \
  -p timeout:=5.0 -p height_range:='[0.5, 20.0]' >/tmp/pbtm.log 2>&1 & echo $!
sleep 1
ros2 topic info -v /agent001/trajectory/points | cat   # 期望：Sub=1（pbtm_node）
```

3) 适配（位姿/点云）
```bash
nohup python3 /home/ctx/hifisim_ws/pose_to_odom_adapter.py \
  --in /agent001/global/sim_nwu_pose --out /drone_0/odom \
  --frame world --child base_link >/tmp/odom_adapt.log 2>&1 & echo $!

# 若需点云中转（Unity 未直发 /drone_0/cloud）
nohup python3 /home/ctx/hifisim_ws/cloud_relay.py \
  --in /agent001/lidar01 --out /drone_0/cloud --pose /agent001/global/sim_nwu_pose \
  >/tmp/cloud_relay.log 2>&1 & echo $!
```

4) 规划器（短名参数）
```bash
nohup bash -lc 'ros2 launch ego_planner advanced_param.launch.py \
  drone_id:=0 odometry_topic:=odom cloud_topic:=cloud odom_world:=odom \
  flight_type:=1 planning_horizon:=5.0 \
  map_size_x_:=120.0 map_size_y_:=80.0 map_size_z_:=40.0 \
  local_update_range_x_:=20.0 local_update_range_y_:=20.0 local_update_range_z_:=15.0 \
  resolution_:=0.2 obstacles_inflation_:=0.6 virtual_ceil_height_:=40.0 ground_height_:=0.0' \
  >/tmp/ego_planner.launch.log 2>&1 & echo $!
```

5) 目标过滤/偏移（隔离 raw/filtered）
```bash
nohup python3 /home/ctx/hifisim_ws/tools/goal_filter_offset.py \
  --node goal_filter_offset_v2 \
  --in /move_base_simple/goal_raw --out /move_base_simple/goal \
  --odom /drone_0/odom --backoff 2.5 --z_min 1.8 --z_max 8.0 --min_interval 0.4 \
  >/tmp/goal_filter.log 2>&1 & echo $!
```

6) 任务（从 Signpost 发布序列目标）
```bash
nohup python3 /home/ctx/hifisim_ws/tools/signpost_mission_runner.py \
  --json /home/ctx/Unity/build/hifi_simulator_unity_Data/StreamingAssets/AppData/EnvScenarios/Env1Scenario1.json \
  --pose /agent001/global/sim_nwu_pose --goal /move_base_simple/goal_raw \
  --alt 2.0 --pre 2.0 --post 2.0 --reach 2.5 --filter Signpost_ --limit 3 \
  >/tmp/signpost_runner.log 2>&1 & echo $!
```

7) 控制桥（Bspline → JointTrajectory）
```bash
nohup python3 /home/ctx/hifisim_ws/tools/controller_adapter_fixed.py \
  --agent_id 1 --hz 20 --horizon 4.0 --mission_code 3 \
  --z_mode clamp --z_min 1.8 --z_max 8.0 --drive_mode jt \
  >/tmp/ctrl_1.log 2>&1 & echo $!
```

8) 在线观测
```bash
# 话题拓扑
ros2 topic info -v /move_base_simple/goal_raw | cat   # Pub=1(signpost), Sub=1(goal_filter)
ros2 topic info -v /move_base_simple/goal | cat       # Pub=1(goal_filter), Sub=1(planner)
ros2 topic info -v /drone_0_planning/bspline | cat    # Pub=1(planner), Sub=1(controller_adapter)

# 数据速率
ros2 topic hz /drone_0_planning/bspline | head -n 20 | cat
ros2 topic hz /agent001/trajectory/points | head -n 20 | cat  # 约 20Hz

# RViz：Fixed Frame=world；显示 /drone_0/cloud、/drone_0/odom、/drone_0_planning/bspline、/drone_0_grid/grid_map/occupancy_inflate
```

#### 调试记录
- 目标过滤链路未接通
  - Observation：`/move_base_simple/goal` 无订阅或订阅数≠1
  - Diagnosis：过滤节点未启动或参数错误（`--in/--out` 混用；`--odom` 话题名需为 `/drone_0/odom`）
  - Change：修正为上面的标准启动与话题名；确认计数 Pub=1/Sub=1
  - Result：Planner 能持续收到过滤后的目标
- `bspline` 出现但无人机不动
  - Observation：`/drone_0_planning/bspline` 在流，`/agent001/trajectory/points` 无数据
  - Diagnosis：控制桥未订阅 bspline 或 pbtm 未在 `agent001` 命名空间
  - Change：按第7步启动控制桥；复核 pbtm 订阅计数（Sub=1）
  - Result：`trajectory/points` ≈20Hz，Unity 中机体跟随
- 占据栅格无内容
  - Observation：`/drone_0_grid/grid_map/occupancy_inflate` 宽高=0
  - Diagnosis：点云为空/帧不一致或栅格范围/分辨率不合适
  - Change：保证点云 `frame=world、x/y/z 存在、width*height>0`；保守调 `resolution_`、`obstacles_inflation_`、`local_update_range_*`
  - Result：占据逐步可见；轨迹绕障
- 目标贴墙或抖动
  - Observation：靠近障碍时规划失败或反复重规划
  - Diagnosis：原始目标紧贴墙面/频繁发布
  - Change：`goal_filter_offset.py` 增大 `--backoff`，设 `--min_interval` 去抖；`--z_min/z_max` 合法化高度
  - Result：轨迹更稳定，绕障余量更好

#### 验收标准
- 目标序列发布后 `t ≤ 2 s` 内出现首个 `/drone_0_planning/bspline` 包
- `/drone_0_planning/bspline` 持续输出（≥ 5 Hz），`/agent001/trajectory/points` 稳定 ≈ 20 Hz
- RViz 可见占据膨胀与轨迹；路径不穿越明显障碍，Unity 中机体按轨迹行进

#### 已知问题/风险
- `goal_filter_offset.py` 的 `--odom` 默认值为 `/drone_0_odom`（下划线），需显式传入 `/drone_0/odom`
- 场景差异较大时需适配 `resolution_`、`obstacles_inflation_`、`local_update_range_*`；否则易出现“地图空/过稀/过密”
- `signpost_mission_runner.py` 的坐标系转换以当前 Env1 假设为准，切换场景需复核（已通过参数 `--filter/--limit` 控制样本）

