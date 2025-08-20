# 阶段2（Signpost → 规划器）

> 仅覆盖目标输入链路；使用**短名参数**与**分终端最小命令**验证。沿用阶段1命名与 QoS/frame 约定（`frame=world`，`QoS=SensorData`）。

---

## 1. 目标与范围

* **目标**：将 TestCase 生成的目标点（`geometry_msgs/PoseStamped`）送入 Ego-Planner 的目标订阅接口，使规划器进入 `PLANNING/EXEC` 并在 `/drone_0_planning/bspline` 发布 B-spline。
* **范围**：单机（`agent001 ↔ drone_0`），只验证**目标输入 → 轨迹输出**闭环；不涉及控制器/多机互避/一键脚本。
* **验证方式**：终端分步、短名参数、RViz 最小显示。

---

## 2. 系统与数据通路

* **输入**（阶段2主链）

  * 目标：`/move_base_simple/goal` · `geometry_msgs/msg/PoseStamped` · `header.frame_id=world`
* **依赖输入**（阶段1已打通）

  * 里程计：`/drone_0/odom` · `nav_msgs/msg/Odometry` · `frame=world`
  * 点云：`/drone_0/cloud` · `sensor_msgs/msg/PointCloud2` · `frame=world`
* **输出**

  * 轨迹：`/drone_0_planning/bspline` · `traj_utils/msg/Bspline`（期望 ≥ 5 Hz）

---


## 3. 改动清单

* **适配层**

  * `pose_to_odom_adapter.py`：订阅 `sim_nwu_pose` → 发布 `Odometry`；`QoS=SensorData`；首帧缓存后 \~20 Hz 重发；新增 CLI：`--frame world --child base_link`。
  * `cloud_relay.py`：中转 `PointCloud2`；可选 `--pose /agent001/global/sim_nwu_pose` 做时间/坐标对齐；统一输出 `header.frame_id=world`。
  * **动机/影响**：解决“RViz 有计数但不可见（空包/帧不一致）”“地图宽高为 0”；确保规划器可消费。
* **规划器启动用法**

  * 全部使用**短名参数**：`odometry_topic:=odom cloud_topic:=cloud odom_world:=odom`；避免 `--ros-args -r` 引发参数识别错误与命名空间重复。
* **非本阶段工具（仅记录，不启用）**

  * `tools/goal_filter_offset.py`、`tools/controller_adapter_fixed.py`、`tools/signpost_mission_runner.py`：用于后续闭环/任务层。
* **文档**

  * 新增 `docs/stages/stage-02-signpost-to-planner.md`（本说明与命令清单）。

---

## 4. 复现测试

**终端 A：位姿 → 里程计**

```bash
python3 /home/ctx/hifisim_ws/pose_to_odom_adapter.py \
  --in /agent001/global/sim_nwu_pose \
  --out /drone_0/odom \
  --frame world --child base_link
```

**终端 B：点云中转（如 Unity 未直发 /drone\_0/cloud）**

```bash
python3 /home/ctx/hifisim_ws/cloud_relay.py \
  --in /agent001/lidar01 \
  --out /drone_0/cloud \
  --pose /agent001/global/sim_nwu_pose
```

**终端 C：启动规划器（短名参数）**

```bash
ros2 launch ego_planner advanced_param.launch.py \
  drone_id:=0 odometry_topic:=odom cloud_topic:=cloud odom_world:=odom \
  flight_type:=1 planning_horizon:=5.0
```

**终端 D：发布目标（强制 frame=world）**

```bash
ros2 topic pub -1 /move_base_simple/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: world}, pose: {position: {x: 10.0, y: 5.0, z: 2.5}, orientation: {w: 1.0}}}"
```

**监听**

```bash
ros2 topic info -v /move_base_simple/goal | cat
ros2 topic info -v /drone_0_planning/bspline | cat
ros2 topic hz /drone_0_planning/bspline | head -n 20 | cat
```



## 5. 调试记录

* **目标未生效/报参错**
  Observation：使用 `--ros-args -r` 报 “unrecognized arguments” 或 planner 未收到目标
  Diagnosis：planner 期望用 launch 参数传话题名，remap 冲突
  Change：统一短名参数
  Result：`/move_base_simple/goal` 订阅计数=1，发布后产生 `bspline`
* **RViz 有计数无点云/地图**
  Observation：`PointCloud2` 宽高为 0 或不可见；地图宽高为 0
  Diagnosis：空包/`frame_id` 与 Fixed Frame 不一致
  Change：适配层统一 `frame_id=world`，必要时 `--pose` 对齐，确保 `fields` 含 `x,y,z`
  Result：点云可见，地图更新
* **命名空间前缀重复**
  Observation：出现 `/drone_0_/drone_0_*`
  Diagnosis：绝对路径叠加命名空间
  Change：仅用短名（`odom/cloud/odom`）
  Result：订阅表规范、唯一
* **目标帧不一致**
  Observation：发布目标后无轨迹
  Diagnosis：目标 `frame_id` 与地图帧不一致
  Change：目标统一 `frame_id=world`
  Result：`bspline` 正常输出

---

## 6. 验收标准

* 目标发布后 **t ≤ 2 s** 出现首个 `/drone_0_planning/bspline` 包
* `/drone_0_planning/bspline` **频率 ≥ 5 Hz**（连续 ≥ 5 s）
* 订阅/发布计数匹配：`/move_base_simple/goal`（Sub=1）、`/drone_0/odom`（Planner Sub=1）、`/drone_0/cloud`（Planner Sub=1）

---

## 7. 已知问题

* 仓库内部分工具默认 `frame_id=map`，与本阶段 `world` 设定不一致（阶段2不启用以避免混淆）。
* 个别环境点云 QoS 默认非 SensorData，可能导致初期不同步；建议在节点端显式设为 SensorData。
* `waypoint_feeder_node.py` 存在硬编码里程计话题格式的问题（非本阶段使用）。

---



### 附录｜提交与文件索引

| 路径                        | 变更摘要                                                                                   | 相关提交     |
| ------------------------- | -------------------------------------------------------------------------------------- | -------- |
| `pose_to_odom_adapter.py` | `QoS=SensorData`；输出 `Odometry`；首帧后 \~20 Hz 重发；CLI 增加 `--frame world --child base_link` | 未提交（工作区） |
| `cloud_relay.py`          | 点云中转；可选 `--pose` 做对齐；统一 `header.frame_id=world`                                        | 未提交（工作区） |
| `signpost_router.py`      | TestCase → 路点/目标（默认 `frame_id=map`）；阶段2未使用                                             | 初始提交     |
| `waypoint_feeder.py`      | 发布 `PoseStamped` 至 `/traj_start_trigger`（`frame_id=map`）；阶段2未使用                        | 初始提交     |
| `waypoint_feeder_node.py` | 路点推进/重发；里程计话题命名缺陷；阶段2未使用                                                               | 初始提交     |

