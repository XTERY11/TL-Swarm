# Unity-EGO-Swarm集成验证计划

## 阶段一：底层模块稳定性验证

### 1.1 Unity数据发布验证
**目标**: 确认Unity能正常发布传感器数据
**测试方法**:
```bash
# 启动Unity并检查话题
ros2 topic list | grep agent
ros2 topic echo /agent000/global/sim_nwu_pose --once
ros2 topic echo /agent000/lidar01 --once
```
**成功标准**: 
- Unity正常发布PoseStamped和PointCloud2
- 数据格式正确，无异常值

### 1.2 模拟数据发布验证  
**目标**: 确认mock_unity_publisher稳定工作
**测试方法**:
```bash
# 终端A
/home/ctx/hifisim_ws/install/unity_bridge/bin/mock_unity_publisher

# 终端B - 监控输出
ros2 topic hz /agent000/global/sim_nwu_pose
ros2 topic hz /agent000/lidar01
```
**成功标准**:
- 稳定10Hz发布率
- 5分钟无崩溃

### 1.3 桥接节点验证
**目标**: 确认数据格式转换正确
**测试方法**:
```bash
# 启动桥接
/home/ctx/hifisim_ws/install/unity_bridge/bin/unity_bridge_node --ros-args -p drone_id:=0

# 验证转换
ros2 topic echo /drone_0_visual_slam/odom --once
ros2 topic echo /drone_0_cloud --once
```
**成功标准**:
- PoseStamped → Odometry转换正确
- PointCloud2正确转发

## 阶段二：EGO-Planner核心功能验证

### 2.1 单独启动测试
**目标**: 确认ego_planner可独立稳定运行
**测试方法**:
```bash
# 提供稳定odometry和点云
/home/ctx/hifisim_ws/install/unity_bridge/bin/mock_unity_publisher &
/home/ctx/hifisim_ws/install/unity_bridge/bin/unity_bridge_node --ros-args -p drone_id:=0 &

# 启动ego_planner (注意参数语法)
/home/ctx/hifisim_ws/install/ego_planner/lib/ego_planner/ego_planner_node --ros-args \
  -p manager/drone_id:=0 \
  -p fsm/single_drone:=1 \
  -p fsm/flight_type:=1 \
  -p manager/max_vel:=2.0 \
  -p manager/max_acc:=3.0 \
  -p manager/planning_horizon:=4.0 \
  --remap /odom_world:=/drone_0_visual_slam/odom \
  --remap /grid_map/cloud:=/drone_0_cloud \
  --remap /grid_map/odom:=/drone_0_visual_slam/odom
```
**成功标准**:
- 状态从INIT → WAIT_TARGET
- "no odom"消失
- 10分钟无崩溃

### 2.2 轨迹规划功能验证
**目标**: 确认能接收目标并生成轨迹
**测试方法**:
```bash
# 发布目标点 (修正语法)
ros2 topic pub /move_base_simple/goal geometry_msgs/msg/PoseStamped --once \
'{header: {frame_id: "world"}, pose: {position: {x: 5.0, y: 5.0, z: 2.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

# 监控轨迹输出
ros2 topic echo /planning/bspline --once
```
**成功标准**:
- 状态转为GEN_NEW_TRAJ
- 成功输出B样条轨迹
- 无std::length_error异常

## 阶段三：完整数据流验证

### 3.1 端到端数据流测试
**目标**: Unity → Bridge → EGO → 轨迹输出全链路
**测试方法**:
```bash
# 按顺序启动所有节点
# 1. Unity或mock_publisher
# 2. unity_bridge_node  
# 3. ego_planner_node
# 4. goal_publisher (自动发布目标)

# 监控关键数据流
ros2 topic hz /agent000/global/sim_nwu_pose
ros2 topic hz /drone_0_visual_slam/odom  
ros2 topic hz /planning/bspline
```
**成功标准**:
- 所有话题稳定发布
- 轨迹规划响应目标变化
- 30分钟连续无错误

## 阶段四：Unity UserData集成

### 4.1 Unity配置分析
**目标**: 理解Unity传感器配置
**任务**:
- 分析 `/home/ctx/Unity/build/hifi_simulator_unity_Data/StreamingAssets/UserData`
- 确认实际话题名称和数据格式
- 对比与模拟数据的差异

### 4.2 接口适配
**目标**: 修改桥接节点适配真实Unity数据
**任务**:
- 更新话题映射配置
- 处理数据格式差异
- 验证坐标系转换

## 关键注意事项 (基于历史错误)

### ❌ 避免的错误
1. **参数语法**: 必须用 `manager/drone_id` 而非 `manager.drone_id`
2. **话题确认**: 先用 `ros2 topic info` 确认订阅关系
3. **路径依赖**: 使用绝对路径避免 `ros2 run` 问题  
4. **异常处理**: Python节点需要完善信号处理
5. **顺序启动**: 确保依赖节点先启动并稳定

### ✅ 成功要素
1. **逐层验证**: 每个模块单独测试通过再集成
2. **数据流跟踪**: 用 `ros2 topic hz/echo` 实时监控
3. **参数验证**: 用 `ros2 param get` 确认参数加载
4. **节点状态**: 用 `ros2 node list` 检查节点存活
5. **异常记录**: 保存所有错误日志便于排查

## 下一步执行

按照上述验证计划，从阶段一开始，逐步验证每个模块的稳定性，只有前一阶段完全通过才进入下一阶段。 