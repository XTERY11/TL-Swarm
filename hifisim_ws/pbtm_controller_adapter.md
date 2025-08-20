### 为什么可以“不再依赖 pbtm_ros2”
- **pbtm 的定位**：它是“板载控制器”的仿真件，读取 JointTrajectory 的“任务码”（起飞/降落/航点等），在内部生成 B 样条，然后发布期望位姿到 `/agentXXX/global/nwu_pose`。它不向 Unity 的执行话题发指令。
- **Unity 的执行接口**：Unity 只执行 `/agentXXX/trajectory/points`（JointTrajectory）。真正“驱动电机”的是这个话题。
- **因此**：最终方案用 `controller_adapter.py` 直接把规划器输出（Bspline）采样成 Unity 所需的 `JointTrajectory`，即可闭环运动；`pbtm` 不再是必需组件。

### 代码证据（pbtm 的话题与行为）
```220:238:/home/ctx/hifisim_ws/src/pbtm_ros2/include/pbtm_ros2/pbtm.h
            _nwu_pos_sub = _nh->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/" + _id + "/global/sim_nwu_pose", 20, 
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->curr_nwu_pose_callback(msg);
                    });

            /** @brief Subscriber that receives waypoint information from user */
            _waypoint_sub = _nh->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "/" + _id + "/trajectory/points", 20, 
                [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
                    this->waypoint_command_callback(msg);
                    });
           
                /* ------------ Publishers ------------ */
            _pose_nwu_pub = _nh->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/" + _id + "/global/nwu_pose", 20);
```

```354:410:/home/ctx/hifisim_ws/src/pbtm_ros2/src/pbtm.cpp
int pbtm_class::joint_trajectory_to_waypoint(trajectory_msgs::msg::JointTrajectory jt)
{
    // ...
    int mission_type = static_cast<int>(jt.points[0].time_from_start.sec +
                                    jt.points[0].time_from_start.nanosec / 1e9);
    // mission_type: 1=Takeoff, 3=Mission...
    if (mission_type == 3) { ... push_back 路点 ... }
    else if (mission_type == 1) { ... 生成起飞目标高度 ... }
    // ...
    return mission_type;
}
```

```91:104:/home/ctx/hifisim_ws/src/pbtm_ros2/src/pbtm.cpp
    geometry_msgs::msg::PoseStamped global_nwu;
    global_nwu.header.stamp =  _nh->now();
    global_nwu.header.frame_id = "world";
    global_nwu.pose.position.x = cmd_nwu.pos.x();
    global_nwu.pose.position.y = cmd_nwu.pos.y();
    global_nwu.pose.position.z = cmd_nwu.pos.z();
    // 姿态由 yaw 计算
    _pose_nwu_pub->publish(global_nwu);
```

- 结论：`pbtm` 的“执行输出”是 `PoseStamped` 到 `/global/nwu_pose`，不是 Unity 接口的 `JointTrajectory`。除非你再写一层“Pose→JointTrajectory”的桥，否则 Unity 不会跟随 `pbtm` 的输出。

### 两条执行链的对比（本质差异）
- **链路A：pbtm（不驱动 Unity）**
  - 输入：`/agent001/trajectory/points`（只用其“任务码/航点”）
  - 计算：内部 B 样条 + 定时器
  - 输出：`/agent001/global/nwu_pose`（Position/Yaw）
  - 预条件：必须高频收到 `/agent001/global/sim_nwu_pose`，否则超时拒命令
  - 适用：模拟板载控制/MAVROS风格；或Unity能够订阅Pose时
- **链路B：controller_adapter（推荐，驱动 Unity）**
  - 输入：`/drone_0_planning/bspline`（Ego-Planner）
  - 计算：离散采样+时间化窗口
  - 输出：`/agent001/trajectory/points`（Unity执行）
  - 预条件：Unity 对该话题为订阅者=1
  - 适用：Unity 执行实际运动；与规划器无缝衔接

### 为什么我们之前“要跑 pbtm”，现在“可以不用”
- 之前：为验证“任务码起飞”链路是否正常，尝试给 `/agent001/trajectory/points` 发 `mission_code=1`，看到 `pbtm` 日志响应；但 Unity 是否执行，取决于其是否订阅该话题。若当时 Unity 未订阅，就会出现“pbtm 在动（发 Pose），Unity 不动”的观感差异。
- 现在：我们已验证 Unity 对 `/agent001/trajectory/points` 的订阅与执行“是通的”，并引入 `controller_adapter.py` 直接驱动 Unity，因此 `pbtm` 变为“冗余组件”。为了减少歧义和话题混用，推荐不再并行运行 `pbtm`。

### 测试目的与方法的前后区别
- 早期目的：验证“命令路径是否能被消费”与“任务码解析是否正确”（pbtm 用于自证内部时序与 B 样条）。
- 目前目的：验证“规划器→执行器→Unity”的整链运动与避障表现。最佳做法是“规划器输出→adapter 直接发给 Unity”，省去中间不必要的控制栈。

### 什么时候还会用到 pbtm
- 需要“板载控制栈”或 MAVROS 接口仿真；或 Unity 将来支持直接跟随 `/global/nwu_pose`；
- 若坚持用 `pbtm` 驱动 Unity，需要额外桥接：`/agent001/global/nwu_pose` → 生成 Unity 的 `JointTrajectory`。

- 小结
  - `pbtm` 发布 Pose，不是 Unity 的执行接口；`controller_adapter` 发布 JointTrajectory，正好是 Unity 所需。
  - 最终工程里保留“规划器→controller_adapter→Unity”，不再并行跑 `pbtm`，避免认知与调试复杂度。