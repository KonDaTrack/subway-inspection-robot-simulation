# 地铁巡检机器人仿真 · Subway Inspection Robot Simulation

ROS Noetic + Gazebo 仿真项目:**一台麦克纳姆轮全向底盘 + 一台 Franka Panda 机械臂**,在地铁站场景中执行"导航到导航点 → 按点执行机械臂动作"的自动巡检任务。

**核心技术点**:SLAM 建图、导航定位、RViz 交互标记导航点、MoveIt 机械臂示教回放、以及一个把"导航点数据"与"到达后任务"解耦的轻量任务编排框架(`mission_handler`)。

---

## 特性 / Features

- 🛞 **麦克纳姆轮全向底盘**:支持横移(全向),Gazebo 中通过自定义 `wpr_plugin` 驱动(`cmd_vel` → `odom`)
- 🦾 **Franka Panda 7 自由度机械臂**:MoveIt 运动规划,`ros_control` 关节控制
- 🗺️ **建图 / 定位 / 导航**:gmapping 建图、AMCL 定位、move_base + 自定义 XY 优先规划器
- 📍 **RViz 可视化标记导航点**:`waterplus_map_tools` 的 Add Waypoint 工具,存成 `waypoints.xml`
- 🧠 **mission_handler 任务编排**:导航点名即任务名,把"去哪个点"和"到了干什么"彻底解耦
- 🏭 **地铁站仿真场景**:`metro.world`,含 lidar 点云过滤

---

## 机器人 & 场景

```
        ┌───────────────┐
        │  Franka Panda │  7-DOF 机械臂 (MoveIt / move_group)
        │   机械臂       │
        └──────┬────────┘
               │ panda_base
        ┌──────┴────────┐
        │  麦克纳姆轮底盘 │  全向移动 (linear.x / linear.y / angular.z)
        │  base_footprint│
        └───────────────┘
```

- 机器人模型:`robot_description/urdf/robot.urdf.xacro`(组合 `base_mecanum` + `panda`)
- 场景:地铁站 `robot_description/world/metro.world`
- 传感器:2D 激光雷达(经 `lidar_filter` 过滤)

---

## 目录结构

| 功能包 | 作用 |
|---|---|
| `robot_description` | 机器人 URDF、Gazebo 仿真主入口、地铁站世界 |
| `base_controller` | 底盘导航:gmapping / amcl / move_base / 地图 |
| `arm_controller` | 机械臂动作服务 + 示教保存(`arm_poses.yaml`) |
| `robot_moveit_config` | MoveIt 规划配置 |
| `xy_first_planner` | 自定义"纯追踪"XY 优先导航服务 |
| `waterplus_map_tools` * | 导航点编辑/管理(第三方,见文末) |
| `state_manager` | 任务调度:`mission_handler.py` 任务编排框架 |
| `wpr_simulation` * | Waterplus 机器人仿真插件与场景(第三方,见文末) |

\* = 并入的第三方 ROS 包,见 [THIRD_PARTY_NOTICES.md](./THIRD_PARTY_NOTICES.md)

---

## 快速开始

### 方式 A:用封装好的 Docker 镜像(推荐,开箱即用)

已封装为 **`metro_detection_robot`** 镜像,内含:编译好的完整工作区、预置地图与 16 个导航点、机械臂位姿、已修复的环境与 `.bashrc`。

```bash
# 1. 导入镜像(拿到 .tar 时)
docker load -i metro_detection_robot.tar

# 2. 宿主机放行 X11(GUI,重启宿主机后需重跑一次)
xhost +local: >> /dev/null

# 3. 启动容器(需 NVIDIA 驱动 + Container Toolkit;无 GPU 删除 --gpus 及 NVIDIA_* 行)
docker run -dit \
  --name=metro_robot \
  -u robot -w /home/robot \
  --gpus all --privileged \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --net=host \
  metro_detection_robot:latest

# 4. 进入容器(ROS 与工作区环境已自动加载)
docker exec -it metro_robot bash
```

### 方式 B:源码构建

```bash
# 依赖:ROS Noetic(noetic-desktop-full 含 Gazebo/RViz)+ moveit + navigation + ros-control 等
mkdir -p ~/catkin_ws/src
cp -r src_subway/* ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> 注意:部分模块(地图、导航点加载路径)目前按 `/home/robot/catkin_ws` 的约定路径写死,源码方式需对应修改 `base_controller/launch/amcl_test.launch`、`movebase_with_amcl.launch` 中的路径。

---

## 使用指南

所有命令在已加载工作区的终端内执行。

### 1. 启动仿真

```bash
roslaunch robot_description simulation.launch
```

启动 Gazebo(地铁站)+ 底盘 + Panda 机械臂 + MoveIt + lidar 过滤。

### 2. 建图 (SLAM)

```bash
roslaunch base_controller gmapping.launch      # 建图 + 键盘遥控(i/j/k/l 移动,q/z 调速)
# 跑完地图保存:
roslaunch base_controller map_save.launch      # → base_controller/map/map.yaml
```

### 3. 标记导航点

```bash
roslaunch base_controller movebase_with_amcl.launch   # 导航+定位+rviz+导航点管理器
```

在 RViz 用 **Add Waypoint** 工具(快捷键 `a`)点击放点 → 保存:

```bash
rosservice call /waterplus/save_waypoints "{filename: '/home/robot/catkin_ws/src/waterplus_map_tools/waypoints.xml'}"
```

**命名约定(重要)**:`mission_handler` 按导航点名决定到达后动作——只有下列名字会被任务处理:

| 导航点名 | 到达后的机械臂动作 |
|---|---|
| `A1` | 机械臂到 `pose1` |
| `A2` | 机械臂到 `pose2` 再回 home |
| `Home` | 机械臂回 home |
| `None` | 纯导航点,不动机械臂 |

RViz 工具自动命名数字,需编辑 `waypoints.xml` 把名字改成上述约定值。

### 4. 机械臂示教

```bash
rosrun arm_controller teach_save_pose.py       # 启动示教保存节点
# 在 RViz MoveIt 面板把机械臂摆到目标位姿后保存:
rostopic pub /save_arm_pose std_msgs/String "pose1" -1   # 存入 arm_poses.yaml

rosrun arm_controller arm_move_service.py       # 机械臂动作服务
rosservice call /move_arm_to_pose "action_name: 'pose1'" # 回放测试
```

### 5. 启动自动巡检

```bash
# 终端1:仿真 + 导航全套 + 机械臂服务
roslaunch state_manager mission.launch
# 终端2:XY 优先导航规划服务
roslaunch xy_first_planner planner.launch
# 终端3:运行巡检(导航到点 → 机械臂动作,循环)
rosrun state_manager mission_handler.py
```

---

## 设计详解

### mission_handler:把"导航点数据"和"到达后任务"解耦

核心文件 `state_manager/scripts/mission_handler.py`。每个导航点被包装成 `PoseWithActionBase` 子类实例,基类自动接好两个服务:

| 服务 | 用途 |
|---|---|
| `/xy_first_planner/move_to_pose` | 底盘导航到点 |
| `/move_arm_to_pose` | 执行机械臂动作 |

**导航点名 = 任务名**,通过注册表 `action_class_map` 把名字映射到任务类:

```python
action_class_map = {
    'A1':   MoveitAction1,     # 到 pose1
    'A2':   MoveitAction2,     # 到 pose2 再回 home
    'Home': MoveitActionHome,
    'None': MoveitActionNone,
}
```

执行流程:

```python
handler = MissionHandler()
for task in handler.get_all_waypoints():   # 顺序 = waypoints.xml 索引
    task.send_navigation_goal()            # 导航到点
    task.action()                          # 到达后执行任务
```

**新增一个"到达后任务"(如拍照)**:① 继承 `PoseWithActionBase` 重写 `action()` → ② 加进 `action_class_map` → ③ 把某个导航点命名为对应名字。业务逻辑全在任务类里,导航数据全在 `waypoints.xml`,两者靠名字关联——加任务不用改导航坐标。

### XYFirstPlanner:点到点纯追踪导航(可替换)

`xy_first_planner/scripts/xy_first_planner_serivce.py` 是**纯追踪控制器**(非路径规划器):在目标方向线上取 `lookahead_distance` 前瞻点,用 PID 直扑目标,无避障——适合地铁站这类开阔场景。三段状态机:移动中(大转角先走位置 / 侧向 >70° 全向横移)→ 到位原地对齐 → 停止,双 PID 调参。

**替换方式**:只需让新节点提供同名服务 `/xy_first_planner/move_to_pose`(输出 `/cmd_vel`),`mission_handler` 一行不改;或改 `send_navigation_goal()` 用 move_base action。

---

## 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `roslaunch` 找不到功能包 | 未 source 工作区:`source ~/catkin_ws/devel/setup.bash` |
| 机械臂不动 | `arm_poses.yaml` 缺位姿,或 `move_group` 未启动 |
| 巡检只导航不动机械臂 | `main()` 里 `action()` 被注释,或位姿文件缺失 |

---

## 第三方组件与致谢

本仓库并入了第三方 ROS 包以做到开箱即用,包括 **Waterplus 的 `wpr_simulation`(GPL-3.0)**、**`waterplus_map_tools`(GPL-2.0)**,以及 **Franka Emika Panda** 机械臂模型。来源、许可证与改动说明详见:

➡️ [THIRD_PARTY_NOTICES.md](./THIRD_PARTY_NOTICES.md)

## License

仓库整体以 **GPL-3.0** 发布(见 [LICENSE](./LICENSE))。并入的第三方包保留各自许可证,请勿按单一许可证重授。
