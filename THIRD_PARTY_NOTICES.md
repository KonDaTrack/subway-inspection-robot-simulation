# Third-Party Notices · 第三方组件声明

本仓库是个人项目"地铁巡检机器人仿真"。为了让代码**开箱即跑**,仓库内**直接并入了**以下第三方 ROS 包(而非以外部依赖引用)。这些包保留各自的许可证与版权声明。

---

## 1. wpr_simulation

- **上游来源**: <https://github.com/6-robot/wpr_simulation>
- **版权 / 作者**: Waterplus (6-robot)
- **许可证**: **GPL-3.0**(以包内 `LICENSE` 文件为准;个别源码文件头部标注 BSD,系上游自身不一致)
- **引入目的**: WPR 系列移动机器人仿真模型、底盘 `wpr_plugin` 插件、`lidar_filter` 激光节点、仿真场景素材
- **本地改动**: 为地铁场景更换/新增了部分 `media/`(如 QR、书本图)与 `meshes/`(床、杂物等)素材;微调安装脚本。核心代码未改动

## 2. waterplus_map_tools

- **上游来源**: <https://github.com/6-robot/waterplus_map_tools>
- **版权 / 作者**: Waterplus (6-robot)
- **许可证**: **GPL-2.0**(见包内 `LICENSE`)
- **引入目的**: RViz **Add Waypoint** 工具 / `wp_manager` 导航点的编辑、保存与查询服务(`/waterplus/...`),供 mission_handler 读取
- **本地改动**: 部分图标、`charger` 网格、各 ROS 发行版安装脚本等。核心逻辑未改动

## 3. Franka Emika Panda 机械臂模型

- `robot_description/urdf/panda.urdf.xacro` 及其配套 mesh,采用社区公开的 **Franka Emika Panda 7 自由度机械臂**运动学模型
- 该模型在 MoveIt 生态中广泛流传(常来自 `panda_moveit_config` / `franka_ros`);本仓库内此文件未携带作者/许可证声明,故单独列出致谢

## 4. 作者原创部分

- 麦克纳姆轮底盘模型 `base_mecanum.urdf.xacro` 与其 `meshes/`
- `base_controller`、`arm_controller`、`state_manager`(含 mission_handler 任务编排)、`xy_first_planner`、`robot_moveit_config`、`robot_description` 的组合与集成代码
- 以上为作者原创,按仓库整体许可证(见下)授权

---

## 许可证关系说明

- 仓库整体以 **GPL-3.0** 发布(见根目录 `LICENSE`)。
- 并入的 `wpr_simulation`(**GPL-3.0**)与 `waterplus_map_tools`(**GPL-2.0**)保留各自许可证;**waterplus_map_tools 为 GPL-2.0,并未升级到 3.0**。
- 上述各包以相互独立的 ROS 包形式存在、通过 ROS 消息/服务通信,视为相互独立的作品聚合,各自许可证分别适用;请勿把它们与仓库自有代码混为一谈后按单一许可证重授。
