# Silver2/Stonefish 坐标系与坐标轴速查

本文档整理 `silver2_stonefish` 与 `stonefish_ros2` 中当前仿真涉及的坐标系、轴方向、外参与 TF 发布规则，供后续查阅。

## 1. 全局坐标系（世界系）

- 场景环境显式使用 NED（North-East-Down）：
  - `silver2_stonefish/src/stonefish_silver/scenarios/simulation.scn:31`
  - `silver2_stonefish/src/stonefish_silver/scenarios/simulation_2.scn:41`
- ROS 侧世界帧统一命名为 `world_ned`（不是 `map`/`odom`）：
  - `stonefish_ros2/src/stonefish_ros2/ROS2Interface.cpp:281`
  - `stonefish_ros2/src/stonefish_ros2/ROS2Interface.cpp:333`
  - `stonefish_ros2/src/stonefish_ros2/ROS2Interface.cpp:570`
  - `stonefish_ros2/src/stonefish_ros2/ROS2Interface.cpp:662`

## 2. 场景中的位姿定义方式

- 场景内实体（光源、地形、障碍物）使用 `world_transform xyz + rpy` 放置：
  - 示例：`simulation.scn:55`, `simulation.scn:65`
  - 示例：`simulation_2.scn:64`, `simulation_2.scn:73`, `simulation_2.scn:91`, `simulation_2.scn:101`, `simulation_2.scn:110`, `simulation_2.scn:131`, `simulation_2.scn:158`, `simulation_2.scn:171`
- 机器人实例化位姿由 include 参数传入：
  - `simulation.scn:86`, `simulation.scn:87`
  - `simulation_2.scn:194`, `simulation_2.scn:195`
  - 在机器人定义中应用：`robot.scn:5`

## 3. 机器人本体与关节轴（`robot.scn`）

### 3.1 机体基准

- 机器人基座 link 为 `Main_Body`：
  - `silver2_stonefish/src/stonefish_silver/scenarios/robot.scn:7`

### 3.2 18 个关节轴方向规律

- Coxa 关节轴：`+Z`（`axis="0 0 1"`）
  - `robot.scn:41`, `robot.scn:118`, `robot.scn:195`, `robot.scn:272`, `robot.scn:349`, `robot.scn:426`
- Femur 关节轴：`+Y`（`axis="0 1 0"`）
  - `robot.scn:66`, `robot.scn:143`, `robot.scn:220`, `robot.scn:297`, `robot.scn:374`, `robot.scn:451`
- Tibia 关节轴：`-Y`（`axis="0 -1 0"`）
  - `robot.scn:91`, `robot.scn:168`, `robot.scn:245`, `robot.scn:322`, `robot.scn:399`, `robot.scn:476`

### 3.3 左右腿对称时的额外旋转

- 右侧三条腿 Coxa 关节原点存在 yaw 约 `pi`（`rpy ... 3.14`）：
  - `robot.scn:271`, `robot.scn:348`, `robot.scn:425`

## 4. 传感器安装外参与坐标

### 4.1 双目相机

- 左相机 `stereo_left`：
  - `origin xyz="0.174 0.25 0.0" rpy="${pi/2} 0 ${pi}"`
  - `robot.scn:485`, `robot.scn:487`
- 右相机 `stereo_right`：
  - `origin xyz="-0.174 0.25 0.0" rpy="${pi/2} 0 ${pi}"`
  - `robot.scn:492`, `robot.scn:494`
- 两相机都挂在 `Main_Body`（`link name="Main_Body"`），基线约 `0.348 m`（x 向对称）。

### 4.2 IMU

- IMU 安装在 `Main_Body` 原点：
  - `robot.scn:500`, `robot.scn:505`, `robot.scn:506`

### 4.3 Odom 传感器

- Odometry 传感器也在 `Main_Body` 原点：
  - `robot.scn:511`, `robot.scn:513`

## 5. ROS2 中的 TF 与 frame_id 规则

### 5.1 TF 发布

- TF 广播由 `ROS2Interface::PublishTF()` 完成：
  - `stonefish_ros2/src/stonefish_ros2/ROS2Interface.cpp:90-106`
- 机器人 base TF 仅在开关打开时发布：
  - 发布点：`ROS2SimulationManager.cpp:394-395`
  - 父子关系：`world_ned -> <robot>/base_link`

### 5.2 `ros_base_link_transform` 开关

- 解析 `<ros_base_link_transform publish="...">`：
  - `ROS2ScenarioParser.cpp:297-300`
- 默认值是 `false`（未配置则不发该 TF）：
  - `stonefish_ros2/include/stonefish_ros2/ROS2SimulationManager.h:91`

### 5.3 各类消息 frame_id

- Odometry/INS odom 的 `header.frame_id` 为 `world_ned`：
  - `ROS2Interface.cpp:281`, `ROS2Interface.cpp:333`
- 很多传感器默认用自身名称作为 `frame_id`（如 IMU、DVL、GPS、FT、编码器、声呐、相机等）：
  - IMU：`ROS2Interface.cpp:162`
  - DVL：`ROS2Interface.cpp:214`
  - GPS：`ROS2Interface.cpp:252`
  - 编码器：`ROS2Interface.cpp:372`
  - 相机图像：`ROS2Interface.cpp:714`

## 6. NED 与 OKVIS 相关的轴变换

- IMU 发布时有注释明确写了 NED 到 Z-up（给 OKVIS）：
  - `stonefish_ros2/src/stonefish_ros2/ROS2Interface.cpp:170`
- 实际处理是只对角速度和线加速度做 Y/Z 取反：
  - `ROS2Interface.cpp:171-176`
- INS 原生消息保留 NED 语义（north/east/down, roll/pitch/yaw）：
  - `ROS2Interface.cpp:312-317`

## 7. 在线更新 origin（传感器/执行器）

- 执行器支持通过 `geometry_msgs/Transform` 在线改相对安装位姿：
  - 解析订阅入口：`ROS2ScenarioParser.cpp:667-673`
  - 回调实现：`ROS2SimulationManager.cpp:1061-1076`
- 传感器也支持在线改相对安装位姿：
  - LINK 传感器：`ROS2ScenarioParser.cpp:842-849`
  - VISION 传感器：`ROS2ScenarioParser.cpp:975-982`
  - 回调实现：`ROS2SimulationManager.cpp:1148-1162`

## 8. 相机 frame_id 的一个注意点

- 解析器支持在 `<ros_publisher>` 里配置 `frame_id`：
  - `ROS2ScenarioParser.cpp:714-715`
- 但 `GenerateCameraMsgPrototypes()` 中：
  - `Image.header.frame_id` 可用外部传入 `frame_id`：`ROS2Interface.cpp:714`
  - `CameraInfo.header.frame_id` 固定为 `cam->getName()`：`ROS2Interface.cpp:723`
- 这可能导致 `image` 与 `camera_info` 的 frame_id 不一致（若显式配置了 `frame_id`）。

## 9. 当前仓库配置下的结论（基于现有 `.scn`）

- 现有 `simulation.scn` / `simulation_2.scn` / `robot.scn` 中未看到 `<ros_base_link_transform ...>`，因此默认不发布 `world_ned -> base_link` TF（除非后续在场景里加该标签）。
- 现有 `robot.scn` 传感器 `<ros_publisher>` 未设置 `frame_id` 属性，因此相机图像 frame 默认是传感器名（如 `stereo_left` / `stereo_right`）。
