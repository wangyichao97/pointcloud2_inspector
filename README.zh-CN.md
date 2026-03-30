# pointcloud2_inspector

`pointcloud2_inspector` 是一个面向生产环境的 ROS 包，用于在激光雷达联调、驱动适配和数据排查阶段，检查并打印 `sensor_msgs/PointCloud2` 的消息布局与字段信息。

该项目基于**单一代码库**，通过 CMake 构建选项切换目标中间件，支持：
- ROS1 Noetic（`roscpp`）
- ROS2 Foxy / Humble（`rclcpp`）

---

## 功能特性

- 通过 CMake 选项选择构建目标：
  - `-DBUILD_ROS1=ON`
  - `-DBUILD_ROS2=ON`
- 配置阶段强制互斥：
  - 两者同时 ON -> 报错
  - 两者同时 OFF -> 报错
- 运行时参数：
  - `topic_name`（默认：`/points_raw`）
  - `print_every_n_msg`（默认：`1`，当 `<= 0` 时自动夹紧为 `1`）
- 打印 PointCloud2 消息级元数据 + 全部 `PointField` 字段详情。
- `datatype` 数值映射：
  - `1 INT8`、`2 UINT8`、`3 INT16`、`4 UINT16`
  - `5 INT32`、`6 UINT32`、`7 FLOAT32`、`8 FLOAT64`
  - 其它 -> `UNKNOWN`
- 健壮性检查：
  - `fields` 为空时告警
  - `width == 0` 时告警
  - `point_step == 0` 时告警
- 启动时一次性打印：
  - 当前运行模式（ROS1/ROS2）
  - 订阅话题
  - 打印间隔

---

## 目录结构

```text
pointcloud2_inspector/
├── CMakeLists.txt
├── package.xml
├── README.md
├── README.zh-CN.md
├── include/
│   └── pointcloud2_inspector/
│       └── pointcloud2_inspector.hpp
├── launch/
│   ├── inspector_ros1.launch
│   └── inspector_ros2.launch.py
└── src/
    ├── main_ros1.cpp
    ├── main_ros2.cpp
    └── pointcloud2_inspector.cpp
```

---

## 设计说明

### 关于单 package.xml 的取舍
ROS1（`catkin`）与 ROS2（`ament_cmake`）在构建系统上不同。为了保持单包结构，本项目使用 `package.xml` format 3，并基于 `ROS_VERSION` 使用条件依赖（REP-149 风格）。

这是一种工程上实用的折中方案：在常见 ROS1/ROS2 环境中可工作，同时保持代码库与目录统一。

---

## 构建说明

> 必须且只能启用一个构建选项：`BUILD_ROS1` 或 `BUILD_ROS2`。

### ROS1 Noetic

```bash
source /opt/ros/noetic/setup.bash

cd ~/catkin_ws/src
git clone <your_repo_url> pointcloud2_inspector
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release -DBUILD_ROS1=ON -DBUILD_ROS2=OFF
source devel/setup.bash
```

### ROS2 Foxy

```bash
source /opt/ros/foxy/setup.bash

cd ~/ros2_ws/src
git clone <your_repo_url> pointcloud2_inspector
cd ..
colcon build --packages-select pointcloud2_inspector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_ROS1=OFF -DBUILD_ROS2=ON
source install/setup.bash
```

### ROS2 Humble

```bash
source /opt/ros/humble/setup.bash

cd ~/ros2_ws/src
git clone <your_repo_url> pointcloud2_inspector
cd ..
colcon build --packages-select pointcloud2_inspector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_ROS1=OFF -DBUILD_ROS2=ON
source install/setup.bash
```

---

## 运行说明

## ROS1

### 默认参数运行

```bash
source ~/catkin_ws/devel/setup.bash
rosrun pointcloud2_inspector pointcloud2_inspector_node
```

### 自定义 `topic_name`

```bash
rosrun pointcloud2_inspector pointcloud2_inspector_node _topic_name:=/lidar/points
```

### 自定义 `print_every_n_msg`

```bash
rosrun pointcloud2_inspector pointcloud2_inspector_node _print_every_n_msg:=10
```

### 使用 ROS1 launch 文件

```bash
roslaunch pointcloud2_inspector inspector_ros1.launch
roslaunch pointcloud2_inspector inspector_ros1.launch topic_name:=/lidar/points print_every_n_msg:=5
```

## ROS2（Foxy / Humble）

### 默认参数运行

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pointcloud2_inspector pointcloud2_inspector_node
```

### 自定义 `topic_name`

```bash
ros2 run pointcloud2_inspector pointcloud2_inspector_node --ros-args -p topic_name:=/lidar/points
```

### 自定义 `print_every_n_msg`

```bash
ros2 run pointcloud2_inspector pointcloud2_inspector_node --ros-args -p print_every_n_msg:=10
```

### 使用 ROS2 launch 文件

```bash
ros2 launch pointcloud2_inspector inspector_ros2.launch.py
ros2 launch pointcloud2_inspector inspector_ros2.launch.py topic_name:=/lidar/points print_every_n_msg:=5
```

---

## 输出示例

```text
[INFO] [pointcloud2_inspector]: ========== PointCloud2 Inspector ==========
topic       : /points_raw
frame_id    : lidar_link
stamp       : sec=1710000000, nanosec=123456789, to_sec=1710000000.123456717
width/height: 1024 / 1
point_step  : 16
row_step    : 16384
is_dense    : true
is_bigendian: false
fields:
  [0] name=x, offset=0, datatype=7(FLOAT32), count=1
  [1] name=y, offset=4, datatype=7(FLOAT32), count=1
  [2] name=z, offset=8, datatype=7(FLOAT32), count=1
  [3] name=intensity, offset=12, datatype=7(FLOAT32), count=1
summary     : total fields=4
===========================================
```

---

## 补充说明

- 默认每条消息都打印（`print_every_n_msg=1`）。
- 如果点云频率很高，建议增大 `print_every_n_msg`，避免终端刷屏。
- 该工具用于“检查结构/字段布局”，不做点云内容解析与可视化。
