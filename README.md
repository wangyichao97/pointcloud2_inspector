# lidar_tools（统一 LiDAR 工具包）

`lidar_tools` 是一个统一的 LiDAR 工程化工具包，面向以下场景：
- 驱动联调：确认 `PointCloud2` 字段布局是否符合预期。
- 预处理验证：在接入 Point-LIO/LIO/SLAM 前验证 ring、时间戳、盲区、降采样、排序等规则。
- 长期扩展：在单一大包内持续增加新的 LiDAR 类型处理器与调试模块。

## 设计哲学

- **一个统一的预处理测试节点**：`lidar_preprocess_tester_node`。
- **内部多处理器分发**：通过 `lidar_type` 参数切换，不为每个雷达单独建节点。
- **模块化共享代码**：`common/` 提供通用工具，`pointcloud2_inspector/` 和 `preprocess/` 复用。
- **兼容 ROS1/ROS2**：单代码树，通过 CMake 选项二选一构建。

## 功能模块

1. `pointcloud2_inspector`
   - 打印 PointCloud2 元数据与每个 PointField。
   - 参数：`topic_name`、`print_every_n_msg`。

2. `lidar_preprocess_tester`
   - 中央类：`LidarPreprocessTester`。
   - 枚举支持：`VELO16/LS16/RS16/VELO16SIM/AVIA/MID360/HESAIXT32/JT128`。
   - MID360 与 JT128 提供更完整处理逻辑：
     - `pcl::fromROSMsg`
     - ring 过滤 / 非有限值过滤 / 盲区过滤 / 降采样
     - 时间顺序检查（JT128）
     - 相对时间写入输出点的 `intensity`（等价“调试曲率槽位”）
   - 可选发布：
     - `/lidar_preprocess_tester/pl_full`
     - `/lidar_preprocess_tester/pl_surf`

## 目录结构

```text
lidar_tools/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── lidar_preprocess_tester.yaml
│   └── pointcloud2_inspector.yaml
├── docs/
│   ├── lidar_preprocess_tester/README.md
│   └── pointcloud2_inspector/README.md
├── include/lidar_tools/
│   ├── common/
│   │   ├── pointcloud_types.hpp
│   │   ├── pointcloud_utils.hpp
│   │   └── preprocess_utils.hpp
│   ├── pointcloud2_inspector/
│   │   └── pointcloud2_inspector.hpp
│   └── preprocess/
│       ├── custom_points.hpp
│       └── lidar_preprocess_tester.hpp
├── launch/
│   ├── lidar_preprocess_tester_ros1.launch
│   ├── lidar_preprocess_tester_ros2.launch.py
│   ├── pointcloud2_inspector_ros1.launch
│   └── pointcloud2_inspector_ros2.launch.py
└── src/
    ├── common/
    │   ├── pointcloud_utils.cpp
    │   └── preprocess_utils.cpp
    ├── pointcloud2_inspector/
    │   ├── node_ros1.cpp
    │   ├── node_ros2.cpp
    │   └── pointcloud2_inspector.cpp
    └── preprocess/
        ├── lidar_preprocess_tester.cpp
        ├── node_ros1.cpp
        └── node_ros2.cpp
```

## 构建（ROS1/ROS2 二选一）

> 必须满足：仅一个 ON。否则 CMake 直接报错。

### ROS1 Noetic

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/src
git clone <repo> lidar_tools
cd ..
catkin_make -DBUILD_ROS1=ON -DBUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### ROS2 Foxy

```bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws/src
git clone <repo> lidar_tools
cd ..
colcon build --packages-select lidar_tools --cmake-args -DBUILD_ROS1=OFF -DBUILD_ROS2=ON -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### ROS2 Humble

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
git clone <repo> lidar_tools
cd ..
colcon build --packages-select lidar_tools --cmake-args -DBUILD_ROS1=OFF -DBUILD_ROS2=ON -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 运行

### 1) pointcloud2_inspector

ROS1：
```bash
rosrun lidar_tools pointcloud2_inspector_node _topic_name:=/points_raw _print_every_n_msg:=1
```

ROS2：
```bash
ros2 run lidar_tools pointcloud2_inspector_node --ros-args -p topic_name:=/points_raw -p print_every_n_msg:=1
```

### 2) lidar_preprocess_tester

ROS1：
```bash
rosrun lidar_tools lidar_preprocess_tester_node _topic_name:=/points_raw _lidar_type:=6 _blind:=1.0 _point_filter_num:=1 _n_scans:=6 _sort_by_time:=false
```

ROS2：
```bash
ros2 run lidar_tools lidar_preprocess_tester_node --ros-args -p topic_name:=/points_raw -p lidar_type:=6 -p blind:=1.0 -p point_filter_num:=1 -p n_scans:=6 -p sort_by_time:=false
```

### 3) 指定 JT128

ROS2 示例：
```bash
ros2 run lidar_tools lidar_preprocess_tester_node --ros-args -p topic_name:=/jt128_points -p lidar_type:=8 -p sort_by_time:=true
```

### 4) 指定 MID360

ROS2 示例：
```bash
ros2 run lidar_tools lidar_preprocess_tester_node --ros-args -p topic_name:=/mid360/points -p lidar_type:=6 -p blind:=1.5 -p point_filter_num:=2
```

## 如何新增一种 LiDAR

1. 在 `LidarType` 增加枚举值（`include/lidar_tools/common/preprocess_utils.hpp`）。
2. 在 `lidarTypeFromInt()` 与 `lidarTypeToString()` 增加映射（`src/common/preprocess_utils.cpp`）。
3. 在 `LidarPreprocessTester::process()` 中扩展 dispatch 分支（`src/preprocess/lidar_preprocess_tester.cpp`）。
4. 在 ROS1/ROS2 节点 `convertPoints()` 中增加该雷达消息解析逻辑（通常 `pcl::fromROSMsg + 自定义点类型`）。
5. 更新配置与文档。

## 模块文档

- `docs/pointcloud2_inspector/README.md`
- `docs/lidar_preprocess_tester/README.md`
