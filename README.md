# pointcloud2_inspector

`pointcloud2_inspector` is a production-style ROS package for inspecting incoming `sensor_msgs/PointCloud2` metadata and field layout during LiDAR bring-up, sensor integration, and driver adaptation.

It supports a **single shared codebase** with a **build-time switch** for:
- ROS1 Noetic (`roscpp`)
- ROS2 Foxy/Humble (`rclcpp`)

---

## Features

- Build-mode selection via CMake options:
  - `-DBUILD_ROS1=ON`
  - `-DBUILD_ROS2=ON`
- Enforced mutual exclusivity at configure time.
- Runtime parameters:
  - `topic_name` (default: `/points_raw`)
  - `print_every_n_msg` (default: `1`, clamped to `1` if `<= 0`)
- Prints message-level PointCloud2 metadata and all `PointField` entries.
- Datatype mapping:
  - `1 INT8`, `2 UINT8`, `3 INT16`, `4 UINT16`, `5 INT32`, `6 UINT32`, `7 FLOAT32`, `8 FLOAT64`, else `UNKNOWN`.
- Validation warnings if:
  - `fields` is empty
  - `width == 0`
  - `point_step == 0`
- Startup log includes middleware mode, topic, and print interval.

---

## Directory tree

```text
pointcloud2_inspector/
├── CMakeLists.txt
├── package.xml
├── README.md
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

## Design notes

### Why one package.xml can be awkward
ROS1 (`catkin`) and ROS2 (`ament_cmake`) use different build tools and metadata expectations. A truly universal package manifest is sometimes awkward in mixed environments.

This project uses a **single `package.xml` (format 3)** with **conditional dependencies** based on `ROS_VERSION` (REP-149 style). In practice this is a pragmatic compromise that works for typical ROS1/ROS2 environments while keeping one shared package layout.

---

## Build instructions

> Build with **exactly one** of `BUILD_ROS1` or `BUILD_ROS2` set to `ON`.

If both are ON or both are OFF, CMake stops with a configure error.

### ROS1 Noetic

```bash
# terminal 1
source /opt/ros/noetic/setup.bash

# catkin workspace assumed
cd ~/catkin_ws/src
git clone <your_repo_url> pointcloud2_inspector
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release -DBUILD_ROS1=ON -DBUILD_ROS2=OFF
source devel/setup.bash
```

### ROS2 Foxy

```bash
# terminal 1
source /opt/ros/foxy/setup.bash

# colcon workspace assumed
cd ~/ros2_ws/src
git clone <your_repo_url> pointcloud2_inspector
cd ..
colcon build --packages-select pointcloud2_inspector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_ROS1=OFF -DBUILD_ROS2=ON
source install/setup.bash
```

### ROS2 Humble

```bash
# terminal 1
source /opt/ros/humble/setup.bash

# colcon workspace assumed
cd ~/ros2_ws/src
git clone <your_repo_url> pointcloud2_inspector
cd ..
colcon build --packages-select pointcloud2_inspector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_ROS1=OFF -DBUILD_ROS2=ON
source install/setup.bash
```

---

## Run instructions

## ROS1

### Default topic and interval

```bash
source ~/catkin_ws/devel/setup.bash
rosrun pointcloud2_inspector pointcloud2_inspector_node
```

### Custom `topic_name`

```bash
rosrun pointcloud2_inspector pointcloud2_inspector_node _topic_name:=/lidar/points
```

### Custom `print_every_n_msg`

```bash
rosrun pointcloud2_inspector pointcloud2_inspector_node _print_every_n_msg:=10
```

### ROS1 launch file

```bash
roslaunch pointcloud2_inspector inspector_ros1.launch
roslaunch pointcloud2_inspector inspector_ros1.launch topic_name:=/lidar/points print_every_n_msg:=5
```

## ROS2 (Foxy/Humble)

### Default topic and interval

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pointcloud2_inspector pointcloud2_inspector_node
```

### Custom `topic_name`

```bash
ros2 run pointcloud2_inspector pointcloud2_inspector_node --ros-args -p topic_name:=/lidar/points
```

### Custom `print_every_n_msg`

```bash
ros2 run pointcloud2_inspector pointcloud2_inspector_node --ros-args -p print_every_n_msg:=10
```

### ROS2 launch file

```bash
ros2 launch pointcloud2_inspector inspector_ros2.launch.py
ros2 launch pointcloud2_inspector inspector_ros2.launch.py topic_name:=/lidar/points print_every_n_msg:=5
```

---

## Example output

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

## Notes

- The inspector prints every received message by default (`print_every_n_msg=1`).
- Invalid `print_every_n_msg <= 0` is automatically clamped to `1` and logged.
- If your sensor publishes very high rate clouds, increase `print_every_n_msg` to reduce console load.
