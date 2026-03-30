# pointcloud2_inspector 模块说明

## 作用
用于快速查看驱动发布的 `sensor_msgs/PointCloud2` 消息结构，重点检查：
- frame_id / 时间戳
- width/height / point_step / row_step
- is_dense / is_bigendian
- PointField 列表（name/offset/datatype/count）

## 参数
- `topic_name`：订阅话题，默认 `/points_raw`
- `print_every_n_msg`：每 N 帧打印一次，`<=0` 自动夹紧到 `1`

## 运行示例
ROS1:
```bash
rosrun lidar_tools pointcloud2_inspector_node _topic_name:=/points_raw _print_every_n_msg:=5
```

ROS2:
```bash
ros2 run lidar_tools pointcloud2_inspector_node --ros-args -p topic_name:=/points_raw -p print_every_n_msg:=5
```

## 输出说明
输出按“消息头 + 字段列表 + 警告 + summary”组织，便于 bring-up 阶段快速判定字段是否匹配算法预期。
