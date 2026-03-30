# lidar_preprocess_tester 模块说明

## 设计理念（重点）
本模块采用类似 Point-LIO preprocess 的工程思想，但进一步通用化：
- 只提供一个统一节点：`lidar_preprocess_tester_node`
- 节点内部按 `lidar_type` 分发到不同 handler
- 通过参数切换雷达，不通过改代码或换节点
- 便于后续扩展更多雷达类型

## 支持的 LiDAR 类型
- 1: VELO16
- 2: LS16
- 3: RS16
- 4: VELO16SIM
- 5: AVIA
- 6: MID360
- 7: HESAIXT32
- 8: JT128

## 关键参数
- `topic_name`：输入点云话题
- `lidar_type`：雷达类型枚举
- `blind`：盲区阈值（米）
- `point_filter_num`：点抽样步长
- `n_scans`：线数限制（ring 过滤）
- `sort_by_time`：是否按时间排序（JT128 推荐可开）
- `publish_full_cloud` / `publish_surf_cloud`：是否发布调试输出

## MID360 / JT128 特化行为
- MID360：`pcl::fromROSMsg` + 自定义点类型 + tag/ring/盲区/采样处理 + 统计输出
- JT128：`pcl::fromROSMsg` + 自定义点类型 + NaN/Inf/ring/盲区/时间顺序检查 + 可选时间排序 + 统计输出

## 发布输出
- `/lidar_preprocess_tester/pl_full`
- `/lidar_preprocess_tester/pl_surf`

## 运行示例
ROS2 MID360：
```bash
ros2 run lidar_tools lidar_preprocess_tester_node --ros-args -p topic_name:=/mid360/points -p lidar_type:=6 -p blind:=1.2 -p point_filter_num:=2
```

ROS2 JT128：
```bash
ros2 run lidar_tools lidar_preprocess_tester_node --ros-args -p topic_name:=/jt128/points -p lidar_type:=8 -p sort_by_time:=true
```

## 新增雷达类型流程
1. 增加枚举
2. 增加 enum 与字符串/参数映射
3. 在 `process()` 增加 dispatch
4. 在 ROS wrapper 增加消息解析与点类型转换
5. 更新配置与文档
