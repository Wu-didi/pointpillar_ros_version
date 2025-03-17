**catkin_pkg
使用conda 安装**

使用colcon build 编译前，记得清空之前编译结果

```
rm -rf build log install
```

# 启动指令：

 ros2 run lidar_detection trt_infer_node   --ros-args   -p point_cloud_path:=/home/wudi/learn_tensorrt/PointPillars_ros2/box_data/1726305992941000000.bin   -p  trt_path:=/home/wudi/learn_tensorrt/pointpillar_ros_version/weight/model_52.trt

# 发布指定

ros2 run lidar_detection trt_infer_node --ros-args -p trt_path:=/home/wudi/learn_tensorrt/pointpillar_ros_version/weight/model_52.trt

# 查看ros的输出

你可以 `ros2 topic echo /detection_results` 观察字符串输出。

# 文件组织说明

```
├── include
│   ├── common_trt.h
│   ├── io_trt.hpp
│   ├── nms_trt.h
│   ├── utils.h
│   └── voxelization_trt.h
├── src
│   ├── nms_trt.cpp
│   ├── utils.cpp
│   ├── voxelization_trt.cpp
│   └── voxelization_trt.cu
├── test.txt
├── trt_infer.cpp # 原文件，c++纯推理
├── trt_infer_node_1.cpp # ros2节点，简单的推理
├── trt_infer_node.cpp    # 完整的订阅、检测、发布流程
└── trt_infer_node_fileresult.cpp # 将检测结果保存到txt，同时也是一个ros2节点
```
