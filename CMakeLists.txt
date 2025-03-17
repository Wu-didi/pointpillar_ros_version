cmake_minimum_required(VERSION 3.16)
project(lidar_detection LANGUAGES CXX CUDA)


# 原先设置的 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 不给 CUDA STANDARD 赋值或只赋 c++14
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# 使用 ament_cmake 以便 colcon 能够识别并构建
find_package(ament_cmake REQUIRED)

# 如果你需要 rclcpp 或 std_msgs，手动添加，如下(可选):
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# 如果你有 TensorRT 的根路径
set(TENSORRT_ROOT "/home/wudi/TensorRT-8.5.1.7")

# 查找 CUDA
find_package(CUDA REQUIRED)
if(CUDA_FOUND)
    message(STATUS "CUDA library status:")
    message(STATUS "    version: ${CUDA_VERSION}")
    message(STATUS "    libraries: ${CUDA_LIBRARIES}")
    message(STATUS "    include path: ${CUDA_INCLUDE_DIRS}")
endif()

# 链接 TensorRT 库所在目录
link_directories(${TENSORRT_ROOT}/lib)

# include 路径
include_directories(
    src/trt_infer/include
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_ROOT}/include
)

# 源文件
set(SOURCE_FILES
    src/trt_infer/trt_infer_node.cpp
    src/trt_infer/src/voxelization_trt.cpp
    src/trt_infer/src/utils.cpp
    src/trt_infer/src/nms_trt.cpp
    src/trt_infer/src/voxelization_trt.cu
)



# 生成可执行文件
add_executable(trt_infer_node ${SOURCE_FILES})

# 强制设置 trt_infer_node 使用 CUDA 14 标准
set_target_properties(trt_infer_node PROPERTIES CUDA_STANDARD 14 CUDA_STANDARD_REQUIRED ON)


# 链接 TensorRT 和 CUDA
target_link_libraries(trt_infer_node
    nvinfer
    nvinfer_plugin
    ${CUDA_LIBRARIES}
)

# 如果你使用 rclcpp 等，需要:
ament_target_dependencies(trt_infer_node
  rclcpp
  std_msgs
  sensor_msgs
)

# 安装可执行文件到 ROS2 的 install 目录
install(TARGETS trt_infer_node
  DESTINATION lib/${PROJECT_NAME}
)

# 关键: 声明这是一个 ament_cmake 包
ament_package()
