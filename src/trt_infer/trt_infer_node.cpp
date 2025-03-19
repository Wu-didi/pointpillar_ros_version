#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <sstream>
#include <fstream>
#include <memory>
#include <cmath>

// ------------------ ROS2 相关头文件 ------------------
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// ------------------ TensorRT 头文件 ------------------
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include "NvInferPlugin.h"
#include <cuda_runtime.h>

#include "include/common_trt.h"
#include "include/io_trt.hpp"
#include "include/voxelization_trt.h"
#include "include/utils.h"
#include "include/nms_trt.h"

// 如果你已在工程里有 "common_trt.h" 或类似头文件包含下列结构体/函数声明，
// 请直接 #include "common_trt.h" ，并把这里的重复定义删掉；
// 否则，单文件示例里，我们先把核心结构体、函数都写上。

// ============================================================================
// 1) 常见数据结构: Point / Voxel / Box2d / Box3d / Box3dfull
// ============================================================================

// ============================================================================
// 2) Logger 与一些宏
// ============================================================================
class Logger : public nvinfer1::ILogger           
{
    void log(Severity severity, const char* msg) noexcept override
    {
        switch(severity) {
            case Severity::kINTERNAL_ERROR:
            case Severity::kERROR:
            case Severity::kWARNING:
                std::cerr << "[TensorRT] " << msg << std::endl;
                break;
            case Severity::kINFO:
            case Severity::kVERBOSE:
                // 可根据需要选择是否打印
                // std::cout << "[TensorRT] " << msg << std::endl;
                break;
        }
    }
} gLogger;

// 如果你原先有 CHECK 宏可保留，这里仅做示例
#define CHECK(status) \
    if (status != 0) \
    { \
        std::cerr << "Cuda failure: " << status << std::endl; \
        abort(); \
    }


// ============================================================================
// 4) trtInfer 两个版本 (引用你原先的cpp实现)。如果你已有 .cpp 实现，请删此处
// ============================================================================

// // 4.1 基于 std::vector<Voxel> 的 trtInfer
// void trtInfer(std::vector<Voxel>& voxels, 
//               std::vector<std::vector<int>>& coors, 
//               std::vector<int>& num_points_per_voxel, 
//               int& max_points,
//               nvinfer1::ICudaEngine* engine, 
//               nvinfer1::IExecutionContext* context, 
//               std::vector<float>& output)
// {
//     // 这里是你原先给出的实现，如下(截取版)：
//     int inputPillarsIndex = engine->getBindingIndex("input_pillars");
//     int inputCoorsBatchIndex = engine->getBindingIndex("input_coors_batch");
//     int inputNpointsPerPillarIndex = engine->getBindingIndex("input_npoints_per_pillar");
//     int outputIndex = engine->getBindingIndex("output_x");

//     int pillar_num = voxels.size();

//     void* inputPillarsDevice;
//     void* inputCoorsBatchDevice;
//     void* inputNpointsPerPillarDevice;

//     CHECK(cudaMalloc(&inputPillarsDevice, pillar_num * max_points * sizeof(Point)));
//     CHECK(cudaMalloc(&inputCoorsBatchDevice, pillar_num * 4 * sizeof(int)));
//     CHECK(cudaMalloc(&inputNpointsPerPillarDevice, pillar_num * sizeof(int)));

//     // 拷贝 host -> device
//     Point* tempHostMemoryPillar = new Point[pillar_num * max_points];
//     Point* currentHostPtrPillar = tempHostMemoryPillar;
//     for (const auto& voxel : voxels) {
//         memcpy(currentHostPtrPillar, voxel.points.data(), voxel.points.size() * sizeof(Point));
//         currentHostPtrPillar += voxel.points.size();
//     }

//     int* tempHostMemoryCoor = new int[pillar_num * 4];
//     int* currentHostPtrCoor = tempHostMemoryCoor;
//     for (const auto& c : coors) {
//         memcpy(currentHostPtrCoor, c.data(), c.size() * sizeof(int));
//         currentHostPtrCoor += c.size();
//     }

//     CHECK(cudaMemcpy(inputPillarsDevice, tempHostMemoryPillar, pillar_num * max_points * sizeof(Point), cudaMemcpyHostToDevice));
//     CHECK(cudaMemcpy(inputCoorsBatchDevice, tempHostMemoryCoor, pillar_num * 4 * sizeof(int), cudaMemcpyHostToDevice));
//     CHECK(cudaMemcpy(inputNpointsPerPillarDevice, num_points_per_voxel.data(), pillar_num * sizeof(int), cudaMemcpyHostToDevice));

//     delete[] tempHostMemoryPillar;
//     delete[] tempHostMemoryCoor;

//     // 分配输出
//     void* outputDevice;
//     CHECK(cudaMalloc(&outputDevice, output.size() * sizeof(float)));

//     // 设置 input dims
//     nvinfer1::Dims inputPilllarDims, inputCoorsDims, inputNpointsPerPillarDims;
//     inputPilllarDims.nbDims = 3;
//     inputPilllarDims.d[0] = pillar_num;
//     inputPilllarDims.d[1] = max_points;
//     inputPilllarDims.d[2] = sizeof(Point)/sizeof(float);

//     inputCoorsDims.nbDims = 2;
//     inputCoorsDims.d[0] = pillar_num;
//     inputCoorsDims.d[1] = 4;

//     inputNpointsPerPillarDims.nbDims = 1;
//     inputNpointsPerPillarDims.d[0] = pillar_num;

//     if (!context->setBindingDimensions(inputPillarsIndex, inputPilllarDims)) {
//         std::cerr << "setBindingDimensions error for input_pillars.\n";
//     }
//     if (!context->setBindingDimensions(inputCoorsBatchIndex, inputCoorsDims)) {
//         std::cerr << "setBindingDimensions error for input_coors_batch.\n";
//     }
//     if (!context->setBindingDimensions(inputNpointsPerPillarIndex, inputNpointsPerPillarDims)) {
//         std::cerr << "setBindingDimensions error for input_npoints_per_pillar.\n";
//     }

//     void* buffers[4];
//     buffers[inputPillarsIndex]          = inputPillarsDevice;
//     buffers[inputCoorsBatchIndex]       = inputCoorsBatchDevice;
//     buffers[inputNpointsPerPillarIndex] = inputNpointsPerPillarDevice;
//     buffers[outputIndex]                = outputDevice;

//     context->enqueueV2(buffers, 0, nullptr);

//     CHECK(cudaMemcpy(output.data(), outputDevice, output.size() * sizeof(float), cudaMemcpyDeviceToHost));

//     // 释放
//     cudaFree(inputPillarsDevice);
//     cudaFree(inputCoorsBatchDevice);
//     cudaFree(inputNpointsPerPillarDevice);
//     cudaFree(outputDevice);
// }

// 4.2 基于 GPU指针(float* d_voxels等)的 trtInfer
void trtInfer(float* d_voxels,
              int* d_coors,
              int* d_num_points_per_voxel,
              const int pillar_num,
              int& max_points,
              nvinfer1::ICudaEngine* engine,
              nvinfer1::IExecutionContext* context,
              std::vector<float>& output)
{
    int inputPillarsIndex          = engine->getBindingIndex("input_pillars");
    int inputCoorsBatchIndex       = engine->getBindingIndex("input_coors_batch");
    int inputNpointsPerPillarIndex = engine->getBindingIndex("input_npoints_per_pillar");
    int outputIndex                = engine->getBindingIndex("output_x");

    void* inputPillarsDevice;
    void* inputCoorsBatchDevice;
    void* inputNpointsPerPillarDevice;

    CHECK(cudaMalloc(&inputPillarsDevice,         pillar_num * max_points * sizeof(Point)));
    CHECK(cudaMalloc(&inputCoorsBatchDevice,      pillar_num * 4 * sizeof(int)));
    CHECK(cudaMalloc(&inputNpointsPerPillarDevice,pillar_num * sizeof(int)));

    CHECK(cudaMemcpy(inputPillarsDevice,         d_voxels,               pillar_num * max_points * sizeof(Point), cudaMemcpyDeviceToDevice));
    CHECK(cudaMemcpy(inputCoorsBatchDevice,      d_coors,                pillar_num * 4 * sizeof(int),            cudaMemcpyDeviceToDevice));
    CHECK(cudaMemcpy(inputNpointsPerPillarDevice,d_num_points_per_voxel, pillar_num * sizeof(int),                cudaMemcpyDeviceToDevice));

    // 可视需要在这里 free 传进来的 d_voxels/d_coors/d_num_points_per_voxel
    // 如果外部还要用，则别 free
    cudaFree(d_voxels);
    cudaFree(d_coors);
    cudaFree(d_num_points_per_voxel);

    void* outputDevice;
    CHECK(cudaMalloc(&outputDevice, output.size() * sizeof(float)));

    // 设置dims
    nvinfer1::Dims inputPilllarDims, inputCoorsDims, inputNpointsPerPillarDims;
    inputPilllarDims.nbDims = 3;
    inputPilllarDims.d[0]   = pillar_num;
    inputPilllarDims.d[1]   = max_points;
    inputPilllarDims.d[2]   = sizeof(Point)/sizeof(float);

    inputCoorsDims.nbDims = 2;
    inputCoorsDims.d[0]   = pillar_num;
    inputCoorsDims.d[1]   = 4;

    inputNpointsPerPillarDims.nbDims = 1;
    inputNpointsPerPillarDims.d[0]   = pillar_num;

    if (!context->setBindingDimensions(inputPillarsIndex,          inputPilllarDims)) {
        std::cerr << "setBindingDimensions error (pillar).\n";
    }
    if (!context->setBindingDimensions(inputCoorsBatchIndex,       inputCoorsDims)) {
        std::cerr << "setBindingDimensions error (coors).\n";
    }
    if (!context->setBindingDimensions(inputNpointsPerPillarIndex, inputNpointsPerPillarDims)) {
        std::cerr << "setBindingDimensions error (npoints).\n";
    }

    void* buffers[4];
    buffers[inputPillarsIndex]          = inputPillarsDevice;
    buffers[inputCoorsBatchIndex]       = inputCoorsBatchDevice;
    buffers[inputNpointsPerPillarIndex] = inputNpointsPerPillarDevice;
    buffers[outputIndex]                = outputDevice;

    context->enqueueV2(buffers, 0, nullptr);

    CHECK(cudaMemcpy(output.data(), outputDevice, output.size() * sizeof(float), cudaMemcpyDeviceToHost));

    cudaFree(inputPillarsDevice);
    cudaFree(inputCoorsBatchDevice);
    cudaFree(inputNpointsPerPillarDevice);
    cudaFree(outputDevice);
}

// ============================================================================
// 5) postProcessing
// ============================================================================
void postProcessing(std::vector<float>& output,
                    int& num_class,
                    float& nms_thr,
                    float& score_thr,
                    int& max_num,
                    std::vector<Box3dfull>& bboxes_full)
{
    std::vector<Box2d> bboxes_2d;
    std::vector<Box3d> bboxes_3d;
    std::vector<std::vector<float>> scores_list;
    std::vector<float> direction_list;

    // 1) decode
    decodeDetResults(output, num_class, bboxes_2d, bboxes_3d, scores_list, direction_list);

    // 2) 按类别做分数过滤 + NMS
    std::vector<Box3dfull> bboxes_3d_nms;
    for (int i = 0; i < num_class; i++){
        std::vector<int> score_filter_inds;
        std::vector<float> scores;
        filterByScores(i, scores_list, score_thr, score_filter_inds, scores);

        std::vector<Box2d> bboxes_2d_filtered;
        std::vector<Box3d> bboxes_3d_filtered;
        std::vector<float> direction_filtered;
        obtainBoxByInds(score_filter_inds, bboxes_2d, bboxes_2d_filtered,
                        bboxes_3d, bboxes_3d_filtered,
                        direction_list, direction_filtered);

        std::vector<int> nms_filter_inds;
        nms(bboxes_2d_filtered, scores, nms_thr, nms_filter_inds);

        for (auto ind : nms_filter_inds) {
            Box3dfull box3d_full;
            box3d_full.x     = bboxes_3d_filtered[ind].x;
            box3d_full.y     = bboxes_3d_filtered[ind].y;
            box3d_full.z     = bboxes_3d_filtered[ind].z;
            box3d_full.w     = bboxes_3d_filtered[ind].w;
            box3d_full.l     = bboxes_3d_filtered[ind].l;
            box3d_full.h     = bboxes_3d_filtered[ind].h;
            float limited_th = limitPeriod(bboxes_3d_filtered[ind].theta);
            box3d_full.theta = (1.f - direction_filtered[ind]) * float(M_PI) + limited_th;
            box3d_full.score = scores[ind];
            box3d_full.label = i;
            bboxes_3d_nms.push_back(box3d_full);
        }
    }

    // 3) 只保留前 max_num
    getTopkBoxes(bboxes_3d_nms, max_num, bboxes_full);
}

// ============================================================================
// 6) TensorRT Engine 初始化（如你所示例）
// ============================================================================


std::pair<nvinfer1::ICudaEngine*, nvinfer1::IExecutionContext*> 
initializeTensorRTComponents(const std::string& engineFilePath)
{
    bool didInitPlugins = initLibNvInferPlugins(nullptr, "");
    if (!didInitPlugins) {
        std::cerr << "initLibNvInferPlugins failed!\n";
        return {nullptr, nullptr};
    }

    auto engineData = readEngineFile(engineFilePath);
    if (engineData.empty()) {
        return {nullptr, nullptr};
    }

    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(gLogger);
    if (!runtime) {
        std::cerr << "Failed to create TRT runtime." << std::endl;
        return {nullptr, nullptr};
    }

    nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(engineData.data(), engineData.size(), nullptr);
    if (!engine) {
        std::cerr << "Failed to create TRT engine." << std::endl;
        runtime->destroy(); // deprecated warning 可暂时忽略
        return {nullptr, nullptr};
    }

    nvinfer1::IExecutionContext* context = engine->createExecutionContext();
    if (!context) {
        std::cerr << "Failed to create TRT context." << std::endl;
        engine->destroy();
        runtime->destroy();
        return {nullptr, nullptr};
    }

    runtime->destroy(); // 释放runtime(若不需重复加载)
    return {engine, context};
}

// ============================================================================
// 7) 转换 sensor_msgs::msg::PointCloud2 -> std::vector<Point>
// ============================================================================
void convertCloud2ToPoints(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg,
                           std::vector<Point> &points_out,
                           rclcpp::Logger logger)
{
    points_out.clear();
    points_out.reserve(cloud_msg->width * cloud_msg->height);

    // 查找 x, y, z, intensity 对应的 offset
    int offset_x = -1, offset_y = -1, offset_z = -1, offset_i = -1;
    for (auto &field : cloud_msg->fields) {
        if (field.name == "x") {
            offset_x = field.offset;
        } else if (field.name == "y") {
            offset_y = field.offset;
        } else if (field.name == "z") {
            offset_z = field.offset;
        } else if (field.name == "intensity") {
            offset_i = field.offset;
        }
    }

    // 基本检查
    if (offset_x < 0 || offset_y < 0 || offset_z < 0) {
        RCLCPP_ERROR(logger, "PointCloud2 fields do not match x,y,z float32!");
        return;
    }
    // 对于 intensity，可以允许没有（offset_i < 0 表示可能没有），
    // 如果一定需要 intensity 才能继续，则也可以在这里 return

    const size_t total_points = cloud_msg->width * cloud_msg->height;
    const uint8_t *ptr = cloud_msg->data.data();

    for (size_t i = 0; i < total_points; ++i) {
        float px = *reinterpret_cast<const float*>(ptr + offset_x);
        float py = *reinterpret_cast<const float*>(ptr + offset_y);
        float pz = *reinterpret_cast<const float*>(ptr + offset_z);

        Point pt;
        pt.x = px;
        pt.y = py;
        pt.z = pz;
        
        // 如果找到了 intensity 字段，就解析它
        if (offset_i >= 0) {
            float intensity = *reinterpret_cast<const float*>(ptr + offset_i);
            pt.feature = intensity;
        } else {
            pt.feature = 0.0f;  // 或者给一个默认值
        }

        points_out.push_back(pt);

        // 跳到下一个点的数据开始位置
        ptr += cloud_msg->point_step;
    }
}

// ============================================================================
// 8) (可选) 如果你想把“一次推理流程”封装成一个函数 doOneInference
//    但可视需要，也可直接在回调里写
// ============================================================================
std::vector<Box3dfull> doOneInference(const std::vector<Point> &points,
                                      nvinfer1::ICudaEngine* engine,
                                      nvinfer1::IExecutionContext* context)
{
    // 示例: GPU 体素化 => TRT推理 => 后处理

    // 这些参数可改成全局或动态
    std::vector<float> voxel_size   = {0.16f, 0.16f, 4.0f};
    std::vector<float> coors_range  = {0.f, -39.68f, -3.f, 69.12f, 39.68f, 1.f};
    int max_points = 32;
    int max_voxels = 40000;
    int NDim       = 3;

    // 1) 分配 GPU buffer
    int* d_num_points_per_voxel = nullptr;
    CHECK(cudaMalloc((void**)&d_num_points_per_voxel, max_voxels * sizeof(int)));
    CHECK(cudaMemset(d_num_points_per_voxel, 0, max_voxels * sizeof(int)));

    float* d_voxels = nullptr;
    CHECK(cudaMalloc((void**)&d_voxels, max_voxels * max_points * sizeof(Point)));
    CHECK(cudaMemset(d_voxels, 0.f, max_voxels * max_points * sizeof(Point)));

    int* d_coors = nullptr;
    CHECK(cudaMalloc((void**)&d_coors, max_voxels * NDim * sizeof(int)));
    CHECK(cudaMemset(d_coors, 0, max_voxels * NDim * sizeof(int)));

    // 2) voxelize
    int voxel_num = voxelizeGpu(points, voxel_size, coors_range,
                                max_points, max_voxels,
                                d_voxels, d_coors, d_num_points_per_voxel,
                                NDim);

    // 3) pad coors
    int* d_coors_padded = nullptr;
    CHECK(cudaMalloc((void**)&d_coors_padded, voxel_num*(NDim+1)*sizeof(int)));
    CHECK(cudaMemset(d_coors_padded, 0, voxel_num*(NDim+1)*sizeof(int)));

    padCoorsGPU(d_coors, d_coors_padded, voxel_num);
    cudaFree(d_coors);

    // 4) TRT推理
    int num_class = 3;
    int num_box   = 100;
    std::vector<float>   output(num_box*(7+num_class+1));
    // 这里用 GPU指针版本:
    trtInfer(d_voxels, d_coors_padded, d_num_points_per_voxel,
             voxel_num, max_points, engine, context, output);

    // 5) 后处理
    float nms_thr   = 0.01f;
    float score_thr = 0.1f;
    int max_num     = 50;
    std::vector<Box3dfull> bboxes;
    postProcessing(output, num_class, nms_thr, score_thr, max_num, bboxes);

    return bboxes;
}

// ============================================================================
// 9) ROS2 Node 定义
// ============================================================================
class TrtInferNode : public rclcpp::Node
{
public:
    TrtInferNode()
    : Node("trt_infer_node")
    {
        this->declare_parameter<std::string>("trt_path", "");
        trt_path_ = this->get_parameter("trt_path").as_string();
        if (trt_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "必须通过 -p trt_path:=/path/to/model.trt 指定引擎路径");
            return;
        }

        auto engine_context = initializeTensorRTComponents(trt_path_);
        engine_  = engine_context.first;
        context_ = engine_context.second;
        if (!engine_ || !context_) {
            RCLCPP_ERROR(this->get_logger(), "加载 TensorRT 引擎失败");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "成功加载 TensorRT 引擎: %s", trt_path_.c_str());

        // 订阅点云
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "bin_points", 10,   // 改成你实际的话题
            std::bind(&TrtInferNode::pointCloudCallback, this, std::placeholders::_1)
        );

        // 发布检测结果（文本示例）
        result_pub_ = this->create_publisher<std_msgs::msg::String>("detection_results", 10);

        RCLCPP_INFO(this->get_logger(), "TrtInferNode 启动，等待点云...");
    }

    ~TrtInferNode()
    {
        // 释放资源：这会有 deprecated 警告，可忽略或升级API
        if (context_) { context_->destroy();  context_ = nullptr; }
        if (engine_)  { engine_->destroy();   engine_  = nullptr; }
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        if (!engine_ || !context_) {
            RCLCPP_WARN(this->get_logger(), "引擎/上下文无效, 无法推理");
            return;
        }

        // 1) 转为 std::vector<Point>
        std::vector<Point> points;
        convertCloud2ToPoints(cloud_msg, points, this->get_logger());
        if (points.empty()) {
            RCLCPP_WARN(this->get_logger(), "空点云, 跳过");
            return;
        }

        // 2) 执行一次推理
        auto bboxes = doOneInference(points, engine_, context_);

        // 3) 发布结果
        std_msgs::msg::String result_msg;
        std::ostringstream oss;
        oss << "检测到 " << bboxes.size() << " 个box:\n";
        for (size_t i = 0; i < bboxes.size(); ++i) {
            const auto& box = bboxes[i];
            oss << "  [" << i << "]: x=" << box.x << ", y=" << box.y << ", z=" << box.z
                << ", w=" << box.w << ", l=" << box.l << ", h=" << box.h
                << ", theta=" << box.theta << ", score=" << box.score << ", label=" << box.label << "\n";
        }
        result_msg.data = oss.str();
        result_pub_->publish(result_msg);

        // 也可以在日志打印
        RCLCPP_INFO(this->get_logger(), "%s", result_msg.data.c_str());
    }

    // ROS 相关
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
    // 参数
    std::string trt_path_;
    // TRT
    nvinfer1::ICudaEngine* engine_{nullptr};
    nvinfer1::IExecutionContext* context_{nullptr};
};

// ============================================================================
// 10) main
// ============================================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrtInferNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
