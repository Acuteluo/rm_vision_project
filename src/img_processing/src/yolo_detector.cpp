#include "yolo_detector.hpp"
#include <fstream>
#include <cmath>

inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

YoloDetector::YoloDetector(const std::string& model_path) 
{
    // 强制开启底层数学优化 (会唤醒我们刚才编译的 OpenBLAS 和 AVX2)
    cv::setUseOptimized(true);

    RCLCPP_INFO(rclcpp::get_logger("YoloDetector"), "[YOLO] 正在初始化 OpenCV DNN 模型...");

    std::ifstream f(model_path.c_str());
    if (!f.good()) {
        RCLCPP_ERROR(rclcpp::get_logger("YoloDetector"), "[YOLO] 找不到模型: %s", model_path.c_str());
        return; 
    }

    try 
    {
        net_ = cv::dnn::readNetFromONNX(model_path);

        // 原代码是锁定 CPU：
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU); 

        // 【修改为】：强行调用你刚刚编译进 OpenCV 4.9.0 的 OpenCL 显卡加速模块！
        // net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        // net_.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);

        is_ready_ = true; 
        RCLCPP_INFO(rclcpp::get_logger("YoloDetector"), "[YOLO] OpenCV DNN 模型加载配置成功 (运行在 CPU)！");

        RCLCPP_INFO(rclcpp::get_logger("YoloDetector"), "OpenCV threads: %d", cv::getNumThreads());
    } 
    catch (const cv::Exception& e) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("YoloDetector"), "[YOLO] DNN 异常: %s", e.what());
    }
}

std::vector<YoloObject> YoloDetector::Detect(const cv::Mat& raw_img, int enemy_color) 
{
    if (!is_ready_ || raw_img.empty()) return {};

    cv::Mat blob = cv::dnn::blobFromImage(raw_img, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true, false);
    
    float scale_x = static_cast<float>(raw_img.cols) / 640.0f; 
    float scale_y = static_cast<float>(raw_img.rows) / 640.0f;

    net_.setInput(blob);
    std::vector<cv::Mat> net_outputs;

    try {
        net_.forward(net_outputs, net_.getUnconnectedOutLayersNames());
    } catch (const cv::Exception& e) {
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME); // 声明静态时钟
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("YoloDetector"), steady_clock, 1000, 
            "[YOLO] 推理失败: %s", e.what());
        return {};
    }

    if (net_outputs.empty()) return {};

    cv::Mat output = net_outputs[0];
    int rows = 0, cols = 0;
    float* data_ptr = (float*)output.data;

    if (output.dims == 3) {
        rows = output.size[1];
        cols = output.size[2];
    } else if (output.dims == 2) {
        rows = output.size[0];
        cols = output.size[1];
    } else {
        return {};
    }

    cv::Mat output_buffer;
    if (rows < cols) {
        cv::Mat temp(rows, cols, CV_32F, data_ptr);
        cv::transpose(temp, output_buffer); 
    } else {
        output_buffer = cv::Mat(rows, cols, CV_32F, data_ptr);
    }

    // 【优化】：每帧只做清空，不做分配
    class_ids.clear(); color_ids.clear(); confidences.clear(); 
    boxes.clear(); all_keypoints.clear(); 

    // 【优化】：使用指针直接访问内存，不要用 .at()！
    float* data = (float*)output_buffer.data;
    int stride = output_buffer.cols;

    for (int r = 0; r < output_buffer.rows; r++) 
    {
        float* row_ptr = data + r * stride; // 获取当前行头指针
        float conf = sigmoid(row_ptr[8]);
        if (conf < score_threshold_) continue;

        cv::Mat color_scores = output_buffer.row(r).colRange(9, 13);
        cv::Mat class_scores = output_buffer.row(r).colRange(13, 22);

        cv::Point class_id_pt, color_id_pt;
        cv::minMaxLoc(class_scores, nullptr, nullptr, nullptr, &class_id_pt);
        cv::minMaxLoc(color_scores, nullptr, nullptr, nullptr, &color_id_pt);

        int raw_color = color_id_pt.x; 
        int raw_class = class_id_pt.x; 

        // 恢复在网络底层的直接过滤，极大减轻 NMS 和 PnP 的计算压力！
        if (raw_color == 2 || raw_color == 3) continue;
        if (raw_color != enemy_color) continue;

        std::vector<cv::Point2f> armor_kpts(4);
        float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
        for (int i = 0; i < 4; i++) 
        {
            float kx = output_buffer.at<float>(r, i * 2) * scale_x;
            float ky = output_buffer.at<float>(r, i * 2 + 1) * scale_y;
            
            armor_kpts[i] = cv::Point2f(kx, ky);

            if (kx < min_x) min_x = kx;
            if (kx > max_x) max_x = kx;
            if (ky < min_y) min_y = ky;
            if (ky > max_y) max_y = ky;
        }

        cv::Rect box(static_cast<int>(min_x), static_cast<int>(min_y), 
                     static_cast<int>(max_x - min_x), static_cast<int>(max_y - min_y));

        class_ids.push_back(raw_class);
        color_ids.push_back(raw_color);
        confidences.push_back(conf);
        boxes.push_back(box);
        all_keypoints.push_back(armor_kpts);
    }

    indices.clear();
    if (!boxes.empty()) {
        cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices); // 会按置信度从大到小返回保留下来的框的索引！
    }

    std::vector<YoloObject> final_objects;
    for (int idx : indices) 
    {
        YoloObject obj;
        obj.prob = confidences[idx];
        obj.box = boxes[idx];
        obj.pts = all_keypoints[idx];
        obj.color = color_ids[idx];

        int raw_class = class_ids[idx];
        int number_map[] = {6, 1, 2, 3, 4, 5, 7, 8, 8};
        
        if(raw_class >= 0 && raw_class <= 8) 
        {
            obj.number = number_map[raw_class];
            obj.is_big = (raw_class == 1 || raw_class == 8); 
        } 
        else 
        {
            obj.number = 0;
            obj.is_big = false;
        }
        
        final_objects.push_back(obj);
    }

    return final_objects;
}