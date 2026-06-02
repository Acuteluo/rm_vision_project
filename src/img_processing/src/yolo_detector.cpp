#include "yolo_detector.hpp"
#include <fstream>
#include <cmath>

inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

YoloDetector::YoloDetector(const std::string& model_path) 
{
    RCLCPP_INFO(rclcpp::get_logger("YoloDetector"), "[YOLO] 正在初始化 OpenCV DNN 模型...");

    std::ifstream f(model_path.c_str());
    if (!f.good()) {
        RCLCPP_ERROR(rclcpp::get_logger("YoloDetector"), "[YOLO] 找不到模型: %s", model_path.c_str());
        return; 
    }

    try 
    {
        net_ = cv::dnn::readNetFromONNX(model_path);
        
        // =========================================================================
        // 【核心修改】：强制使用纯 CPU 运算！彻底避开 OpenCL (ocl4dnn) 的底层 Bug
        // =========================================================================
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU); // 锁定 CPU

        is_ready_ = true; 
        RCLCPP_INFO(rclcpp::get_logger("YoloDetector"), "[YOLO] OpenCV DNN 模型加载配置成功 (运行在 CPU)！");
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

    std::vector<int> class_ids;
    std::vector<int> color_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> all_keypoints;

    for (int r = 0; r < output_buffer.rows; r++) 
    {
        float conf = sigmoid(output_buffer.at<float>(r, 8));
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

    std::vector<int> indices;
    if (!boxes.empty()) {
        cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);
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