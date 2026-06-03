#ifndef YOLO_DETECTOR_HPP
#define YOLO_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>      // 引入 OpenCV DNN 模块
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 解析后的纯净 YOLO 目标结构体
 */
struct YoloObject 
{
    int color;                    // 颜色: 0->红, 1->蓝, 2->灰(熄灭), 3->紫
    int number;                   // 编号: 1(英雄) 2(工程) 3/4/5(步兵) 6(哨兵) 7(前哨站) 8(基地)
    bool is_big;                  // 尺寸: true->大装甲板(225mm), false->小装甲板(135mm)
    
    float prob;                   // 综合置信度
    cv::Rect box;                 // 生成的外接矩形框 (用于NMS)
    std::vector<cv::Point2f> pts; // 模型回归出的4个物理角点(左上,左下,右下,右上)
};

/**
 * @brief YOLO 目标检测器类 (基于 OpenCV DNN)
 */
class YoloDetector 
{
public:
    YoloDetector(const std::string& model_path);
    std::vector<YoloObject> Detect(const cv::Mat& raw_img, int enemy_color);
 
private:
    bool is_ready_ = false;              // 模型加载状态标志位

    cv::dnn::Net net_;                   // OpenCV DNN 网络对象

    // --- 深大模型的超参数 ---
    const float score_threshold_ = 0.65; // 置信度阈值
    const float nms_threshold_ = 0.45;   // NMS重叠阈值

    std::vector<int> class_ids;
    std::vector<int> color_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> all_keypoints;
    std::vector<int> indices;
};

#endif // YOLO_DETECTOR_HPP