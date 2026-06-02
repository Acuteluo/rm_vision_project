#ifndef YOLO_DETECTOR_HPP
#define YOLO_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/dnn.hpp>      // 新增：使用 OpenCV 的 DNN 预处理

/**
 * @brief 解析后的纯净 YOLO 目标结构体
 * 包含装甲板的所有物理信息和语义信息，作为后续解算层(PnP)的输入
 */
struct YoloObject 
{
    int class_id;                 // 原始类别 ID (0-37)
    
    // 【新增翻译后属性】
    int color;                    // 颜色: 0->红, 1->蓝, 2->灰(熄灭), 3->紫
    int number;                   // 编号: 1(英雄) 2(工程) 3/4/5(步兵) 6(哨兵) 7(前哨站) 8(基地)
    bool is_big;                  // 尺寸: true->大装甲板(225mm), false->小装甲板(135mm)
    
    float prob;                   // 置信度 0.00 - 1.00
    cv::Rect box;                 // 目标的 2D 边框 (xywh)
    std::vector<cv::Point2f> pts; // 排序好的 4 个物理角点 (左上, 左下, 右下, 右上)
};

/**
 * @brief YOLO 目标检测器类 (基于 Intel OpenVINO)
 * 职责：只负责“看图说话”，输入单帧图像，输出清洗好的 2D 目标列表
 */
class YoloDetector 
{
public:
    /**
     * @brief 构造函数，初始化 OpenVINO 推理引擎并加载模型
     * @param model_path .xml 模型文件的绝对路径
     */
    YoloDetector(const std::string& model_path);

    /**
     * @brief 核心检测函数
     * @param raw_img 相机传来的原始图像 (cv::Mat)
     * @return 经过 NMS 和坐标还原后的 2D 目标列表
     */
    std::vector<YoloObject> Detect(const cv::Mat& raw_img);
 
private:
    ov::Core core_;                      // OpenVINO 核心对象，负责管理所有插件
    ov::CompiledModel compiled_model_;   // 编译后的可执行模型 (已针对具体硬件优化)
    ov::InferRequest infer_request_;     // 推理请求句柄，管理输入输出内存

    // --- 模型超参数 (严格对应同济训练时的设定) ---
    const int class_num_ = 38;           // 模型训练的类别总数
    const float score_threshold_ = 0.7;  // 置信度阈值：低于 70% 把握的框直接丢弃，防止噪点乱飘
    const float nms_threshold_ = 0.3;    // NMS重叠阈值：两个框重叠面积 > 30% 则认为识别了同一个装甲板，杀掉低分框

    // --- 内部功能函数 ---
    /**
     * @brief 强行物理排序关键点
     * 消除网络输出关键点的乱序，严格匹配 PnP 解算时自定义的 3D 本地坐标系顺序
     */
    void SortKeypoints(std::vector<cv::Point2f>& keypoints);

    // 【新增】用于自动翻译 38 分类 
    void MapArmorProperty(YoloObject& obj); 
};

#endif // YOLO_DETECTOR_HPP