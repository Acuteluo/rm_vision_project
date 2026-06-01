#include "yolo_detector.hpp"


// 【提取自同济源码的终极映射表】
// 定义: color (0红, 1蓝, 2灰, 3紫), number (1~8), is_big (大/小板)
struct ArmorProperty 
{ 
    int color; 
    int number; 
    bool is_big; 
};

const ArmorProperty ARMOR_PROPS[38] = 
{
    /*0-2:   哨兵小*/ {1, 6, false}, {0, 6, false}, {2, 6, false},
    /*3-5:   英雄小*/ {1, 1, false}, {0, 1, false}, {2, 1, false},
    /*6-8:   工程小*/ {1, 2, false}, {0, 2, false}, {2, 2, false},
    /*9-11:  步兵3小*/{1, 3, false}, {0, 3, false}, {2, 3, false},
    /*12-14: 步兵4小*/{1, 4, false}, {0, 4, false}, {2, 4, false},
    /*15-17: 步兵5小*/{1, 5, false}, {0, 5, false}, {2, 5, false},
    /*18-20: 前哨站小*/{1, 7, false}, {0, 7, false}, {2, 7, false},
    /*21-24: 基地大*/ {1, 8, true},  {0, 8, true},  {2, 8, true},  {3, 8, true},
    /*25-28: 基地小*/ {1, 8, false}, {0, 8, false}, {2, 8, false}, {3, 8, false},
    /*29-31: 步兵3大*/{1, 3, true},  {0, 3, true},  {2, 3, true},
    /*32-34: 步兵4大*/{1, 4, true},  {0, 4, true},  {2, 4, true},
    /*35-37: 步兵5大*/{1, 5, true},  {0, 5, true},  {2, 5, true}
};



YoloDetector::YoloDetector(const std::string& model_path) 
{
    RCLCPP_INFO(rclcpp::get_logger("YoloDetector"), "[YOLO] 正在初始化 OpenVINO 模型...");

    try
    {
        // 加入这两行测试设备插件！
        std::vector<std::string> available_devices = core_.get_available_devices();
        for (auto&& device : available_devices) {
            RCLCPP_INFO(rclcpp::get_logger("YoloDetector"), "可用硬件加速设备: %s", device.c_str());
        }

        // 1. 读取 XML 网络拓扑结构和 BIN 权重参数
        auto model = core_.read_model(model_path);

        // =========================================================================
        // 2. PrePostProcessor (PPP) 硬件加速预处理
        // 作用：将原本需要 CPU 用 OpenCV 做的图像格式转换，内嵌到模型指令中，
        // 利用 CPU 的向量指令集(AVX)或 iGPU/NPU 实现零拷贝极速转换。
        // =========================================================================
        ov::preprocess::PrePostProcessor ppp(model);
        auto & input = ppp.input();

        // 声明你传入 C++ 的图片长什么样：u8(8位无符号即0~255)，尺寸 640x640，通道 BGR，内存排列为 NHWC (通道在最后)
        input.tensor()
        .set_element_type(ov::element::u8)
        .set_shape({1, 640, 640, 3})
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);

        // 声明模型训练时要求的排列：NCHW (通道在前面)
        input.model().set_layout("NCHW");

        // 声明预处理操作：转为 f32 浮点数，BGR 转 RGB，数值除以 255.0 进行归一化
        input.preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale(255.0);

        // 固化这些预处理操作到模型中
        model = ppp.build();

        // =========================================================================
        // 3. 编译模型并分配硬件资源
        // "CPU" : 使用 Core Ultra 5 的 CPU 算力
        // "GPU" : 如果系统配置正确，可使用 Intel Arc 核显
        // "NPU" : 针对神经网络特化的加速器
        // 性能提示设置为 LATENCY(延迟优先)，保证摄像头每一帧能以最快速度出结果
        // =========================================================================
        compiled_model_ = core_.compile_model(model, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
        
        // 创建推理上下文句柄，后续所有的 detect 都在此内存空间进行，避免重复分配
        infer_request_ = compiled_model_.create_infer_request();
        
        RCLCPP_INFO(rclcpp::get_logger("YoloDetector"), "[YOLO] 模型编译加载成功！硬件准备就绪。");
    }
    catch (const ov::Exception& e) 
    {
        // 如果 OpenVINO 报错，这里会把具体的错误原因打印出来
        RCLCPP_ERROR(rclcpp::get_logger("YoloDetector"), "[致命错误] OpenVINO 崩溃: %s", e.what());
    }
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("YoloDetector"), "[致命错误] C++ 标准异常: %s", e.what());
    }
    
}

std::vector<YoloObject> YoloDetector::Detect(const cv::Mat& raw_img) 
{
    if (raw_img.empty()) return {};

    // =========================================================================
    // [模块1]：图像缩放与填充 (左上角对齐的 Letterbox)
    // 作用：将任意长宽比的相机画面，不丢失比例地放入 640x640 的模型输入框内
    // =========================================================================
    auto x_scale = static_cast<double>(640) / raw_img.rows;
    auto y_scale = static_cast<double>(640) / raw_img.cols;
    auto scale = std::min(x_scale, y_scale); // 以最长边为基准计算缩放比例
    
    // 缩放后的实际长宽
    auto h = static_cast<int>(raw_img.rows * scale);
    auto w = static_cast<int>(raw_img.cols * scale);

    // 创建 640x640 全黑底板
    cv::Mat input_img = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    
    // 将缩放后的画面贴在黑板的 左上角(0, 0)。
    // 好处：后续解码还原坐标时，x 和 y 不用减去 padding 黑边偏移量！
    cv::Rect roi(0, 0, w, h);
    cv::resize(raw_img, input_img(roi), {w, h});

    // =========================================================================
    // [模块2]：执行神经网络推理
    // =========================================================================
    // 将处理好的图绑定到内存指针 (零拷贝技术)
    ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input_img.data);
    infer_request_.set_input_tensor(input_tensor);
    
    // 推理
    infer_request_.infer();

    // =========================================================================
    // [模块3]：获取输出张量并解析 (Post-Processing)
    // =========================================================================
    auto output_tensor = infer_request_.get_output_tensor();
    auto output_shape = output_tensor.get_shape(); // 形如 [1, 50, 8400]
    
    // 极其精妙的 OpenCV 转换：用 cv::Mat 包裹一维内存，并进行转置 (transpose)
    // 转置后变为 [8400, 50]，意味着每一行 (row) 就是一个备选框的所有信息，极易遍历！
    cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
    cv::transpose(output, output); 

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> all_keypoints;

    // 遍历所有的 8400 个盲猜框
    for (int r = 0; r < output.rows; r++) 
    {
        // 提取前 4 位：cx, cy, w, h
        auto xywh = output.row(r).colRange(0, 4);
        // 提取中间 38 位：各个分类的概率
        auto scores = output.row(r).colRange(4, 4 + class_num_);
        // 提取最后 8 位：四个角点的 x 和 y (同济优化版，去掉了关键点置信度)
        auto kpts = output.row(r).colRange(4 + class_num_, 50);

        // 寻找概率最大的类别
        double max_score;
        cv::Point max_point;
        cv::minMaxLoc(scores, nullptr, &max_score, nullptr, &max_point);

        // 【阈值过滤】：垃圾框不要
        if (max_score < score_threshold_) continue;

        // 【坐标还原】：YOLO 算出的是在 640x640 上的坐标，现在要除以 scale 还原到 原图 尺度
        float x = xywh.at<float>(0);
        float y = xywh.at<float>(1);
        float width = xywh.at<float>(2);
        float height = xywh.at<float>(3);
        
        // 转换为左上角坐标，提供给 OpenCV 的 NMS 使用
        int left = static_cast<int>((x - 0.5 * width) / scale);
        int top = static_cast<int>((y - 0.5 * height) / scale);
        int box_w = static_cast<int>(width / scale);
        int box_h = static_cast<int>(height / scale);

        // 还原 4 个角点坐标
        std::vector<cv::Point2f> armor_kpts;
        for (int i = 0; i < 4; i++) 
        {
            float kx = kpts.at<float>(0, i * 2) / scale;
            float ky = kpts.at<float>(0, i * 2 + 1) / scale;
            armor_kpts.push_back(cv::Point2f(kx, ky));
        }

        // 把合格的数据先暂存起来
        class_ids.push_back(max_point.x);
        confidences.push_back(static_cast<float>(max_score));
        boxes.push_back(cv::Rect(left, top, box_w, box_h));
        all_keypoints.push_back(armor_kpts);
    }


    // =========================================================================
    // [模块4]：非极大值抑制 (NMS)
    // 作用：同一个装甲板如果被画了 3 个框，利用重叠率(IoU)过滤，只保留概率最大的 1 个
    // =========================================================================
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

    // =========================================================================
    // [模块5]：打包最终结果
    // =========================================================================
    std::vector<YoloObject> final_objects;
    for (int idx : indices) 
    {
        YoloObject obj;
        obj.class_id = class_ids[idx];
        obj.prob = confidences[idx];
        obj.box = boxes[idx];   // 保存需要的 Box
        obj.pts = all_keypoints[idx];

        // 【最核心】：将散乱的关键点按明确规律排序
        SortKeypoints(obj.pts);
        MapArmorProperty(obj);  // 调用翻译函数，把 ID 转为人话
        
        final_objects.push_back(obj);
    }

    return final_objects;
}

void YoloDetector::SortKeypoints(std::vector<cv::Point2f>& keypoints) 
{
    if (keypoints.size() != 4) return;

    // 1. 先按 y 坐标把四个点分成“上半部分”和“下半部分”
    std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.y < b.y;
    });

    std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
    std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

    // 2. 上面两个点再按 x 排序 -> [0]是左上，[1]是右上
    std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });

    // 3. 下面两个点再按 x 排序 -> [0]是左下，[1]是右下
    std::sort(bottom_points.begin(), bottom_points.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });

    // 4. 重组成与 ArmorPlate::local_corners 严格对应的顺序！
    // 你的坐标系定义：0->左上, 1->左下, 2->右下, 3->右上
    // 如果顺序错乱，PnP 算出来的姿态会直接发散倒转！
    keypoints[0] = top_points[0];     // 左上
    keypoints[1] = bottom_points[0];  // 左下
    keypoints[2] = bottom_points[1];  // 右下
    keypoints[3] = top_points[1];     // 右上
}


// 【自动翻译机制】
void YoloDetector::MapArmorProperty(YoloObject& obj) 
{
    if (obj.class_id >= 0 && obj.class_id < 38) 
    {
        // 获取映射表中对应 class_id 的属性
        obj.color = ARMOR_PROPS[obj.class_id].color;
        obj.number = ARMOR_PROPS[obj.class_id].number;
        obj.is_big = ARMOR_PROPS[obj.class_id].is_big;
    } 
    else 
    {
        // 异常兜底保护
        obj.color = 2; // 默认灰色
        obj.number = 0; 
        obj.is_big = false;
    }
}