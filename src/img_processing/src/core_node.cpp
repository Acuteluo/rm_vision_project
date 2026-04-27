// 我的 pitch 是你的 yaw
// 我的 roll 是你的 pitch
// 我的 yaw 是你的 roll

#include "prepare_algorithm.h"
#include "tf.hpp"
#include "ekf.hpp"
#include "kf_data.hpp"
#include "kf_position.hpp"

class ProcessNode: public rclcpp::Node
{
public:
    ProcessNode(): Node("core_node_cpp")
    {

        // 改为动态传参：

        // 声明参数（带默认值）

        // core 节点
        this->declare_parameter("core.logger.show_logger_time", true); // 是否打印时间相关日志
        this->declare_parameter("core.logger.show_logger_else", false); // 是否打印corenode其他的相关日志
        this->declare_parameter("core.image.show_img", true); // 是否显示图片
        
        // prepare 类
        this->declare_parameter("core.logger.show_logger_prepare", false); // 是否打印 prepare 类中的日志
        this->declare_parameter("core.param.chosen_color", "red"); // 选择的装甲板颜色
        this->declare_parameter("core.param.camera_name", "galaxy"); // 使用的相机名称
        this->declare_parameter("core.param.armor_type", "normal"); // 识别装甲板的类型

        // EKF相关（传递给ekf_）
        this->declare_parameter("ekf.predict_time", 0.05); // 预测时间
        this->declare_parameter("ekf.show_logger_debug", true); // ekf 调试

        this->declare_parameter("ekf.q_x", 0.05);
        this->declare_parameter("ekf.q_y", 0.05);
        this->declare_parameter("ekf.q_z", 0.05);
        this->declare_parameter("ekf.q_v_x", 0.5);
        this->declare_parameter("ekf.q_v_y", 0.5);
        this->declare_parameter("ekf.q_v_z", 0.5);
        this->declare_parameter("ekf.q_yaw", 0.008);
        this->declare_parameter("ekf.q_omega", 0.05);
        this->declare_parameter("ekf.q_a_omega", 0.1);
        this->declare_parameter("ekf.r_x", 0.05);
        this->declare_parameter("ekf.r_y", 0.05);
        this->declare_parameter("ekf.r_z", 0.1);
        this->declare_parameter("ekf.r_yaw", 0.4);
        this->declare_parameter("ekf.radius", 0.25);

        // TF 参数声明
        this->declare_parameter("tf.show_logger_error", false);
        this->declare_parameter("tf.show_result", true);
        this->declare_parameter("is_standalone", false); // 单机 / 联调模式


        // corenode 节点变量获取初始值
        this->SHOW_LOGGER_TIME = this->get_parameter("core.logger.show_logger_time").as_bool();
        this->SHOW_LOGGER_ELSE = this->get_parameter("core.logger.show_logger_else").as_bool();
        this->SHOW_IMG_SHOW = this->get_parameter("core.image.show_img").as_bool();
        this->SHOW_LOGGER_PREPARE = this->get_parameter("core.logger.show_logger_prepare").as_bool();
        this->CHOSEN_COLOR = this->get_parameter("core.param.chosen_color").as_string();
        this->CAMERA_NAME = this->get_parameter("core.param.camera_name").as_string();
        this->ARMOR_TYPE = this->get_parameter("core.param.armor_type").as_string();
        this->PREDICT_TIME = this->get_parameter("ekf.predict_time").as_double();
        this->SHOW_LOGGER_DEBUG = this->get_parameter("ekf.show_logger_debug").as_bool();
        this->IS_STANDALONE = this->get_parameter("is_standalone").as_bool();


        // 注册参数变化回调（用于运行时动态修改）
        param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ProcessNode::onParameterChange, this, std::placeholders::_1));

        // --------------------------- 配置 -> 相机相关 qos sub pub ---------------------------


        // 声明 QoS 参数

        // 适用于 mind_vision 的 qos
        this->declare_parameter("use_sensor_data_qos", false);
        bool qos1 = this->get_parameter("use_sensor_data_qos").as_bool();
        
        // 适用于 galaxy 的 qos
        auto qos2 = rclcpp::SensorDataQoS(); 

        // 根据 CAMERA_NAME 选择 qos，以创建订阅原图方
        if(this->CAMERA_NAME == "mind_vision") sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", qos1, std::bind(&ProcessNode::process_callback, this, std::placeholders::_1));
        else sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", qos2, std::bind(&ProcessNode::process_callback, this, std::placeholders::_1));

        // 发布 pnp 消息
        serial_pub_ = this->create_publisher<serial_driver_interfaces::msg::SerialDriver>("/serial_driver", 10);

        RCLCPP_INFO_ONCE(this->get_logger(), "CoreNode 节点创建成功! ");

        // 初始化pnp帧率的计时器
        this->last_print = this->now();

        this->tf = std::make_unique<TF>(this); // 传 this 指针给 TF 类来构造，让它能创建 ROS2 相关对象，同时发布静态变换

        // 初始化 KF 类 和 EKF 类 指针
        // kf_position_ = std::make_unique<KalmanFilter>(); 
        // kf_data_ = std::make_unique<KF>(); 
        ekf_ = std::make_unique<EKF>(this); 
        ekf_->updateParamsFromServer(); // 设置一大堆参数
        ekf_->setParam(this->ARMOR_TYPE); // 装甲板类型
        ekf_->setDebugLogger(this->SHOW_LOGGER_DEBUG);

        // 初始化上一次收到图像的时间，一切的 dt 都依照这个来算
        this->last_image_time = this->now(); 

        prepare.setParam(this->SHOW_LOGGER_PREPARE, this->CHOSEN_COLOR, this->CAMERA_NAME, this->ARMOR_TYPE); // 传入从配置文件读取的参数
    }


private:

    // 核心逻辑
    void process_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rclcpp::Time start = this->now();
    
        // 1. 接收图像 并 记录时间和更新 dt，dt 是整个的基准！！
        this->img = cv_bridge::toCvCopy(msg, "bgr8")->image; // 需要 SharedPtr 作为参数
        this->img_show = this->img.clone(); // 复制一份用来显示信息
        
        rclcpp::Time current_image_time = msg->header.stamp; // 从消息头获取图像的时间戳
        double dt = (current_image_time - this->last_image_time).seconds(); // 计算与上一次图像的时间间隔，注意sec已经是精确时间！ 
        this->last_image_time = current_image_time; // 更新上一次图像的时间戳


        // ... 步骤1 接收图像完成时间戳
        rclcpp::Time t1 = this->now();


        // 2. 预处理（掩码、灯条筛选、灯条配对）
        prepare.setImgShow(this->img_show); // 设置 img_show
        rclcpp::Time t1_1 = this->now();
        prepare.preProcessing(img); // 图像预处理
        rclcpp::Time t1_2 = this->now();
        this->strip = prepare.findAndJudgeLightStrip(); // 找灯带 返回灯带集合
        rclcpp::Time t1_3 = this->now();
		this->armorplate = prepare.pairStrip(); // 灯条配对 【修改】返回按置信度排序后的装甲板集合
        rclcpp::Time t1_4 = this->now();
        this->img_show = prepare.getImgShow(); // 获取预处理后带有信息的 img_show

        // ... 步骤2 预处理完成时间戳
        rclcpp::Time t2 = this->now();
        


        // 3. 解算 pnp
        if(this->armorplate.size() > 0) // 存在装甲板
        {
            this->armorplate[0].setImgShow(this->img_show); // 设置 img_show
            this->armorplate[0].perspectiveNPoint(); // 解算 pnp
            this->armorplate[0].drawArmorPlateAndPrintPNPInfo(this->CHOSEN_COLOR); // 画置信度最高的装甲板，并打印 pnp 信息
            this->img_show = this->armorplate[0].getImgShow(); // 获取带有装甲板信息的 img_show
        }

        // ... 步骤3 pnp结算完成时间戳
        rclcpp::Time t3 = this->now();


        
        // 4. 发送 tf 变换消息 + 滤波
        int CONTINUOUS_THRESHOLD = 2; // 连续发布的帧数阈值，丢弃前面连续的 2 帧不稳定数据
        int FILTER_INIT_THRESHOLD = 2; // 有效帧被用来初始化滤波器的帧数

        /*
            逻辑：
                有数据
                    连续很少帧 0 - 2 检测到，
                        初始两帧直接丢
                    连续一般帧 3 - 4 检测到，
                        初始化滤波器
                    连续很多帧 >= 5 检测到，
                        ekf 稳定
                        那无论怎么丢都外推（tf 查询失败）
                    
                没数据
                    ekf 稳定
                        那无论怎么丢都外推
                    ekf 不稳定
                        那直接重置得了
        */

        double pitch_result = -999;
        double yaw_result = -999;


        Eigen::Vector3d armorplate_center_predict; // 预测点位置
        Eigen::Vector3d car_center_predict; // 整车中心的预测点位置


        // 首先 pnp结算必须要有结果 才能发布tf变换
        if(this->armorplate.size() > 0 && this->armorplate[0].is_success) 
        {
            // 引入分层丢失机制，根据丢失帧数来采取不同策略
            if(this->lost_count > 0)
            {
                if(this->lost_count <= MAX_LOST_COUNT * 0.1)
                {
                    // 丢失很少，基本可以认为是偶尔的误检或者轻微的遮挡，相信 ekf 外推，继续保持当前状态，等待恢复
                    RCLCPP_WARN(this->get_logger(), "【轻度丢失】%d 帧后找回, 啥也不动, 相信 ekf", this->lost_count);
                }
                else if(this->lost_count < MAX_LOST_COUNT * 0.4)
                {
                    // 丢失一般，可能是持续的遮挡或者误检，直接重置滤波器，并用这一帧来初始化喵
                    this->ekf_->reset(); 
                    this->tracking_id = 3; // 重置追踪装甲板 ID
                    RCLCPP_WARN(this->get_logger(), "【中度丢失】%d 帧后找回，仅仅重置滤波器！", this->lost_count);
                }
                else
                {
                    // 丢太多了，放弃吧
                    this->ekf_ready = false; // 丢的太多了 不ready
                    this->continuous_count = 0; // 数据作废
                    this->ekf_->reset(); // 重置滤波器为未初始化状态
                    this->tracking_id = 3; // 重置追踪装甲板 ID
                    RCLCPP_WARN(this->get_logger(), "【重度丢失】%d 帧后找回，全部作废重来", this->lost_count);
                }
            }


            ++this->continuous_count; // 连续计数器 + 1
            this->lost_count = 0; // 连续丢失计数器清零

            // 丢弃前面 2 帧不稳定的数据，纯丢。这两个数据不计重复丢失次数
            if(this->continuous_count <= CONTINUOUS_THRESHOLD)  
            {
                RCLCPP_WARN_EXPRESSION(this->get_logger(), this->SHOW_LOGGER_ELSE, "已丢弃连续帧的第 %d 帧", this->continuous_count);
                showImg();
                return;
            }


            // 发送【相机坐标系】->【装甲板坐标系】的 tf，仅仅是可视化使用，不可能刚发就查毕竟查不到喵，会有延迟
            this->tf->updateCameraToArmorplate(this->armorplate[0].R, this->armorplate[0].t_vec, msg->header.stamp);
        
            // 查找【父坐标系】->【装甲板坐标系】变换，也就是实时值
            // 单机模式下是【相机坐标系】->【装甲板坐标系】，联调模式下是【世界坐标系】->【装甲板坐标系】

            Eigen::Vector3d armorplate_center_now; // 实时点位置
            double yaw_armorplate_now; // 实时装甲板偏航角
            bool flag = this->tf->getFatherToArmorplateTransform(this->armorplate[0].R, this->armorplate[0].t_vec, armorplate_center_now, yaw_armorplate_now, msg->header.stamp); // 获取父坐标系到装甲板坐标系的变换，单机模式下父坐标系是相机坐标系，联调模式下父坐标系是世界坐标系。返回值表示是否成功获取变换

            // 如果查到了变换，就开始滤波
            if(flag)
            {
                // 修改：增加猜板和切板的逻辑

                if(this->ekf_ready)
                {
                    double min_diff = 10.0;
                    // EKF 中定义：1前(PI), 2右(PI/2), 3后(0), 4左(-PI/2)
                    double offsets[4] = {M_PI, M_PI/2, 0.0, -M_PI/2}; 
                    for (int i = 0; i < 4; i++) 
                    {
                        // 根据 EKF 内部的底盘朝向，推算出这 4 块板子【理论上应该在什么角度】
                        double target_yaw = this->ekf_->X(6) + offsets[i]; 
                        
                        // 将【理论角度】与【你手里的实际角度】进行相减，并用 atan2 规范化到 [-pi, pi]
                        double diff = std::abs(std::atan2(std::sin(target_yaw - yaw_armorplate_now), std::cos(target_yaw - yaw_armorplate_now)));
                        
                        // 找出一个误差最小的，那就是你当前正拿着的这面！
                        if (diff < min_diff) 
                        {
                            min_diff = diff;
                            this->tracking_id = i + 1; // 得到 1, 2, 3, 4
                        }
                    }
                }

                this->ekf_->setParam(this->ARMOR_TYPE); // 设置装甲板尺寸，方便后期转换装甲板的四个角点到世界下
                this->ekf_->getKalman(armorplate_center_now, yaw_armorplate_now, this->tracking_id, dt);
                
                armorplate_center_predict = armorplate_center_now; // 【3-4帧】先用观测值

                // 只有 用有效帧的前几帧初始化了滤波器 才用预测值，否则就用观测值
                if(this->continuous_count > CONTINUOUS_THRESHOLD + FILTER_INIT_THRESHOLD)
                {
                    this->ekf_ready = true; // 已经稳定，无论怎么丢我都外推
                    this->ekf_->getArmorPredict(armorplate_center_predict, this->tracking_id, this->PREDICT_TIME);
                    this->ekf_->getCenterPredict(car_center_predict, this->PREDICT_TIME);
                }
                else
                {
                    RCLCPP_INFO_EXPRESSION(this->get_logger(), this->SHOW_LOGGER_ELSE, "第 %d 帧，正在初始化滤波器，使用了观测值", this->continuous_count);
                }

            }
            else // 丢失方法2：没查到变换
            {
                ++this->lost_count; // 连续丢失计数器 + 1
                if(this->IS_STANDALONE) RCLCPP_WARN(this->get_logger(), "未查到【相机坐标系】->【装甲板坐标系】变换");
                else RCLCPP_WARN(this->get_logger(), "未查到【世界坐标系】->【装甲板坐标系】变换");

                if(this->ekf_ready && this->lost_count <= this-> MAX_LOST_COUNT)
                {
                    this->ekf_->predictOnly(dt);  // 继续预测 dt 时间，维持状态外推

                    // 用外推的信息发
                    this->ekf_->getArmorPredict(armorplate_center_predict, this->tracking_id, this->PREDICT_TIME);
                    this->ekf_->getCenterPredict(car_center_predict, this->PREDICT_TIME);
                }
                else
                {
                    this->ekf_ready = false; // 丢的太多了 不ready
                    this->continuous_count = 0; // 数据作废
                    this->tracking_id = 3; // 重置追踪装甲板 ID
                    this->ekf_->reset(); // 重置滤波器为未初始化状态。因为当前帧已经没有用了，等下一帧初始化
                    RCLCPP_WARN_EXPRESSION(this->get_logger(), this->SHOW_LOGGER_ELSE, "有目标但未查到变换，目标丢失超过 %d 帧。或者滤波器未初始化好。EKF 已重置", this->MAX_LOST_COUNT);
                    showImg();
                    return;
                }
            }

        }
        else // 丢失方法1：不存在装甲板 或者 pnp解算失败
        {
            ++this->lost_count; // 连续丢失计数器 + 1

            // 提示信息
            if(this->armorplate.size() == 0) 
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 500, "未检测到装甲板");
            }
            else RCLCPP_ERROR(this->get_logger(), "pnp 解算失败");


            if(this->ekf_ready && this->lost_count <= this-> MAX_LOST_COUNT)
            {
                this->ekf_->predictOnly(dt);  // 继续预测 dt 时间，维持状态外推

                // 用外推的信息发
                this->ekf_->getArmorPredict(armorplate_center_predict, this->tracking_id, this->PREDICT_TIME);
                this->ekf_->getCenterPredict(car_center_predict, this->PREDICT_TIME);
            }
            else
            {
                this->ekf_ready = false; // 丢的太多了 不ready
                this->continuous_count = 0; // 数据作废
                this->tracking_id = 3; // 重置追踪装甲板 ID
                this->ekf_->reset(); // 重置滤波器为未初始化状态。因为当前帧已经没有用了，等下一帧初始化
                RCLCPP_WARN_EXPRESSION(this->get_logger(), this->SHOW_LOGGER_ELSE, "不存在装甲板 或者 pnp解算失败。目标丢失超过 %d 帧, EKF 已重置", this->MAX_LOST_COUNT);
                showImg();
                return;
            }

        }

        // 计算最终角（滤波器没初始好就用原始值，连续检测多就用预测值，丢帧但滤波器稳定就用外推值）
        pitch_result = -std::atan2(armorplate_center_predict[2], std::sqrt(armorplate_center_predict[0] * armorplate_center_predict[0] + armorplate_center_predict[1] * armorplate_center_predict[1])) * 180.0 / M_PI;   
        yaw_result = std::atan2(armorplate_center_predict[1], armorplate_center_predict[0]) * 180.0 / M_PI;
        RCLCPP_INFO_EXPRESSION(this->get_logger(), this->SHOW_LOGGER_ELSE, "armorplate_center_predict: [%.3f, %.3f, %.3f]", armorplate_center_predict[0], armorplate_center_predict[1], armorplate_center_predict[2]);

        
        // ... 步骤4 发布 tf变换 + 滤波 完成时间戳
        rclcpp::Time t4 = this->now();



        // 5. 重投影 ekf_->getArmorFourCorners 拿到的是实时值
        // ========== 重投影可视化 ==========
        if (this->ekf_ready && pitch_result != -999 && yaw_result != -999)
        {
            // 先获取相机内参和畸变系数（可从当前处理的装甲板对象获取，因为 ArmorPlate 已保存）
            //    注意：如果本帧没有成功检测到装甲板，则无法获取相机参数，此时跳过重投影。
            if (!this->armorplate.empty() && this->armorplate[0].is_success)
            {
                // 再查询 camera → 父坐标系 的 TF，因为要转到相机下投影！
                tf2::Transform T_world_cam;
                if (this->tf->getCameraToWorldTransform(T_world_cam, msg->header.stamp)) 
                {
                    // 对每个装甲板 ID 进行重投影（用不同颜色区分）（实时滤波位置）
                    std::vector<cv::Scalar> colors = 
                    {
                        cv::Scalar(0, 255, 0),   // 前: 绿
                        cv::Scalar(0, 165, 255), // 右: 橙
                        cv::Scalar(255, 255, 255), // 后: 白
                        cv::Scalar(255, 255, 0)  // 左: 青
                    };

                    for (int id = 1; id <= 4; id++)
                    {
                        std::vector<Eigen::Vector3d> corners_world;
                        this->ekf_->getArmorFourCorners(corners_world, id); // id=1 前, id=2 右, id=3 后, id=4 左
                        projectAndDraw(corners_world, this->armorplate[0].K, this->armorplate[0].D, T_world_cam, colors[id - 1]);
                    }

                    // 也绘制整车中心点（实时滤波位置）
                    std::vector<Eigen::Vector3d> center_world(1);
                    this->ekf_->getCenterPredict(center_world[0], 0.00); // 获取实时位置下的整车中心点 在父坐标系下 的位置坐标
                    projectAndDraw(center_world, this->armorplate[0].K, this->armorplate[0].D, T_world_cam, cv::Scalar(255, 255, 255));
                
                    // 也绘制当前装甲板 dt后的 预测值（不是外推值）
                    std::vector<Eigen::Vector3d> armorplate_center_world(1);
                    armorplate_center_world[0] = armorplate_center_predict; // 预测位置的装甲板中心点 在父坐标系下 的位置坐标
                    projectAndDraw(armorplate_center_world, this->armorplate[0].K, this->armorplate[0].D, T_world_cam, cv::Scalar(255, 0, 255));
                }
            }
        }
        
            
        
        // 6. 发送 最终信息
        auto send_msg = serial_driver_interfaces::msg::SerialDriver();
        send_msg.pitch = pitch_result;
        send_msg.yaw = yaw_result;
        serial_pub_->publish(send_msg);
        RCLCPP_INFO_EXPRESSION(this->get_logger(), this->SHOW_LOGGER_ELSE, "已发送信息");


        // 7. 联调模式下，在图上打印要发布的目标角度 和 电控发来的目前芯片姿态
        if(!this->IS_STANDALONE)
        {
            double pitch_chip;
            double yaw_chip;
            bool flag = this->tf->getWorldToChipTransform(pitch_chip, yaw_chip, msg->header.stamp); // 获取【世界坐标系】->【芯片坐标系的变换】
            
            if(flag)
            {
                // 1. 提前计算好差值 (目标值 - 当前查到的值)
                double pitch_diff = pitch_chip - pitch_result;
                double yaw_diff = yaw_chip - yaw_result;

                // 2. 打印第一行 Target 信息 (Y 坐标 = 700，白色)
                cv::putText(this->img_show, 
                            cv::format("Target: pitch = %.2f, yaw = %.2f", pitch_result, yaw_result), 
                            cv::Point2f(0, 700), 
                            cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);

                // 3. 打印第二行 Current 信息及差值 (Y 坐标下移到 740 防止重叠，我顺手给你用上了樱花粉方便区分)
                cv::putText(this->img_show, 
                            cv::format("Current: pitch = %.2f ( %+.2f ), yaw = %.2f ( %+.2f )", 
                                    pitch_chip, pitch_diff, yaw_chip, yaw_diff), 
                            cv::Point2f(0, 750), 
                            cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(193, 182, 255), 2.5);
            }
        }
        


        // ... 步骤5 重投影 + 发送消息 + 打印 完成时间戳
        rclcpp::Time t5 = this->now();



        // 8. 计算每个步骤的耗时，并打印
        if(this->SHOW_LOGGER_TIME && this->armorplate.size() > 0 && this->armorplate[0].is_success)
        {
            double duration1 = (t1 - start).seconds() * 1000.0; // 转换为毫秒
            double duration1_1 = (t1_1 - t1).seconds() * 1000.0; // 转换为毫秒
            double duration1_2 = (t1_2 - t1_1).seconds() * 1000.0; // 转换为毫秒
            double duration1_3 = (t1_3- t1_2).seconds() * 1000.0; // 转换为毫秒
            double duration1_4 = (t1_4 - t1_3).seconds() * 1000.0; // 转换为毫秒
            double duration2 = (t2 - t1).seconds() * 1000.0; // 转换为毫秒
            double duration3 = (t3 - t2).seconds() * 1000.0; // 转换为毫秒
            double duration4 = (t4 - t3).seconds() * 1000.0; // 转换为毫秒
            double duration5 = (t5 - t4).seconds() * 1000.0; // 转换为毫秒
            double total_duration = (t5 - start).seconds() * 1000.0; // 转换为毫秒
            RCLCPP_INFO(this->get_logger(), "本帧处理耗时: 接受图像 = %.4f ms, 预处理 = %.4f ms, pnp 解算 = %.4f ms, tf 变换 + 滤波 = %.4f ms, 重投影 + 发布消息 = %.4f ms. 总耗时 = %.4f ms, t1_1 = %.4f ms, t1_2 = %.4f ms, t1_3 = %.4f ms, t1_4 = %.4f ms", duration1, duration2, duration3, duration4, duration5, total_duration, duration1_1, duration1_2, duration1_3, duration1_4);
        }
        


        ///////////////////////////////////////////////////////

        // cv::imshow("img", img);
        showImg();

        RCLCPP_INFO_ONCE(this->get_logger(), "coreNode 正在发布...");
    }



    /**
     * @brief 将世界坐标系下的点投影到图像平面并绘制
     * @param world_points 世界坐标系下的三维点
     * @param K            相机内参矩阵 (3x3)
     * @param D            畸变系数 (1x5)
     * @param T_world_cam  世界到相机的变换
     * @param color        绘制颜色
     */
    void projectAndDraw(const std::vector<Eigen::Vector3d>& world_points,
                        const cv::Mat& K, const cv::Mat& D,
                        const tf2::Transform& T_world_cam,
                        const cv::Scalar& color)
    {
        if (world_points.empty()) return;

        // P: 你的 FLU -> OpenCV 相机系
        Eigen::Matrix3d P_flu2cv;
        P_flu2cv << 0, -1,  0,
                    0,  0, -1,
                    1,  0,  0;

        std::vector<cv::Point3f> cam_points;
        cam_points.reserve(world_points.size());
        
        for (const auto& wp : world_points) 
        {
            // 1. 【空间刚体变换】：World -> Camera (此时坐标系依然是 FLU)
            tf2::Vector3 p_world(wp.x(), wp.y(), wp.z());
            tf2::Vector3 p_cam_tf = T_world_cam * p_world;   
            
            // 转换为 Eigen 向量
            Eigen::Vector3d p_cam_flu(p_cam_tf.x(), p_cam_tf.y(), p_cam_tf.z());
            
            // 2. 【纯粹的数学基变换】：利用矩阵乘法，将 FLU 坐标系切换为 OpenCV 坐标系
            Eigen::Vector3d p_cam_cv = P_flu2cv * p_cam_flu;
            
            // 存入 OpenCV 容器
            cam_points.emplace_back(p_cam_cv.x(), p_cam_cv.y(), p_cam_cv.z()); 
        }

        // ================= 投影到像素平面 =================
        std::vector<cv::Point2f> img_points;
        // 因为我们已经把点转换到了相机的真实物理位置，所以 rvec 和 tvec 必须为 0
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::projectPoints(cam_points, rvec, tvec, K, D, img_points);

        // ================= 绘制部分 =================
        // 绘制：假设传入的是四个角点（左上、右上、右下、左下），连成矩形
        if (img_points.size() == 4) 
        {
            for (int i = 0; i < 4; ++i) 
            {
                cv::line(this->img_show, img_points[i], img_points[(i + 1) % 4], color, 2);
            }

            // 绘制装甲板中心点
            cv::Point2f center = (img_points[0] + img_points[1] + img_points[2] + img_points[3]) / 4.0f;
            cv::circle(this->img_show, center, 2.5, color, -1);
        }
        else // 否则就是投影整车中心点
        {
            cv::circle(this->img_show, img_points[0], 5, color, -1);
        }
    }

    // 由于写了return，必须在每次 return 前 显示图片
    void showImg()
    {
        if(this->SHOW_IMG_SHOW)
        {
            cv::imshow("img_show_armorplate", this->img_show);
            int key = cv::waitKey(1);
        }
    }



    // 回调：当参数被外部修改时触发
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>& params)
    {
        RCLCPP_INFO(this->get_logger(), "修改参数的回调函数已被调用! ");
        for (const auto& p : params) 
        {
            // 注意 p 一次 只能被当做一种类型来看
            const std::string& name = p.get_name();

            if (name == "core.logger.show_logger_time") 
            {
                this->SHOW_LOGGER_TIME = p.as_bool();
                RCLCPP_INFO(this->get_logger(), "打印 corenode 节点耗时开关已打开! ");
            } 

            else if (name == "core.logger.show_logger_else") 
            {
                this->SHOW_LOGGER_ELSE = p.as_bool();
                RCLCPP_INFO(this->get_logger(), "打印 corenode 节点其他日志的开关已打开! ");
            } 

            else if (name == "core.image.show_img") 
            {
                this->SHOW_IMG_SHOW = p.as_bool();
                RCLCPP_INFO(this->get_logger(), "显示图片开关已打开! ");
            } 

            else if (name == "core.logger.show_logger_prepare") 
            {
                this->SHOW_LOGGER_PREPARE = p.as_bool();
                prepare.setParam(this->SHOW_LOGGER_PREPARE, this->CHOSEN_COLOR, this->CAMERA_NAME, this->ARMOR_TYPE);
                RCLCPP_INFO(this->get_logger(), "预处理日志开关已更新!");
            } 

            else if (name == "core.param.chosen_color") 
            {
                this->CHOSEN_COLOR = p.as_string();
                prepare.setParam(this->SHOW_LOGGER_PREPARE, this->CHOSEN_COLOR, this->CAMERA_NAME, this->ARMOR_TYPE);
                RCLCPP_INFO(this->get_logger(), "装甲板颜色已更新!");
            } 

            else if (name == "core.param.armor_type") 
            {
                this->ARMOR_TYPE = p.as_string();
                prepare.setParam(this->SHOW_LOGGER_PREPARE, this->CHOSEN_COLOR, this->CAMERA_NAME, this->ARMOR_TYPE);
                RCLCPP_INFO(this->get_logger(), "装甲板类型已更新!");
            }

            else if (name == "ekf.q_x" || name == "ekf.q_y" || name == "ekf.q_z" || 
                    name == "ekf.q_v_x" || name == "ekf.q_v_y" || name == "ekf.q_v_z" ||
                    name == "ekf.q_yaw" || name == "ekf.q_omega" || name == "ekf.q_a_omega" ||
                    name == "ekf.r_x" || name == "ekf.r_y" || name == "ekf.r_z" ||
                    name == "ekf.r_yaw" || name == "ekf.radius" || name == "ekf.show_logger_debug") 
            {
                RCLCPP_INFO(this->get_logger(), "EKF 参数已更新! ");
                this->ekf_->updateParamsFromServer();  // 让EKF自己重新读取参数
            }

            else if (name == "tf.show_logger_error" || name == "tf.show_result")
            {
                RCLCPP_INFO(this->get_logger(), "TF 节点参数已更新! ");
                this->tf->updateParamsFromServer();  // 通知 TF 刷新
            }

            else if(name == "is_standalone")
            {
                this->IS_STANDALONE = p.as_bool();
                if(this->IS_STANDALONE) RCLCPP_INFO(this->get_logger(), "已切换到单机模式! 父坐标系是 camera_frame");
                else RCLCPP_INFO(this->get_logger(), "已切换到联调模式! 父坐标系是 world_frame");
                this->tf->updateParamsFromServer();  // 通知 TF 刷新
                this->ekf_->updateParamsFromServer();  // 通知 EKF 刷新坐标系
                this->ekf_->reset(); // 必须重置一下滤波器，因为坐标系都变了，之前的数据都没用了
            }

            else if (name == "ekf.predict_time") // 更新 ekf 预测时间
            {
                this->PREDICT_TIME = p.as_double();
                RCLCPP_INFO(this->get_logger(), "EKF 预测时间已更新! ");
            }

            else if (name == "ekf.show_logger_debug") // 是否打印 ekf 调参参数
            {
                this->SHOW_LOGGER_DEBUG = p.as_bool();
                this->ekf_->setDebugLogger(this->SHOW_LOGGER_DEBUG);
                RCLCPP_INFO(this->get_logger(), "EKF 调参日志开关已打开! ");
            }
            
            // 注意：检测颜色、相机名称等通常不应运行时改变，如需改变可类似处理
        }
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        return res;
    }





    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; // 订阅 mindvision_publisher 发过来的原图
    rclcpp::Publisher<serial_driver_interfaces::msg::SerialDriver>::SharedPtr serial_pub_; // 发布 最终 信息


    Prepare prepare; // prepare 类对象，包含预处理函数、寻找灯条函数、配对函数等
    
    std::vector<Strip> strip; // 灯条类集合，接收从 prepare.findAndJudgeLightStrip() 返回的灯条集合
    std::vector<ArmorPlate> armorplate; // 【修改】置信度最高的那个装甲板，接收从 prepare.pairStrip() 返回的装甲板



    ////////////////////// 图像相关 //////////////////////

    cv::Mat img; // 原图
    cv::Mat img_show; // 用来显示信息的图



    ////////////////////// 从配置文件读取的变量 //////////////////////
    
    bool SHOW_LOGGER_TIME; // 是否显示 core 节点中的每帧耗时日志（计算耗时）
    bool SHOW_LOGGER_ELSE; // 是否显示 core 节点中的其他日志（不计算耗时的日志）
    bool SHOW_IMG_SHOW; // 是否显示 img_show 窗口

    // 单机模式 / 联调模式的切换，决定了父坐标系是谁（主要是为了调试时不依赖电控的 TF 发布）
    bool IS_STANDALONE; // true 就是单机模式，父坐标系是 camera_frame；false 就是联调模式，父坐标系是 world// 单机模式 / 联调模式的切换，决定了父坐标系是谁（主要是为了调试时不依赖电控的 TF 发布）

    bool SHOW_LOGGER_PREPARE; // 是否显示 prepare 中的日志
    std::string CHOSEN_COLOR; // 选择检测的颜色 red / blue
    std::string CAMERA_NAME; // 选择相机名称 mind_vision / galaxy ，注意改对应的 qos
    std::string ARMOR_TYPE; // 选择装甲板类型 normal / hero ，决定了配对的参数

    double PREDICT_TIME; // ekf 预测时间
    bool SHOW_LOGGER_DEBUG; // ekf 打印调试日志

    ///////// 工具类参数 /////////
    rclcpp::Time last_print; // 记录上一次打印 pnp 发布频率的时间戳，用于统计 pnp 发布频率

    int continuous_count = 0; // 记录连续有效的帧数，刚收到数据的前几帧不要（因为不稳定）
    int lost_count = 0; // 记录连续丢失的帧数，丢失一定次数后不给ekf了，并且重置


    // tf 类对象指针
    std::unique_ptr<TF> tf;

    // EKF 类对象指针
    std::unique_ptr<EKF> ekf_;

    // 参数量
    int MAX_LOST_COUNT = 60; // 最大连续丢失量，超过这个就不用 ekf 的预测了，大概 60 / 200 = 0.3s

    // 上一次查询的的时间戳
    rclcpp::Time last_image_time; // 上一帧收到图像的时间戳，是一切 dt 的基础

    // ekf 是否已经稳定跟踪
    bool ekf_ready = false; 

    // 动态参数变化 
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 当前正在追踪的最好的装甲板 ID
    int tracking_id = 3;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProcessNode>());
    rclcpp::shutdown();
    return 0;
}