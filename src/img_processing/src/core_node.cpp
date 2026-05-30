// 我的 pitch 是你的 yaw
// 我的 roll 是你的 pitch
// 我的 yaw 是你的 roll

#include "core_node.hpp"


// ============================== 构造与析构 ==============================

CoreNode::CoreNode(): Node("core_node_cpp")
{
    // 1. 初始化 ROS 参数
    InitParams();

    // 2. 初始化核心算法模块
    prepare_ = std::make_unique<Prepare>(); // 构造 Prepare 类对象
    prepare_->SetParam(show_logger_prepare_, chosen_color_, camera_name_, armorplate_type_); 

    tf_ = std::make_unique<TF>(this); // 传 this 指针给 TF 类来构造，让它能创建 ROS2 相关对象，同时发布静态变换
    ekf_ = std::make_unique<EKF>(this); 
    ekf_->UpdateParamsFromServer(); // 设置一大堆参数
    ekf_->SetArmorplateSize(armorplate_type_); // 装甲板类型
    ekf_->SetDebugLogger(show_logger_ekf_debug_); // 是否打印 ekf 调试日志
    ekf_->SetArmorNum(armor_num_); // 设置装甲板数量

    // 3. 初始化 ROS 通信与多线程
    InitROS2();
    

    RCLCPP_INFO_ONCE(this->get_logger(), "CoreNode 节点创建成功! ");
}



CoreNode::~CoreNode()
{
    // 析构时安全退出线程
    if (video_thread_.joinable()) 
    {
        video_thread_.join();
    }
}



// ============================== 声明和获取 ROS2 参数 ==============================
void CoreNode::InitParams()
{
    
    // 说明：单机模式决定是否依赖串口（最终坐标系看 相机/云台），本地视频模式决定是否以来相机（图像来源于 本地视频/摄像头）
    // 如果单机，就改一下 camera_name (galaxy -> mind_vision) 和 is_standalone_mode (false -> true)


    // core 节点
    this->declare_parameter("core.logger.show_logger_time", false); // 是否打印时间相关日志
    this->declare_parameter("core.logger.show_logger_else", false); // 是否打印corenode其他的相关日志
    this->declare_parameter("core.image.show_img", true); // 是否显示图片
    
    // prepare 类
    this->declare_parameter("core.logger.show_logger_prepare", false); // 是否打印 prepare 类中的日志
    this->declare_parameter("core.param.chosen_color", "blue"); // 选择的装甲板颜色
    this->declare_parameter("core.param.camera_name", "galaxy"); // 使用的相机名称
    this->declare_parameter("core.param.armor_type", "normal"); // 识别装甲板的类型

    // EKF相关（传递给ekf）
    this->declare_parameter("ekf.predict_time", 0.2); // 预测时间（记得改！）0.2? 0.225? 其实未来还要根据速度来确定
    this->declare_parameter("ekf.show_logger_debug", false); // ekf 调试

    this->declare_parameter("ekf.q_x", 0.05);
    this->declare_parameter("ekf.q_y", 0.05);
    this->declare_parameter("ekf.q_z", 0.05);
    this->declare_parameter("ekf.q_v_x", 0.5);
    this->declare_parameter("ekf.q_v_y", 0.5);
    this->declare_parameter("ekf.q_v_z", 0.5);
    this->declare_parameter("ekf.q_yaw", 0.05);
    this->declare_parameter("ekf.q_omega", 0.5);
    this->declare_parameter("ekf.q_a_omega", 1.0);
    
    this->declare_parameter("ekf.r_los_yaw", 0.002);   // 相机角度极其精准，给极小方差
    this->declare_parameter("ekf.r_los_pitch", 0.002); // 相机角度极其精准，给极小方差
    this->declare_parameter("ekf.r_distance", 7.5);    // PnP 测距极其垃圾！给巨大方差 (5.0~10.0都行)
    this->declare_parameter("ekf.r_euler_yaw", 0.05); // 目前观测到的装甲板的角度（需要转换到整车下）

    this->declare_parameter("ekf.radius", 0.25);

    // TF 参数声明
    this->declare_parameter("tf.show_logger_error", false);
    this->declare_parameter("tf.show_result", false);

    // 在 CoreNode 构造函数里加一个控制开关和对象实例化：
    this->declare_parameter("core.image.show_plot", true); 

    // 新增：模式选择与视频路径参数
    this->declare_parameter("core.mode.is_standalone_mode", true); // 单机 / 联调模式
    this->declare_parameter("core.mode.is_video_mode", true); // 是否为读取 本地视频模式
    this->declare_parameter("core.mode.video_path", "/home/cly/下载/rm_test_videos/20260501_160636__camera_0_rgb_output.mp4"); // 视频的绝对路径




    // 参数变量获取初始值

    // corenode 节点变量获取初始值
    show_logger_about_time_ = this->get_parameter("core.logger.show_logger_time").as_bool();
    show_logger_about_else_ = this->get_parameter("core.logger.show_logger_else").as_bool();
    show_image_ = this->get_parameter("core.image.show_img").as_bool();
    show_logger_prepare_ = this->get_parameter("core.logger.show_logger_prepare").as_bool();
    chosen_color_ = this->get_parameter("core.param.chosen_color").as_string();
    camera_name_ = this->get_parameter("core.param.camera_name").as_string();
    armorplate_type_ = this->get_parameter("core.param.armor_type").as_string();
    ekf_predict_time_ = this->get_parameter("ekf.predict_time").as_double();
    show_logger_ekf_debug_ = this->get_parameter("ekf.show_logger_debug").as_bool();
    show_plot_ = this->get_parameter("core.image.show_plot").as_bool(); 

    // 新增：关于 本地视频模式 的参数获取
    is_standalone_mode_ = this->get_parameter("core.mode.is_standalone_mode").as_bool();
    is_video_mode_ = this->get_parameter("core.mode.is_video_mode").as_bool();
    video_path_ = this->get_parameter("core.mode.video_path").as_string();

    // 本地视频模式下强制为单机模式（因为没有电控发TF）
    if (is_video_mode_) 
    {
        is_standalone_mode_ = true; // 本地视频模式 下强制为单机模式（一定为 相机+corenode）

        // 必须把这个覆盖后的值同步回参数服务器！
        this->set_parameter(rclcpp::Parameter("core.mode.is_standalone_mode", true));
    } 
    else 
    {
        is_standalone_mode_ = this->get_parameter("core.mode.is_standalone_mode").as_bool(); // 否则按照原本的声明（单机模式下就是 相机+corenode，不要串口）
    }

    // 注册参数变化回调（用于运行时动态修改）
    param_callback_handle_ = add_on_set_parameters_callback(std::bind(&CoreNode::OnParameterChange, this, std::placeholders::_1));
}



// ========================= 初始化 Pub/Sub 和 线程 ========================
void CoreNode::InitROS2()
{
    // 发布给串口所需消息的 publisher
    serial_pub_ = this->create_publisher<serial_driver_interfaces::msg::SerialDriver>("/serial_driver", 10);

    // 初始化上一次收到图像的时间，一切的 dt 都依照这个来算
    last_image_time_ = this->now(); 

    // 初始化动态示波器 (参数: 宽 1200, 高 800, 存最近 200 帧)
    plotter_ = std::make_unique<Plotter>(1200, 800, 200);



    // 分支启动逻辑（本地视频模式/使用相机）

    // 声明 QoS 参数

    // 适用于 mind_vision 的 qos
    this->declare_parameter("use_sensor_data_qos", false);
    bool qos1 = this->get_parameter("use_sensor_data_qos").as_bool();
    // 适用于 galaxy 的 qos
    auto qos2 = rclcpp::SensorDataQoS(); 


    // 如果是本地读取视频模式
    if (is_video_mode_) 
    {
        RCLCPP_INFO(this->get_logger(), "【本地视频模式】将读取本地视频: %s", video_path_.c_str());
        
        video_thread_ = std::thread(&CoreNode::VideoReading, this); // 启动一个子线程专门用来读视频，防止阻塞 rclcpp::spin()
    }
    else
    {
        // 否则将根据 camera_name_ 选择 qos，以创建 订阅相机原图的 subscription
        if (camera_name_ == "mind_vision") 
        {
            sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", qos1, std::bind(&CoreNode::CameraImageCallback, this, std::placeholders::_1));
        }
        else 
        {
            sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", qos2, std::bind(&CoreNode::CameraImageCallback, this, std::placeholders::_1));
        }
    }

}






// ================= 新增：视频读取专用线程函数 =================
void CoreNode::VideoReading()
{
    // 既然是读取视频而不是用相机，那么真实时间并不重要！

    // 打开视频
    cv::VideoCapture cap(video_path_);
    if (!cap.isOpened()) 
    {
        RCLCPP_ERROR(this->get_logger(), "无法打开视频文件: %s, 请检查路径是否正确", video_path_.c_str());
        return;
    }

    cv::Mat frame;
    rclcpp::Time base_time = this->now(); // 记录开始时间
    rclcpp::Time virtual_time = base_time; // 模拟时间。初始值为当前时间，后续每一帧都加上 视频帧时刻 来模拟时间流逝
    rclcpp::Time last_virtual_time = this->now(); // 记录上一帧模拟时间，用来看 fps

    while(rclcpp::ok()) 
    {
        cap >> frame;
        if (frame.empty()) 
        {
            RCLCPP_WARN(this->get_logger(), "视频播放结束，循环重播...");
            cap.set(cv::CAP_PROP_POS_FRAMES, 0); // 跳回第一帧循环播放
            base_time = virtual_time; // 重置基准时间为当前模拟时间，保证时间戳连续
            continue;
        }
        
        // 视频本身自带的图像时间戳 (ms)
        double video_timestamp_ms = cap.get(cv::CAP_PROP_POS_MSEC);

        // 得到当前帧的模拟图像时间戳 (s)
        virtual_time = base_time + rclcpp::Duration::from_seconds(video_timestamp_ms / 1000.00); 

        RCLCPP_INFO_EXPRESSION(this->get_logger(), show_logger_about_else_, "fps = %.2f", 1.0 / (virtual_time - last_virtual_time).seconds());


        // 调用核心算法主逻辑, 使用 virtual_time 作为当前帧的时间戳
        CoreLogic(frame, virtual_time);

        // 手动控制视频播放速度
        int key = cv::waitKey(200);

        // 按 ESC 或 q 退出节点
        if (key == 27 || key == 'q') 
        { 
            rclcpp::shutdown();
            break;
        } 
        else if (key == ' ') 
        {       
            cv::waitKey(0); 
        }

        last_virtual_time = virtual_time; // 更新上一帧的模拟时间
    }
}



// ================= 相机图传 回调函数 =================
void CoreNode::CameraImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image; // 需要 SharedPtr 作为参数
    CoreLogic(frame, msg->header.stamp);
}



// =========================== 独立主逻辑 ===========================
void CoreNode::CoreLogic(cv::Mat& frame, rclcpp::Time current_image_time)
{
    rclcpp::Time calculate_start = this->now(); // 以当前真实时间作为计时器开始，所有的 dt 都基于这个来算


    // 1. 设置图像 并 记录时间和更新 dt，dt 是整个的基准！！
    img_ = frame.clone();
    img_show_ = img_.clone(); // 复制一份用来显示信息
    
    double dt = (current_image_time - last_image_time_).seconds(); // 计算与上一次图像的时间间隔，注意sec已经是精确时间！ 
    last_image_time_ = current_image_time; // 更新上一次图像的时间戳


    // ... 步骤1 接收图像完成时间戳
    rclcpp::Time t1 = this->now();


    // 2. 预处理（掩码、灯条筛选、灯条配对）
    prepare_->SetImgShow(img_show_); // 设置 img_show_
    rclcpp::Time t1_1 = this->now();
    prepare_->PreProcessing(img_); // 图像预处理
    rclcpp::Time t1_2 = this->now();
    strips_ = prepare_->FindAndJudgeLightStrip(); // 找灯带 返回灯带集合
    rclcpp::Time t1_3 = this->now();
    armorplates_ = prepare_->PairStrip(); // 灯条配对 【修改】返回按置信度排序后的装甲板集合
    rclcpp::Time t1_4 = this->now();
    img_show_ = prepare_->GetImgShow(); // 获取预处理后带有信息的 img_show_

    // ... 步骤2 预处理完成时间戳
    rclcpp::Time t2 = this->now();
    


    // 3. 解算 pnp
    if (armorplates_.size() > 0) // 存在装甲板
    {
        armorplates_[0].perspectiveNPoint(); // 解算 pnp，算出了初版的 t_vec 和 辣鸡 R

        // 【新增】：一键优化！内部直接用极品 R 覆盖掉辣鸡 R！
        armorplates_[0].OptimizeEulerYaw(img_show_);

        // 画出所有装甲板（按置信度排序的）并打印信息
        for (int i = 0; i < armorplates_.size(); i++) 
        { 
            // 画所有的装甲板，并打印综合置信度（考虑与上一帧追踪装甲板的距离）最高的装甲板也就是 0号 的 pnp 信息
            armorplates_[i].drawArmorPlateAndPrintPNPInfo(img_show_, chosen_color_, i); 
        }
    }

    // ... 步骤3 pnp解算完成时间戳
    rclcpp::Time t3 = this->now();


    
    // ====================================================================
    // 4. Tracker 状态机流转 (决策)
    // ====================================================================
    bool is_found = (armorplates_.size() > 0 && armorplates_[0].is_success);
    
    UpdateTrackerState(is_found); // 调用状态机判定函数

    // 预先声明并初始化核心数据，这些数据将被 ExecuteTracker 修改并带出

    Eigen::Vector3d armorplate_center_now(-999, -999, -999); 
    double yaw_armorplate_now = -999; 

    Eigen::Vector3d armorplate_center_filter(-999, -999, -999); 
    Eigen::Vector3d armorplate_center_predict(-999, -999, -999); 
    Eigen::Vector3d car_center_predict(-999, -999, -999); 


    // ====================================================================
    // 5. 状态机执行 (动作)
    // ====================================================================
    ExecuteTracker(dt, current_image_time, 
                   yaw_armorplate_now, 
                   armorplate_center_now, armorplate_center_filter, armorplate_center_predict, 
                   car_center_predict);



    // ====================================================================
    // 6. 计算最终预测云台偏角
    // ====================================================================
    // 只有在预测出有效坐标时才进行解算

    double pitch_result = -999;
    double yaw_result = -999;

    if (armorplate_center_predict[0] != -999 && armorplate_center_predict[1] != -999 && armorplate_center_predict[2] != -999)
    {
        // 算出云台需要瞄准的目标 Pitch 和 Yaw
        pitch_result = -std::atan2(armorplate_center_predict[2], std::sqrt(armorplate_center_predict[0] * armorplate_center_predict[0] + armorplate_center_predict[1] * armorplate_center_predict[1])) * 180.0 / M_PI;   
        yaw_result = std::atan2(armorplate_center_predict[1], armorplate_center_predict[0]) * 180.0 / M_PI;
        RCLCPP_INFO_EXPRESSION(this->get_logger(), show_logger_about_else_, "armorplate_center_predict: [%.3f, %.3f, %.3f]", armorplate_center_predict[0], armorplate_center_predict[1], armorplate_center_predict[2]);
    }


    // ... 步骤4 发布 tf变换 + 滤波 完成时间戳
    rclcpp::Time t4 = this->now();


    // 5. 重投影 可视化 (颜色匹配 红0,黄1,蓝2,绿3)
    // ekf_->GetArmorplateFourCorners 拿到的是 实时滤波值
    if (ekf_ready_ && pitch_result != -999 && yaw_result != -999)
    {
        // 先获取相机内参和畸变系数（可从当前处理的装甲板对象获取，因为 ArmorPlate 已保存）
        //    注意：如果本帧没有成功检测到装甲板，则无法获取相机参数，此时跳过重投影。
        if (!armorplates_.empty() && armorplates_[0].is_success)
        {
            // 再查询 camera → 父坐标系 的 TF，因为要转到相机下投影！
            tf2::Transform T_world_cam;
            if (tf_->GetCameraToWorldTransform(T_world_cam, current_image_time)) 
            {
                // 对每个装甲板 ID 进行重投影（0红, 1黄, 2蓝, 3绿）（实时滤波位置）
                std::vector<cv::Scalar> colors = 
                {
                    cv::Scalar(0, 0, 255),   // ID=0: 红色 -> 前
                    cv::Scalar(0, 255, 255), // ID=1: 黄色 -> 右
                    cv::Scalar(255, 0, 0),   // ID=2: 蓝色 -> 后
                    cv::Scalar(0, 255, 0)    // ID=3: 绿色 -> 左
                };

                for (int id = 0; id < armor_num_; id++)
                {
                    std::vector<Eigen::Vector3d> corners_world;
                    ekf_->GetArmorplateFourCorners(corners_world, id); // id=0 前 红色, id=1 右 黄色, id=2 后 蓝色, id=3 左 绿色
                    tf_->ProjectAndDraw(img_show_, corners_world, armorplates_[0].K, armorplates_[0].D, T_world_cam, colors[id], id);
                }

                // 也绘制整车中心点（实时滤波位置）
                std::vector<Eigen::Vector3d> center_world(1);
                ekf_->GetCarCenterPredict(center_world[0], 0.00); // 获取实时位置下的整车中心点 在父坐标系下 的位置坐标
                tf_->ProjectAndDraw(img_show_, center_world, armorplates_[0].K, armorplates_[0].D, T_world_cam, cv::Scalar(255, 255, 255));
            
                // 也绘制当前装甲板 dt后的 预测值（不是外推值）
                std::vector<Eigen::Vector3d> armorplate_center_world(1);
                armorplate_center_world[0] = armorplate_center_predict; // 预测位置的装甲板中心点 在父坐标系下 的位置坐标
                tf_->ProjectAndDraw(img_show_, armorplate_center_world, armorplates_[0].K, armorplates_[0].D, T_world_cam, cv::Scalar(255, 0, 255));
            }
        }
    }
    
        
    
    // 6. 发送 最终信息
    auto send_msg = serial_driver_interfaces::msg::SerialDriver();
    send_msg.pitch = pitch_result;
    send_msg.yaw = yaw_result;
    serial_pub_->publish(send_msg);
    RCLCPP_INFO_EXPRESSION(this->get_logger(), show_logger_about_else_, "已发送信息");


    // 7. 联调模式下，在图上打印要发布的目标角度 和 电控发来的目前芯片姿态
    if (!is_standalone_mode_)
    {
        double pitch_chip;
        double yaw_chip;
        bool flag = tf_->GetWorldToChipTransform(pitch_chip, yaw_chip, current_image_time); // 获取【世界坐标系】->【芯片坐标系的变换】
        
        if (flag)
        {
            // 1. 提前计算好相对偏差值 (当前查到的值 - 目标值)
            double pitch_diff = pitch_chip - pitch_result;
            double yaw_diff = yaw_chip - yaw_result;

            // 2. 打印第一行 Target 信息 
            cv::putText(img_show_, 
                        cv::format("Target: pitch = %.2f, yaw = %.2f", pitch_result, yaw_result), 
                        cv::Point2f(0, 700), 
                        cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);

            // 3. 打印第二行 Current 信息及差值 
            cv::putText(img_show_, 
                        cv::format("Current: pitch = %.2f ( %+.2f ), yaw = %.2f ( %+.2f )", 
                                pitch_chip, pitch_diff, yaw_chip, yaw_diff), 
                        cv::Point2f(0, 750), 
                        cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(193, 182, 255), 2.5);
        }
    }
    


    // ... 步骤5 重投影 + 发送消息 + 打印 完成时间戳
    rclcpp::Time t5 = this->now();



    // 8. 计算每个步骤的耗时，并打印
    if (show_logger_about_time_ && armorplates_.size() > 0 && armorplates_[0].is_success)
    {
        double duration1 = (t1 - calculate_start).seconds() * 1000.0; // 转换为毫秒
        double duration1_1 = (t1_1 - t1).seconds() * 1000.0; // 转换为毫秒
        double duration1_2 = (t1_2 - t1_1).seconds() * 1000.0; // 转换为毫秒
        double duration1_3 = (t1_3- t1_2).seconds() * 1000.0; // 转换为毫秒
        double duration1_4 = (t1_4 - t1_3).seconds() * 1000.0; // 转换为毫秒
        double duration2 = (t2 - t1).seconds() * 1000.0; // 转换为毫秒
        double duration3 = (t3 - t2).seconds() * 1000.0; // 转换为毫秒
        double duration4 = (t4 - t3).seconds() * 1000.0; // 转换为毫秒
        double duration5 = (t5 - t4).seconds() * 1000.0; // 转换为毫秒
        double total_duration = (t5 - calculate_start).seconds() * 1000.0; // 转换为毫秒
        RCLCPP_INFO(this->get_logger(), "本帧处理耗时: 接受图像 = %.4f ms, 预处理 = %.4f ms, pnp 解算 = %.4f ms, tf 变换 + 滤波 = %.4f ms, 重投影 + 发布消息 = %.4f ms. 总耗时 = %.4f ms, t1_1 = %.4f ms, t1_2 = %.4f ms, t1_3 = %.4f ms, t1_4 = %.4f ms", duration1, duration2, duration3, duration4, duration5, total_duration, duration1_1, duration1_2, duration1_3, duration1_4);
    }
    

    ShowImg();

    UpdatePlotter(armorplate_center_now, armorplate_center_filter, armorplate_center_predict);

    RCLCPP_INFO_ONCE(this->get_logger(), "coreNode 正在发布...");
}




// ============================== 工具类 ==============================

// 由于写了return，必须在每次 return 前 显示图片
void CoreNode::ShowImg()
{
    if (show_image_)
    {
        cv::imshow("img_show_armorplate", img_show_);
        int key = cv::waitKey(1);
    }
}

// 统一处理动态示波器数据的辅助函数
void CoreNode::UpdatePlotter(Eigen::Vector3d armorplate_center_now, Eigen::Vector3d armorplate_center_filter, Eigen::Vector3d armorplate_center_predict)
{
    // 如果没开画图，或者指针为空，直接跳过，绝不占用系统资源
    if (!show_plot_ || !plotter_) return;

    // 全部初始化为 -999，表示无效值，画图类收到 -999 就会自动断开
    double pitch_now = -999;
    double yaw_now = -999;
    double pitch_filter = -999; 
    double yaw_filter = -999;
    double pitch_result = -999;
    double yaw_result = -999;

    // 计算最终角（实时）
    if (armorplate_center_now[0] != -999 && armorplate_center_now[1] != -999 && armorplate_center_now[2] != -999)
    {
        pitch_now = -std::atan2(armorplate_center_now[2], std::sqrt(armorplate_center_now[0] * armorplate_center_now[0] + armorplate_center_now[1] * armorplate_center_now[1])) * 180.0 / M_PI;   
        yaw_now = std::atan2(armorplate_center_now[1], armorplate_center_now[0]) * 180.0 / M_PI;
    }
    

    // 计算最终角（实时滤波）
    if (armorplate_center_filter[0] != -999 && armorplate_center_filter[1] != -999 && armorplate_center_filter[2] != -999)
    {
        pitch_filter = -std::atan2(armorplate_center_filter[2], std::sqrt(armorplate_center_filter[0] * armorplate_center_filter[0] + armorplate_center_filter[1] * armorplate_center_filter[1])) * 180.0 / M_PI;   
        yaw_filter = std::atan2(armorplate_center_filter[1], armorplate_center_filter[0]) * 180.0 / M_PI;
    }


    // 计算最终角（预测）（滤波器没初始好就用原始值，连续检测多就用预测值，丢帧但滤波器稳定就用外推值）
    if (armorplate_center_predict[0] != -999 && armorplate_center_predict[1] != -999 && armorplate_center_predict[2] != -999)
    {
        pitch_result = -std::atan2(armorplate_center_predict[2], std::sqrt(armorplate_center_predict[0] * armorplate_center_predict[0] + armorplate_center_predict[1] * armorplate_center_predict[1])) * 180.0 / M_PI;   
        yaw_result = std::atan2(armorplate_center_predict[1], armorplate_center_predict[0]) * 180.0 / M_PI;
    }


    // 3. 把这帧的状态送入示波器
    plotter_->updateAndDraw(
        pitch_now, yaw_now, 
        pitch_filter, yaw_filter, 
        pitch_result, yaw_result
    );
}


// ==============================================================================
// 【函数 1】状态机决策大脑：只负责判定现在是什么阶段，绝对不触碰具体算法！
// ==============================================================================
void CoreNode::UpdateTrackerState(bool is_found)
{
    // 如果这一帧看到了装甲板
    if (is_found) 
    {
        if (tracker_state_ == TrackerState::LOST) 
        {
            tracker_state_ = TrackerState::DETECTING; // 刚看到，进入防抖观察期
            detect_count_ = 1;
        } 
        else if (tracker_state_ == TrackerState::DETECTING) 
        {
            detect_count_++;

            if (detect_count_ >= min_detect_frames_) 
            {
                tracker_state_ = TrackerState::TRACKING; // 连续看到多帧，确认为实体，进入追踪！
            }
        } 
        else if (tracker_state_ == TrackerState::TRACKING) 
        {
            // 持续看到，相安无事，保持追踪
        } 
        else if (tracker_state_ == TrackerState::TEMP_LOST) 
        {
            tracker_state_ = TrackerState::TRACKING; // 盲推期间突然又看到了，王者归来！
        }
    } 
    // 如果这一帧没看到装甲板
    else 
    {
        if (tracker_state_ == TrackerState::LOST) 
        {
            // 依然没看到，保持丢失
        } 
        else if (tracker_state_ == TrackerState::DETECTING) 
        {
            tracker_state_ = TrackerState::LOST; // 还没观察稳就消失了，直接丢弃！
        } 
        else if (tracker_state_ == TrackerState::TRACKING) 
        {
            tracker_state_ = TrackerState::TEMP_LOST; // 稳定追踪时突然消失（可能被遮挡），进入盲推外推期！
            temp_lost_count_ = 1;
        } 
        else if (tracker_state_ == TrackerState::TEMP_LOST) 
        {
            temp_lost_count_++;

            if (temp_lost_count_ >= max_lost_frames_) 
            {
                tracker_state_ = TrackerState::LOST; // 盲推太久还没看到，彻底凉了，转为彻底丢失
            }
        }
    }
}

// ==============================================================================
// 【函数 2】追踪执行者：完全听令于 tracker_state_，执行对应的 EKF 动作！
// ==============================================================================
void CoreNode::ExecuteTracker(double dt, rclcpp::Time current_image_time,
                              double& yaw_armorplate_now,
                              Eigen::Vector3d& armorplate_center_now, 
                              Eigen::Vector3d& armorplate_center_filter, 
                              Eigen::Vector3d& armorplate_center_predict, 
                              Eigen::Vector3d& car_center_predict)
{
    // ====== 场景 A：看到目标了 (处于 DETECTING 预热期 或 TRACKING 稳定期) ======
    if (tracker_state_ == TrackerState::DETECTING || tracker_state_ == TrackerState::TRACKING) 
    {
        tf_->UpdateCameraToArmorplate(armorplates_[0].R, armorplates_[0].t_vec, current_image_time);

        // 用 armorplate_center_now 和 yaw_armorplate_now 接收 TF 查询的结果（如果成功的话）
        bool tf_lookup_flag = tf_->GetFatherToArmorplateTransform(armorplates_[0].R, armorplates_[0].t_vec, armorplate_center_now, yaw_armorplate_now, current_image_time);

        if (tf_lookup_flag) 
        {
            if (ekf_->is_initialized) 
            {
                // ===============================================================
                // 【核心切板防抖逻辑】
                // ===============================================================
                if (tracker_state_ == TrackerState::DETECTING) 
                {
                    // 预热期：锁定的板子就是 0 号
                    tracking_id_ = 0; 
                }
                else if (tracker_state_ == TrackerState::TRACKING) 
                {
                    // 稳定期：死死咬住当前的 tracking_id_！
                    // 计算正在追踪的板的理论角度
                    double current_target_yaw = tools::limit_rad(ekf_->X(6) + tracking_id_ * 2.0 * M_PI / armor_num_);
                    double diff = std::abs(tools::limit_rad(current_target_yaw - yaw_armorplate_now));
                    
                    // 只有当误差大于 45 度 (0.785 rad) 时，说明当前板子已经转到了侧面，才允许切板！
                    if (diff > 0.785) 
                    {
                        double min_diff = 10.0;
                        for (int i = 0; i < armor_num_; i++) 
                        {
                            double target_yaw = tools::limit_rad(ekf_->X(6) + i * 2.0 * M_PI / armor_num_);
                            double test_diff = std::abs(tools::limit_rad(target_yaw - yaw_armorplate_now)); // 看看哪个板的理论位置和当前检测到的位置是否更接近，选最接近的那个作为 tracking_id_
                            if (test_diff < min_diff) 
                            {
                                min_diff = test_diff;
                                tracking_id_ = i; // 0, 1, 2, 3
                            }
                        }
                        RCLCPP_WARN(this->get_logger(), "发生切板！切换至 %d 号装甲板", tracking_id_);
                    }
                }
            }

            // 更新EKF：无论是 DETECTING 还是 TRACKING，只要有数据就必须喂，让它收敛！
            // EKF 即使 Reset 了，喂一次数据就会 Initialize
            ekf_->SetArmorplateSize(armorplate_type_); 
            ekf_->UpdateExtendedKalman(armorplate_center_now, yaw_armorplate_now, tracking_id_, dt);

            
            // 【还在预热期】
            if (tracker_state_ == TrackerState::DETECTING) 
            {
                // 【预热期】：EKF 的速度和加速度还在剧烈震荡。此时它的预测值是垃圾。
                // 此时，滤波值和预测值直接强制等于原始观测值
                armorplate_center_filter = armorplate_center_now;
                armorplate_center_predict = armorplate_center_now;
            }
            else if (tracker_state_ == TrackerState::TRACKING) 
            {
                // 【稳定期】：EKF 已经完全收敛。使用 EKF 的输出作为画图和云台控制的基准！
                
                // future_time = 0.0，获取的就是纯正的【当前滤波值】！
                ekf_->GetArmorplatePredict(armorplate_center_filter, tracking_id_, 0.00);
                
                // future_time = ekf_predict_time_，获取未来的【预测值】！
                ekf_->GetArmorplatePredict(armorplate_center_predict, tracking_id_, ekf_predict_time_);
                ekf_->GetCarCenterPredict(car_center_predict, ekf_predict_time_);
                
                ekf_ready_ = true; // ekf 已经准备好，可以开始盲推了
            }
            // ===============================================================
        }
        else 
        {
            // 查不到 TF，也不应该盲推，只有 TEMP_LOST 阶段才盲推！其他阶段应直接报警，说明哪里出了问题！
            if (ekf_ready_) 
            {
                // ekf_->PredictOnly(dt);
                // ekf_->GetArmorplatePredict(armorplate_center_filter, tracking_id_, 0.00); // 盲推时的滤波即为预测
                // ekf_->GetArmorplatePredict(armorplate_center_predict, tracking_id_, ekf_predict_time_);
                // ekf_->GetCarCenterPredict(car_center_predict, ekf_predict_time_);
                // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "TF 查询失败，EKF 盲推维系中...");

                RCLCPP_ERROR(this->get_logger(), "TRACKING 或 DETECTING 状态下 TF 查不到 父坐标系 到 装甲板坐标系 的变换！");
            }
        }
    }
    // ====== 场景 B：短暂丢失阶段 (核心：抗小陀螺盲推) ======
    else if (tracker_state_ == TrackerState::TEMP_LOST)
    {
        if (ekf_ready_) 
        {
            // 目标短暂消失，完全不给测量值，让 EKF 靠惯性纯算未来位置！
            ekf_->PredictOnly(dt);
            
            // 此时取 future=0 就是外推出来的“假想当前位置”
            ekf_->GetArmorplatePredict(armorplate_center_filter, tracking_id_, 0.00);
            ekf_->GetArmorplatePredict(armorplate_center_predict, tracking_id_, ekf_predict_time_);
            ekf_->GetCarCenterPredict(car_center_predict, ekf_predict_time_);
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "目标遮挡或丢失，EKF 正在盲推...");
        }
    }
    // ====== 场景 C：彻底丢失 ======
    else if (tracker_state_ == TrackerState::LOST)
    {
        // 彻底丢了，立刻重置滤波器，避免发散
        ekf_ready_ = false; 
        tracking_id_ = 0; // ID 归位
        ekf_->Reset(); 
    }
}





// =========================== 回调函数：当参数被外部修改时触发 ===========================
rcl_interfaces::msg::SetParametersResult CoreNode::OnParameterChange(const std::vector<rclcpp::Parameter>& params)
{
    RCLCPP_INFO(this->get_logger(), "修改参数的回调函数已被调用! ");
    for (const auto& p : params) 
    {
        // 注意 p 一次 只能被当做一种类型来看
        const std::string& name = p.get_name();

        if (name == "core.logger.show_logger_time") 
        {
            show_logger_about_time_ = p.as_bool();
            RCLCPP_INFO(this->get_logger(), "打印 corenode 节点耗时开关已打开! ");
        } 

        else if (name == "core.logger.show_logger_else") 
        {
            show_logger_about_else_ = p.as_bool();
            RCLCPP_INFO(this->get_logger(), "打印 corenode 节点其他日志的开关已打开! ");
        } 

        else if (name == "core.image.show_img") 
        {
            show_image_ = p.as_bool();
            RCLCPP_INFO(this->get_logger(), "显示图片开关已打开! ");
        } 

        else if (name == "core.logger.show_logger_prepare") 
        {
            show_logger_prepare_ = p.as_bool();
            prepare_->SetParam(show_logger_prepare_, chosen_color_, camera_name_, armorplate_type_);
            RCLCPP_INFO(this->get_logger(), "预处理日志开关已更新!");
        } 

        else if (name == "core.param.chosen_color") 
        {
            chosen_color_ = p.as_string();
            prepare_->SetParam(show_logger_prepare_, chosen_color_, camera_name_, armorplate_type_);
            RCLCPP_INFO(this->get_logger(), "装甲板颜色已更新!");
        } 

        else if (name == "core.param.armor_type") 
        {
            armorplate_type_ = p.as_string();
            prepare_->SetParam(show_logger_prepare_, chosen_color_, camera_name_, armorplate_type_);
            RCLCPP_INFO(this->get_logger(), "装甲板类型已更新!");
        }

        else if (name == "ekf.q_x" || name == "ekf.q_y" || name == "ekf.q_z" || 
                name == "ekf.q_v_x" || name == "ekf.q_v_y" || name == "ekf.q_v_z" ||
                name == "ekf.q_yaw" || name == "ekf.q_omega" || name == "ekf.q_a_omega" ||
                name == "ekf.r_x" || name == "ekf.r_y" || name == "ekf.r_z" ||
                name == "ekf.r_yaw" || name == "ekf.radius" || name == "ekf.show_logger_debug") 
        {
            RCLCPP_INFO(this->get_logger(), "EKF 参数已更新! ");
            ekf_->UpdateParamsFromServer();  // 让EKF自己重新读取参数
        }

        else if (name == "tf.show_logger_error" || name == "tf.show_result")
        {
            RCLCPP_INFO(this->get_logger(), "TF 节点参数已更新! ");
            tf_->UpdateParamsFromServer();  // 通知 TF 刷新
        }

        else if(name == "core.mode.is_standalone_mode")
        {
            is_standalone_mode_ = p.as_bool();
            if (is_standalone_mode_) RCLCPP_INFO(this->get_logger(), "已切换到单机模式! 父坐标系是 camera_frame");
            else RCLCPP_INFO(this->get_logger(), "已切换到联调模式! 父坐标系是 world_frame");
            tf_->UpdateParamsFromServer();  // 通知 TF 刷新
            ekf_->UpdateParamsFromServer();  // 通知 EKF 刷新坐标系
            ekf_->Reset(); // 必须重置一下滤波器，因为坐标系都变了，之前的数据都没用了
        }

        else if (name == "ekf.predict_time") // 更新 ekf 预测时间
        {
            ekf_predict_time_ = p.as_double();
            RCLCPP_INFO(this->get_logger(), "EKF 预测时间已更新! ");
        }

        else if (name == "ekf.show_logger_debug") // 是否打印 ekf 调参参数
        {
            show_logger_ekf_debug_ = p.as_bool();
            ekf_->SetDebugLogger(show_logger_ekf_debug_);
            RCLCPP_INFO(this->get_logger(), "EKF 调参日志开关已打开! ");
        }
        
        // 注意：检测颜色、相机名称等通常不应运行时改变，如需改变可类似处理
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
}



// ============================== main 函数 ==============================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoreNode>());
    rclcpp::shutdown();
    return 0;
}