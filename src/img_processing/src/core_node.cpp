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

    tf_ = std::make_unique<TF>(this); // 传 this 指针给 TF 类来构造，让它能创建 ROS2 相关对象，同时发布静态变换
    ekf_ = std::make_unique<EKF>(this); 
    ekf_->UpdateParamsFromServer(); // 设置一大堆参数
    ekf_->SetArmorNum(armor_num_); // 设置装甲板数量
    ballistic_solver_ = std::make_unique<RungeKutta>();

    // 3. 初始化 ROS 通信与多线程
    InitROS2();

    // 4. 初始化 yolo 模型
    std::string yolo_model_path = "/home/cly/project/src/img_processing/model/0526.onnx"; 
    yolo_detector_ = std::make_unique<YoloDetector>(yolo_model_path);

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
    this->declare_parameter("core.logger.show_logger_time", true); // 是否打印时间相关日志
    this->declare_parameter("core.logger.show_logger_else", true); // 是否打印corenode其他的相关日志
    this->declare_parameter("core.image.show_img", true); // 是否显示图片
    
    this->declare_parameter("core.param.chosen_color", "blue"); // 选择的装甲板颜色（blue=0 red=1 gray=2 purple=3）
    this->declare_parameter("core.param.camera_name", "galaxy"); // 使用的相机名称

    this->declare_parameter("pnp.show_logger_debug", true); // 打印 pnp 调试日志

    // EKF相关（传递给ekf）
    this->declare_parameter("ekf.predict_time", 0.1); // 预测时间（记得改！）0.2? 0.225? 其实未来还要根据速度来确定
    this->declare_parameter("ekf.show_logger_debug", true); // ekf 调试

    // 注意 q_z 作为 z轴的平移加速度方差，其它参数弃用
    this->declare_parameter("ekf.q_x", 0.02);
    this->declare_parameter("ekf.q_y", 0.02);  
    this->declare_parameter("ekf.q_v_x", 0.05); 
    this->declare_parameter("ekf.q_v_y", 0.05);
    this->declare_parameter("ekf.q_v_z", 0.01);
    this->declare_parameter("ekf.q_yaw", 0.05); 
    this->declare_parameter("ekf.q_omega", 2.0);

    this->declare_parameter("ekf.q_z", 0.5);  

    this->declare_parameter("ekf.q_v1", 500.0); // 平移加速度方差 
    this->declare_parameter("ekf.q_v2", 1000.0);  // 旋转角加速度方差，小陀螺可以突然转的非常快，给个夸张值 50000
    
    // [ATTENTION]: .0才可以让它是 double，否则会被当成 int 解析，导致 ekf.cpp 里读取参数时出问题

    // 【新增】：模型几何噪声，给很小的值让它平滑收敛
    this->declare_parameter("ekf.q_r", 1e-8); 
    this->declare_parameter("ekf.q_dz", 1e-8);
    
    this->declare_parameter("ekf.r_los_yaw", 4e-3);   // 相机角度极其精准，给极小方差 4e-3
    this->declare_parameter("ekf.r_los_pitch", 4e-3); // 相机角度极其精准，给极小方差 4e-3
    this->declare_parameter("ekf.r_distance", 0.05);  // PnP 测距
    this->declare_parameter("ekf.r_euler_yaw", 0.1);  // 观测到的装甲板欧拉角，经过优化后误差会小很多 0.8

    // TF 参数声明
    this->declare_parameter("tf.show_logger_error", false);
    this->declare_parameter("tf.show_result", false);

    // 开启示波器：
    this->declare_parameter("core.image.show_plot", false); 

    // 新增：模式选择与视频路径参数
    this->declare_parameter("core.mode.is_standalone_mode", true); // 单机 / 联调模式
    this->declare_parameter("core.mode.is_video_mode", true); // 是否为读取 本地视频模式
    this->declare_parameter("core.mode.video_path", "/home/cly/下载/rm_test_videos/20260501_160636__camera_0_rgb_output.mp4"); // 默认的本地视频绝对路径




    // 参数变量获取初始值

    // corenode 节点变量获取初始值
    show_logger_about_time_ = this->get_parameter("core.logger.show_logger_time").as_bool();
    show_logger_about_else_ = this->get_parameter("core.logger.show_logger_else").as_bool();
    show_image_ = this->get_parameter("core.image.show_img").as_bool();
    chosen_color_ = this->get_parameter("core.param.chosen_color").as_string();
    camera_name_ = this->get_parameter("core.param.camera_name").as_string();
    ekf_predict_time_ = this->get_parameter("ekf.predict_time").as_double();
    show_plot_ = this->get_parameter("core.image.show_plot").as_bool(); 

    show_logger_pnp_ = this->get_parameter("pnp.show_logger_debug").as_bool();

    // 新增：关于 本地视频模式 的参数获取
    is_standalone_mode_ = this->get_parameter("core.mode.is_standalone_mode").as_bool();
    is_video_mode_ = this->get_parameter("core.mode.is_video_mode").as_bool();
    video_path_ = this->get_parameter("core.mode.video_path").as_string();

    // 本地视频模式下强制为单机模式，默认是大恒相机得到的图像（因为没有电控发TF）
    if (is_video_mode_) 
    {
        is_standalone_mode_ = true; // 本地视频模式下 强制为单机模式（没有串口）

        // 必须把这个覆盖后的值同步回参数服务器！
        this->set_parameter(rclcpp::Parameter("core.mode.is_standalone_mode", true));
    } 
    else 
    {
        is_standalone_mode_ = this->get_parameter("core.mode.is_standalone_mode").as_bool(); // 否则按照原本的声明（单机模式下就是 相机+corenode，不要串口）
    }

    // 设置相机内参：相机内参K，畸变系数D
    // todo: 之后可以改成从相机配置文件 yaml 获取
    if (is_video_mode_) // 如果是本地视频模式，默认使用大恒相机的内参
    {
        K_ = (cv::Mat_<double>(3, 3) << 1359.21385,    0.     ,  635.62767,
                                                0.     , 1361.75423,  478.48483,
                                                0.     ,    0.     ,    1.     );

        D_ = (cv::Mat_<double>(5, 1) << -0.081521, 0.153947, -0.006919, -0.003306, 0.000000);
    }
    else // 如果不是本地视频模式，根据 camera_name_ 选择相机内参
    {
        if (camera_name_  == "mind_vision") // mind_vision
        {
            K_ = (cv::Mat_<double>(3, 3) << 1359.21385,    0.     ,  635.62767,
                                                0.     , 1361.75423,  478.48483,
                                                0.     ,    0.     ,    1.     );

            D_ = (cv::Mat_<double>(5, 1) << -0.081521, 0.153947, -0.006919, -0.003306, 0.000000);
        }

        else // galaxy
        {
            K_ = (cv::Mat_<double>(3, 3) << 1305.07013,    0.     ,  666.71169,
                                                0.     , 1307.67428,  495.17044,
                                                0.     ,    0.     ,    1.     );

            D_ = (cv::Mat_<double>(5, 1) << -0.209067, 0.129977, -0.002895, -0.000558, 0.000000);
        }
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
        RCLCPP_INFO(this->get_logger(), "[本地视频模式] 将读取本地视频: %s", video_path_.c_str());
        
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
    rclcpp::Time last_virtual_time = virtual_time; // 记录上一帧模拟时间，用来看 fps

    while(rclcpp::ok()) 
    {
        cap >> frame;
        if (frame.empty()) 
        {
            RCLCPP_WARN(this->get_logger(), "视频播放结束...");
            exit(0);
        }
        
        // 视频本身自带的图像时间戳 (ms)
        double video_timestamp_ms = cap.get(cv::CAP_PROP_POS_MSEC);

        // 得到当前帧的模拟图像时间戳 (s)
        virtual_time = base_time + rclcpp::Duration::from_seconds(video_timestamp_ms / 1000.00); 

        RCLCPP_INFO_EXPRESSION(this->get_logger(), show_logger_about_else_, "fps = %.2f", 1.0 / (virtual_time - last_virtual_time).seconds());


        // 调用核心算法主逻辑, 使用 virtual_time 作为当前帧的时间戳
        CoreLogic(frame, virtual_time);

        last_virtual_time = virtual_time; // 更新上一帧的模拟时间
    }
}



// ================= 相机图传 回调函数 =================
void CoreNode::CameraImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 直接把底层的内存映射为 OpenCV 的 Mat
    cv::Mat frame(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data()), msg->step);
    
    // 【修改点在这里】：必须深拷贝！因为 msg 的生命周期在回调结束就会销毁
    cv::Mat safe_frame = frame.clone(); 
    
    // 【修改点在这里】：你刚才传的是 frame！必须传克隆好的 safe_frame！否则一旦切线程直接指针越界段错误！
    CoreLogic(safe_frame, msg->header.stamp);
}



// =========================== 独立主逻辑 ===========================
void CoreNode::CoreLogic(cv::Mat& frame, rclcpp::Time current_image_time)
{
    rclcpp::Time start_time = this->now(); // 记录核心逻辑开始的时间戳，因为算法会有延迟，不能直接用图像的时间戳算！

    // ====================================================================
    // 0. 设置图像 并 记录时间和更新 dt，dt 是整个的基准！！
    // ====================================================================

    img_show_ = frame.clone(); // 复制一份用来显示信息
                               // frame 就是原图，一般不会对原图修改，要用就直接用 frame 就好
    
    double dt = (current_image_time - last_image_time_).seconds(); // 计算与上一次图像的时间间隔，注意sec已经是精确时间！ 
    last_image_time_ = current_image_time; // 更新上一次图像的时间戳


    // t1 = 完成 接收原图 的时间戳
    rclcpp::Time t1 = this->now();


    // ====================================================================
    // 1. 调用 YOLO 提取绝对精准的目标对象
    // ====================================================================
    
    // 映射目标颜色 (0蓝 1红 2灰 3紫)
    int target_color = COLOR_MAP.count(chosen_color_) ? COLOR_MAP.at(chosen_color_) : -1;
    if (target_color == -1) 
    {
        RCLCPP_ERROR(this->get_logger(), "目标颜色 %s 不在映射表内", chosen_color_.c_str());
        return;
    }

    // 调用 yolo 检测，得到仅含敌方装甲板颜色的结果
    auto detected_armors = yolo_detector_->Detect(frame, target_color); // frame 就是原图

    // t2 = 完成 yolo检测 的时间戳
    rclcpp::Time t2 = this->now();

    yolo_armors_.clear();

    // ====================================================================
    // 2. 目标筛选与 PnP 解算
    // ====================================================================
    for (const auto& obj : detected_armors) 
    {
        // 构造新装甲板对象
        // 【核心】：从你的 CoreNode 全局参数里把相机内参传过去！
        // 在 InitParams() 里根据 CAMERA_NAME 设置了 K_ 和 D_
        YoloArmor armor(obj.number, obj.color, obj.is_big, obj.prob, obj.box, obj.pts, K_, D_);

        armor.SetArmorplateSize(); // 设置动态 3D 尺寸
        armor.PrintDebugLog(show_logger_pnp_); // 设置是否打印 PnP 日志
        armor.PNP(); // 执行 PnP，并且内部自动优化 EulerYaw

        if (armor.pnp_success_) 
        {
            armor.DrawAndPrintInfo(img_show_, "simple"); // "simple" / "complex"
            yolo_armors_.push_back(armor);
        }
    }

    // 装甲板排序！也就是如果有多个装甲板，优先选哪个？目前是按照 yolo 置信度排序的，但也可以考虑离中心更近的优先，或者综合评分（距离中心+置信度）
    /*
        既然两块板都要送入 EKF，为什么还要按靠近图像中心排序？
        
        非线性系统的线性化点（EKF 的数学本质）：
            EKF 在计算雅可比矩阵 H 时，是在当前的预测状态 X_est 处进行泰勒展开的。
            靠近图像中心的装甲板，镜头畸变小，YOLO 框最准，PnP 算出的 t 向量也最准。
            将最准的观测（靠近中心的板）放在第一位更新：它能将状态 $X$ 修正到一个非常接近真值的点。
            当进行第二次更新（边缘的、较模糊的板）时，EKF 已经在上一步得到了极准的基准状态。
            此时它的雅可比矩阵 H 计算会非常精确，从而能完美榨取第二块板带来的“角度约束”价值，同时用较低的卡尔曼增益过滤掉它位置不准的劣势。
            
        状态机安全冗余（Set Target）：
            当 Tracker 处于 Lost 状态时，第一帧必须选一块板作为初始化基准。
            选最靠近中心的板，能保证初始化的车心坐标不带有太大的偏置。
    
    */
    if (yolo_armors_.size() > 1) 
    {
        // 1. 获取画面中心点
        cv::Point2f img_center(frame.cols / 2.0f, frame.rows / 2.0f); // frame 就是原图

        // 2. 按装甲板 2D 框中心到图像中心的距离从小到大排序，越近越优先
        std::sort(yolo_armors_.begin(), yolo_armors_.end(), 
        [&img_center](const YoloArmor& a, const YoloArmor& b) {
            float dist_a = std::pow((a.box_.x + a.box_.width / 2.0f) - img_center.x, 2) + 
                           std::pow((a.box_.y + a.box_.height / 2.0f) - img_center.y, 2);
            float dist_b = std::pow((b.box_.x + b.box_.width / 2.0f) - img_center.x, 2) + 
                           std::pow((b.box_.y + b.box_.height / 2.0f) - img_center.y, 2);
            return dist_a < dist_b; 
        });

        // 3. 稳定排序 (保留中心距离的相对顺序)，优先高价值目标 
        std::stable_sort(yolo_armors_.begin(), yolo_armors_.end(), [](const YoloArmor& a, const YoloArmor& b) {
            return a.is_big_ > b.is_big_; // true(1) 排在 false(0) 前面
        });
    }

    // t3 = 完成 目标筛选与pnp解算 的时间戳
    rclcpp::Time t3 = this->now();
    


    
    // ====================================================================
    // 4. Tracker 状态机流转 (决策)
    // ====================================================================
    bool is_found = (yolo_armors_.size() > 0); // yolo 是否检测到目标
    
    UpdateTrackerState(is_found); // 调用状态机判定函数

    // 预先声明并初始化核心数据，这些数据将被 ExecuteTracker 修改并带出

    Eigen::Vector3d armorplate_center_now(-999, -999, -999); 
    double yaw_armorplate_now = -999; 

    Eigen::Vector3d armorplate_center_filter(-999, -999, -999); 
    Eigen::Vector3d armorplate_center_predict(-999, -999, -999);
    Eigen::Vector3d armorplate_center_predict_rungekutta(-999, -999, -999);
    Eigen::Vector3d car_center_predict(-999, -999, -999); 


    // ====================================================================
    // 5. 状态机执行 (动作)
    // ====================================================================
    ExecuteTracker(dt, current_image_time, 
                   yaw_armorplate_now, 
                   armorplate_center_now, armorplate_center_filter, armorplate_center_predict, 
                   car_center_predict);

    cv::putText(img_show_, "r1 = " + std::to_string((double)ekf_->X(8)), cv::Point2f(0, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
    cv::putText(img_show_, "r2 = " + std::to_string((double)ekf_->X(9)), cv::Point2f(0, 100), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
    cv::putText(img_show_, "dz = " + std::to_string((double)ekf_->X(10)), cv::Point2f(0, 150), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);


    // ====================================================================
    // 6. 计算最终预测云台偏角 并 发送最终信息
    // ====================================================================
    // 只有在预测出有效坐标时才进行解算

    double pitch_result = -999, pitch_result_revised = -999;
    double yaw_result = -999, yaw_result_revised = -999;;

    if (armorplate_center_predict[0] != -999 && armorplate_center_predict[1] != -999 && armorplate_center_predict[2] != -999)
    {
        // 算出云台需要瞄准的目标 Pitch 和 Yaw
        pitch_result = tools::rad2deg(-std::atan2(armorplate_center_predict[2], std::sqrt(armorplate_center_predict[0] * armorplate_center_predict[0] + armorplate_center_predict[1] * armorplate_center_predict[1])));   
        yaw_result = tools::rad2deg(std::atan2(armorplate_center_predict[1], armorplate_center_predict[0]));
        RCLCPP_INFO_EXPRESSION(this->get_logger(), show_logger_about_else_, "armorplate_center_predict: [%.3f, %.3f, %.3f]", armorplate_center_predict[0], armorplate_center_predict[1], armorplate_center_predict[2]);
    
        // 进行弹道解算得到实际角度
        std::pair<double, double> result = ballistic_solver_->SolveAim(armorplate_center_predict[0], armorplate_center_predict[1], -armorplate_center_predict[2]);
        pitch_result_revised = tools::rad2deg(result.first);
        yaw_result_revised   = tools::rad2deg(result.second);

        // 有可能失败，暂时使用直线瞄准
        if (std::isnan(result.first) || std::isnan(result.second)) 
        {
            // 回退到直线瞄准
            pitch_result_revised = pitch_result;
            yaw_result_revised   = yaw_result;
            RCLCPP_WARN(this->get_logger(), "[弹道解算] 弹道解算失败，使用直线瞄准！");
        }

        // 构造 rk4 得到的实际瞄准角，在预测装甲板中心处，z轴的投影点坐标
        double dist = std::sqrt(armorplate_center_predict[0] * armorplate_center_predict[0] + armorplate_center_predict[1] * armorplate_center_predict[1]);
        armorplate_center_predict_rungekutta = Eigen::Vector3d(armorplate_center_predict[0], armorplate_center_predict[1], -(dist * std::tan(result.first)));
        RCLCPP_INFO_EXPRESSION(this->get_logger(), show_logger_about_else_, "armorplate_center_predict_rungekutta: [%.3f, %.3f, %.3f]", armorplate_center_predict_rungekutta[0], armorplate_center_predict_rungekutta[1], armorplate_center_predict_rungekutta[2]);
    }

    auto send_msg = serial_driver_interfaces::msg::SerialDriver();
    send_msg.pitch = pitch_result_revised;
    send_msg.yaw = yaw_result_revised;
    serial_pub_->publish(send_msg);
    RCLCPP_INFO_EXPRESSION(this->get_logger(), show_logger_about_else_, "[弹道解算] 原计算视场角: pitch = %.2f, yaw = %.2f", pitch_result, yaw_result);
    RCLCPP_INFO_EXPRESSION(this->get_logger(), show_logger_about_else_, "[弹道解算] rk4 的修正角: pitch = %.2f, yaw = %.2f", pitch_result_revised, yaw_result_revised);


    // t4 = 完成 状态机流转执行、计算和发送数据 的时间戳
    rclcpp::Time t4 = this->now();


    // 7. 重投影 可视化 (颜色匹配 红0,黄1,蓝2,绿3)
    // ekf_->GetArmorplateFourCorners 拿到的是 实时滤波值
    if (ekf_ready_ && pitch_result != -999 && yaw_result != -999)
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
                tf_->ProjectAndDraw(img_show_, corners_world, K_, D_, T_world_cam, colors[id], id);
            }

            // 也绘制整车中心点（实时滤波位置）
            std::vector<Eigen::Vector3d> center_world(1);
            ekf_->GetCarCenterPredict(center_world[0], 0.00); // 获取实时位置下的整车中心点 在父坐标系下 的位置坐标
            tf_->ProjectAndDraw(img_show_, center_world, K_, D_, T_world_cam, cv::Scalar(255, 255, 255));
        
            // 也绘制当前装甲板 dt后的 预测值（不是外推值）
            std::vector<Eigen::Vector3d> armorplate_center_world(1);
            armorplate_center_world[0] = armorplate_center_predict; // 预测位置的装甲板中心点 在父坐标系下 的位置坐标
            tf_->ProjectAndDraw(img_show_, armorplate_center_world, K_, D_, T_world_cam, cv::Scalar(255, 0, 255));
        
            // 也绘制当前装甲板 dt后的 预测值的 rk4 得到的实际瞄准角，在预测装甲板中心处，z轴的投影点坐标
            std::vector<Eigen::Vector3d> armorplate_center_world_rungekutta(1);
            armorplate_center_world_rungekutta[0] = armorplate_center_predict_rungekutta; // 预测位置的装甲板中心点 在父坐标系下 的位置坐标
            tf_->ProjectAndDraw(img_show_, armorplate_center_world_rungekutta, K_, D_, T_world_cam, cv::Scalar(0, 165, 255));
        }
    }
    

    // 8. 联调模式下，在图上打印要发布的目标角度 和 电控发来的目前芯片姿态
    if (!is_standalone_mode_)
    {
        double pitch_chip;
        double yaw_chip;
        bool tf_flag = tf_->GetWorldToChipTransform(pitch_chip, yaw_chip, current_image_time); // 获取【世界坐标系】->【芯片坐标系的变换】
        
        if (tf_flag)
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
    


    // t5 = 完成 重投影和可视化 的时间戳
    rclcpp::Time t5 = this->now();



    // 9. 计算每个步骤的耗时，并打印

        double duration1 = (t1 - start_time).seconds() * 1000.0; // 完成 接收原图 的时间
        double duration2 = (t2 - t1).seconds() * 1000.0; // 完成 yolo检测 的时间
        double duration3 = (t3 - t2).seconds() * 1000.0; // 完成 目标筛选与pnp解算 的时间
        double duration4 = (t4 - t3).seconds() * 1000.0; // 完成 状态机流转执行、计算和发送数据 的时间
        double duration5 = (t5 - t4).seconds() * 1000.0; // 完成 重投影和可视化 的时间
        double total_duration = (t5 - start_time).seconds() * 1000.0; // 完成 整个 CoreLogic 的时间
        double algorithm_duration = total_duration - duration2;
        RCLCPP_INFO_EXPRESSION(this->get_logger(), show_logger_about_time_ && yolo_armors_.size(), 
            "本帧处理耗时: "
            "接收原图 = %.4f ms, "
            "yolo检测 = %.4f ms, "
            "目标筛选与pnp解算 = %.4f ms, "
            "状态机流转执行计算发送 = %.4f ms, "
            "重投影和可视化 = %.4f ms, "
            "总耗时 = %.4f ms (算法耗时 = %.4f ms)", duration1, duration2, duration3, duration4, duration5, total_duration, algorithm_duration);
    

    ShowImg();

    UpdatePlotter(armorplate_center_now, armorplate_center_filter, armorplate_center_predict);

    RCLCPP_INFO_ONCE(this->get_logger(), "CoreLogic 正在运行...");
}




// ============================== 工具类 ==============================


// 由于写了return，必须在每次 return 前 显示图片
void CoreNode::ShowImg()
{
    if (show_image_)
    {
        cv::imshow("img_show_armorplate", img_show_);
        cv::waitKey(1);
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
        pitch_now = tools::rad2deg(-std::atan2(armorplate_center_now[2], std::sqrt(armorplate_center_now[0] * armorplate_center_now[0] + armorplate_center_now[1] * armorplate_center_now[1])));   
        yaw_now = tools::rad2deg(std::atan2(armorplate_center_now[1], armorplate_center_now[0]));
    }
    

    // 计算最终角（实时滤波）
    if (armorplate_center_filter[0] != -999 && armorplate_center_filter[1] != -999 && armorplate_center_filter[2] != -999)
    {
        pitch_filter = tools::rad2deg(-std::atan2(armorplate_center_filter[2], std::sqrt(armorplate_center_filter[0] * armorplate_center_filter[0] + armorplate_center_filter[1] * armorplate_center_filter[1])));   
        yaw_filter = tools::rad2deg(std::atan2(armorplate_center_filter[1], armorplate_center_filter[0]));
    }


    // 计算最终角（预测）（滤波器没初始好就用原始值，连续检测多就用预测值，丢帧但滤波器稳定就用外推值）
    if (armorplate_center_predict[0] != -999 && armorplate_center_predict[1] != -999 && armorplate_center_predict[2] != -999)
    {
        pitch_result = tools::rad2deg(-std::atan2(armorplate_center_predict[2], std::sqrt(armorplate_center_predict[0] * armorplate_center_predict[0] + armorplate_center_predict[1] * armorplate_center_predict[1])));   
        yaw_result = tools::rad2deg(std::atan2(armorplate_center_predict[1], armorplate_center_predict[0]));
    }


    // 3. 把这帧的状态送入示波器
    plotter_->UpdateAndDraw(
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
                              double& yaw_armorplate_now,                   // 输出：tf 查到的装甲板当前角度
                              Eigen::Vector3d& armorplate_center_now,       // 输出：tf 查到的装甲板当前位置
                              Eigen::Vector3d& armorplate_center_filter,    // 输出：滤波器得到的装甲板当前位置
                              Eigen::Vector3d& armorplate_center_predict,   // 输出：滤波器得到的装甲板预测位置
                              Eigen::Vector3d& car_center_predict)          // 输出：滤波器得到的车体中心预测位置
{
    // ====== 场景 A：看到目标了 (处于 DETECTING 预热期 或 TRACKING 稳定期) ======
    if (tracker_state_ == TrackerState::DETECTING || tracker_state_ == TrackerState::TRACKING) 
    {
        
        // 1. EKF 初始化

        if (!ekf_->is_initialized_) 
        {
            if (yolo_armors_.size() > 0) 
            {
                // 使用距离视野中心最近的目标（已经排序好了，即 yolo_armors_[0]）进行初始化
                Eigen::Vector3d init_center(-999, -999, -999);
                double init_yaw = -999.99;
                
                tf_->UpdateCameraToArmorplate(yolo_armors_[0].R_, yolo_armors_[0].t_vec_, current_image_time);
                bool tf_ok = tf_->GetFatherToArmorplateTransform(yolo_armors_[0].R_, yolo_armors_[0].t_vec_, init_center, init_yaw, current_image_time);
                
                if (tf_ok) 
                {
                    ekf_->Initialized(init_center, init_yaw, tracking_id_, current_image_time);
                    ekf_ready_ = false; // 刚初始化，处于预热期，绝对不可信任其盲推能力

                    // 修复：立刻赋原始值，确保第一帧就有输出！
                    armorplate_center_now = init_center;
                    yaw_armorplate_now = init_yaw;

                    // 此时，滤波值和预测值直接强制等于原始观测值
                    armorplate_center_filter = armorplate_center_now;
                    armorplate_center_predict = armorplate_center_now;
                }
            }
            return; // 初始化这一帧只做初始化，直接结束
        }


        
        // 2. 先验预测，先把 EKF 的状态 从 current_image_time - dt 推到 current_image_time 现在
        
        ekf_->PredictState(dt, current_image_time);  


        // 一些统计和记录的参数

        bool has_valid_update = false;          // 是否存在有效的更新
        bool has_checked_to_switch_id = false;  // 是否已经检查过 要不要切换跟踪的板子

        // 临时变量，用于 存储 追踪的装甲板的 TF 变换的结果（原始数据）
        Eigen::Vector3d tracking_center_now(-999, -999, -999);
        double tracking_yaw_now = -999.99;

        std::vector<int> id_set; // 存储识别到的装甲板 ID


        // 3. EKF 序贯匹配与后验更新，循环将本帧有效装甲板依次送入 EKF

        for (size_t i = 0; i < yolo_armors_.size(); i++)
        {
            // 临时变量，用于 接收 当前该装甲板的 TF 变换的结果（原始数据）
            Eigen::Vector3d current_center_now(-999, -999, -999);
            double current_yaw_now = -999.99;

            // ========== 3.1. 获得当前板子的 TF 变换 ==========

            tf_->UpdateCameraToArmorplate(yolo_armors_[i].R_, yolo_armors_[i].t_vec_, current_image_time);
            bool tf_ok = tf_->GetFatherToArmorplateTransform(yolo_armors_[i].R_, yolo_armors_[i].t_vec_, current_center_now, current_yaw_now, current_image_time);
        
            if (!tf_ok) 
            {
                RCLCPP_ERROR(this->get_logger(), "[tf] 在 TRACKING / DETECTING 状态下, 板子 %d 查不到 父坐标系 到 装甲板坐标系 的 TF！跳过...", i);
                continue;
            }

            // ========== 3.2. ID 匹配逻辑，判断当前识别的这块板对应哪个 ID ==========

            int current_id = tracking_id_; // 先默认当前追踪 ID 是上一帧的追踪 ID，后续根据 EKF 的预测来识别当前这块板子真正的 ID
            double min_angle_error = 1e5;

            // 3.2.1 获取所有预测板的距离，按距离排序，剔除最远的背板
            std::vector<std::pair<double, int>> dist_id_list;
            for (int k = 0; k < armor_num_; k++) 
            {
                Eigen::Vector3d pred_center;
                ekf_->GetArmorplatePredict(pred_center, k, 0.0); // 获得第 k 个板子的 目前时刻 的预测位置
                double dist = pred_center.norm();
                dist_id_list.push_back({dist, k}); // todo: 这里有点奇怪
            }
            std::sort(dist_id_list.begin(), dist_id_list.end());

            // 3.2.2 只遍历离 Father Frame 最近的 3 块板
            for (int k = 0; k < 3; k++) 
            {
                int id = dist_id_list[k].second;
                
                Eigen::Vector3d pred_center;
                ekf_->GetArmorplatePredict(pred_center, id, 0.0);
                
                double pred_yaw = tools::limit_rad(ekf_->X(6) + id * 2.0 * M_PI / armor_num_);
                double pred_los_yaw = std::atan2(pred_center[1], pred_center[0]);
                double obs_los_yaw = std::atan2(current_center_now[1], current_center_now[0]);

                // 同济复合误差：姿态误差 + 视线误差
                double error = std::abs(tools::limit_rad(current_yaw_now - pred_yaw)) + 
                                std::abs(tools::limit_rad(obs_los_yaw - pred_los_yaw));
                
                if (error < min_angle_error) 
                {
                    min_angle_error = error;
                    current_id = id;
                }
            }

            //（粗筛，仅仅筛掉绝对不可能的候选项）防止漏了高质量的可以用的板子
            // 3.2.3 限制一帧内目标（当前观测的装甲板）可能发生的最大位移（角度），也就是与 EKF 预测的误差值最大可以多大
            // 极限小陀螺的角速度下，一帧大概能转多少弧度 -> 20 * 0.016 * 4(考虑误差) = 1.28 rad，而 1.25 rad = 71.6 度
            if (min_angle_error > 1.25) 
            {
                RCLCPP_WARN(this->get_logger(), "板子 %d 是畸形板，误差高达 %.2f, 启动门限拒绝，丢弃数据！", i, min_angle_error);
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "[识别器] 当前 识别 的第 %d / %zu 块装甲板, 对应 ID = %d", i + 1, yolo_armors_.size(), current_id);

            // 无论是 DETECTING 还是 TRACKING，只要有数据就必须给 EKF。
            std::string size = yolo_armors_[i].is_big_ ? "hero" : "normal";
            ekf_->SetArmorplateSize(size);

            tools::limit_rad_inplace(current_yaw_now);
            ekf_->UpdateState(current_center_now, current_yaw_now, current_id, current_image_time); 
            has_valid_update = true;
            id_set.push_back(current_id);

            // 如果当前板子 就是 追踪目标，那么就保存追踪版对应的 tf 数据（原始数据）
            if (current_id == tracking_id_)
            {
                tracking_center_now = current_center_now;
                tracking_yaw_now = current_yaw_now;
            }

            // 注意！我们用现在的 实时的装甲板目标 更新完 EKF，EKF 就已经可以推出任何一个 ID 对应的状态了，接下来的 tracking_id 只是决定要打和跟踪谁而已！
            
            // 只用 最靠近中心的板 来决定要不要切换跟踪对象
            if (!has_checked_to_switch_id)
            {
                // 发现现在 最靠近中心 的装甲板 ID 和之前追踪的 ID 不一样了，直接去跟踪他
                // todo: 切板（跟踪板）逻辑需要制定！
                if (current_id != tracking_id_) 
                {
                    RCLCPP_WARN(this->get_logger(), "[跟踪器] 已经切换跟踪装甲板，不再跟踪 ID = %d。开始跟踪 ID = %d", tracking_id_, current_id);

                    // 更新当前装甲板目标，因此要保存 tf 查到的原始结果，让 corenode 使用
                    tracking_center_now = current_center_now;
                    tracking_yaw_now = current_yaw_now;
                    tracking_id_ = current_id; // 跟踪新板
                }
                has_checked_to_switch_id = true;
            }
        }
            
        // 4. 拿走 EKF 得到的滤波值和预测值

        // 在图上打印出装甲板类别，和 识别到的装甲板 ID 集合，与 正在追踪的装甲板的 ID
        cv::putText(img_show_, "Armor Type ID: " + std::to_string(yolo_armors_[0].armor_id_), cv::Point2f(0, 200), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
        std::sort(id_set.begin(), id_set.end());
        if (id_set.empty()) 
        {
            cv::putText(img_show_, "Detecting: None  Tracking: " + std::to_string(tracking_id_), cv::Point2f(0, 250), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
        } 
        else if (id_set.size() == 1) 
        {
            cv::putText(img_show_, "Detecting: id=" + std::to_string(id_set[0]) + "   Tracking: " + std::to_string(tracking_id_), cv::Point2f(0, 250), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
        } 
        else 
        {
            cv::putText(img_show_, "Detecting: id=" + std::to_string(id_set[0]) + " " + std::to_string(id_set[1]) + " Tracking: " + std::to_string(tracking_id_), cv::Point2f(0, 250), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
        }
        
        // 先拿到 当前装甲板目标 的原始数据
        armorplate_center_now = tracking_center_now;
        yaw_armorplate_now = tracking_yaw_now;  

        if (has_valid_update) // 存在有效的更新，就可以获取滤波值和预测值了
        {
            reject_count_ = 0; // 没有拒绝 yolo 结果，拒绝计数器重置

            // 【还在预热期】
            if (tracker_state_ == TrackerState::DETECTING) 
            {
                //【预热期】：EKF 的速度和加速度还在剧烈震荡。此时它的预测值是垃圾。
                // 此时，滤波值和预测值直接强制等于原始观测值
                armorplate_center_filter = armorplate_center_now;
                armorplate_center_predict = armorplate_center_now;
                ekf_ready_ = false; // 预热期不能盲推，不能直接使用对当前帧的预测数据
            }
            else if (tracker_state_ == TrackerState::TRACKING) 
            {
                //【稳定期】：EKF 已经完全收敛。使用 EKF 的输出作为画图和云台控制的基准！
                
                // future_time = 0.0，获取的就是纯正的【当前滤波值】！
                ekf_->GetArmorplatePredict(armorplate_center_filter, tracking_id_, 0.00);
                
                // future_time = ekf_predict_time_，获取未来的【预测值】！
                ekf_->GetArmorplatePredict(armorplate_center_predict, tracking_id_, ekf_predict_time_);
                ekf_->GetCarCenterPredict(car_center_predict, ekf_predict_time_);
                
                ekf_ready_ = true; // 稳定期可以开始盲推，可以直接使用对当前帧的预测数据
            }
        }
        else // 没有有效的更新，看看能不能盲推（使用对当前帧的预测数据）
        {
            ++reject_count_;

            if (reject_count_ >= max_reject_frames_)
            {
                RCLCPP_ERROR(this->get_logger(), "yolo 检测到装甲板，但连续 %d 帧数据都全被 EKF 拒绝，强制重置！", reject_count_);
                tracker_state_ = TrackerState::LOST;
                ekf_ready_ = false; 
                tracking_id_ = 0; 
                ekf_->Reset(); 
                reject_count_ = 0; // 重置
            }

            if (ekf_ready_) 
            {
                ekf_->GetArmorplatePredict(armorplate_center_filter, tracking_id_, 0.00);
                ekf_->GetArmorplatePredict(armorplate_center_predict, tracking_id_, ekf_predict_time_);
                ekf_->GetCarCenterPredict(car_center_predict, ekf_predict_time_);
            }
        }

        // 5. 最终发散保护，如果 EKF 察觉自己内部崩塌了就赶紧重置
        if (ekf_->IsDiverged() || ekf_->IsNISFailed()) 
        {
            if (ekf_->IsDiverged()) RCLCPP_ERROR(this->get_logger(), "[发散检测] 滤波器 物理发散！强制重置！");
            else RCLCPP_ERROR(this->get_logger(), "[发散检测] 滤波器 NIS 崩溃！强制重置！");
            tracker_state_ = TrackerState::LOST;
            ekf_ready_ = false; 
            tracking_id_ = 0; // 默认追踪 0 板
            ekf_->Reset(); 
        }
        
    }
    // ====== 场景 B：短暂丢失阶段 (没有测量值，只能利用盲推的预测，维持数据) ======
    else if (tracker_state_ == TrackerState::TEMP_LOST)
    {
        if (ekf_ready_) 
        {
            // 把 EKF 的状态 从 current_image_time - dt 推到 current_image_time 现在
            // 传入当前帧图像时间，确保tf发布时，tf 的时间戳是当前帧图像时间！
            ekf_->PredictState(dt, current_image_time);
            
            // 此时取 future=0 就是外推出来的 当前帧的 预估现在位置
            ekf_->GetArmorplatePredict(armorplate_center_filter, tracking_id_, 0.00);
            ekf_->GetArmorplatePredict(armorplate_center_predict, tracking_id_, ekf_predict_time_);
            ekf_->GetCarCenterPredict(car_center_predict, ekf_predict_time_);
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "[ekf] 目标遮挡或丢失, EKF 正在盲推，直接使用对当前帧的预测数据...");
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
            RCLCPP_INFO(this->get_logger(), "打印 corenode 节点耗时开关已切换! ");
        } 

        else if (name == "core.logger.show_logger_else") 
        {
            show_logger_about_else_ = p.as_bool();
            RCLCPP_INFO(this->get_logger(), "打印 corenode 节点其他日志的开关已切换! ");
        } 

        else if (name == "pnp.show_logger_debug") 
        {
            show_logger_pnp_ = p.as_bool();
            RCLCPP_INFO(this->get_logger(), "打印 PNP 调试信息开关已切换! ");
        }

        else if (name == "core.image.show_img") 
        {
            show_image_ = p.as_bool();
            RCLCPP_INFO(this->get_logger(), "显示图片开关已切换! ");
        } 

        else if (name == "core.param.chosen_color") 
        {
            chosen_color_ = p.as_string();
            RCLCPP_INFO(this->get_logger(), "装甲板颜色已更新!");
        } 

        else if (name == "ekf.q_x" || name == "ekf.q_y" || name == "ekf.q_z" || 
                name == "ekf.q_v_x" || name == "ekf.q_v_y" || name == "ekf.q_v_z" ||
                name == "ekf.q_yaw" || name == "ekf.q_omega" ||
                name == "ekf.q_r" || name == "ekf.q_dz" ||
                name == "ekf.q_v1" || name == "ekf.q_v2" ||
                name == "ekf.r_x" || name == "ekf.r_y" || name == "ekf.r_z" ||
                name == "ekf.r_yaw" || name == "ekf.show_logger_debug") 
        {
            ekf_->UpdateParamsFromServer();  // 让EKF自己重新读取参数
            RCLCPP_INFO(this->get_logger(), "EKF 参数已更新! ");
        }

        else if (name == "tf.show_logger_error" || name == "tf.show_result")
        {
            tf_->UpdateParamsFromServer();  // 通知 TF 刷新
            RCLCPP_INFO(this->get_logger(), "TF 节点参数已更新! ");
        }

        else if (name == "core.mode.is_standalone_mode")
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

        else
        {
            RCLCPP_WARN(this->get_logger(), "未知的参数被修改! ");
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