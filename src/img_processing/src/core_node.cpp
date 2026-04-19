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

        this->tf = std::make_unique<TF>(this); // 传 this 指针给 TF 类来构造，让它能创建 ROS2 相关对象，同时发布静态变换

        // 初始化 KF 类 和 EKF 类 指针
        // kf_position_ = std::make_unique<KalmanFilter>(); 
        // kf_data_ = std::make_unique<KF>(); 
        ekf_ = std::make_unique<EKF>(this); 

        this->last_lookup_time_ = this->now(); // 初始化 上一次滤波时间

        // ------------------ 进行一个配置文件的读取 -----------------

        std::ifstream file("config.txt");  // 打开配置文件，注意是在工作空间下
        if (!file.is_open()) 
        {
            RCLCPP_ERROR(this->get_logger(), "【 EXIT 】无法打开 config.txt 配置文件。。。。即将退出 core 节点\n");
            exit(-1);
        }

        std::string each_line;
        int line_count = 0; // 记录行数

        while (std::getline(file, each_line)) 
        {
            // 处理每一行，each_line 即为当前行的字符串
            if (each_line.empty() || each_line[0] == '#' || each_line[0] == '/') continue;
            else
            {
                ++line_count;
                RCLCPP_INFO(this->get_logger(), "已读取配置文件第 %d 个有效行: %s", line_count, each_line.c_str());
                if(line_count == 1)
                {
                    if(each_line == "false" || each_line == "False" || each_line == "FALSE") 
                    {
                        this->SHOW_LOGGER_TIME = false;
                    }
                    else this->SHOW_LOGGER_TIME = true;
                    RCLCPP_INFO(this->get_logger(), "【 设置参数 】SHOW_LOGGER_TIME = %s", each_line.c_str());
                }

                else if(line_count == 2)
                {
                    if(each_line == "false" || each_line == "False" || each_line == "FALSE") 
                    {
                        this->SHOW_IMG_SHOW = false;
                    }
                    else this->SHOW_IMG_SHOW = true;
                    RCLCPP_INFO(this->get_logger(), "【 设置参数 】SHOW_IMG_SHOW = %s", each_line.c_str());
                }

                else if(line_count == 3)
                {
                    if(each_line == "false" || each_line == "False" || each_line == "FALSE") 
                    {
                        this->SHOW_LOGGER_PREPARE = false;
                    }
                    else this->SHOW_LOGGER_PREPARE = true;
                    RCLCPP_INFO(this->get_logger(), "【 设置参数 】SHOW_LOGGER_PREPARE = %s", each_line.c_str());
                }

                else if(line_count == 4)
                {
                    this->CHOSEN_COLOR = each_line;
                    RCLCPP_INFO(this->get_logger(), "【 设置参数 】CHOSEN_COLOR = %s", each_line.c_str());
                }

                else if(line_count == 5)
                {
                    this->CAMERA_NAME = each_line;
                    RCLCPP_INFO(this->get_logger(), "【 设置参数 】CAMERA_NAME = %s", each_line.c_str());
                }

                else if(line_count == 6)
                {
                    this->ARMOR_TYPE = each_line;
                    RCLCPP_INFO(this->get_logger(), "【 设置参数 】ARMOR_TYPE = %s", each_line.c_str());
                }
            }
        }

        if(line_count < 6)
        {
            RCLCPP_ERROR(this->get_logger(), "配置文件的有效行数不足6行, 检查配置文件。即将退出 core 节点\n");
            exit(-1);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "【 设置参数完成 】ALL SET! 共设置了 %d 个有效参数", line_count);
        }

        file.close();


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
    }


private:

    // 核心逻辑
    void process_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rclcpp::Time start = this->now();
    
        // 1. 接收图像
        this->img = cv_bridge::toCvCopy(msg, "bgr8")->image; // 需要 SharedPtr 作为参数
        this->img_show = this->img.clone(); // 复制一份用来显示信息

        // ... 步骤1 接收图像完成时间戳
        rclcpp::Time t1 = this->now();

        // 2. 预处理（掩码、灯条筛选、灯条配对）
        prepare.setParam(this->SHOW_LOGGER_PREPARE, this->CHOSEN_COLOR, this->CAMERA_NAME, this->ARMOR_TYPE); // 传入从配置文件读取的参数
        prepare.setImgShow(this->img_show); // 设置 img_show
        prepare.preProcessing(img); // 图像预处理
        this->strip = prepare.findAndJudgeLightStrip(); // 找灯带 返回灯带集合
		this->armorplate = prepare.pairStrip(); // 灯条配对 【修改】返回按置信度排序后的装甲板集合
        this->img_show = prepare.getImgShow(); // 获取预处理后带有信息的 img_show

        // ... 步骤2 预处理完成时间戳
        rclcpp::Time t2 = this->now();
        

        // 3. 解算 pnp
        if(this->armorplate.size() > 0) // 存在装甲板
        {
            this->armorplate[0].setImgShow(this->img_show); // 设置 img_show
            this->armorplate[0].perspectiveNPoint(); // 解算 pnp
            this->armorplate[0].drawArmorPlateAndPrintPNPInfo(); // 画置信度最高的装甲板，并打印 pnp 信息
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

        double pitch_result;
        double yaw_result;

        double dt = (this->now() - this->last_lookup_time_).seconds(); // 距离上一次滤波的时间间隔

        Eigen::Vector3d armorplate_center_predict; // 预测点位置
        Eigen::Vector3d car_center_predict; // 整车中心的预测点位置


        // 首先 pnp结算必须要有结果 才能发布tf变换
        if(this->armorplate.size() > 0 && this->armorplate[0].is_success) 
        {
            ++this->continuous_count; // 连续计数器 + 1
            this->lost_count = 0; // 连续丢失计数器清零

            // 丢弃前面 2 帧不稳定的数据，纯丢。这两个数据不计重复丢失次数
            if(this->continuous_count <= CONTINUOUS_THRESHOLD)  
            {
                return;
            }


            // 发送【相机坐标系】->【装甲板坐标系】的 tf
            this->tf->updateCameraToArmorplate(this->armorplate[0].R, this->armorplate[0].t_vec);
        
            // 查找【世界坐标系】->【装甲板坐标系】变换
            Eigen::Vector3d armorplate_center;
            double yaw_armorplate;
            bool flag = this->tf->getWorldToArmorplateTransform(armorplate_center, yaw_armorplate);

            // 如果查到了变换，就开始滤波
            if(flag)
            {
                this->ekf_->getKalman(armorplate_center, yaw_armorplate, 3, dt);
                
                armorplate_center_predict = armorplate_center; // 【3-4帧】先用观测值

                // 只有 用有效帧的前几帧初始化了滤波器 才用预测值，否则就用观测值
                if(this->continuous_count > CONTINUOUS_THRESHOLD + FILTER_INIT_THRESHOLD)
                {
                    this->ekf_ready = true; // 已经稳定，无论怎么丢我都外推
                    this->ekf_->getArmorPredict(armorplate_center_predict, 3, 0.20);
                    this->ekf_->getCenterPredict(car_center_predict, 0.20);
                }

            }
            else // 丢失方法2：没查到变换
            {
                ++this->lost_count; // 连续丢失计数器 + 1

                if(this->ekf_ready && this->lost_count <= this-> MAX_LOST_COUNT)
                {
                    this->ekf_->predictOnly(dt);  // 继续预测 dt 时间，维持状态外推

                    // 用外推的信息发
                    this->ekf_->getArmorPredict(armorplate_center_predict, 3, 0.20);
                    this->ekf_->getCenterPredict(car_center_predict, 0.20);
                }
                else
                {
                    this->ekf_ready = false; // 丢的太多了 不ready
                    this->continuous_count = 0; // 数据作废
                    this->ekf_->reset();
                    RCLCPP_WARN_ONCE(this->get_logger(), "目标丢失超过 %d 帧, 或者滤波器未初始化好。EKF 已重置", this->MAX_LOST_COUNT);
                    return;
                }
            }

            

        }
        else // 丢失方法1：不存在装甲板 或者 pnp解算失败
        {
            ++this->lost_count; // 连续丢失计数器 + 1

            if(this->armorplate.size() == 0) RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 500, "未检测到装甲板");
            else RCLCPP_ERROR(this->get_logger(), "pnp 解算失败");


            if(this->ekf_ready && this->lost_count <= this-> MAX_LOST_COUNT)
            {
                this->ekf_->predictOnly(dt);  // 继续预测 dt 时间，维持状态外推

                // 用外推的信息发
                this->ekf_->getArmorPredict(armorplate_center_predict, 3, 0.20);
                this->ekf_->getCenterPredict(car_center_predict, 0.20);
            }
            else
            {
                this->ekf_ready = false; // 丢的太多了 不ready
                this->continuous_count = 0; // 数据作废
                this->ekf_->reset();
                RCLCPP_WARN_ONCE(this->get_logger(), "目标丢失超过 %d 帧, EKF 已重置", this->MAX_LOST_COUNT);
                return;
            }

        }

        // 计算最终角（滤波器没初始好就用原始值，连续检测多就用预测值，丢帧但滤波器稳定就用外推值）
        pitch_result = -std::atan2(armorplate_center_predict[2], std::sqrt(armorplate_center_predict[0] * armorplate_center_predict[0] + armorplate_center_predict[1] * armorplate_center_predict[1])) * 180.0 / M_PI;   
        yaw_result = std::atan2(armorplate_center_predict[1], armorplate_center_predict[0]) * 180.0 / M_PI;

        this->last_lookup_time_ = this->now(); // 更新上一次滤波时间
            
        
        // 5. 发送 最终信息
        auto send_msg = serial_driver_interfaces::msg::SerialDriver();
        send_msg.pitch = pitch_result;
        send_msg.yaw = yaw_result;
        serial_pub_->publish(send_msg);
         
        



        // 6. 计算每个步骤的耗时，并打印
        if(this->SHOW_LOGGER_TIME && this->armorplate.size() > 0 && this->armorplate[0].is_success)
        {
            double duration1 = (t1 - start).seconds() * 1000.0; // 转换为毫秒
            double duration2 = (t2 - t1).seconds() * 1000.0; // 转换为毫秒
            double duration3 = (t3 - t2).seconds() * 1000.0; // 转换为毫秒
            double total_duration = (t3 - start).seconds() * 1000.0; // 转换为毫秒
            RCLCPP_INFO(this->get_logger(), "本帧处理耗时: 接受图像 = %.4f ms, 预处理 = %.4f ms, pnp 解算 = %.4f ms, 总耗时 = %.4f ms", duration1, duration2, duration3, total_duration);
        }
        


        ///////////////////////////////////////////////////////

        // cv::imshow("img", img);

        if(this->SHOW_IMG_SHOW)
        {
            cv::imshow("img_show_armorplate", this->img_show);
            int key = cv::waitKey(1);
        }
        

        RCLCPP_INFO_ONCE(this->get_logger(), "coreNode 正在发布...");
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
    bool SHOW_IMG_SHOW; // 是否显示 img_show 窗口
    
    bool SHOW_LOGGER_PREPARE; // 是否显示 prepare 中的日志
    std::string CHOSEN_COLOR; // 选择检测的颜色 red / blue
    std::string CAMERA_NAME; // 选择相机名称 mind_vision / galaxy ，注意改对应的 qos
    std::string ARMOR_TYPE; // 选择装甲板类型 normal / hero ，决定了配对的参数

    ///////// 工具类参数 /////////
    rclcpp::Time last_print; // 记录上一次打印 pnp 发布频率的时间戳，用于统计 pnp 发布频率

    int continuous_count = 0; // 记录连续有效的帧数，刚收到数据的前几帧不要（因为不稳定）
    int lost_count = 0; // 记录连续丢失的帧数，丢失一定次数后不给ekf了，并且重置


    // tf 类对象指针
    std::unique_ptr<TF> tf;

    // EKF 类对象指针
    std::unique_ptr<EKF> ekf_;

    // 参数量
    int MAX_LOST_COUNT = 10; // 最大连续丢失量，超过这个就不用 ekf 的预测了

    // 上一次查询的的时间戳
    rclcpp::Time last_lookup_time_;

    // ekf 是否已经稳定跟踪
    bool ekf_ready = false; 

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProcessNode>());
    rclcpp::shutdown();
    return 0;
}