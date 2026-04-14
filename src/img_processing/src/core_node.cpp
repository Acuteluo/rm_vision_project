/*
    接收来自 mindvision_publisher 相机图片的框架
*/

#include "prepare_algorithm.h"

class ProcessNode: public rclcpp::Node
{
public:
    ProcessNode(): Node("core_node_cpp")
    {
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
        pnp_pub_ = this->create_publisher<serial_driver_interfaces::msg::SendPNPInfo>("/send_pnp_info", 10);

        RCLCPP_INFO_ONCE(this->get_logger(), "CoreNode 节点创建成功! ");

        // 初始化pnp帧率的计时器
        this->last_print = this->now();
    }


private:

    void process_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rclcpp::Time start = this->now();
    
        // 1. 接收图像
        this->img = cv_bridge::toCvCopy(msg, "bgr8")->image; // 需要 SharedPtr 作为参数
        this->img_show = this->img.clone(); // 复制一份用来显示信息

        // ... 步骤1 接收图像完成时间戳
        rclcpp::Time t1 = this->now();

        // 2. 预处理
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
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 500, "未识别到装甲板! corenode 未 进行 pnp 解算");
        }

        // ... 步骤3 pnp结算完成时间戳
        rclcpp::Time t3 = this->now();

        
        // 4. 发送 pnp 消息，注意是转换后的右手系
        if(this->armorplate.size() > 0 && this->armorplate[0].is_success) // 只有当 pnp结算有结果 才发布消息
        {
            auto msg = serial_driver_interfaces::msg::SendPNPInfo();

            msg.matrix_r = {
                this->armorplate[0].R(0, 0), 
                this->armorplate[0].R(0, 1), 
                this->armorplate[0].R(0, 2), 
                this->armorplate[0].R(1, 0), 
                this->armorplate[0].R(1, 1), 
                this->armorplate[0].R(1, 2), 
                this->armorplate[0].R(2, 0), 
                this->armorplate[0].R(2, 1), 
                this->armorplate[0].R(2, 2), 
            };

            msg.tvec = {this->armorplate[0].t_vec(0), this->armorplate[0].t_vec(1), this->armorplate[0].t_vec(2)};

            // 添加：填充帧头的时间戳信息，以便得到传输消息时间，以及后续在 serial_driver 节点中计算消息的延迟
            msg.header.stamp = this->now();


            pnp_pub_->publish(msg); // 发布消息 到 /send_pnp_info 话题
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "[1000ms提示一次] pnp 消息已发布到 /send_pnp_info 话题 下");
        
            // 手动统计帧率
            // 在构造函数中已经初始化了 last_print，所以这里不需要再赋值
            static int pub_count = 0;
            pub_count++;
            auto now = this->now();
            if ((now - this->last_print).seconds() >= 1.0) 
            {
                double fps = pub_count / (now - this->last_print).seconds();
                RCLCPP_INFO(this->get_logger(), "PNP 发布频率: %.2f Hz", fps);
                pub_count = 0;
                this->last_print = now;
            }
        
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 500, "[500ms提示一次] 未识别到装甲板! corenode 未 发布 PNP 消息");
        }
         
        
        // 5. 计算每个步骤的耗时，并打印
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
        

        // if(key == 27)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "【 EXIT 】coreNode 节点已被手动退出! ");
        //     rclcpp::shutdown();
        //     return;
        // }

        // else if (key == 's' || key == 'S') 
        // { 
        //     // 保存当前原图
        //     std::string filename = "saved_image_" + std::to_string(cv::getTickCount()) + ".jpg";
        //     cv::imwrite(filename, img);
        //     RCLCPP_INFO(this->get_logger(), "Image saved as %s", filename.c_str());

        //     // 保存当前数值图
        //     std::string filename_changed = "saved_changed_image_" + std::to_string(cv::getTickCount()) + ".jpg";
        //     cv::imwrite(filename_changed, this->img_show);
        //     RCLCPP_INFO(this->get_logger(), "Image saved as %s", filename_changed.c_str());
        // }

        RCLCPP_INFO_ONCE(this->get_logger(), "coreNode 正在发布...");
    }



    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; // 订阅 mindvision_publisher 发过来的原图
    rclcpp::Publisher<serial_driver_interfaces::msg::SendPNPInfo>::SharedPtr pnp_pub_; // 发布 pnp 信息


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

    rclcpp::Time last_print;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProcessNode>());
    rclcpp::shutdown();
    return 0;
}