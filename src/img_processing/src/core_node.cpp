/*
    接收来自 mindvision_publisher 相机图片的框架
*/

#include "prepare_algorithm.h"
#include "serial_driver_interfaces/msg/serial_driver.hpp" // 发布方 的 话题类型 为 [serial_driver] 类

class ProcessNode: public rclcpp::Node
{
public:
    ProcessNode(): Node("core_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "CoreNode 节点创建成功! ");

        // 声明 QoS 参数
        this->declare_parameter("use_sensor_data_qos", false);
        bool use_sensor_data_qos = this->get_parameter("use_sensor_data_qos").as_bool();

        // 订阅原图
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", use_sensor_data_qos, std::bind(&ProcessNode::process_callback, this, std::placeholders::_1));
    
        // 发布
        pub_ = this->create_publisher<serial_driver_interfaces::msg::SerialDriver>("serial_driver", 10);
    }

private:

    void process_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image; // 需要 SharedPtr 作为参数
        
        // 预处理
        prepare.preProcessing(img); // 图像预处理
        this->strip = prepare.findAndJudgeLightStrip(); // 找灯带 返回灯带集合
		this->armorplate = prepare.pairStrip(); // 灯条配对 返回装甲板集合


        cv::Mat img_show_prepare = prepare.getImgShow(); // 获取prepare画的图（已经标上了灯条信息）
        


        // 对于每一个装甲板，画出装甲板，解算 pnp，记住谁的置信度最大
        int moderation_max_index = -1; // 置信度最高的装甲板编号
        double moderation_max = 0.00; // 最高的置信度
        for(int i = 0; i < this->armorplate.size(); i++)
        {
            this->armorplate[i].setImgShow(img_show_prepare); // 传入 img_show_prepare
            this->armorplate[i].drawArmorPlate(); // 画装甲板
            this->armorplate[i].perspectiveNPoint(); // 解算 pnp
            this->armorplate[i].printPNPInfo(i); // 传入index，打印 pnp 信息
            img_show_prepare = this->armorplate[i].getImgShow(); // 获取装甲板画的图，传给下一个装甲板继续画，这样就能把所有装甲板的信息都画在 img_show 上了

            // 记录置信度最高的装甲板编号和置信度
            if(this->armorplate[i].moderation > moderation_max)
            {
                moderation_max = this->armorplate[i].moderation;
                moderation_max_index = i;
            }
        }
        
        cv::Mat img_show_armorplate = img_show_prepare; // 获取最终图像



        /////////////////// 发送串口消息 ////////////////////////

        auto serial_driver = serial_driver_interfaces::msg::SerialDriver();

        // 只要有装甲板，就发送第一个装甲板的信息，优先级最高
        if(this->armorplate.size() > 0)
        {
            serial_driver.yaw = this->armorplate[moderation_max_index].t_yaw; // 发送 置信度max 的装甲板的 yaw
            serial_driver.pitch = this->armorplate[moderation_max_index].t_pitch; // 发送 置信度max 装甲板的 pitch
        }
        else
        {
            serial_driver.yaw = 0.00;
            serial_driver.pitch = 0.00;
        }

        pub_->publish(serial_driver); // 发布消息 到 serial_driver 话题

        RCLCPP_INFO(this->get_logger(), " ------------> 串口消息 正在发布...");



        ///////////////////////////////////////////////////////

        //cv::imshow("img", img);
        cv::imshow("img_show_armorplate", img_show_armorplate);



        int key = cv::waitKey(1);

        if(key == 27)
        {
            RCLCPP_ERROR(this->get_logger(), "ProcessNode 节点已被手动退出! ");
            rclcpp::shutdown();
            return;
        }

        else if (key == 's' || key == 'S') 
        { 
            // 保存当前原图
            std::string filename = "saved_image_" + std::to_string(cv::getTickCount()) + ".jpg";
            cv::imwrite(filename, img);
            RCLCPP_INFO(this->get_logger(), "Image saved as %s", filename.c_str());

            // 保存当前数值图
            std::string filename_changed = "saved_changed_image_" + std::to_string(cv::getTickCount()) + ".jpg";
            cv::imwrite(filename_changed, prepare.getImgShow());
            RCLCPP_INFO(this->get_logger(), "Image saved as %s", filename_changed.c_str());
        }

        RCLCPP_INFO_ONCE(this->get_logger(), "ProcessNode 正在发布...");
    }



    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; // 订阅 mindvision_publisher 发过来的原图
    rclcpp::Publisher<serial_driver_interfaces::msg::SerialDriver>::SharedPtr pub_; // 发布信息


    Prepare prepare; // prepare 类对象，包含预处理函数、寻找灯条函数、配对函数等
    
    std::vector<Strip> strip; // 灯条类集合，接收从 prepare.findAndJudgeLightStrip() 返回的灯条集合
    std::vector<ArmorPlate> armorplate; // 装甲板类集合，接收从 prepare.pairStrip() 返回的装甲板集合

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProcessNode>());
    rclcpp::shutdown();
    return 0;
}