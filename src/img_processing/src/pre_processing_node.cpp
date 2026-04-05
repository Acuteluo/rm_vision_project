/*
    接收来自 mindvision_publisher 相机图片的框架
*/

#include "prepare_algorithm.h"

class ProcessNode: public rclcpp::Node
{
public:
    ProcessNode(): Node("pre_processing_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "ProcessNode 节点创建成功! ");

        // 声明 QoS 参数
        this->declare_parameter("use_sensor_data_qos", false);
        bool use_sensor_data_qos = this->get_parameter("use_sensor_data_qos").as_bool();
        // rclcpp::QoS qos = rclcpp::QoS(10); // 先给个默认值
        // if (use_sensor_data_qos) 
        // {
        //     qos = rclcpp::SensorDataQoS();
        // } 
        // else 
        // {
        //     qos = rclcpp::SystemDefaultsQoS();
        // }       
        // 在构造函数中
        // rclcpp::QoS qos = rclcpp::SensorDataQoS();  // 直接使用传感器 QoS


        // 订阅原图
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", use_sensor_data_qos, std::bind(&ProcessNode::process_callback, this, std::placeholders::_1));
    }

private:

    void process_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image; // 需要 SharedPtr 作为参数
        
        prepare.preProcessing(img);
        this->strip = prepare.findAndJudgeLightStrip(); // 找灯带
		this->armorplate = prepare.pairStrip(); // 灯条配对 输出结果
        img_show = prepare.getImgShow(); // 获取画图的图（已经标上了灯条信息）

        for(int i = 0; i < armorplate.size(); i++)
        {
            armorplate[i].drawArmorPlate(this->img_show); // 画装甲板
        }
        

        cv::imshow("img", img);
        cv::imshow("img_show", this->img_show);



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
    Prepare prepare; // prepare 类对象，包含预处理函数、寻找灯条函数、配对函数等
    
    std::vector<Strip> strip; // 灯条类集合，接收从 prepare.findAndJudgeLightStrip() 返回的灯条集合
    std::vector<ArmorPlate> armorplate; // 装甲板类集合，接收从 prepare.pairStrip() 返回的装甲板集合

    cv::Mat img_show; // 当需要传入画画的图像的时候需要

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProcessNode>());
    rclcpp::shutdown();
    return 0;
}