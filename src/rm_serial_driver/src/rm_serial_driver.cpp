/*
  安装依赖
  sudo apt install ros-humble-serial-driver
  以下仅供参考
*/


// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <vector>
#include <cstring>   // 用于 memcpy
#include <algorithm> // 用于 std::copy
#include <vector>
#include <deque>     // 用于接收缓冲区

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"


// 添加信号处理器，打印错误堆栈
#include <signal.h>
#include <execinfo.h>
#include <unistd.h>

void signal_handler(int sig) 
{
    void *array[20];
    size_t size = backtrace(array, 20);
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}


namespace rm_serial_driver
{

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options): 
    Node("serial_driver", options),
    owned_ctx_{new IoContext(2)},
    serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
    {

        // // 启用话题统计，并设置相关参数
        // rclcpp::SubscriptionOptions sub_options;
        
        // sub_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable; // 启用统计
        // sub_options.topic_stats_options.publish_period = std::chrono::seconds(5); // 发布周期 5 秒
        // sub_options.topic_stats_options.publish_topic = "/send_pnp_info"; // 指定统计话题

        RCLCPP_INFO_ONCE(get_logger(), "SerialDriver 构造函数启动!");

        signal(SIGSEGV, signal_handler);
        signal(SIGABRT, signal_handler);
        signal(SIGFPE, signal_handler);
        RCLCPP_INFO_ONCE(get_logger(), "Step 1/7: 函数注册信号处理器 创建完成");


        // ---------- 动态参数声明 ----------
        this->declare_parameter("serial.show_logger_receive", false);
        this->declare_parameter("serial.show_logger_try_send", false);

        // 读取初始值
        this->SHOW_LOGGER_RECEIVE = this->get_parameter("serial.show_logger_receive").as_bool();
        this->SHOW_LOGGER_TRY_AND_SEND = this->get_parameter("serial.show_logger_try_send").as_bool();

        // 注册参数变化回调
        param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&RMSerialDriver::onParameterChange, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Step 2/7: 串口节点动态参数已初始化: SHOW_LOGGER_RECEIVE=%d, SHOW_LOGGER_TRY_AND_SEND=%d", SHOW_LOGGER_RECEIVE, SHOW_LOGGER_TRY_AND_SEND);



        getParams();
        RCLCPP_INFO_ONCE(get_logger(), "Step 3/7: 已经从 config 文件读取参数, device_name=%s", device_name_.c_str());


        // 串口初始化核心
        try 
        {
            serial_driver_->init_port(device_name_, *device_config_);
            RCLCPP_INFO_ONCE(get_logger(), "Step 4/7: 初始化串口 成功");

            // 如果串口没打开，尝试打开
            if (!serial_driver_->port()->is_open()) 
            {
                serial_driver_->port()->open();
                RCLCPP_INFO_ONCE(get_logger(), "Step 5/7: 未打开串口，现在串口已经打开成功");

                receive_thread_ = std::thread(&RMSerialDriver::receiveData, this); 
                RCLCPP_INFO_ONCE(get_logger(), "Step 6/7: 启动接收线程 成功");
            }
        } 
        catch (const std::exception & ex) 
        {
            RCLCPP_ERROR(get_logger(), "创建串口时 发生错误: %s - %s", device_name_.c_str(), ex.what());

            RCLCPP_ERROR(get_logger(), "【串口初始化异常】串口设备不存在（/dev/ttyACM0 未连接）或无法打开。。。。。。。。。。");
            // RCLCPP_ERROR(get_logger(), "即将抛出 ex, 串口节点将退出。。。。。。。。。。。");

            // throw ex; // 如果要仅仅测试sendData函数，请注释掉 throw ex这一行，允许程序继续运行，并在sendData函数中仅仅打开test那一行
                        // 否则节点会直接退出
            
        }

        // 创建 core 消息订阅者，话题 /serial_driver
        data_sub_ = this->create_subscription<serial_driver_interfaces::msg::SerialDriver>("/serial_driver", 10, std::bind(&RMSerialDriver::CheckData, this, std::placeholders::_1));
        RCLCPP_INFO_ONCE(get_logger(), "Step 7/7: 成功创建 /serial_driver 话题订阅者");

        // 初始化 last_receive_time，上一次接收电控数据的时间，用于统计电控发来消息的频率
        this->last_receive_time = this->now(); 

        this->chip_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);


        RCLCPP_INFO_ONCE(get_logger(), ">>>>>>>>>>>>>>> 串口构造函数已经初始化完成。");

    }


RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}


rcl_interfaces::msg::SetParametersResult RMSerialDriver::onParameterChange(
    const std::vector<rclcpp::Parameter>& params)
{
    std::lock_guard<std::mutex> lock(state_mutex_);  // 线程安全

    for (const auto& p : params) 
    {
        RCLCPP_INFO(this->get_logger(), "RMSerialDriver 节点参数更新! ");
        const std::string& name = p.get_name();
        if (name == "serial.show_logger_receive") 
        {
            this->SHOW_LOGGER_RECEIVE = p.as_bool();
            RCLCPP_INFO(get_logger(), "SHOW_LOGGER_RECEIVE 更新为 %d", SHOW_LOGGER_RECEIVE);
        } 
        else if (name == "serial.show_logger_try_send")
        {
            this->SHOW_LOGGER_TRY_AND_SEND = p.as_bool();
            RCLCPP_INFO(get_logger(), "SHOW_LOGGER_TRY_AND_SEND 更新为 %d", SHOW_LOGGER_TRY_AND_SEND);
        }
    }

    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
}



// 01【世界坐标系】->【芯片坐标系】当前芯片位姿的坐标系 -> 动态，用 R 矩阵
void RMSerialDriver::updateWorldToChip(double roll, double pitch, double yaw)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now(); // 帧头 -> 直接获取当前时刻
    tf.header.frame_id = "world_frame"; // 父坐标系 -> 世界坐标系
    tf.child_frame_id = "chip_frame"; // 子坐标系 -> 芯片坐标系

    // 先忽略 t
    tf.transform.translation.x = 0.00;
    tf.transform.translation.y = 0.00;
    tf.transform.translation.z = 0.00;
    

    // 欧拉角（度）->（弧度）再转四元数

    // 先把 度 -> 弧度，注意 setRPY() 需要弧度
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;

    // 然后转四元数
    tf2::Quaternion q;
    q.setRPY(roll_rad, pitch_rad, yaw_rad);  // 顺序：roll, pitch, yaw （XYZ） 

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    this->chip_broadcaster_->sendTransform(tf);
}



// 接收数据 线程函数，持续监听串口数据，解析数据包
void RMSerialDriver::receiveData()
{
    std::vector<uint8_t> header(1);
    std::vector<uint8_t> data;
    data.reserve(sizeof(ReceivePacket));

    rclcpp::Time t1, t2;

    // 通过 while 实现持续监听串口数据，直到节点关闭
    while (rclcpp::ok()) 
    {
        // ... 步骤1 记录初始时刻
        rclcpp::Time start = this->now();
        try 
        {
            // 读取帧头
            serial_driver_->port()->receive(header);

            // 帧头正确
            if (header[0] == 0xFF) 
            {
                // 读取剩余 13 字节 -> roll picth yaw 0xFE
                data.resize(sizeof(ReceivePacket) - 1);
                serial_driver_->port()->receive(data);

                // 将帧头插入开头，组装成完整数据包
                data.insert(data.begin(), header[0]);
                ReceivePacket packet = fromVector(data);

                // 帧尾正确
                if (packet.checksum == 0xFE) 
                {
                    // 接收数据部分

                    // ... 步骤2 确认数据可用的时间戳
                    t1 = this->now();
                    
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    this->receive_time = this->now(); // 最新收到电控数据的时间戳

                    // 手动统计帧率
                    // 在构造函数中已经初始化了 this->receive_time，所以这里不需要再赋值
                    static int pub_count = 0;
                    pub_count++;
                    if ((this->receive_time - this->last_receive_time).seconds() >= 1.0) 
                    {
                        double fps = pub_count / (this->receive_time - this->last_receive_time).seconds();
                        RCLCPP_INFO(this->get_logger(), "电控消息 发布频率: %.2f Hz", fps);
                        pub_count = 0;
                        this->last_receive_time = this->receive_time;
                    }


                    // 打印接收到的数据
                    RCLCPP_INFO_EXPRESSION(get_logger(), this->SHOW_LOGGER_RECEIVE, "收到电控数据: euler_roll=%.7f euler_pitch=%.7f euler_yaw=%.7f", packet.euler_roll, packet.euler_pitch, packet.euler_yaw);

                    // 更新发布【世界坐标系】->【芯片坐标系】
                    updateWorldToChip(packet.euler_roll, packet.euler_pitch, packet.euler_yaw);


                    // ... 步骤3 更新发布【世界坐标系】->【芯片坐标系】完成时间戳
                    t2 = this->now();
                    
                    if(this->SHOW_LOGGER_RECEIVE)
                    {
                        double duration1 = (t1 - start).seconds() * 1000.0; // 转换为毫秒
                        double duration2 = (t2 - t1).seconds() * 1000.0; // 转换为毫秒
                        double total_duration = (t2 - start).seconds() * 1000.0; // 转换为毫秒
                        RCLCPP_INFO(this->get_logger(), "处理电控数据耗时: 确认数据有效 = %.4f ms, 发布坐标变换 = %.4f ms, 总耗时 = %.4f ms", duration1, duration2, total_duration);
                    }

                } 
                else 
                {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 100, "帧尾错误！期望 0xFE, 实际 %02X", packet.checksum);
                }
            } 

            else 
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "帧头错误！期望 0xFF, 实际 %02X", header[0]);
                continue;
            }
        } 

        catch (const std::exception & ex) 
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 100, "串口 接收数据时发生错误: %s, 尝试重启串口...", ex.what());
            reopenPort();
        }
    }
    

}


// 确定数据是有效的

void RMSerialDriver::CheckData(const serial_driver_interfaces::msg::SerialDriver msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_); // 加锁，避免线程打架    

    // 单位是 ms
    double dt_receive = (this->now() - this->receive_time).seconds() * 1000; // 距离上次收到电控回传消息的时间 
    
    // 如果超过100ms，电控回传还没有新的数据，那么认为数据无效，毕竟 tf 变换太滞后了
    if(dt_receive > 100.00)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "由于 电控回传数据 距离上一次 > 100ms, 数据无效不发送");
        return;
    }


    // 与上一次发送的数据进行对比，假如完全相同，则跳过发送
    if(msg.pitch == this->last_send_pitch && msg.yaw == this->last_send_yaw)
    {
        ++this->send_data_same_count;
    }
    else
    {
        if(this->send_data_same_count >= 1)
        {
            RCLCPP_WARN(this->get_logger(), "之前准备发送给串口的数据和上一次完全相同，累计相同 %d 帧。该帧不同", this->send_data_same_count);
        }
        this->send_data_same_count = 0; 
    }


    // // 与上一次发送的数据进行对比，假如相差特别大，则认为是异常数据，也跳过发送
    // double ANGLE_THRESHOLD = 5.00; // 角度阈值，单位为度

    // if(id != 1 && (std::fabs(pitch - this->last_pitch) > ANGLE_THRESHOLD || std::fabs(yaw - this->last_yaw) > ANGLE_THRESHOLD))
    // {
    //     RCLCPP_WARN(this->get_logger(), "【偏差数据】id != 1 的准备发送给串口的数据 与上一次偏差超过了 %.2f度，已舍弃", ANGLE_THRESHOLD);
    //     return;
    // }

    
    // 更新上一次发送的最终数据
    this->last_send_pitch = msg.pitch;
    this->last_send_yaw = msg.yaw;
    

    // 发送数据给串口
    SendData(msg.pitch, msg.yaw);

}  



// 最终发送给串口的函数（已经确认过的数据）
void RMSerialDriver::SendData(float pitch, float yaw)
{
    // 再一次检查设备是否已经打开
    if (!serial_driver_->port()->is_open()) 
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "串口未打开，数据未发送");
        return;
    }

    try 
    {
        ////////// ------- 填充数据起 ------- ////////// 

        SendPacket packet;
        packet.header = 0xFF; // 填充帧头

        // 发绝对角
        packet.absolute_pitch = pitch; // 用滤波后的 pitch -> 填充 pitch
        packet.absolute_yaw = yaw; // 用滤波后的 yaw -> 填充 yaw
        
        packet.crc = 0xFE; // 直接固定帧尾
        
        ////////// ------- 填充数据止 ------- ////////// 

        std::vector<uint8_t> data = toVector(packet);
        

        auto before_send = this->now(); //【计时开始】send 前

        // serial_driver_->port()->send(data); // 同步发送

        // 异步发送
        serial_driver_->port()->async_send(data);

        // 看日志决定输出
        if(this->SHOW_LOGGER_TRY_AND_SEND)
        {
            auto after_send = this->now(); //【计时结束】send 后

            RCLCPP_INFO(get_logger(), "-------------SUCCESSED--------------> 串口已经发送数据 absolute_pitch=%.2f absolute_yaw=%.2f", pitch, yaw);

            auto diff_send_duration = (after_send - before_send).seconds(); // 调用 send 过程前后的时间差
            RCLCPP_INFO(get_logger(), "[内_执行异步发送] send_duration = %.5f s", diff_send_duration);
        }
         

        // 添加一个计时器，用来统计实际发送的fps
        static int send_count = 0;
        static rclcpp::Time last_time = this->now();
        send_count++;
        rclcpp::Time now = this->now();
        double elapsed = (now - last_time).seconds();
        if (elapsed >= 1.000) 
        {
            double fps = send_count / elapsed;
            RCLCPP_INFO(this->get_logger(), "串口实际发送帧率:--------------------->【 %.2f Hz】", fps);
            send_count = 0;
            last_time = now;
        }

    } 
    
    // 异步发送失败
    catch (const std::exception & ex) 
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 200, "串口 发送数据时发生错误: %s, 尝试重启串口...", ex.what());
    
        // 必须：仅在设备可用时重试，避免无设备时无限重试（会直接导致串口崩溃）
        if (serial_driver_->port()->is_open()) reopenPort(); //【修改】启动 reopenPort
    }

    // RCLCPP_INFO(get_logger(), "-------------TEST--------------> 串口已经发送数据 yaw=%.2f pitch=%.2f", msg->yaw, msg->pitch);
}




void RMSerialDriver::getParams()
{
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate{};
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    try 
    {
        device_name_ = declare_parameter<std::string>("device_name", "");
    } 
    catch (rclcpp::ParameterTypeException & ex) 
    {
        RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
        throw ex;
    }

    try 
    {
        baud_rate = declare_parameter<int>("baud_rate", 0);
    } 
    catch (rclcpp::ParameterTypeException & ex) 
    {
        RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
        throw ex;
    }

    try 
    {
        const auto fc_string = declare_parameter<std::string>("flow_control", "");
        RCLCPP_INFO(get_logger(), "flow_control string = '%s'", fc_string.c_str());

        if (fc_string == "none") 
        {
            fc = FlowControl::NONE;
        } 
        else if (fc_string == "hardware") 
        {
            fc = FlowControl::HARDWARE;
        } 
        else if (fc_string == "software") 
        {
            fc = FlowControl::SOFTWARE;
        } 
        else 
        {
            throw std::invalid_argument
            {
                "The flow_control parameter must be one of: none, software, or hardware."
            };
        }
    } 
    catch (rclcpp::ParameterTypeException & ex) 
    {
        RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
        throw ex;
    }

    try 
    {
        const auto pt_string = declare_parameter<std::string>("parity", "");

        if (pt_string == "none") 
        {
            pt = Parity::NONE;
        } 
        else if (pt_string == "odd") 
        {
            pt = Parity::ODD;
        } 
        else if (pt_string == "even") 
        {
            pt = Parity::EVEN;
        } 
        else 
        {
            throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
        }
    } 
    catch (rclcpp::ParameterTypeException & ex) 
    {
        RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
        throw ex;
    }

    try 
    {
        const auto sb_string = declare_parameter<std::string>("stop_bits", "");

        if (sb_string == "1" || sb_string == "1.0") 
        {
            sb = StopBits::ONE;
        } 
        else if (sb_string == "1.5") 
        {
            sb = StopBits::ONE_POINT_FIVE;
        } 
        else if (sb_string == "2" || sb_string == "2.0") 
        {
            sb = StopBits::TWO;
        } 
        else 
        {
            throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
        }
    } 
    catch (rclcpp::ParameterTypeException & ex) 
    {
        RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
        throw ex;
    }

    device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");
    try 
    {
        if (serial_driver_->port()->is_open()) 
        {
            serial_driver_->port()->close();
        }
        serial_driver_->port()->open();
        RCLCPP_INFO(get_logger(), "Successfully reopened port");
    } 
    catch (const std::exception & ex) 
    {
        RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
        if (rclcpp::ok()) 
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            reopenPort();
        }
    }
}


}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
