/*
  安装依赖
  sudo apt install ros-humble-serial-driver
  以下仅供参考
*/


// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cstring>   // 用于 memcpy
#include <algorithm> // 用于 std::copy
#include <vector>
#include <deque>     // 用于接收缓冲区

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

#include "serial_driver_interfaces/msg/serial_driver.hpp"
#include "serial_driver_interfaces/msg/send_pnp_info.hpp"


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
        RCLCPP_INFO_ONCE(get_logger(), "SerialDriver 构造函数启动!");

        signal(SIGSEGV, signal_handler);
        signal(SIGABRT, signal_handler);
        signal(SIGFPE, signal_handler);
        RCLCPP_INFO_ONCE(get_logger(), "Step 1/10: 函数注册信号处理器 创建完成");

        getParams();
        RCLCPP_INFO_ONCE(get_logger(), "Step 2/10: 已经从 config 文件读取参数, device_name=%s", device_name_.c_str());

        // 串口初始化核心
        try 
        {
            serial_driver_->init_port(device_name_, *device_config_);
            RCLCPP_INFO_ONCE(get_logger(), "Step 3/10: 初始化串口 成功");

            // 如果串口没打开，尝试打开
            if (!serial_driver_->port()->is_open()) 
            {
                serial_driver_->port()->open();
                RCLCPP_INFO_ONCE(get_logger(), "Step 4/10: 未打开串口，现在串口已经打开成功");

                receive_thread_ = std::thread(&RMSerialDriver::receiveData, this); 
                RCLCPP_INFO_ONCE(get_logger(), "Step 5/10: 启动接收线程 成功");
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

        // 创建 PNP 消息订阅者
        pnp_sub_ = this->create_subscription<serial_driver_interfaces::msg::SendPNPInfo>("/send_pnp_info", 10, std::bind(&RMSerialDriver::PNPCallback, this, std::placeholders::_1));
        RCLCPP_INFO_ONCE(get_logger(), "Step 6/10: 成功创建 pnp 话题订阅者");

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


// 自定义接收数据处理函数，发送数据到话题 receive_data 下
void RMSerialDriver::processReceivedPacket(const ReceivePacket& packet) 
{
    // 1. 先打印接收到的数据
    RCLCPP_INFO(get_logger(), "++++++++++++ Received: euler_roll=%.2f euler_pitch=%.2f euler_yaw=%.2f", packet.euler_roll, packet.euler_pitch, packet.euler_yaw);

    // 2. 创建并填充新消息
    auto pub_msg = serial_driver_interfaces::msg::ReceiveData();
    pub_msg.header.stamp = this->now(); // 帧头获取当前时刻
    pub_msg.roll = packet.euler_roll;      // 翻滚角
    pub_msg.pitch = packet.euler_pitch;    // 俯仰角
    pub_msg.yaw = packet.euler_yaw;        // 航向角
    
}


// 接收数据线程函数，持续监听串口数据，解析数据包
void RMSerialDriver::receiveData()
{
    std::vector<uint8_t> header(1);
    std::vector<uint8_t> data;
    data.reserve(sizeof(ReceivePacket));

    while (rclcpp::ok()) 
    {
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

                    // 打印接收到的数据
                    RCLCPP_INFO(get_logger(), "++++++++++++ Received: euler_roll=%.2f euler_pitch=%.2f euler_yaw=%.2f", packet.euler_roll, packet.euler_pitch, packet.euler_yaw);

                    // 更新发布【世界坐标系】->【芯片坐标系】
                    this->tf->updateWorldToChip(packet.euler_roll, packet.euler_pitch, packet.euler_yaw);

                    // 已经更新
                    this->world_to_chip_updated_ = true; 
            
                } 
                else 
                {
                    RCLCPP_ERROR(get_logger(), "帧尾错误！期望 0xFE, 实际 %02X", packet.checksum);
                }
            } 

            else 
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "帧头错误！期望 0xFF, 实际 %02X", header[0]);
                continue;
            }
        } 

        catch (const std::exception & ex) 
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20, "串口 接收数据时发生错误: %s, 尝试重启串口...", ex.what());
            reopenPort();
        }
    }
}


// 确定两个坐标系是否都已经更新，尝试查询 TF 变换，得到最终数据，并发送串口
void RMSerialDriver::confirmIfCanSendData()
{

}  



// 最终发送给串口的函数（已经确认过）
void RMSerialDriver::ultimateSendData(float pitch, float yaw)
{
    // 再一次检查设备是否已经打开
    if (!serial_driver_->port()->is_open()) 
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10, "串口未打开，数据未发送");
        return;
    }


    try 
    {
        SendPacket packet;
        packet.header = 0xFF; // 填充帧头

        // 发绝对角
        packet.absolute_pitch = pitch; // 用滤波后的 pitch -> 填充 pitch
        packet.absolute_yaw = yaw; // 用滤波后的 yaw -> 填充 yaw
        
        packet.crc = 0xFE; //直接固定帧尾
        
        std::vector<uint8_t> data = toVector(packet);



        RCLCPP_INFO(get_logger(), "-------------READY--------------> 串口 sendData 准备发送数据, sizeof = %d", sizeof(SendPacket));
        
        static rclcpp::Time send_once_start = this->now(); // 获取当前时刻

        auto before_send = this->now(); //【计时开始】send 前

        // serial_driver_->port()->send(data); // 同步发送

        // 异步发送
        serial_driver_->port()->async_send(data);

        auto after_send = this->now(); //【计时结束】send 后

        RCLCPP_INFO(get_logger(), "-------------SUCCESSED--------------> 串口已经发送数据 absolute_pitch=%.2f absolute_yaw=%.2f", pitch, yaw);

        auto diff_send_interval = (after_send - send_once_start).seconds(); // 整次 send 堵塞的时间差
        auto diff_send_duration = (after_send - before_send).seconds(); // 调用 send 过程前后的时间差
        RCLCPP_INFO(get_logger(), "[内_执行异步发送] send_duration = %.5f s", diff_send_duration);
        RCLCPP_INFO(get_logger(), "[外_单次间隔] send_interval = %.5f s", diff_send_interval);
        
        send_once_start = after_send; //【计时开始】整次 send 

    } 
    
    // 异步发送失败
    catch (const std::exception & ex) 
    {
        RCLCPP_ERROR(get_logger(), "串口 发送数据时发生错误: %s, 尝试重启串口...", ex.what());
    
        // 必须：仅在设备可用时重试，避免无设备时无限重试（会直接导致串口崩溃）
        if (serial_driver_->port()->is_open()) reopenPort(); //【修改】启动 reopenPort
    }

    // RCLCPP_INFO(get_logger(), "-------------TEST--------------> 串口已经发送数据 yaw=%.2f pitch=%.2f", msg->yaw, msg->pitch);
}



// pnp 回调函数
void RMSerialDriver::PNPCallback(const serial_driver_interfaces::msg::SendPNPInfo msg)
{
    // 检查 pnp 的 t 矩阵数据是否和上次发送的一模一样
    if(msg.tvec[0] == last_tvec[0] && msg.tvec[1] == this->last_pnp_t_y && msg.tvec[2] == this->last_pnp_t_z)
    {
        ++this->pnp_same_t_count;
        RCLCPP_WARN(this->get_logger(), "【 警告！ 】pnp 收到的 t 矩阵和上次的完全相同，已经相同 %d 帧。跳过发布", this->pnp_same_t_count);
        // return;
    }

    this->pnp_same_t_count = 0; // 重置计数器

    // 更新 上一次的 pnp 的 t 矩阵数据 
    this->last_pnp_t_x = msg.tvec[0];
    this->last_pnp_t_y = msg.tvec[1];
    this->last_pnp_t_z = msg.tvec[2];


    // 更新【芯片坐标系】->【装甲板坐标系】的变换


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

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");
    RCLCPP_INFO(get_logger(), "flow_control string = '%s'", fc_string.c_str());

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
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
