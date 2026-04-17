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

        getParams();
        RCLCPP_INFO_ONCE(get_logger(), "Step 2/7: 已经从 config 文件读取参数, device_name=%s", device_name_.c_str());

        this->tf = std::make_unique<TF>(this); // 传 this 指针给 TF 类来构造，让它能创建 ROS2 相关对象，同时发布静态变换


        // 串口初始化核心
        try 
        {
            serial_driver_->init_port(device_name_, *device_config_);
            RCLCPP_INFO_ONCE(get_logger(), "Step 3/7: 初始化串口 成功");

            // 如果串口没打开，尝试打开
            if (!serial_driver_->port()->is_open()) 
            {
                serial_driver_->port()->open();
                RCLCPP_INFO_ONCE(get_logger(), "Step 4/7: 未打开串口，现在串口已经打开成功");

                receive_thread_ = std::thread(&RMSerialDriver::receiveData, this); 
                RCLCPP_INFO_ONCE(get_logger(), "Step 5/7: 启动接收线程 成功");
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
        RCLCPP_INFO_ONCE(get_logger(), "Step 6/7: 成功创建 pnp 话题订阅者");

        this->receive_time = this->now();
        this->pnp_time = this->now();

        this->last_print = this->now(); // 初始化 last_print，用于统计电控发来消息的频率


        // ------------------ 进行一个配置文件的读取 -----------------

        std::ifstream file("config.txt");  // 打开配置文件，注意是在工作空间下
        if (!file.is_open()) 
        {
            RCLCPP_ERROR(this->get_logger(), "【 EXIT 】无法打开 config.txt 配置文件。。。。即将退出 tf\n");
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
                if(line_count == 9)
                {
                    if(each_line == "false" || each_line == "False" || each_line == "FALSE") 
                    {
                        this->SHOW_LOGGER_PNP = false;
                    }
                    else this->SHOW_LOGGER_PNP = true;
                    RCLCPP_INFO(this->get_logger(), "【 设置参数 】SHOW_LOGGER_PNP = %s", each_line.c_str());
                }

                else if(line_count == 10)
                {
                    if(each_line == "false" || each_line == "False" || each_line == "FALSE") 
                    {
                        this->SHOW_LOGGER_RECEIVE = false;
                    }
                    else this->SHOW_LOGGER_RECEIVE = true;
                    RCLCPP_INFO(this->get_logger(), "【 设置参数 】SHOW_LOGGER_RECEIVE = %s", each_line.c_str());
                }

                else if(line_count == 11)
                {
                    if(each_line == "false" || each_line == "False" || each_line == "FALSE") 
                    {
                        this->SHOW_LOGGER_TRY_AND_SEND = false;
                    }
                    else this->SHOW_LOGGER_TRY_AND_SEND = true;
                    RCLCPP_INFO(this->get_logger(), "【 设置参数 】SHOW_LOGGER_TRY_AND_SEND = %s", each_line.c_str());
                }

            }
        }

        if(line_count < 11)
        {
            RCLCPP_ERROR(this->get_logger(), "配置文件的有效行数不足 11 行, 检查配置文件。即将退出 core 节点\n");
            exit(-1);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Step 7/7: 串口 ALL SET! 共设置了 %d 个有效参数", line_count);
        }

        file.close();

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
                    this->receive_time = this->now(); // 最新收到电控数据的时间戳

                    // 手动统计帧率
                    // 在构造函数中已经初始化了 last_print，所以这里不需要再赋值
                    static int pub_count = 0;
                    pub_count++;
                    auto now = this->now();
                    if ((now - this->last_print).seconds() >= 1.0) 
                    {
                        double fps = pub_count / (now - this->last_print).seconds();
                        RCLCPP_INFO(this->get_logger(), "电控消息 发布频率: %.2f Hz", fps);
                        pub_count = 0;
                        this->last_print = now;
                    }


                    // 打印接收到的数据
                    RCLCPP_INFO_EXPRESSION(get_logger(), this->SHOW_LOGGER_RECEIVE, "收到电控数据: euler_roll=%.7f euler_pitch=%.7f euler_yaw=%.7f", packet.euler_roll, packet.euler_pitch, packet.euler_yaw);

                    // 更新发布【世界坐标系】->【芯片坐标系】
                    this->tf->updateWorldToChip(packet.euler_roll, packet.euler_pitch, packet.euler_yaw);

                    // 已经收到过了（丢失后再次初始化了），可以查询
                    this->world_to_chip_init_ = true;

                    // 更新新的电控传来的数据
                    this->euler_pitch = packet.euler_pitch;
                    this->euler_yaw = packet.euler_yaw;

                    // ... 步骤3 更新发布【世界坐标系】->【芯片坐标系】完成时间戳
                    t2 = this->now();
                    
                    if(this->SHOW_LOGGER_RECEIVE)
                    {
                        double duration1 = (t1 - start).seconds() * 1000.0; // 转换为毫秒
                        double duration2 = (t2 - t1).seconds() * 1000.0; // 转换为毫秒
                        double total_duration = (t2 - start).seconds() * 1000.0; // 转换为毫秒
                        RCLCPP_INFO(this->get_logger(), "处理电控数据耗时: 确认数据有效 = %.4f ms, 发布坐标变换 = %.4f ms, 总耗时 = %.4f ms", duration1, duration2, total_duration);
                    }


                    // 串口频率高，可以多更新一点，但是超过50ms，pnp还没有新的数据，就别更新了吧
                    // 尝试查询 TF，得到最终数据，并发送串口（在这个函数里有写，如果pnp信息新发布了才发）
                    confirmIfCanSendData();

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


// 确定两个坐标系是否都已经更新，尝试查询 TF 变换，得到最终数据，并发送串口

void RMSerialDriver::confirmIfCanSendData()
{
    std::lock_guard<std::mutex> lock(state_mutex_); // 加锁，避免线程打架

    // 如果超过50ms，pnp还没有新的数据，认为暂时丢失。就别更新了吧，这样可以避免丢失后还在发数据
    double dt_pnp = (this->now() - this->pnp_time).seconds() * 1000; // 单位是ms
    if(dt_pnp > 50.00)
    {
        this->chip_to_armorplate_init_ = false;
    }

    double dt_receive = (this->now() - this->receive_time).seconds() * 1000; // 单位是ms
    if(dt_receive > 50.00)
    {
        this->world_to_chip_init_ = false;
    }

    // 只有数据都更新了，才能查询到最新变换啊
    if(this->world_to_chip_init_ == false || this->chip_to_armorplate_init_ == false)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 50, "等待 两个坐标系变换 至少还有一个未更新完或未收到 退出尝试函数");
        return;
    }


    // 可以尝试查询 TF 变换，得到最终数据，并发送串口

    // ... 步骤6 记录开始获取数据的时间戳
    rclcpp::Time start = this->now();

    // 通过引用回传最终的 pitch 和 yaw，加上判断确保值有效
    float pitch = -9999.99;
    float yaw = -9999.99; 
    bool flag = this->tf->getTransform(pitch, yaw); 

    if(!flag || pitch == -9999.99 || yaw == -9999.99)
    {
        RCLCPP_ERROR(get_logger(), "获取最终数据失败或无效，无法发送串口");
        return;
    }

    // ... 步骤7 记录获取到数据的时间戳
    rclcpp::Time t7 = this->now();


    // 与上一次发送的数据进行对比，假如完全相同，则跳过发送
    if(pitch == this->last_pitch && yaw == this->last_yaw)
    {
        ++this->data_same_count;
        // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "准备发送给串口的数据和上一次完全相同, 已经相同 %d 帧。跳过发送", this->data_same_count);
        // return;
    }
    else
    {
        if(this->data_same_count >= 1) RCLCPP_WARN(this->get_logger(), "之前准备发送给串口的数据和上一次完全相同，累计相同 %d 帧。该帧不同", this->data_same_count);
        this->data_same_count = 0; 
    }


    // // 与上一次发送的数据进行对比，假如相差特别大，则认为是异常数据，也跳过发送
    // double ANGLE_THRESHOLD = 5.00; // 角度阈值，单位为度

    // if(id != 1 && (std::fabs(pitch - this->last_pitch) > ANGLE_THRESHOLD || std::fabs(yaw - this->last_yaw) > ANGLE_THRESHOLD))
    // {
    //     RCLCPP_WARN(this->get_logger(), "【偏差数据】id != 1 的准备发送给串口的数据 与上一次偏差超过了 %.2f度，已舍弃", ANGLE_THRESHOLD);
    //     return;
    // }

    
    // 更新上一次发送的最终数据
    this->last_pitch = pitch;
    this->last_yaw = yaw;
    

    // 发送数据给串口
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "OK! 准备发送数据给串口");
    ultimateSendData(pitch, yaw);

    // ... 步骤8 记录发完数据的时间戳
    rclcpp::Time t8 = this->now();

    if(this->SHOW_LOGGER_TRY_AND_SEND)
    {
        double duration7 = (t7 - start).seconds() * 1000.0; // 转换为毫秒
        double duration8 = (t8 - t7).seconds() * 1000.0; // 转换为毫秒
        double total_duration = (t8 - start).seconds() * 1000.0; // 转换为毫秒
        RCLCPP_INFO(this->get_logger(), "尝试得到变换和发布耗时: 得到变换 = %.4f ms, 发送给串口 = %.4f ms, 总耗时 = %.4f ms", duration7, duration8, total_duration);
    }

}  



// 最终发送给串口的函数（已经确认过的数据）
void RMSerialDriver::ultimateSendData(float pitch, float yaw)
{
    // 再一次检查设备是否已经打开
    if (!serial_driver_->port()->is_open()) 
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "串口未打开，数据未发送");
        return;
    }

    try 
    {
        SendPacket packet;
        packet.header = 0xFF; // 填充帧头

        // 发绝对角
        packet.absolute_pitch = pitch; // 用滤波后的 pitch -> 填充 pitch
        packet.absolute_yaw = yaw; // 用滤波后的 yaw -> 填充 yaw
        
        packet.crc = 0xFE; // 直接固定帧尾
        
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



// pnp 回调函数
void RMSerialDriver::PNPCallback(const serial_driver_interfaces::msg::SendPNPInfo msg)
{
    // ... 步骤4 接收到pnp数据完成时间戳
    rclcpp::Time t4 = this->now();
    this->pnp_time = this->now(); // 最新收到 pnp数据 的时间戳
    

    // 检查 pnp 的 t 矩阵数据是否和上次收到的一模一样
    if(msg.tvec[0] == this->last_tvec[0] && msg.tvec[1] == this->last_tvec[1] && msg.tvec[2] == this->last_tvec[2])
    {
        ++this->pnp_same_t_count;
        // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 200, "pnp 收到的 t 矩阵和上次的完全相同，已经相同 %d 帧。跳过此次发布", this->pnp_same_t_count);
    }
    else if(this->pnp_same_t_count > 0)
    {
        if(this->data_same_count >= 1) RCLCPP_WARN(this->get_logger(), "之前 pnp 收到的 t 矩阵和前几次的完全相同，累计相同 %d 帧。该帧不同", this->pnp_same_t_count);
        this->pnp_same_t_count = 0; 
    }
    

    // 更新 上一次的 pnp 的 t 矩阵数据 
    this->last_tvec[0] = msg.tvec[0];
    this->last_tvec[1] = msg.tvec[1];
    this->last_tvec[2] = msg.tvec[2];


    // 更新【相机坐标系】->【装甲板坐标系】的变换
    this->tf->updateCameraToArmorplate(msg);

    // 已经丢失后重新更新过了，可以查询
    this->chip_to_armorplate_init_ = true;
    
    
    // ... 步骤5 更新【相机坐标系】->【装甲板坐标系】的变换 完成时间戳
    rclcpp::Time t5 = this->now();

    if(this->SHOW_LOGGER_PNP)
    {
        // 计算每个步骤的耗时，并打印
        double duration4 = (t4 - msg.header.stamp).seconds() * 1000.0; // 转换为毫秒
        double duration5 = (t5 - t4).seconds() * 1000.0; // 转换为毫秒
        double total_duration = (t5 - msg.header.stamp).seconds() * 1000.0; // 转换为毫秒
        RCLCPP_INFO(this->get_logger(), "pnp处理耗时: 话题通信传输pnp数据 = %.4f ms, 更新变换 = %.4f ms, 总耗时 = %.4f ms。现在尝试查询并发布", duration4, duration5, total_duration);
    }

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
