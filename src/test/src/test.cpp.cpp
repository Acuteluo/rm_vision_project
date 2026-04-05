#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp> // 订阅方 的 话题类型
#include <cv_bridge/cv_bridge.h> // 用于在 ROS2 的 sensor_msgs::msg::Image 和 OpenCV 的 cv::Mat 之间进行转换
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/objdetect.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<iostream>
#include<string>
#include<cmath>
#include<ctime>
#include<deque>
#include<random>
#include<Eigen/Dense>
#include<Eigen/QR>
#include<Eigen/Core>
#include<Eigen/LU>


class MyNode: public rclcpp::Node
{
public:
    MyNode(): Node("mynode_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "节点创建成功! ");

        std::vector<cv::Mat> channels_bgr;
        std::vector<cv::Mat> channels_hsv;
        cv::Mat B, G, R; // bgr三通道值
        cv::Mat H, S, V; // hsv三通道值
        cv::Mat red_differ, blue_differ, mask; // 红色和蓝色灯条 BGR最高最低相差值掩码（需要吗？）
        cv::Mat img_red, img_blue;
        cv::Mat high_red, high_blue;
        cv::Mat high_value;
        cv::Mat img_low_green; // 低绿值掩码
        cv::Mat img_hsv; // hsv

        cv::Mat img = cv::imread("/home/cly/project/saved_image_8039951240884.jpg"); // 读图

        // // 1. 亮度调整 同时调整对比度和亮度，但目前不是必须
        // cv::Mat bright;
        // img.convertTo(bright, -1, 1.2, 30);

        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

        // 2. 分离bgr三通道的值
        split(img, channels_bgr);

        B = channels_bgr[0];
        G = channels_bgr[1];
        R = channels_bgr[2];

        // 4. 高阈值二值化
        cv::threshold(R, high_red, 220, 255, cv::THRESH_BINARY);
        cv::threshold(B, high_blue, 220, 255, cv::THRESH_BINARY);

        // // 5. 垂直方向模糊 重新变为二值化
        /*
            但实际效果需要验证

            建议你用实际图像测试一下，观察二值化后的灯条掩码是否有以下情况：

            断裂：如果灯条中心仍有空洞，导致掩码分成上下两段，则需要垂直模糊来连接。
            小缺口：边缘有小缺口，但整体连通，可以不处理。
            横向散光：如果仍有轻微横向扩散，垂直模糊也能帮助抑制（因为它只在垂直方向平滑）。
        */
        // cv::blur(high_red, high_red, cv::Size(1, 3));
        // cv::blur(high_blue, high_blue, cv::Size(1, 3));


        // cv::threshold(high_red, high_red, 1, 255, cv::THRESH_BINARY);
        // cv::threshold(high_blue, high_blue, 1, 255, cv::THRESH_BINARY);

        cv::namedWindow("img_show", cv::WINDOW_NORMAL);
        cv::namedWindow("B", cv::WINDOW_NORMAL);
        cv::namedWindow("r", cv::WINDOW_NORMAL);

        cv::imshow("img_show", img);
        cv::imshow("B", high_blue);
        cv::imshow("r", high_red);
        
        findAndJudgeLightStrip(img, img_hsv, high_blue); // 找灯带


        /*
            代码参考
            // const int IMAGE_BRIGHT = 30;       // 全局亮度增益（过暗就加大）
            // const int THRESHOLD_VALUE = 200;   // 二值化阈值（噪点多就加大）
            // const int KERNEL_SIZE = 2;         // 核大小（实际未用于膨胀）

            // cv::Mat dst_BR, dst;
            // std::vector<cv::Mat> channels;

            // // 步骤1：全局亮度调整
            // {
            //     cv::Mat BrightnessLut(1, 256, CV_8UC1); 
            //     for (int i = 0; i < 256; i++) {
            //         BrightnessLut.at<uchar>(i) = cv::saturate_cast<uchar>(i + IMAGE_BRIGHT);
            //     }
            //     cv::LUT(img, BrightnessLut, dst_BR);
            // }

            // // 步骤2：颜色通道差分（此处是直接取蓝色通道）
            // cv::split(dst_BR, channels);

            // // 步骤3：二值化（对蓝色通道）
            // cv::threshold(channels[0], dst, THRESHOLD_VALUE, 255, cv::THRESH_BINARY);

            // // 步骤4：垂直方向模糊（仅垂直方向）
            // cv::blur(dst, dst, cv::Size(1, 3)); // 只模糊垂直方向，保留灯条形状

            // // 步骤5：显示（调试用）
            // cv::imshow("img", img);   
            // cv::imshow("最终掩码", dst);    
        
        */


    }

private:
    void findAndJudgeLightStrip(cv::Mat img, cv::Mat& img_hsv, cv::Mat& mask)
    {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


        int sum = 0; // 目前检测到的候选灯带数量 Strip Count（需要吗？）
        int corners_num = 0; //当前灯条角点数

        for (int i = 0; i < contours.size(); i++)
        {
            corners_num = 0;
            int area = cv::contourArea(contours[i]);
            if (area > 100) // 面积去除噪点
            {
                /*
                    所以旋转矩形的angle和width height是这样定义的
                    首先 angle范围理论上为(0, 90]
                    然后，过旋转矩形的中心点作一条水平直线，现在顺时针旋转这条直线，当直线和矩形某一边平行时停止旋转，此时这条矩形平行边记为width，水平直线顺时针转过的角度就是angle，
                    当然，归一化到(0, 90]
                    但是0度 和 90度没法区分倒是
                */

                // 先获得旋转矩形

                cv::RotatedRect temp_rotatedRect = cv::minAreaRect(contours[i]);



                // 获取参数

                double angle = temp_rotatedRect.angle; // 获取角度

                cv::Size2d size = temp_rotatedRect.size;  // 尺寸
                double width = size.width; // 宽
                double height = size.height; // 高

                corners_num = contours[i].size(); // 角点数量


                // 角度处理

                if (width > height)
                {
                    angle -= 90.00; // 如果逆时针旋转矩形，那么让度数变成负的，这样旋转矩形 from -90 to 90
                    std::swap(width, height); // 确保 height 是长边
                }
                if (angle == 90.00) angle = 0.00; // 90度视为0度，确保 angle 连续



                // 高宽比处理

                double ratio = height / width; // [高宽比]不能太小，确保像灯带

                if (ratio < 2.50) continue;



                /*
                    // 类似水平噪声处理 

                    // 但opencv没办法区分90度的水平矩形和90度的竖直矩形，所以算一下跨度差，把水平矩形排除掉
                    cv::Point2f corners[4]; // 四个顶点
                    temp_rotatedRect.points(corners); // 获取顶点
                    float xmin = 999999, xmax = -1;
                    float ymin = 999999, ymax = -1;
                    for (int j = 0; j < 4; j++)
                    {
                        xmin = std::min(xmin, corners[j].x);
                        xmax = std::max(xmax, corners[j].x);
                        ymin = std::min(ymin, corners[j].y);
                        ymax = std::max(ymax, corners[j].y);
                    }
                    if (xmax - xmin > ymax - ymin) continue; // 水平跨度 > 竖直跨度，是水平矩形，不要
                */
                
                // 灯条 + 1
                ++sum;
            

                // 区分颜色之边缘部分


                // 生成一个只有当前灯条边缘及其内部是 255 的掩码图/home/cly/project/saved_image_6780318180280.jpg
                cv::Mat strip_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
                cv::drawContours(strip_mask, std::vector<std::vector<cv::Point>>{contours[i]}, 0, 255, -1);

                int saturation_edge_total = 0; //  总边缘饱和度
                int blue_edge_total = 0; //  总边缘蓝值
                int green_edge_total = 0; //  总边缘绿值
                int red_edge_total = 0; //  总边缘红值

                double blue_edge_average = 0.00; // 边缘平均蓝值
                double green_edge_average = 0.00; // 边缘平均绿值
                double red_edge_average = 0.00; // 边缘平均红值
                double saturation_edge_average = 0.00; // 边缘平均饱和度

                for(int j = 0; j < corners_num; j++)
                {
                    // 注意行在前，列在后
                    if (contours[i][j].x >= 0 && contours[i][j].x < img.cols && contours[i][j].y >= 0 && contours[i][j].y < img.rows)
                    {
                        // 在该点的 strip_mask 值
                        uchar mask_value = strip_mask.at<uchar>(contours[i][j].y, contours[i][j].x);

                        // 掩码图上是有这个点的 -> 这个点是这个灯条上的 -> bgr和hsv有效
                        if(mask_value) 
                        {
                            // bgr
                            cv::Vec3b bgr_value = img.at<cv::Vec3b>(contours[i][j].y, contours[i][j].x);  
                            uchar blue = bgr_value[0];
                            uchar green = bgr_value[1];   
                            uchar red = bgr_value[2];
                            blue_edge_total += blue;
                            green_edge_total += green;
                            red_edge_total += red;

                            // saturation
                            cv::Vec3b hsv_value = img_hsv.at<cv::Vec3b>(contours[i][j].y, contours[i][j].x);
                            uchar saturation = hsv_value[1]; // S 范围 0~255
                            saturation_edge_total += saturation;
                    
                        }
                    }
                }
                
                blue_edge_average = blue_edge_total / corners_num; // 边缘平均蓝值
                green_edge_average = green_edge_total / corners_num; // 边缘平均绿值
                red_edge_average = red_edge_total / corners_num; // 边缘平均红值
                saturation_edge_average = saturation_edge_total / corners_num; // 边缘平均饱和度
                



                // 区分颜色 之 总体部分 注意是针对 strip_mask 与 contours无关！

                int saturation_total = 0; //  总饱和度
                double saturation_average = 0.00; // 平均饱和度
                int pixels_num = 0; // 灯条像素点总数

                for(int j = 0; j < strip_mask.rows; j++) // 行y
                {
                    for(int k = 0; k < strip_mask.cols; k++) // 列x
                    {
                        // 在该点的 strip_mask 值
                        uchar mask_value = strip_mask.at<uchar>(j, k);
                        if(mask_value) 
                        {
                            cv::Vec3b hsv_value = img_hsv.at<cv::Vec3b>(j, k);
                            uchar saturation = hsv_value[1]; // S 范围 0~255
                            saturation_total += saturation;

                            ++pixels_num;
                        }
                    }
                }

                saturation_average = saturation_total / pixels_num; // 总体平均饱和度



                // 中心点饱和度

                double saturation_center = -9999.99;
                uchar mask_value = strip_mask.at<uchar>(temp_rotatedRect.center.y, temp_rotatedRect.center.x);

                if(mask_value) 
                {
                    cv::Vec3b hsv_value = img_hsv.at<cv::Vec3b>(temp_rotatedRect.center.y, temp_rotatedRect.center.x);
                    uchar saturation = hsv_value[1];
                    saturation_center = saturation;
                }
                
                


                // 画画部分

                // 信息
                cv::putText(img, 
                    "strip " + std::to_string((int)sum) + 
                    " corners = " + std::to_string((int)corners_num) + 
                    " b = " + std::to_string((int)blue_edge_average) + 
                    " g = " + std::to_string((int)green_edge_average) + 
                    " r = " + std::to_string((int)red_edge_average) + 
                    " s_edge = " + std::to_string((int)saturation_edge_average) + 
                    " s_total = " + std::to_string((int)saturation_average) +
                    " s_ctr = " + std::to_string((int)saturation_center), cv::Point2f(0, sum * 50), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255), 2);
                
                // 灯条几
                cv::putText(img, "s" + std::to_string((int)sum), cv::Point2f(temp_rotatedRect.center.x, temp_rotatedRect.center.y), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 0.75);

                cv::Point2f corners[4]; // 四个顶点
                temp_rotatedRect.points(corners); // 获取顶点

                // 画出旋转矩形
                for (int j = 0; j < 4; j++)
                {
                    cv::putText(img, "[" + std::to_string((int)j + 1) + "][" + std::to_string((int)corners[j].x) + ", " + std::to_string((int)corners[j].y) + "]", cv::Point(corners[j].x, corners[j].y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 0.65);
                    cv::line(img, corners[j], corners[(j + 1) % 4], cv::Scalar(0, 0, 255), 3); //画线
                }
                

                
            }
        }

        cv::imshow("after_changed", img);

    }


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto a = std::make_shared<MyNode>();
    if(cv::waitKey(100000) == 27)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "MyNode 节点已被手动退出! ");
        rclcpp::shutdown();
    }
    rclcpp::shutdown();
    return 0;
}