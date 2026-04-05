#pragma once
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



//灯条类
class Strip
{
public:

	Strip(); //默认构造


	/**
	* @brief	构造函数
	* @param	cv::RotatedRect rotated_rect 旋转矩形
	* @param	double angle 处理后的角度
	* @param	double width 宽
	* @param	double height 高
	* @param	cv::Mat img_binary 二值图，用来亚像素优化
    * @param	std::string color 灯条颜色
	* @return   无
	*/
	Strip(cv::RotatedRect rotated_rect, double angle, double width, double height, std::string color);


	/**
	* @brief	（静态函数，谁都可以调用）把灯条的旋转矩形的点按从上到下排序
	* @param	const cv::Point2f& a, const cv::Point2f& b 两个排序的点
	* @return   1 或 0
	*/
	static bool sortPointByY(const cv::Point2f& a, const cv::Point2f& b);



    ///////////// 灯条的参数 //////////////

    cv::RotatedRect rotated_rect; // 原始的 灯条旋转矩形

	cv::Point2f upper; // 简化后的上端点（左上 右上 两个角点化为一个）
	cv::Point2f lower; // 简化后的下端点（左下 右下 两个角点化为一个）
	cv::Point2f center; // 旋转矩形中心点

	double angle; // 倾斜角
	double height; // 高度
	double width; // 宽度

	std::string color; // 颜色 "red" / "blue"
};
