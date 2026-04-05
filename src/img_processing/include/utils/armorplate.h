#pragma once
#include "utils/strip.h"


//装甲板类
class ArmorPlate
{
public:
	ArmorPlate(); //默认构造


	//传入一左一右两个灯带组装成一个装甲板
	ArmorPlate(Strip a, Strip b); 


	/**
	* @brief	画出装甲板并标点
	* @param	cv::Mat& img_show 要画的图
	* @return   无返回值，直接画图
	*/
	void drawArmorPlate(cv::Mat& img_show);


	// /**
	// * @brief	用 pnp 算 [R | T]
	// * @return   无返回值
	// */
	// void perspectiveNPoint();


	// /**
	// * @brief	输出 yaw pitch distance 信息
	// * @param	cv::Mat& img_show 要画的图
	// * @return   无返回值，直接画图
	// */
	// void printInfo(cv::Mat& img_show);

	cv::Point2f center; // 中心点
    cv::Point2f tl; // 左上
    cv::Point2f bl; // 左下
    cv::Point2f br; // 右下
    cv::Point2f tr; // 右上


private:

	std::vector<cv::Point2f> vertice_pixel; // [像素坐标系]装甲板四个角点，[顺序：左上，左下，右下，右上]

	std::vector<cv::Point3f> vertice_world; //[世界坐标系]装甲板四个角点，单位mm
	cv::Mat K; //内参矩阵
	cv::Mat D; //畸变矩阵
	cv::Mat R; //旋转矩阵
	cv::Mat r; //旋转向量
	cv::Mat t; //平移向量    
	int image_width; //图像宽 
	int image_height; //图像高

	double yaw; //偏航角
	double pitch; //俯仰角
	double distance; //距离

};