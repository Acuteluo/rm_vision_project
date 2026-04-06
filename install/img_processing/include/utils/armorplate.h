#pragma once
#include "utils/strip.h"


//装甲板类
class ArmorPlate
{
public:

	ArmorPlate(); // 默认构造


	// 传入 一左一右两个灯带 + 置信度 + 相机名称，组装成一个装甲板
	ArmorPlate(Strip a, Strip b, double moderation, std::string camera_name); 


    // 设置参数
    void setParam(); 

    // 传入 img_show
    void setImgShow(cv::Mat& img_show);


    // 获取 img_show
    cv::Mat getImgShow();


	/**
	* @brief	画出装甲板并标点
	* @return   无返回值，直接画图
	*/
	void drawArmorPlate();


	/**
	* @brief	用 pnp 算 [R | T]
	* @return   无返回值
	*/
	void perspectiveNPoint();


	/**
	* @brief	输出 yaw pitch distance 信息
    * @param    int index 装甲板编号
	* @return   无返回值，直接画图
	*/
	void printPNPInfo(int index);


	cv::Point2f center; // 装甲板中心点
    cv::Point2f tl; // 装甲板左上
    cv::Point2f bl; // 装甲板左下
    cv::Point2f br; // 装甲板右下
    cv::Point2f tr; // 装甲板右上

    double t_yaw; //偏航角
	double t_pitch; //俯仰角
	double t_distance; //距离


    double moderation; // 置信度


private:

    // pnp 要求：
	std::vector<cv::Point2f> vertice_pixel; // [像素坐标系]装甲板四个角点，[顺序：左上，左下，右下，右上]
	std::vector<cv::Point3f> vertice_world; // [世界坐标系]装甲板四个角点，[顺序：左上，左下，右下，右上]，单位mm

	cv::Mat K; // 内参矩阵
	cv::Mat D; // 畸变矩阵

    // pnp 解算结果：
	cv::Mat r; // 旋转向量
	cv::Mat t; // 平移向量    

    cv::Mat R; // 旋转矩阵

	int image_width; //图像宽 
	int image_height; //图像高

    double armorplate_width; // 装甲板宽度，单位mm
    double armorplate_height; // 装甲板高度，单位mm

    std::string CAMERA_NAME; // 相机名称，决定了内参矩阵和畸变矩阵

    cv::Mat img_show; // 要画的图

};