#pragma once
#include "strip.h"


//装甲板类
class ArmorPlate
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen 内存对齐：如果你的 ArmorPlate 对象可能被 std::vector 存储，需要包含 EIGEN_MAKE_ALIGNED_OPERATOR_NEW 宏（对于固定大小 Eigen 类型）。但通常 ROS 节点中单个对象问题不大，如果使用 std::vector<ArmorPlate> 则必须添加。

	ArmorPlate(); // 默认构造


	// 传入 一左一右两个灯带 + 置信度 + 相机名称 + 装甲板类型，组装成一个装甲板
	ArmorPlate(Strip a, Strip b, double moderation, std::string camera_name, std::string armor_type);


    // 设置参数
    void setParam(); 


    // 传入 img_show
    void setImgShow(cv::Mat& img_show);


    // 获取 img_show
    cv::Mat getImgShow();



	/**
	* @brief	画出装甲板并标点，打印 pnp 信息
	* @return   无返回值，直接画图
	*/
	void drawArmorPlateAndPrintPNPInfo();


	/**
	* @brief	用 pnp 算 [R | T]
	* @return   无返回值
	*/
	void perspectiveNPoint();


	cv::Point2f center; // 装甲板中心点
    cv::Point2f tl; // 装甲板左上
    cv::Point2f bl; // 装甲板左下
    cv::Point2f br; // 装甲板右下
    cv::Point2f tr; // 装甲板右上

    cv::Mat K; // 内参矩阵
	cv::Mat D; // 畸变矩阵


    ///////// pnp 结算的原始结果（opencv 默认 相机坐标系下） /////////

    bool is_success; // 是否解算成功

    double moderation; // 置信度

    cv::Mat r; // 旋转向量
	cv::Mat t; // 平移向量    


    ///////// pnp 结算的转换结果（右手系） /////////
    Eigen::Vector3d t_vec; // 右手系 平移向量
    Eigen::Matrix3d R; // 右手系 旋转矩阵

private:

    // pnp 要求：
	std::vector<cv::Point2f> vertice_pixel; // [像素坐标系]装甲板四个角点，[顺序：左上，左下，右下，右上]
	std::vector<cv::Point3f> vertice_world; // [世界坐标系]装甲板四个角点，[顺序：左上，左下，右下，右上]，单位mm

    std::string CAMERA_NAME; // 相机名称，决定了内参矩阵和畸变矩阵

    // pnp 解算结果可视化（从 opencv 默认相机坐标系 转换到 右手系后）
    
    double t_yaw; // 偏航角
	double t_pitch; // 俯仰角
	double t_distance; // 距离


    std::string ARMOR_TYPE; // 装甲板类型，英雄/步兵，决定了装甲板的世界坐标系点的位置

    double armorplate_width; // 装甲板宽度，单位mm
    double armorplate_height; // 装甲板高度，单位mm

    

    cv::Mat img_show; // 要画的图

};