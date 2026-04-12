#pragma once
#include "utils/strip.h"
#include "utils/armorplate.h"

class Prepare
{
public:

	/**
	* @brief	预处理函数
	* @param	cv::Mat& img 要处理的图
	* @return   无返回值
	*/
	void preProcessing(cv::Mat& img);
	

    /////// 设置 config.txt 对应的参数 ///////
    void setParam(bool show_logger, std::string chosen_color, std::string camera_name, std::string armor_type);


    /// 设置 img_show
    void setImgShow(cv::Mat& img_show);


    /// 获取 img_show
    cv::Mat getImgShow();
    

	/**
	* @brief	寻找 + 判断灯带的颜色
	* @return   std::vector<Strip> 返回灯条集合
	*/
	std::vector<Strip> findAndJudgeLightStrip();
	

	/**
    * @brief	灯带配对
    * @return   std::vector<ArmorPlate> 【修改】返回按置信度排序后的装甲板集合
    */
    std::vector<ArmorPlate> pairStrip();
	


private:

	/**
	* @brief	灯条从左到右排序 排序规则函数
	* @param	const Strip& a, const Strip& b
	* @return   0 1
	*/
	static bool sortStripByX(const Strip& a, const Strip& b);


    /////// 图像相关 ///////

    cv::Mat img; // 原图
    cv::Mat img_show; // 用来显示信息的图
    cv::Mat img_hsv; // hsv图
	cv::Mat mask; // 最终初步掩码图
    cv::Mat img_gray; // 灰度图 -> 用于亚像素优化



	////// 灯条配对成装甲板相关 /////

	std::vector<Strip> strip; // 灯条集合
	std::vector<ArmorPlate> armorplate; // 装甲板集合



    // ------ 预处理关键参数（从 config.txt 读取）------

    bool SHOW_LOGGER; // 是否显示日志
    std::string CHOSEN_COLOR; // 选择检测的颜色 red / blue
    std::string CAMERA_NAME; // 选择相机名称 mind_vision / galaxy ，注意改对应的 qos
    std::string ARMOR_TYPE; // 选择装甲板类型 normal / hero ，决定了配对的参数
};