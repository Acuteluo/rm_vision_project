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
	

	/**
	* @brief	寻找 + 判断灯带的颜色
	* @return   std::vector<Strip> 返回灯条集合
	*/
	std::vector<Strip> findAndJudgeLightStrip();
	

	/**
    * @brief	灯带配对
    * @return   std::vector<ArmorPlate> 返回装甲板集合
    */
    std::vector<ArmorPlate> pairStrip();
	

    /**
	* @brief	返回 img_show
	* @return   返回 img_show
	*/
	cv::Mat getImgShow();


private:

	/**
	* @brief	灯条从左到右排序 排序规则函数
	* @param	const Strip& a, const Strip& b
	* @return   0 1
	*/
	static bool sortStripByX(const Strip& a, const Strip& b);

    cv::Mat img; // 原图
    cv::Mat img_show; // 用来显示信息的图
    cv::Mat img_hsv; // hsv图
	cv::Mat mask; // 最终初步掩码图

	cv::Mat img_gray; // 灰度图
	std::vector<Strip> strip; // 灯条集合
	std::vector<ArmorPlate> armorplate; // 装甲板集合

    std::string CHOSEN_COLOR; // 选择检测的颜色 red / blue
};