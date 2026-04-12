#include "strip.h"

/**
* @brief	（静态函数，谁都可以调用）把灯条的旋转矩形的点按从上到下排序
* @param	const cv::Point2f& a, const cv::Point2f& b 两个排序的点
* @return   1 或 0
*/
bool Strip::sortPointByY(const cv::Point2f& a, const cv::Point2f& b)
{
	return a.y < b.y; //从上到下排序，用来算upper和lower
}



Strip::Strip()
{
	upper = cv::Point2f(); // 简化后的上端点（左上 右上 两个角点化为一个）
	lower = cv::Point2f(); // 简化后的下端点（左下 右下 两个角点化为一个）
	center = cv::Point2f(); // 旋转矩形的中心点
	angle = 0.00; // 倾斜角
	height = 0.00; // 高度
	width = 0.00; // 宽度
}


/**
* @brief	构造函数
* @param	cv::RotatedRect rotated_rect 灯条的旋转矩形
* @param	double angle 处理后的角度
* @param	double width 宽
* @param	double height 高
* @param	cv::Mat img_binary 二值图，用来亚像素优化
* @return   无
*/
Strip::Strip(cv::RotatedRect rotated_rect, double angle, double width, double height, std::string color)
{
    // 1. 存灯条数据
	this->rotated_rect = rotated_rect; // 原始旋转矩形
	this->angle = angle; // 角度
	this->width = width; // 宽度（短边）
	this->height = height; // 高度（长边）
	this->center = rotated_rect.center; // 灯条中心点
    this->color = color; // 颜色 red/blue/white

 
	// 2. 获取灯条的上下端点 需要用固定大小的数组接收再转成 vector 来排序，以确定上下端点
	cv::Point2f points[4]; 
	rotated_rect.points(points);
    std::vector<cv::Point2f> vertice(points, points + 4);
	std::sort(vertice.begin(), vertice.end(), sortPointByY);

    // 注意 opencv 坐标系是从左上角开始的，所以在图像中较上的点，y 值较小
	this->upper = cv::Point2f((vertice[0].x + vertice[1].x) * 0.5, (vertice[0].y + vertice[1].y) * 0.5); // 上端点
	this->lower = cv::Point2f((vertice[2].x + vertice[3].x) * 0.5, (vertice[2].y + vertice[3].y) * 0.5); // 下端点

}
