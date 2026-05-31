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
Strip::Strip(cv::RotatedRect rotated_rect, double angle, double width, double height, std::string color, std::vector<cv::Point> contour)
{
    // 1. 存灯条数据
	this->rotated_rect = rotated_rect; // 原始旋转矩形
	this->angle = angle; // 角度
	this->width = width; // 宽度（短边）
	this->height = height; // 高度（长边）
    this->color = color; // 颜色 red/blue/white

 
	// 2. 获取灯条的上下端点 需要用固定大小的数组接收再转成 vector 来排序，以确定上下端点
	cv::Point2f points[4]; 
	rotated_rect.points(points);
    std::vector<cv::Point2f> vertice(points, points + 4);
	std::sort(vertice.begin(), vertice.end(), sortPointByY);

    // 注意 opencv 坐标系是从左上角开始的，所以在图像中较上的点，y 值较小
	cv::Point2f subpix_upper = cv::Point2f((vertice[0].x + vertice[1].x) * 0.5, (vertice[0].y + vertice[1].y) * 0.5); // 上端点
	cv::Point2f subpix_lower = cv::Point2f((vertice[2].x + vertice[3].x) * 0.5, (vertice[2].y + vertice[3].y) * 0.5); // 下端点

    // 3. 【核心黑科技】：使用原始 contour 进行最小二乘法拟合中心轴线
    cv::Vec4f line_param; 
    // 拟合出直线，参数为: [vx, vy, x0, y0] (vx, vy 是方向向量，x0, y0 是线上一点)
    cv::fitLine(contour, line_param, cv::DIST_L2, 0, 0.01, 0.01);

    double vx = line_param[0];
    double vy = line_param[1];
    double x0 = line_param[2];
    double y0 = line_param[3];

    cv::Point2f accurate_top, accurate_bottom;

    // 4. 将亚像素确定的高度边界，强制投影到最小二乘法拟合出的中心轴线上！
    // 直线方程：x = x0 + (y - y0) * (vx / vy)
    if (std::abs(vy) > 1e-5) // 正常灯条绝不可能水平
    {
        accurate_top.y = subpix_upper.y;
        accurate_top.x = x0 + (subpix_upper.y - y0) * (vx / vy);

        accurate_bottom.y = subpix_lower.y;
        accurate_bottom.x = x0 + (subpix_lower.y - y0) * (vx / vy);
    }
    else 
    {
        // 如果极端水平（一般不会发生），退回你原本的计算方法
        // ...
    }

    // 5. 将这极致精确的两个点存入 Strip

    this->center = cv::Point2f((accurate_top.x + accurate_bottom.x) / 2.0f, 
                            (accurate_top.y + accurate_bottom.y) / 2.0f);; // 灯条中心点

    this->upper = accurate_top;
    this->lower = accurate_bottom;
}
