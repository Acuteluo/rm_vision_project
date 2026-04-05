#include "utils/armorplate.h"



// 默认构造
ArmorPlate::ArmorPlate()
{
    image_width = 0.00;     // 图像宽
    image_height = 0.00;    // 图像高
    yaw = 0.00;             // 偏航角
    pitch = 0.00;           // 俯仰角
    distance = 0.00;        // 距离
    center = cv::Point2f(); // 中心点
}



// 有参构造，放入两个灯带，构造装甲板
ArmorPlate::ArmorPlate(Strip a, Strip b)
{
    // 先优化装甲板

    // ------- 1. 计算整个装甲板的理论中心点 用四个点来算 -------
    this->center = cv::Point2f(
        (a.upper.x + a.lower.x + b.upper.x + b.lower.x) / 4, 
        (a.upper.y + a.lower.y + b.upper.y + b.lower.y) / 4);



    // ------- 2. 确定 左上 左下 右下 右上 四个点对应哪里 -------
    // 方法：根据两个灯条分别的中心点，看谁在左（左上/左下），谁在右（右上/右下），然后根据装甲板中心点，确定上下

    if(a.center.x < b.center.x) // a 在左（确定 左上/左下），b 在右（确定 右上/右下）
    {
        this->tl = a.upper; // 左上
        this->bl = a.lower; // 左下
        this->br = b.lower; // 右下
        this->tr = b.upper; // 右上
    }
    else // b 在左，a 在右
    {
        this->tl = b.upper; // 左上
        this->bl = b.lower; // 左下
        this->br = a.lower; // 右下
        this->tr = a.upper; // 右上
    }


    // 像素坐标系 顺序是左上(-, -) 左下(-, +) 右下(+, +) 右上(+, -)
    // opencv坐标系 右为x（pitch俯仰） 下为y（yaw偏航） 前为z（roll翻滚）
    
    this->vertice_pixel.push_back(tl);
    this->vertice_pixel.push_back(bl);
    this->vertice_pixel.push_back(br);
    this->vertice_pixel.push_back(tr);


    // //世界坐标系 x向右 y向下 z朝前（转动中心）和相机坐标系建的一样
    // vertice_world.push_back(cv::Point3f(-67.50, -27.50, 0.00)); //左上（-, -）
    // vertice_world.push_back(cv::Point3f(-67.50, 27.50, 0.00)); //左下（-, +）
    // vertice_world.push_back(cv::Point3f(67.50, 27.50, 0.00)); //右下(+, +)
    // vertice_world.push_back(cv::Point3f(67.50, -27.50, 0.00)); //右上（+, -）

    // //相机内参K 3 * 3
    // K = (cv::Mat_<double>(3, 3) << 2374.54248, 0.00000, 698.85288,
    // 	0.00000, 2377.53648, 520.8649,
    // 	0.00000, 0.00000, 1.00000);

    // //畸变矩阵D 5 * 1，和 Eigen 不同，opencv的矩阵是先列后行
    // //p1 p2 k1 k2 p3
    // D = (cv::Mat_<double>(5, 1) << -0.059743, 0.355479, -0.000625, 0.001595, 0.000000);

    // //像素坐标系宽高
    // image_width = 1440;
    // image_height = 1080;

    // //初始化
    // yaw = 0.00; //偏航角
    // pitch = 0.00; //俯仰角
    // distance = 0.00; //距离
}

/**
 * @brief	画出装甲板并标点
 * @param	cv::Mat& img_show 要画的图
 * @return  无返回值，直接画图
 */
void ArmorPlate::drawArmorPlate(cv::Mat &img_show)
{
    for (int i = 0; i < 4; i++)
    {
        cv::putText(img_show, "[" + std::to_string((int)i + 1) + "][" + std::to_string((int)vertice_pixel[i].x) + ", " + std::to_string((int)vertice_pixel[i].y) + "]", cv::Point(vertice_pixel[i].x, vertice_pixel[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 0.65);
        cv::line(img_show, vertice_pixel[i], vertice_pixel[(i + 1) % 4], cv::Scalar(0, 0, 255), 3); // 画线
    }
    cv::circle(img_show, this->center, 3, cv::Scalar(0, 0, 255), cv::FILLED); // 中心点
}

// /**
// * @brief	用 pnp 算 [R | T]
// * @return   无返回值
// */
// void ArmorPlate::perspectiveNPoint()
// {
// 	std::vector<int> inliers;
// 	bool is_success = true;
// 	is_success = cv::solvePnPRansac(vertice_world, vertice_pixel, K, D, r, t, false, 200, 8.0, 0.99, inliers, cv::SOLVEPNP_EPNP);
// 	if (is_success)
// 	{
// 		cv::Rodrigues(r, R);  // 罗德里格斯，把旋转向量变成旋转矩阵

// 		//相机坐标系，z轴朝外，x轴朝右（pitch俯仰角），y轴朝下（yaw偏航角），因为opencv的坐标系就是这样
// 		//旋转：先z roll -> 再y yaw -> 最后x pitch
// 		double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0)); //r11 = cos(yaw)*cos(pitch)，r21 = sin(yaw)*cos(pitch)，求出cos(pitch)

// 		yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0)); //atan2(r21, r11)，atan2(sin(yaw), cos(yaw))
// 		pitch = atan2(-R.at<double>(2, 0), sy); //atan2(-r31, sqrt(r32 ^ 2 + r33 ^ 2))
// 												//r31 = -sin(pitch)，r32 = cos(pitch)*sin(roll)，r33 = cos(pitch)*cos(roll)

// 		//转成角度
// 		yaw = yaw * 180.00 / CV_PI;
// 		pitch = pitch * 180.00 / CV_PI;

// 		distance = sqrt(t.at<double>(0, 0) * t.at<double>(0, 0) + t.at<double>(1, 0) * t.at<double>(1, 0) + t.at<double>(2, 0) * t.at<double>(2, 0)); //距离
// 	}

// }

// /**
// * @brief	输出 yaw pitch distance 信息
// * @param	cv::Mat& img_show 要画的图
// * @return   无返回值，直接画图
// */
// void ArmorPlate::printInfo(cv::Mat& img_show)
// {
// 	cv::putText(img_show, "yaw = " + std::to_string((double)yaw), cv::Point(0, 200), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
// 	cv::putText(img_show, "pitch = " + std::to_string((double)pitch), cv::Point(0, 250), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
// 	cv::putText(img_show, "distance = " + std::to_string((double)distance), cv::Point(0, 300), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
// }
