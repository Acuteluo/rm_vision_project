#include "utils/armorplate.h"



// 默认构造
ArmorPlate::ArmorPlate() = default;



// 设置参数
void ArmorPlate::setParam()
{

    // 后期可以改成从配置文件读取
    /*
        // 从 yaml 文件读取
        cv::FileStorage fs("camera_config.yaml", cv::FileStorage::READ);
        fs["camera_matrix"] >> K;
        fs["distortion_coefficients"] >> D;
        fs["image_width"] >> image_width;
        fs["image_height"] >> image_height;
    */
    

    // 根据不同的装甲板类型设置装甲板的物理参数，目前分为 步兵 和 英雄 两种
    if(this->ARMOR_TYPE == "normal")
    {
        // [步兵] 装甲板的物理参数
        this->armorplate_width = 135.00; // 装甲板宽，单位mm
        this->armorplate_height = 55.00; // 装甲板高，单位mm
    }

    else
    {
        // [英雄] 装甲板的物理参数
        this->armorplate_width = 225.00; // 装甲板宽，单位mm
        this->armorplate_height = 55.00; // 装甲板高，单位mm
    }



    // 装甲板的世界坐标系 以装甲板中心为原点的右手系 -> x朝右，y朝上，z朝外
    this->vertice_world.push_back(cv::Point3f(-armorplate_width/2, +armorplate_height/2, 0.00)); // 左上（-, +）
    this->vertice_world.push_back(cv::Point3f(-armorplate_width/2, -armorplate_height/2, 0.00)); // 左下（-, -）
    this->vertice_world.push_back(cv::Point3f(+armorplate_width/2, -armorplate_height/2, 0.00)); // 右下 (-, +)
    this->vertice_world.push_back(cv::Point3f(+armorplate_width/2, +armorplate_height/2, 0.00)); // 右上（+, +）


    // ---------- [相机参数设置] ----------

    if(this->CAMERA_NAME == "mind_vision")
    {
        // mind_vision 的参数

        // 相机内参K 3 * 3 
        this->K = (cv::Mat_<double>(3, 3) << 1359.21385,    0.     ,  635.62767,
                                            0.     , 1361.75423,  478.48483,
                                            0.     ,    0.     ,    1.     );

        // 相机畸变矩阵D 5 * 1，和 Eigen 不同，opencv的矩阵是先列后行
        // k1 k2 p1 p2 k3
        this->D = (cv::Mat_<double>(5, 1) << -0.081521, 0.153947, -0.006919, -0.003306, 0.000000);

    }

    else
    {
        // galaxy 的参数

        // 相机内参K 3 * 3 
        this->K = (cv::Mat_<double>(3, 3) << 1305.07013,    0.     ,  666.71169,
                                            0.     , 1307.67428,  495.17044,
                                            0.     ,    0.     ,    1.     );

        // 相机畸变矩阵D 5 * 1，和 Eigen 不同，opencv的矩阵是先列后行
        // k1 k2 p1 p2 k3
        this->D = (cv::Mat_<double>(5, 1) << -0.209067, 0.129977, -0.002895, -0.000558, 0.000000);
       
    }
    
    

}


// 传入 img_show
void ArmorPlate::setImgShow(cv::Mat& img_show)
{
    this->img_show = img_show;
}



// 获取 img_show
cv::Mat ArmorPlate::getImgShow()
{
    return this->img_show;
}



// 有参构造，放入两个灯带 + 置信度 + 相机名称 + 装甲板类型，构造装甲板
ArmorPlate::ArmorPlate(Strip a, Strip b, double moderation, std::string camera_name, std::string armor_type)
{
    // ------- 1. 先设置基础参数 -------
    this->moderation = moderation; // 合理性
    this->CAMERA_NAME = camera_name; // 相机名称
    this->ARMOR_TYPE = armor_type; // 装甲板类型
    setParam();

    // 然后构造装甲板

    // ------- 2. 计算整个装甲板的理论中心点 用四个点来算 -------
    this->center = cv::Point2f(
        (a.upper.x + a.lower.x + b.upper.x + b.lower.x) / 4, 
        (a.upper.y + a.lower.y + b.upper.y + b.lower.y) / 4);



    // ------- 3. 确定 图像坐标系上装甲板的 左上 左下 右下 右上 四个点对应哪里 -------
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
    this->vertice_pixel.push_back(tl);
    this->vertice_pixel.push_back(bl);
    this->vertice_pixel.push_back(br);
    this->vertice_pixel.push_back(tr);


    // 相机坐标系 以图像中心为相机光轴中心 -> x朝右 y朝下 z朝前

}




/**
 * @brief	画出装甲板并标点
 * @return  无返回值，直接画图
 */
void ArmorPlate::drawArmorPlate()
{
    for (int i = 0; i < 4; i++)
    {
        cv::putText(this->img_show, "[" + std::to_string((int)i + 1) + "] [" + std::to_string((int)vertice_pixel[i].x) + ", " + std::to_string((int)vertice_pixel[i].y) + "]", cv::Point2f(vertice_pixel[i].x, vertice_pixel[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 0.65);
        cv::line(this->img_show, vertice_pixel[i], vertice_pixel[(i + 1) % 4], cv::Scalar(0, 0, 255), 2); // 画线
    }
    cv::circle(this->img_show, this->center, 3, cv::Scalar(0, 0, 255), cv::FILLED); // 中心点
}



/**
* @brief	用 pnp 算 [R | T]
* @return   无返回值
*/
void ArmorPlate::perspectiveNPoint()
{
	std::vector<int> inliers;
	bool is_success = true;


    /*
        输入：装甲板世界点集 装甲板像素点集 相机内参K 相机畸变D 
        输出：旋转向量r 平移向量t 
        设置：是否使用外点剔除 迭代次数 内点距离阈值 内点置信度阈值 以及使用的pnp算法
    */ 
	is_success = cv::solvePnPRansac(
        this->vertice_world, this->vertice_pixel, K, D, 
        this->r, this->t, 
        false, 200, 8.0, 0.99, inliers, cv::SOLVEPNP_IPPE);


    // 如果解算不成功
	if (!is_success)
	{
		RCLCPP_ERROR(rclcpp::get_logger("ArmorPlate"), "---------->  pnp解算失败! ");
        return;
	}



    // --------------------- 如果解算成功 ---------------------

    // 对于平移向量t，提取出 X Y Z
    double X = this->t.at<double>(0, 0);
    double Y = this->t.at<double>(1, 0);
    double Z = this->t.at<double>(2, 0);

    this->t_distance = std::sqrt(X * X + Y * Y + Z * Z); // 求距离


    // yaw 水平偏角，+表示目标在相机右侧
    // pitch = 垂直偏角，+表示目标在相机上方（Y轴向下，取负更合适）
    this->t_yaw = std::atan2(X, Z) * 180.0 / CV_PI;
    this->t_pitch = -std::atan2(Y, Z) * 180.0 / CV_PI;

}



/**
* @brief	输出 yaw pitch distance 信息
* @param	cv::Mat& img_show 要画的图
* @param    int index 装甲板编号
* @return   无返回值，直接画图
*/
void ArmorPlate::printPNPInfo(int index)
{
	cv::putText(this->img_show, "A" + std::to_string((int)index) + " t_yaw = " + std::to_string((double)this->t_yaw), cv::Point2f(0, 500 + 150*(index-1)), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
	cv::putText(this->img_show, "A" + std::to_string((int)index) + " t_pitch = " + std::to_string((double)this->t_pitch), cv::Point2f(0, 550 + 150*(index-1)), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
	cv::putText(this->img_show, "A" + std::to_string((int)index) + " t_distance = " + std::to_string((double)this->t_distance) + "mm", cv::Point2f(0, 600 + 150*(index-1)), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
}