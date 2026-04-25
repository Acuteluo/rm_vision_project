#include "armorplate.h"



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
        this->armorplate_width = 0.135; // 装甲板宽，单位m
        this->armorplate_height = 0.055; // 装甲板高，单位m
    }

    else
    {
        // [英雄] 装甲板的物理参数
        this->armorplate_width = 0.225; // 装甲板宽，单位m
        this->armorplate_height = 0.055; // 装甲板高，单位m
    }



    // 【修改后】装甲板的世界坐标系 以装甲板中心为原点的右手系 注意是站在【有数字那一面】前来定义！ -> x朝前，y朝左，z朝上
    // 那么 x = 0
    /*
                   z ^   ^ x
                     |  /
                     | /
                     |/
         y <---------O  <一【装甲板中心]

                 右  手  系    
    */
    this->vertice_world.push_back(cv::Point3f(0.00, +armorplate_width/2, +armorplate_height/2)); // 左上（+, +）
    this->vertice_world.push_back(cv::Point3f(0.00, +armorplate_width/2, -armorplate_height/2)); // 左下（+, -）
    this->vertice_world.push_back(cv::Point3f(0.00, -armorplate_width/2, -armorplate_height/2)); // 右下 (-, -)
    this->vertice_world.push_back(cv::Point3f(0.00, -armorplate_width/2, +armorplate_height/2)); // 右上（-, +）
    


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


    // 图像坐标系 顺序是（相对于中心。其实不然，这里有转换保证逆时针-90 ~ 顺时针+90度）
    // 左上(-, -) 左下(-, +) 右下(+, +) 右上(+, -)
    /*
        O----------> x
        |
        |
        |
        v 
    
        y

    */
    this->vertice_pixel.push_back(tl);
    this->vertice_pixel.push_back(bl);
    this->vertice_pixel.push_back(br);
    this->vertice_pixel.push_back(tr);


    // 【修改后】相机坐标系 以图像中心为相机光轴中心 站在相机朝向后来定义-> x朝前 y朝左 z朝上（右手系）
    /*
                   z ^   ^ x
                     |  /
                     | /
                     |/
         y <---------O  <一【镜头中心]

                 右  手  系    
    */

}




/**
* @brief	画出装甲板并标点，打印 pnp 信息
* @return   无返回值，直接画图
*/
void ArmorPlate::drawArmorPlateAndPrintPNPInfo(std::string CHOSEN_COLOR)
{
    cv::Scalar color;
    if(CHOSEN_COLOR == "red") color = cv::Scalar(0, 255, 255); // 红色装甲板 -> 黄色
    else color = cv::Scalar(0, 255, 0); // 蓝色装甲板 -> 绿色

    // 画出装甲板的四个角点、中心点和边框
    for (int i = 0; i < 4; i++)
    {
        cv::putText(this->img_show, "[" + std::to_string((int)i + 1) + "] [" + std::to_string((int)vertice_pixel[i].x) + ", " + std::to_string((int)vertice_pixel[i].y) + "]", cv::Point2f(vertice_pixel[i].x, vertice_pixel[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 128, 255), 0.65);
        cv::line(this->img_show, vertice_pixel[i], vertice_pixel[(i + 1) % 4], color, 2); // 画线
    }
    cv::circle(this->img_show, this->center, 3, color, cv::FILLED); // 中心点

    // 打印 置信度 和 pnp 信息
    cv::putText(this->img_show, "moderation = " + std::to_string((double)this->moderation), cv::Point2f(0, 450), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
	cv::putText(this->img_show, "t_pitch = " + std::to_string((double)this->t_pitch), cv::Point2f(0, 500), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
    cv::putText(this->img_show, "t_yaw = " + std::to_string((double)this->t_yaw), cv::Point2f(0, 550), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
	cv::putText(this->img_show, "t_distance = " + std::to_string((double)this->t_distance) + "m", cv::Point2f(0, 600), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
}



/**
* @brief	用 pnp 算 [R | T]
* @return   无返回值
*/
void ArmorPlate::perspectiveNPoint()
{
	// P: 你的 FLU 右手系 -> OpenCV 相机系
    Eigen::Matrix3d P;
    P << 0, -1,  0,
         0,  0, -1,
         1,  0,  0;

    // vertice_cv = P * vertice_world
    std::vector<cv::Point3f> vertice_cv;
    for (const auto& pt : this->vertice_world) 
    {
        // 1. 取出你定义的 FLU 坐标点
        Eigen::Vector3d pt_flu(pt.x, pt.y, pt.z);
        
        // 2. 【核心】施加线性变换：P_cv = P * P_flu
        Eigen::Vector3d pt_cv = P * pt_flu;
        
        // 3. 存入交给 OpenCV 的点集中
        vertice_cv.push_back(cv::Point3f(pt_cv(0), pt_cv(1), pt_cv(2)));
    }


    /*
        输入：装甲板世界点集 装甲板像素点集 相机内参K 相机畸变D 
        输出：旋转向量r 平移向量t （在 opencv 默认坐标系下）
        设置：是否使用外点剔除 迭代次数 内点距离阈值 内点置信度阈值 以及使用的pnp算法
    */ 

    // 换用 IPPE ，但是需要 z=0 平面，所以索性先转到cv系下，到时候再转回去，也就是相似变换 
	this->is_success = cv::solvePnP(vertice_cv, this->vertice_pixel, K, D, 
        this->r, this->t, 
        false, 
        cv::SOLVEPNP_IPPE); 


    // 如果解算不成功
	if (!this->is_success)
	{
		RCLCPP_ERROR(rclcpp::get_logger("ArmorPlate"), "---------->  pnp解算失败! ");
        return;
	}



    // --------------------- 如果解算成功 ---------------------

    // 对于平移向量t，提取出 X Y Z。
    // 一定注意！！这里的 [R|t] 都是 opencv默认的相机坐标系下的坐标（x朝右，y朝下，z朝前）需要全部转换才能用！！
    /*
                         ^ z
                        /
                       /
                      /
     【镜头中心] 一>   O---------> x
                     |
                     |
                     |
                     |
                     v y

        opencv 默 认 相 机 坐 标 系    
    */

    /// >>>>>>>>>>>>>>>>>>>>> t -> t_vec <<<<<<<<<<<<<<<<<<<<

    // 先获取在 opencv 默认相机系下的坐标
    // 提取平移向量 t_cv
    Eigen::Vector3d t_cv(this->t.at<double>(0, 0), 
                         this->t.at<double>(1, 0), 
                         this->t.at<double>(2, 0));


    /// >>>>>>>>>>>>>>>>>>>>> r -> R <<<<<<<<<<<<<<<<<<<<
    
    // 罗德里格斯变换，把 旋转向量 r -> 旋转矩阵 R_origin
    cv::Mat R_origin;
    cv::Rodrigues(this->r, R_origin);

    // 将 cv::Mat 转换为 Eigen::Matrix3d
    Eigen::Matrix3d A;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            A(i, j) = R_origin.at<double>(i, j);
        }
            
    }
        
    // 构造右手系坐标下的 平移向量 t_vec
    // 用 p逆 也就是 p的转置
    this->t_vec = P.transpose() * t_cv; 


    // 计算新的旋转矩阵并存储到 this->R
    // 注意：这里是 R_transform * R_origin_eigen，左乘
    // 【旋转矩阵转换】：使用相似变换 P_inv * A * P 完美还原模型
    this->R = P.transpose() * A * P;

    double X = this->t_vec(0);
    double Y = this->t_vec(1);
    double Z = this->t_vec(2);

    // ------ 下面信息是给人验证的，因为 yaw pitch 是 t_vec 向量计算出来的，相对于相机坐标系右手系的偏差角 ------


    this->t_distance = std::sqrt(X * X + Y * Y + Z * Z); // 求距离


    // atan2(y, x) 的符号只由 y 决定，与 x 无关。
    // pitch + 表示目标在相机下方。当△z为-时，结果为-。装甲板在下方，需要pitch为+，所以要取负号
    // yaw + 表示目标在相机左侧。因为当△y为-时，结果为-。装甲板在右方，需要yaw为-，没问题
    
    this->t_pitch = -std::atan2(Z, std::sqrt(X * X + Y * Y)) * 180.0 / CV_PI;
    this->t_yaw = std::atan2(Y, X) * 180.0 / CV_PI;
    

}

