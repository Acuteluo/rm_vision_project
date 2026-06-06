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
    this->armorplate_width = rm_constants::getArmorWidth(this->ARMOR_TYPE);
    this->armorplate_height = rm_constants::ARMOR_HEIGHT;



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
    this->vertice_world = rm_utils::buildArmorVerticesFLU(armorplate_width, armorplate_height);
    


    // ---------- [相机参数设置] ----------
    rm_constants::getCameraIntrinsics(this->CAMERA_NAME, this->K, this->D);
    
    

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
void ArmorPlate::drawArmorPlateAndPrintPNPInfo(cv::Mat& img_show, std::string CHOSEN_COLOR, int sum)
{
    cv::Scalar color;
    if(CHOSEN_COLOR == "red") color = cv::Scalar(0, 255, 255); // 红色装甲板 -> 黄色
    else color = cv::Scalar(180, 153, 255); // 蓝色装甲板 -> 粉色

    // 画出装甲板的四个角点、中心点和边框
    for (int i = 0; i < 4; i++)
    {
        cv::putText(img_show, "[" + std::to_string((int)i + 1) + "] [" + std::to_string((int)vertice_pixel[i].x) + ", " + std::to_string((int)vertice_pixel[i].y) + "]", cv::Point2f(vertice_pixel[i].x, vertice_pixel[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 128, 255), 0.65);
        cv::line(img_show, vertice_pixel[i], vertice_pixel[(i + 1) % 4], color, 2); // 画线
    }
    cv::circle(img_show, this->center, 3, color, cv::FILLED); // 中心点

    // 只打印综合置信度（考虑与上一帧追踪装甲板的距离）最高的装甲板的信息，也就是现在跟踪的装甲板的信息
    if(sum == 0)
    {
        // 打印 置信度 和 pnp 信息
        cv::putText(img_show, "moderation = " + std::to_string((double)this->moderation), cv::Point2f(0, 450), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
	    cv::putText(img_show, "t_pitch = " + std::to_string((double)this->t_pitch), cv::Point2f(0, 500), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
        cv::putText(img_show, "t_yaw = " + std::to_string((double)this->t_yaw), cv::Point2f(0, 550), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
	    cv::putText(img_show, "t_distance = " + std::to_string((double)this->t_distance) + "m", cv::Point2f(0, 600), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);
    }
   
}



/**
* @brief	用 pnp 算 [R | T]
* @return   无返回值
*/
void ArmorPlate::perspectiveNPoint()
{
	// vertice_cv = P * vertice_world
    std::vector<cv::Point3f> vertice_cv = rm_utils::fluVerticesToCv(this->vertice_world);


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

    // 将 OpenCV 坐标系结果转换回 FLU 右手系
    auto flu_result = rm_utils::pnpCvToFLU(this->r, this->t);
    this->t_vec = flu_result.t_vec;
    this->R = flu_result.R;

    double X = this->t_vec(0);
    double Y = this->t_vec(1);
    double Z = this->t_vec(2);

    // ------ 下面信息是给人验证的，因为 yaw pitch 是 t_vec 向量计算出来的，相对于相机坐标系右手系的偏差角 ------


    auto spherical = rm_utils::cartesianToSpherical(X, Y, Z);
    this->t_distance = spherical.distance;
    this->t_pitch = spherical.pitch;
    this->t_yaw = spherical.yaw;
    

}





// ==================== 1. 计算给定 Euler Yaw 的重投影误差 ====================
double ArmorPlate::CalculateReprojectionError(double test_euler_yaw)
{
    // 1. FLU -> OpenCV 基变换
    std::vector<cv::Point3f> vertice_cv = rm_utils::fluVerticesToCv(this->vertice_world);

    // 2. 构造测试姿态的旋转矩阵 R_cv
    // 构造测试姿态的旋转矩阵并转换为 OpenCV rvec
    Eigen::Matrix3d R_flu_test = rm_utils::buildRotationFLU(test_euler_yaw, rm_constants::FIXED_ARMOR_PITCH_RAD);
    cv::Mat rvec_test = rm_utils::fluRotationToCvRvec(R_flu_test);

    // 3. 将 3D 模型点重投影到 2D
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(vertice_cv, rvec_test, this->t, this->K, this->D, projected_points);

    // =========================================================================
    // 4. 【核心黑魔法】：计算尺度无关的透视畸变误差！彻底消除 W 型双极小值陷阱！
    // =========================================================================
    
    // (A) 提取实际像素点的左右灯条高度和 Y 中心
    double h_L_pixel = std::abs(this->vertice_pixel[0].y - this->vertice_pixel[1].y);
    double h_R_pixel = std::abs(this->vertice_pixel[3].y - this->vertice_pixel[2].y);
    double y_center_L_pixel = (this->vertice_pixel[0].y + this->vertice_pixel[1].y) / 2.0;
    double y_center_R_pixel = (this->vertice_pixel[3].y + this->vertice_pixel[2].y) / 2.0;

    // (B) 提取投影点的左右灯条高度和 Y 中心
    double h_L_proj = std::abs(projected_points[0].y - projected_points[1].y);
    double h_R_proj = std::abs(projected_points[3].y - projected_points[2].y);
    double y_center_L_proj = (projected_points[0].y + projected_points[1].y) / 2.0;
    double y_center_R_proj = (projected_points[3].y + projected_points[2].y) / 2.0;

    // (C) 极其丝滑的平滑误差函数 (采用 L2 距离的平方，保证底部是 U 型而不是 V 型！)
    double dh_L = h_L_proj - h_L_pixel;
    double dh_R = h_R_proj - h_R_pixel;
    double dy_L = y_center_L_proj - y_center_L_pixel;
    double dy_R = y_center_R_proj - y_center_R_pixel;

    // 平方和误差，保证在 0 附近是极其平滑的碗底，绝对不会来回横跳！
    double error = dh_L*dh_L + dh_R*dh_R + dy_L*dy_L + dy_R*dy_R;

    return error;
}


// ==================== 2. 黄金分割法 (Phi优选 欧拉角) ====================
void ArmorPlate::OptimizeEulerYaw(cv::Mat& img_show, int index)
{
    if (!this->is_success) return;

    // 1. 获取 PnP 粗略算出的 欧拉角 Yaw，作为搜索的基准中心
    // 你的 this->R 已经是 FLU 坐标系下的旋转矩阵了，提取欧拉 Yaw (绕 Z 轴)
    double pnp_euler_yaw = std::atan2(this->R(1, 0), this->R(0, 0));

    // 2. 确定搜索区间：既然 PnP 大体是对的，我们就在 PnP_Yaw 左右各扩展 60 度进行精搜
    double search_radius = 60.0 * CV_PI / 180.0;
    double a = pnp_euler_yaw - search_radius;
    double b = pnp_euler_yaw + search_radius;
    
    const double PHI = 0.618033988749895; // 黄金分割率

    // 3. 初始化探测点
    double c = b - PHI * (b - a);
    double d = a + PHI * (b - a);

    double error_c = CalculateReprojectionError(c);
    double error_d = CalculateReprojectionError(d);

    // 4. 迭代逼近极小值 (25 次迭代)
    int iterations = 25;
    for (int i = 0; i < iterations; i++)
    {
        if (error_c < error_d) 
        {
            b = d;
            d = c;
            error_d = error_c; // 白嫖上次算过的值！极大节省 CPU
            c = b - PHI * (b - a);
            error_c = CalculateReprojectionError(c);
        } 
        else 
        {
            a = c;
            c = d;
            error_c = error_d; // 白嫖！
            d = a + PHI * (b - a);
            error_d = CalculateReprojectionError(d);
        }
    }

    // 5. 收敛得到极致精准的 欧拉角 Yaw
    double best_euler_yaw = (a + b) / 2.0;
    double best_euler_yaw_angle = best_euler_yaw * 180.0 / CV_PI;
    cv::putText(img_show, "best_euler_yaw_" + std::to_string((int)index) + " = " + std::to_string((double)best_euler_yaw_angle), cv::Point2f(0, 650 + 50 * index), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2.5);

    // ==========================================================
    // 6. 终极替换：用最优 Yaw 重新生成旋转矩阵，覆盖 PnP 的垃圾矩阵！
    // ==========================================================
    this->R = rm_utils::buildRotationFLU(best_euler_yaw, rm_constants::FIXED_ARMOR_PITCH_RAD);
    this->r = rm_utils::fluRotationToCvRvec(this->R);
}

