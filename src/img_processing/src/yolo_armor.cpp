#include "yolo_armor.hpp"


YoloArmor::YoloArmor() 
{
    // 初始化转换矩阵 P (FLU -> OpenCV，也就是在 OpenCV 坐标系下，FLU 三个轴基向量 分别的坐标)
    /*
        FLU: x前, y左, z上
        OpenCV: x右(-y), y下(-z), z前(x)
    */
    P_ << 0, -1,  0,
          0,  0, -1,
          1,  0,  0;
}


YoloArmor::YoloArmor(int armor_id, int color, bool is_big, float confidence,
                     const cv::Rect& box, const std::vector<cv::Point2f>& corners,
                     const cv::Mat& K, const cv::Mat& D)
    : armor_id_(armor_id),
      color_(color),
      is_big_(is_big),
      confidence_(confidence),
      box_(box),
      corners_(corners),
      pnp_success_(false),  
      t_distance_(0.0),
      t_yaw_(0.0),
      t_pitch_(0.0),
      K_(K),        
      D_(D)
{
    // 初始化转换矩阵 P (FLU -> OpenCV，也就是在 OpenCV 坐标系下，FLU 三个轴基向量 分别的坐标)
    P_ << 0, -1,  0,
          0,  0, -1,
          1,  0,  0;
}


void YoloArmor::SetArmorplateSize() 
{
    vertice_world_.clear();
    vertice_cv_.clear();
    
    // 1. 根据 YOLO 的判断动态获取真实物理尺寸
    double armorplate_width = is_big_ ? 0.225 : 0.135;
    double armorplate_height = 0.055;

    // 2. 严格按你旧代码的 FLU 坐标系定义装甲板四个角 (左上, 左下, 右下, 右上)
    // 注意是站在【有数字那一面】向车心看！x=0
    vertice_world_.push_back(cv::Point3f(0.00, +armorplate_width/2, +armorplate_height/2)); // 左上
    vertice_world_.push_back(cv::Point3f(0.00, +armorplate_width/2, -armorplate_height/2)); // 左下
    vertice_world_.push_back(cv::Point3f(0.00, -armorplate_width/2, -armorplate_height/2)); // 右下
    vertice_world_.push_back(cv::Point3f(0.00, -armorplate_width/2, +armorplate_height/2)); // 右上

    // 3. 将 FLU 系的点，乘上 P 矩阵，转换到 OpenCV 系下供 PnP 使用
    for (const auto& pt : vertice_world_) 
    {
        Eigen::Vector3d pt_flu(pt.x, pt.y, pt.z);
        Eigen::Vector3d pt_cv = P_ * pt_flu; // P_cv = P * P_flu
        vertice_cv_.push_back(cv::Point3f(pt_cv(0), pt_cv(1), pt_cv(2)));
    }
}


void YoloArmor::perspectiveNPoint() 
{
    // 如果传入的角点数量不对，直接标记失败并返回
    if (corners_.size() != 4 || vertice_cv_.size() != 4) 
    {
        pnp_success_ = false;
        return;
    }

    // 1. 执行 OpenCV 的 solvePnP (极高精度的 2D 物理角点，直接 IPPE)
    pnp_success_ = cv::solvePnP(vertice_cv_, corners_, K_, D_, r_cv_, t_cv_, false, cv::SOLVEPNP_IPPE);

    if (!pnp_success_) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("YoloArmor"), "[PNP] PNP 解算失败! ");
        return;
    }

    // ==========================================================
    // 2. 核心魔法：将 OpenCV 坐标系还原回 FLU 右手系！
    // ==========================================================
    
    // (A) 处理平移向量 t
    Eigen::Vector3d t_cv_eigen(t_cv_.at<double>(0, 0), t_cv_.at<double>(1, 0), t_cv_.at<double>(2, 0));
    t_vec_ = P_.transpose() * t_cv_eigen; // t_flu = P^(-1) * t_cv

    // (B) 处理旋转矩阵 R
    cv::Mat R_cv_mat;
    cv::Rodrigues(r_cv_, R_cv_mat); // 先把 OpenCV系 下的 旋转向量 r_cv 转成 旋转矩阵 R_cv_mat
    Eigen::Matrix3d R_cv_eigen;
    cv::cv2eigen(R_cv_mat, R_cv_eigen);
    R_ = P_.transpose() * R_cv_eigen * P_; // R_flu = P^(-1) * R_cv * P

    // ==========================================================
    // 3. 供人类调试和下位机使用的极坐标 (Yaw, Pitch, Distance)
    // ==========================================================
    double X = t_vec_(0); // 向前
    double Y = t_vec_(1); // 向左
    double Z = t_vec_(2); // 向上

    t_distance_ = std::sqrt(X * X + Y * Y + Z * Z);

    // atan2(y, x) 严格遵循你的原始逻辑
    // Z 为正表示在上方，此时 pitch 需要为负 (下位机协议)，所以加负号
    // Y 为正表示在左方，此时 yaw 为正 (下位机协议)，不需要加负号
    t_pitch_ = -std::atan2(Z, std::sqrt(X * X + Y * Y)) * 180.0 / CV_PI;
    t_yaw_ = std::atan2(Y, X) * 180.0 / CV_PI;
}


void YoloArmor::DrawAndPrintInfo(cv::Mat& img_show)
{
    // 1. 画框与四个角点
    cv::Scalar edge_color = cv::Scalar(235, 206, 135);
    for (int i = 0; i < 4; i++) 
    {
        cv::line(img_show, corners_[i], corners_[(i + 1) % 4], edge_color, 2);
        // 画角点序号，方便验证顺序 (0左上, 1左下, 2右下, 3右上)
        cv::putText(img_show, std::to_string(i), corners_[i], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 128, 255), 2);
    }

    // 2. 找中心点
    cv::Point2f center((corners_[0].x + corners_[2].x)/2, (corners_[0].y + corners_[2].y)/2);
    cv::circle(img_show, center, 3, cv::Scalar(0, 0, 255), cv::FILLED);

    // 3. 打印 PnP 的信息 (距离、Pitch、Yaw) 在角点旁边
    std::string info_dist = "Dist: " + std::to_string(t_distance_).substr(0, 4) + "m";
    std::string info_pitch = "Pitch: " + std::to_string(t_pitch_).substr(0, 5);
    std::string info_yaw = "Yaw: " + std::to_string(t_yaw_).substr(0, 5);

    cv::putText(img_show, info_dist, cv::Point2f(box_.x, box_.y - 55), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(img_show, info_pitch, cv::Point2f(box_.x, box_.y - 35), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(img_show, info_yaw, cv::Point2f(box_.x, box_.y - 15), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    // 4. 打印装甲板 ID 和类型
    std::string type_str = is_big_ ? "BIG" : "SMALL";
    int color_str = color_;
    cv::putText(img_show, "ID:" + std::to_string(armor_id_) + " " + type_str, cv::Point2f(box_.x, box_.y + box_.height + 20), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    cv::putText(img_show, "COLOR:" + std::to_string(color_str), cv::Point2f(box_.x, box_.y + box_.height + 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
}