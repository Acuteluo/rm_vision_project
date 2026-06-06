#ifndef RM_UTILS_HPP
#define RM_UTILS_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <vector>
#include <cmath>

#include "rm_constants.hpp"

namespace rm_utils
{

// ==================== Angle Normalization ====================

inline double normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

inline void normalizeAngleRef(double& angle)
{
    angle = std::atan2(std::sin(angle), std::cos(angle));
}

// ==================== Spherical Coordinates from 3D Point ====================

struct SphericalCoord
{
    double pitch;     // degrees
    double yaw;       // degrees
    double distance;  // meters
};

inline SphericalCoord cartesianToSpherical(double x, double y, double z)
{
    SphericalCoord sc;
    sc.distance = std::sqrt(x * x + y * y + z * z);
    sc.pitch = -std::atan2(z, std::sqrt(x * x + y * y)) * 180.0 / M_PI;
    sc.yaw = std::atan2(y, x) * 180.0 / M_PI;
    return sc;
}

inline SphericalCoord cartesianToSpherical(const Eigen::Vector3d& v)
{
    return cartesianToSpherical(v(0), v(1), v(2));
}

// ==================== cv::Mat <-> Eigen::Matrix3d Conversion ====================

inline Eigen::Matrix3d cvMatToEigen3d(const cv::Mat& mat)
{
    Eigen::Matrix3d eigen_mat;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            eigen_mat(i, j) = mat.at<double>(i, j);
    return eigen_mat;
}

inline cv::Mat eigen3dToCvMat(const Eigen::Matrix3d& eigen_mat)
{
    cv::Mat mat(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            mat.at<double>(i, j) = eigen_mat(i, j);
    return mat;
}

// ==================== FLU <-> OpenCV Vertex Conversion ====================

inline std::vector<cv::Point3f> fluVerticesToCv(const std::vector<cv::Point3f>& flu_vertices)
{
    const auto& P = rm_constants::P_FLU_TO_CV();
    std::vector<cv::Point3f> cv_vertices;
    cv_vertices.reserve(flu_vertices.size());
    for (const auto& pt : flu_vertices) {
        Eigen::Vector3d pt_flu(pt.x, pt.y, pt.z);
        Eigen::Vector3d pt_cv = P * pt_flu;
        cv_vertices.emplace_back(pt_cv(0), pt_cv(1), pt_cv(2));
    }
    return cv_vertices;
}

// ==================== Armor Plate World Vertices (FLU) ====================

inline std::vector<cv::Point3f> buildArmorVerticesFLU(double width, double height)
{
    double hw = width / 2.0;
    double hh = height / 2.0;
    return {
        cv::Point3f(0.0f, +hw, +hh),   // top-left
        cv::Point3f(0.0f, +hw, -hh),   // bottom-left
        cv::Point3f(0.0f, -hw, -hh),   // bottom-right
        cv::Point3f(0.0f, -hw, +hh)    // top-right
    };
}

// ==================== PnP Result: OpenCV -> FLU Inversion ====================

struct PnPResultFLU
{
    Eigen::Vector3d t_vec;  // translation in FLU
    Eigen::Matrix3d R;      // rotation in FLU
};

inline PnPResultFLU pnpCvToFLU(const cv::Mat& r_cv, const cv::Mat& t_cv)
{
    const auto& P = rm_constants::P_FLU_TO_CV();
    PnPResultFLU result;

    // t_flu = P^T * t_cv
    Eigen::Vector3d t_cv_eigen(t_cv.at<double>(0, 0), t_cv.at<double>(1, 0), t_cv.at<double>(2, 0));
    result.t_vec = P.transpose() * t_cv_eigen;

    // R_flu = P^T * R_cv * P
    cv::Mat R_cv_mat;
    cv::Rodrigues(r_cv, R_cv_mat);
    Eigen::Matrix3d R_cv_eigen;
    cv::cv2eigen(R_cv_mat, R_cv_eigen);
    result.R = P.transpose() * R_cv_eigen * P;

    return result;
}

// ==================== Euler Rotation Matrix Builder (FLU: Yaw-Pitch-Roll) ====================

inline Eigen::Matrix3d buildRotationFLU(double yaw, double pitch, double roll = 0.0)
{
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    return (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
}

// ==================== FLU Rotation -> OpenCV rvec ====================

inline cv::Mat fluRotationToCvRvec(const Eigen::Matrix3d& R_flu)
{
    const auto& P = rm_constants::P_FLU_TO_CV();
    Eigen::Matrix3d R_cv_eigen = P * R_flu * P.transpose();
    cv::Mat R_cv_mat = eigen3dToCvMat(R_cv_eigen);
    cv::Mat rvec;
    cv::Rodrigues(R_cv_mat, rvec);
    return rvec;
}

}  // namespace rm_utils

#endif // RM_UTILS_HPP
