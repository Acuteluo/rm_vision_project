#ifndef RM_CONSTANTS_HPP
#define RM_CONSTANTS_HPP

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <cmath>

namespace rm_constants
{

// ==================== Armor Plate Dimensions (meters) ====================

constexpr double NORMAL_ARMOR_WIDTH  = 0.135;
constexpr double HERO_ARMOR_WIDTH    = 0.225;
constexpr double ARMOR_HEIGHT        = 0.055;

inline double getArmorWidth(bool is_big)
{
    return is_big ? HERO_ARMOR_WIDTH : NORMAL_ARMOR_WIDTH;
}

inline double getArmorWidth(const std::string& armor_type)
{
    return (armor_type == "normal") ? NORMAL_ARMOR_WIDTH : HERO_ARMOR_WIDTH;
}

// ==================== Fixed Armor Pitch (radians) ====================

constexpr double FIXED_ARMOR_PITCH_RAD = 15.0 * M_PI / 180.0;

// ==================== FLU <-> OpenCV Coordinate Transform ====================

// P transforms FLU (x-front, y-left, z-up) to OpenCV camera (x-right, y-down, z-front)
// Usage: p_cv = P_FLU_TO_CV * p_flu
//        p_flu = P_FLU_TO_CV.transpose() * p_cv   (P is orthonormal, so P^-1 = P^T)
inline const Eigen::Matrix3d& P_FLU_TO_CV()
{
    static const Eigen::Matrix3d P = (Eigen::Matrix3d() <<
        0, -1,  0,
        0,  0, -1,
        1,  0,  0).finished();
    return P;
}

// ==================== Camera Intrinsics ====================

inline cv::Mat getMindVisionK()
{
    return (cv::Mat_<double>(3, 3) <<
        1359.21385,    0.0,        635.62767,
           0.0,     1361.75423,    478.48483,
           0.0,        0.0,          1.0);
}

inline cv::Mat getMindVisionD()
{
    return (cv::Mat_<double>(5, 1) << -0.081521, 0.153947, -0.006919, -0.003306, 0.000000);
}

inline cv::Mat getGalaxyK()
{
    return (cv::Mat_<double>(3, 3) <<
        1305.07013,    0.0,        666.71169,
           0.0,     1307.67428,    495.17044,
           0.0,        0.0,          1.0);
}

inline cv::Mat getGalaxyD()
{
    return (cv::Mat_<double>(5, 1) << -0.209067, 0.129977, -0.002895, -0.000558, 0.000000);
}

inline void getCameraIntrinsics(const std::string& camera_name, cv::Mat& K, cv::Mat& D)
{
    if (camera_name == "mind_vision") {
        K = getMindVisionK();
        D = getMindVisionD();
    } else {
        K = getGalaxyK();
        D = getGalaxyD();
    }
}

}  // namespace rm_constants

#endif // RM_CONSTANTS_HPP
