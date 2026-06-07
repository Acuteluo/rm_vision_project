#pragma once

#include <cmath>

namespace tools
{
    /**
     * @brief 极速角度归一化，将任意弧度值限制到 [-pi, pi] 之间
     * @note  基于 IEEE 754 的 remainder 实现，零分支开销，性能极高
     * @param angle_rad 输入的角度（弧度制）
     * @return double 归一化后的角度（弧度制）
     */
    inline double limit_rad(double angle_rad)
    {
        return std::remainder(angle_rad, 2.0 * M_PI);
    }

    /**
     * @brief 原地归一化重载版本
     */
    inline void limit_rad_inplace(double& angle_rad)
    {
        angle_rad = std::remainder(angle_rad, 2.0 * M_PI);
    }

    /**
     * @brief 角度转弧度
     */
    inline double deg2rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    /**
     * @brief 弧度转角度
     */
    inline double rad2deg(double rad)
    {
        return rad * 180.0 / M_PI;
    }
} // namespace tools