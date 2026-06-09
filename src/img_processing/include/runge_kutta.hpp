/**
 * @file runge_kutta.hpp
 * @brief 带空气阻力弹道的仰角求解器（基于 RK4 数值积分 + 二分法）
 * 
 * 用途：给定目标点（水平距离 R，高度 z_target），求解弹丸出射的 pitch 角（弧度），
 *       使弹道恰好经过该点。考虑重力和与速度平方成正比的空气阻力。
 * 
 * 原理：
 *   1. 弹道建模为一维常微分方程初值问题，状态变量为 (x, z, vx, vz)
 *   2. 使用四阶 Runge-Kutta 法数值积分，获得弹丸飞行轨迹
 *   3. 在水平距离达到目标距离时进行线性插值，得到该距离处的精确高度
 *   4. 使用二分法求解使得该高度等于目标高度的仰角
 */

#pragma once

#include <cmath>
#include <utility>   // for std::pair

#include "tools/math_tools.hpp"

class RungeKutta 
{
public:

    RungeKutta();

    /**
     * @brief 构造弹道解算器
     * @param k   空气阻力系数 (1/m)  = (ρ·Cd·A) / (2·m)
     * @param v0  弹丸初速 (m/s)
     * @param g   重力加速度 (m/s²)
     */
    RungeKutta(double k, double v0, double g);

    /**
     * @brief 运行时更新初速（可用于电控反馈或在线标定）
     * @param v0 新的初速 (m/s)
     */
    void set_v0(double v0);

    /**
     * @brief 运行时更新阻力系数（可用于在线辨识）
     * @param k 新的阻力系数 (1/m)
     */
    void set_k(double k);

    /**
     * @brief 主要接口：求解最优 pitch 角（弧度）
     * @param target_dist 目标点的水平距离 R = sqrt(x² + y²) (m)
     * @param target_z    目标点的高度 (m) （父坐标系下，注意可能与枪口有偏移）
     * @return 最优 pitch 角（弧度），若不可达（超出最大射程）则返回 NaN
     */
    double SolvePitch(double target_dist, double target_z);

    /**
     * @brief 三维瞄准接口：直接根据目标点坐标（枪口坐标系下）解算 pitch 和 yaw
     * @param x 目标点 X 坐标（向前，m）
     * @param y 目标点 Y 坐标（向左为正，m）
     * @param z 目标点 Z 坐标（向下为正，m）
     * @return std::pair<double, double>  first = pitch (rad), second = yaw (rad)
     *         pitch 方向：向下为正，向上为负
     *         yaw 方向：向左为正，向右为负
     */
    std::pair<double, double> SolveAim(double x, double y, double z);

private:

    /**
     * @brief 弹道状态结构体（平面内）
     * x   : 水平位移 (m)
     * z   : 垂直高度 (m)   (Z 轴向上)
     * vx  : 水平速度 (m/s)
     * vz  : 垂直速度 (m/s)
     */
    struct State 
    {
        double x, z, vx, vz;
    };

    // 计算状态 s 的导数（微分方程右端项）
    State ComputeDerivative(const State& s) const;

    /**
     * @brief 四阶 Runge-Kutta 单步积分
     * @param s  当前状态
     * @param dt 积分步长 (s)
     * @return   积分 dt 后的新状态
     * 
     * 微分方程：
     *   dx/dt = vx
     *   dz/dt = vz
     *   dvx/dt = -k * v * vx
     *   dvz/dt = -g - k * v * vz
     *   其中 v = sqrt(vx² + vz²)
     */
    State rk4_step(const State& s, double dt) const;

    /**
     * @brief 模拟弹道，返回到达目标水平距离时的高度
     * @param pitch_rad    发射仰角 (弧度)
     * @param target_dist  目标水平距离 (m)
     * @return             该仰角下，弹丸飞至 x = target_dist 时的高度 (m)
     *                     若弹丸在到达该水平距离前已落地，返回一个很大的负数（-1e9）
     * 
     * 实现细节：
     *   - 积分步长固定为 0.005 s（可调），兼顾精度与速度
     *   - 使用线性插值获得精确 target_dist 处的高度，避免步长导致的系统误差
     *   - 落地检测：当 z < 0 且 vz < 0 时提前终止
     */
    double SimulateHeight(double pitch_rad, double target_dist) const;

    double k_ = 0.0109;       // 空气阻力系数 (1/m)，值越大阻力越强
    double v0_ = 25.0;        // 弹丸初速 (m/s)
    double g_ = 9.80665;      // 重力加速度 (m/s²)

    double dt_ = 0.004;       // 积分步长 4 ms（可调，建议 0.002~0.01）
    int max_steps_ = 5000;    // 防止无限循环（对应最大飞行时间 25s）
    int iterator_num_ = 25;   // 二分法迭代次数
};