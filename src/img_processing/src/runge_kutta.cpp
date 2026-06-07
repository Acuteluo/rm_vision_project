/**
 * @file runge_kutta.cpp
 * @brief 弹道解算器的实现
 */

#include "runge_kutta.hpp"
#include <cmath>

RungeKutta::RungeKutta()
{
    
}


RungeKutta::RungeKutta(double k, double v0, double g)
    : k_(k), v0_(v0), g_(g) 
{

}

// 计算状态 s 的导数（微分方程右端项）
// 给定状态 s，计算 dx/dt_, dz/dt_, dvx/dt_, dvz/dt_
RungeKutta::State RungeKutta::ComputeDerivative(const State& s) const 
{
    double speed = std::sqrt(s.vx * s.vx + s.vz * s.vz);   // 合速度大小 v

    State d;
    d.x  = s.vx;                          // dx/dt_ = vx 水平位移变化率 = 水平速度
    d.z  = s.vz;                          // dz/dt_ = vz 垂直位移变化率 = 垂直速度
    d.vx = -k_ * speed * s.vx;            // dvx/dt_ = -k * v * vx 水平加速度 = 阻力引起的减速
    d.vz = -g_ - k_ * speed * s.vz;       // dvz/dt_ = -g - k * v * vz 垂直加速度 = 重力 + 阻力
    return d;
}


RungeKutta::State RungeKutta::rk4_step(const State& s, double dt_) const 
{
    // RK4 标准流程：计算四个 k 值
    State k1 = ComputeDerivative(s);
    State k2 = ComputeDerivative({
        s.x + k1.x * dt_ / 2.0,
        s.z + k1.z * dt_ / 2.0,
        s.vx + k1.vx * dt_ / 2.0,
        s.vz + k1.vz * dt_ / 2.0
    });
    State k3 = ComputeDerivative({
        s.x + k2.x * dt_ / 2.0,
        s.z + k2.z * dt_ / 2.0,
        s.vx + k2.vx * dt_ / 2.0,
        s.vz + k2.vz * dt_ / 2.0
    });
    State k4 = ComputeDerivative({
        s.x + k3.x * dt_,
        s.z + k3.z * dt_,
        s.vx + k3.vx * dt_,
        s.vz + k3.vz * dt_
    });

    // 加权平均更新状态
    State next;
    next.x  = s.x  + (k1.x  + 2.0*k2.x  + 2.0*k3.x  + k4.x)  * dt_ / 6.0;
    next.z  = s.z  + (k1.z  + 2.0*k2.z  + 2.0*k3.z  + k4.z)  * dt_ / 6.0;
    next.vx = s.vx + (k1.vx + 2.0*k2.vx + 2.0*k3.vx + k4.vx) * dt_ / 6.0;
    next.vz = s.vz + (k1.vz + 2.0*k2.vz + 2.0*k3.vz + k4.vz) * dt_ / 6.0;
    return next;
}

double RungeKutta::SimulateHeight(double pitch_rad, double target_dist) const 
{
    // 初始状态：从原点 (0,0) 发射，速度为 (v0*cosθ, v0*sinθ)
    State s
    {
        0.0,
        0.0,
        v0_ * std::cos(pitch_rad),
        v0_ * std::sin(pitch_rad)
    };
    
    State prev = s;                // 记录上一步的状态，用于插值

    int step = 0;

    // 逐步积分，直到水平位移超过目标距离
    while (s.x < target_dist && step++ < max_steps_) 
    {
        prev = s;               // 保存步进前的状态
        s = rk4_step(s, dt_);    // 前进一步

        // 落地检测：高度低于地面且正在下降（vz < 0）
        if (s.z < 0.0 && s.vz < 0.0) 
        {
            // 弹丸已落地，且尚未达到目标水平距离，说明该仰角无法到达该距离
            // 返回一个很大的负数，让二分法认为“落点太低，需要增大仰角”
            return -1e9;
        }
    }

    // 如果是因为步数超限退出（极罕见，说明 target_dist 极大或步长太小）
    if (step >= max_steps_) 
    {
        // 此时返回当前高度，但通常应视为不可达（返回极小值）
        return -1e9;
    }

    // 线性插值：在 prev 和 s 之间找到恰好水平距离 = target_dist 的点
    // 假设在最后一步中，水平位移和高度变化是线性的（步长足够小时近似成立）
    double t = (target_dist - prev.x) / (s.x - prev.x);   // 插值系数 [0,1]
    double z_at_target = prev.z + t * (s.z - prev.z);     // 插值得到的高度
    return z_at_target;
}

double RungeKutta::SolvePitch(double target_dist, double target_z) 
{
    // ========== 1. 搜索区间初始化 ==========
    // 物理合理的仰角范围：-20° ~ +55°
    double low  = tools::deg2rad(-20.00);   
    double high = tools::deg2rad(55.00);    
    // 若目标距离为 0（理论上不会发生），直接返回 0
    if (target_dist <= 0.0) return 0.0;

    // ========== 2. 动态扩展 high 边界 ==========
    // 如果当前 high 对应的弹道高度仍然低于目标高度，说明仰角不够大，
    // 需要将 high 向上扩展（最多到 70°）。
    // 最多尝试 5 次，每次增加 0.1 rad (~5.7°)
    for (int i = 0; i < 5; i++) 
    {
        if (SimulateHeight(high, target_dist) < target_z) 
        {
            high += 0.1;
        } 
        else 
        {
            break;
        }
    }

    // 如果 high 扩展后仍然无法达到目标高度，则认为目标点超出最大射程，
    // 直接返回最大射程对应的仰角（通常约为 0.9 rad / 52°）。
    if (SimulateHeight(high, target_dist) < target_z) 
    {
        // 可选：记录警告日志
        return high;   // 返回边界值，至少不会让二分法崩溃
    }

    // ========== 3. 二分法精确求解 ==========
    // 迭代 25 次，精度约为 (high-low)/2^25 ≈ 1e-7 rad，远高于实际需求
    for (int iter = 0; iter < 25; iter++) 
    {
        double mid = (low + high) * 0.5;
        double z_mid = SimulateHeight(mid, target_dist);

        // 如果模拟得到的高度与目标高度之差小于 1 cm，认为已足够精确
        if (std::abs(z_mid - target_z) < 0.01) 
        {
            return mid;
        }

        // 如果模拟高度低于目标高度，说明仰角偏小，增加下界
        if (z_mid < target_z) 
        {
            low = mid;
        } 
        else 
        {
            high = mid;
        }
    }

    // 迭代结束，返回区间中点作为最终结果
    return (low + high) * 0.5;
}


// 给 core_node 调用的主逻辑
std::pair<double, double> RungeKutta::SolveAim(double x, double y, double z_down_positive)
{
    double R = std::sqrt(x * x + y * y);
    double yaw_rad = std::atan2(y, x);   // 向左为正，向右为负

    // 转换为标准坐标系（向上为正）
    double z_up = -z_down_positive;

    double pitch_standard = 0.0;
    if (R < 1e-6)
    {
        // 无水平偏移：标准 pitch 直接由向上为正的高度决定
        pitch_standard = (z_up >= 0.0) ? (M_PI / 2.0) : (-M_PI / 2.0);
    }
    else
    {
        pitch_standard = SolvePitch(R, z_up);
    }

    return {-pitch_standard, yaw_rad};
}


void RungeKutta::set_v0(double v0) 
{ 
    v0_ = v0; 
}


void RungeKutta::set_k(double k) 
{ 
    k_ = k; 
}