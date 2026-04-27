#ifndef PLOTTER_HPP
#define PLOTTER_HPP

#include <opencv2/opencv.hpp>
#include <deque>
#include <string>
#include <algorithm>

class Plotter {
public:
    // 构造函数：宽、高、保存的最大帧数
    Plotter(int width = 1200, int height = 800, int max_frames = 1000);

    // 输入接口：-999 代表无效值，不画线。core_node 调用方式保持不变！
    void updateAndDraw(double p_now, double y_now, double p_filt, double y_filt, double p_pred, double y_pred);

private:
    int width_;
    int height_;
    int max_frames_;

    std::deque<double> buf_p_now, buf_p_filt, buf_p_pred;
    std::deque<double> buf_y_now, buf_y_filt, buf_y_pred;

    void drawChart(cv::Mat& img, cv::Rect rect, const std::string& title,
                   const std::deque<double>& d_now, const std::deque<double>& d_filt, const std::deque<double>& d_pred,
                   cv::Scalar c_now, cv::Scalar c_filt, cv::Scalar c_pred);
};

#endif // PLOTTER_HPP