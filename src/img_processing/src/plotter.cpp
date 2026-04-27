#include "plotter.hpp"

Plotter::Plotter(int width, int height, int max_frames)
    : width_(width), height_(height), max_frames_(max_frames) {}

void Plotter::updateAndDraw(double p_now, double y_now, double p_filt, double y_filt, double p_pred, double y_pred) 
{
    // 1. 数据进栈
    buf_p_now.push_back(p_now);   buf_y_now.push_back(y_now);
    buf_p_filt.push_back(p_filt); buf_y_filt.push_back(y_filt);
    buf_p_pred.push_back(p_pred); buf_y_pred.push_back(y_pred);

    // 2. 控制最大记忆帧数
    if (buf_p_now.size() > (size_t)max_frames_) {
        buf_p_now.pop_front();   buf_y_now.pop_front();
        buf_p_filt.pop_front();  buf_y_filt.pop_front();
        buf_p_pred.pop_front();  buf_y_pred.pop_front();
    }

    // 3. 画布初始化 (纯黑偏深灰的底色，增加对比度)
    cv::Mat plot_img(height_, width_, CV_8UC3, cv::Scalar(20, 20, 20));

    // 4. 配色方案调优
    cv::Scalar c_now(255, 255, 255);    // 观测: 纯白
    cv::Scalar c_filt(0, 255, 0);       // 滤波: 亮绿
    cv::Scalar c_pred(193, 182, 255);   // 预测: 樱花粉

    // 5. 区域切分
    int margin_x = 75, margin_y = 70, space_y = 50;
    int chart_w = width_ - margin_x * 2;
    int chart_h = (height_ - margin_y * 2 - space_y) / 2;

    cv::Rect rect_p(margin_x, margin_y, chart_w, chart_h);
    cv::Rect rect_y(margin_x, margin_y + chart_h + space_y, chart_w, chart_h);

    // 6. 顶部大标题与图例
    cv::putText(plot_img, "EKF Tracking Oscilloscope", cv::Point(margin_x, 35), cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(plot_img, "Observed (White)", cv::Point(width_/2 - 250, 35), cv::FONT_HERSHEY_SIMPLEX, 0.6, c_now, 1, cv::LINE_AA);
    cv::putText(plot_img, "Filtered (Green)", cv::Point(width_/2 - 50, 35), cv::FONT_HERSHEY_SIMPLEX, 0.6, c_filt, 1, cv::LINE_AA);
    cv::putText(plot_img, "Predicted (Pink)", cv::Point(width_/2 + 150, 35), cv::FONT_HERSHEY_SIMPLEX, 0.6, c_pred, 1, cv::LINE_AA);

    // 7. 执行画图
    drawChart(plot_img, rect_p, "Pitch Error (deg)", buf_p_now, buf_p_filt, buf_p_pred, c_now, c_filt, c_pred);
    drawChart(plot_img, rect_y, "Yaw Error (deg)", buf_y_now, buf_y_filt, buf_y_pred, c_now, c_filt, c_pred);

    cv::imshow("EKF Dynamic Plotter", plot_img);
    cv::waitKey(1);
}

void Plotter::drawChart(cv::Mat& img, cv::Rect rect, const std::string& title,
                        const std::deque<double>& d_now, const std::deque<double>& d_filt, const std::deque<double>& d_pred,
                        cv::Scalar c_now, cv::Scalar c_filt, cv::Scalar c_pred) 
{
    // 内部图表底色 (稍微亮一点点的深灰)
    cv::rectangle(img, rect, cv::Scalar(28, 28, 28), -1);
    cv::rectangle(img, rect, cv::Scalar(100, 100, 100), 1);
    
    // 图表标题
    cv::putText(img, title, cv::Point(rect.x + 10, rect.y + 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);

    if (d_now.empty()) return;

    // 1. 获取有效数据的极限值
    double min_v = 9999.0, max_v = -9999.0;
    bool has_valid = false;
    for (size_t i = 0; i < d_now.size(); ++i) {
        if (d_now[i] > -900.0)  { min_v = std::min(min_v, d_now[i]); max_v = std::max(max_v, d_now[i]); has_valid = true; }
        if (d_filt[i] > -900.0) { min_v = std::min(min_v, d_filt[i]); max_v = std::max(max_v, d_filt[i]); has_valid = true; }
        if (d_pred[i] > -900.0) { min_v = std::min(min_v, d_pred[i]); max_v = std::max(max_v, d_pred[i]); has_valid = true; }
    }

    if (!has_valid) { min_v = -5.0; max_v = 5.0; } // 防爆默认极值

    // 2. 扩大 Y 轴跨度，且保证即使静止也有起码的缩放范围 (防止微小噪点被放大成满屏)
    if (max_v - min_v < 2.0) {
        double mid = (max_v + min_v) / 2.0;
        min_v = mid - 1.0; max_v = mid + 1.0;
    }
    double padding = (max_v - min_v) * 0.15;
    min_v -= padding; max_v += padding;

    // 3. 画 8 格精细网格线
    int grid_count = 8;
    for (int i = 0; i <= grid_count; ++i) {
        int y = rect.y + i * rect.height / grid_count;
        double val = max_v - (max_v - min_v) * i / (double)grid_count;
        
        // 刻度线
        if (i > 0 && i < grid_count) {
            cv::line(img, cv::Point(rect.x, y), cv::Point(rect.x + rect.width, y), cv::Scalar(50, 50, 50), 1, cv::LINE_AA);
        }
        // 刻度字
        cv::putText(img, cv::format("%+.2f", val), cv::Point(rect.x - 65, y + 5), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
    }

    // 4. 特殊高亮 0 度基准线 (如果 0 度在当前视野范围内)
    if (min_v < 0.0 && max_v > 0.0) {
        int zero_y = rect.y + rect.height - (0.0 - min_v) / (max_v - min_v) * rect.height;
        cv::line(img, cv::Point(rect.x, zero_y), cv::Point(rect.x + rect.width, zero_y), cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    }

    // 5. 映射比例尺
    double dx = (double)rect.width / max_frames_;
    double dy = (double)rect.height / (max_v - min_v);

    // 6. 画线 Lambda：线宽改成了 1，抗锯齿开启，极度清晰
    auto drawLine = [&](const std::deque<double>& data, cv::Scalar color) {
        for (size_t i = 1; i < data.size(); ++i) {
            if (data[i] < -900.0 || data[i - 1] < -900.0) continue; 
            int x1 = rect.x + (i - 1) * dx;
            int y1 = rect.y + rect.height - (data[i - 1] - min_v) * dy;
            int x2 = rect.x + i * dx;
            int y2 = rect.y + rect.height - (data[i] - min_v) * dy;
            // 【重点优化】：thickness = 1，线不再是一坨
            cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color, 1, cv::LINE_AA);
        }
    };

    // 按层级渲染：樱花粉放最底层，白线观测最上层
    drawLine(d_pred, c_pred); 
    drawLine(d_filt, c_filt); 
    drawLine(d_now, c_now);   

    // ==========================================================
    // 【王炸体验】：在右上角实时打印最新的浮点数值，丢失则显示 [LOST]
    // ==========================================================
    auto fmtVal = [](double v) { return (v < -900.0) ? "[LOST]" : cv::format("%+.2f", v); };
    std::string s_now = "Obs: " + fmtVal(d_now.back());
    std::string s_fil = "Filt: " + fmtVal(d_filt.back());
    std::string s_pre = "Pred: " + fmtVal(d_pred.back());

    int text_x = rect.x + rect.width - 320;
    int text_y = rect.y + 25;
    cv::putText(img, s_now, cv::Point(text_x, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, c_now, 1, cv::LINE_AA);
    cv::putText(img, s_fil, cv::Point(text_x + 110, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, c_filt, 1, cv::LINE_AA);
    cv::putText(img, s_pre, cv::Point(text_x + 220, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, c_pred, 1, cv::LINE_AA);
}