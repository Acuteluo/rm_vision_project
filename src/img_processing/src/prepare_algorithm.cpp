#include "prepare_algorithm.h"


/**
* @brief	灯条从左到右排序 排序规则函数
* @param	const Strip& a, const Strip& b
* @return   0 1
*/
bool Prepare::sortStripByX(const Strip& a, const Strip& b)
{
	return a.center.x < b.center.x;
}




////////////////////////////////////  存储 config 读取的参数  //////////////////////////////////////

void Prepare::setParam(bool show_logger, std::string chosen_color, std::string camera_name, std::string armor_type)
{
    this->SHOW_LOGGER = show_logger;
    this->CHOSEN_COLOR = chosen_color;
    this->CAMERA_NAME = camera_name;
    this->ARMOR_TYPE = armor_type;
}

//////////////////////////////////   设置 和 获取 img_show   //////////////////////////////////

void Prepare::setImgShow(cv::Mat& img_show)
{
    this->img_show = img_show; 
}


cv::Mat Prepare::getImgShow()
{
    return this->img_show;
}




/////////////////////////////////////////////  灯条配对成装甲板函数  /////////////////////////////////////////////

/**
* @brief	灯带配对
* @return   std::vector<ArmorPlate> 【修改】返回按置信度排序后的装甲板集合
*/
std::vector<ArmorPlate> Prepare::pairStrip()
{
	std::vector<ArmorPlate> armorplate; // 装甲板集合
	std::vector<std::vector<double>> moderation(strip.size(), std::vector<double>(strip.size(), 0.00)); // [编号][编号]，合理性
	std::vector<int> moderation_number(strip.size(), -1); // 对于当前灯条，最合理的是谁（ -1 表示没找到目标 ）



    // ------- 1. 灯条从左到右排序 -------  
	std::sort(strip.begin(), strip.end(), sortStripByX); // 从左到右排序


	// ------- 2. 灯条遍历，两两配对，计算合理性 moderation -------
	for (int i = 0; i < strip.size(); i++)
	{
		for (int j = i; j < strip.size(); j++) // 算过的就不用算了，也确保 i 灯条在左边，j 灯条在右边
		{
			if (i == j || moderation[j][i] != 0) continue; // 跳过自己 和 i -> j = j -> i


            ///////////// 合理性评判条件 /////////////

			bool angle_moderation = true; // [两个灯条是否平行]
			bool distance_long_moderation = true; // [灯条间距离是否合理，不能太长]（必要）
            bool distance_short_moderation = true; // [灯条间距离是否合理，不能太短]（必要）
            bool length_moderation = true; // [灯条间的选取是否合理]
            bool connecting_line_moderation = true; // [避免选到共线的灯条]（必要）
            bool no_inner_strip_moderation = true; // [两个灯条构成的装甲板内，不应该有其他灯条]（必要）

			double temp_moderation = 0.00; // 当前灯条 i 与 灯条 j 间的合理性



			// 2.1. 两个灯条是否平行

            const double MAX_ANGLE = 25.00; // 两个灯条最大差角 -> [两个灯条是否平行]

            double angle_diff = std::fabs(strip[i].angle - strip[j].angle); // 两个灯条的角度差
			if (angle_diff > MAX_ANGLE)
			{
				// 不平行
                RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 【不】平行, angle_diff = %.2f > %.2f", i, j, angle_diff, MAX_ANGLE);
				
                angle_moderation = false;
			}
			else
			{   
				// 平行
                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d      平行, angle_diff = %.2f < %.2f", i, j, angle_diff,  MAX_ANGLE);
				
                double score1 = (MAX_ANGLE - angle_diff) / MAX_ANGLE * 25.00; // 和最优的差距，最优时差距为0，得分为25
                temp_moderation += score1;

                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 角度差得分, = %.2f", i, j, score1);
            }   




            // 2.2 两个灯条构成的装甲板 宽高比例是否合理

            double dx = strip[i].center.x - strip[j].center.x;
            double dy = strip[i].center.y - strip[j].center.y;
            double distance = std::sqrt(dx * dx + dy * dy); // 两个灯条中心点距离（装甲板长边）

            double strip_longest = std::max(strip[i].height, strip[j].height); // 两个灯条里长度 长 的那个
            double strip_shortest = std::min(strip[i].height, strip[j].height); // 两个灯条里长度 短 的那个
            double strip_average = (strip_longest + strip_shortest) / 2.00; // 两个灯条长度的平均值（装甲板短边）

            double armorplate_ratio = distance / strip_average; // 实际测量的装甲板比例
			

            // 定义物理标准比例与容忍度 
            const double IDEAL_NORMAL_RATIO = 135.0 / 55.0; // 步兵理想比例 约 2.45
            const double IDEAL_HERO_RATIO = 230.0 / 55.0;   // 英雄理想比例 约 4.18
            const double MAX_DEVIATION_LONG = 0.30;         //（长装甲板）最大允许偏差 30% (最多1.3倍，再长则为0分)
            const double MIN_DEVIATION_SHORT = -0.85;       //（短装甲板）最大允许偏差 85% (最多0.15倍，再短则为0分)

            // 根据类型获取当前目标的理想比例
            double ideal_ratio = (this->ARMOR_TYPE == "hero") ? IDEAL_HERO_RATIO : IDEAL_NORMAL_RATIO;

            // 计算相对偏差: (实际 - 理想) / 理想，包括正负号
            double relative_deviation = (armorplate_ratio - ideal_ratio) / ideal_ratio;

            // 判断与打分
            // 偏差超出最大极限
            if (relative_deviation > MAX_DEVIATION_LONG || relative_deviation < MIN_DEVIATION_SHORT)
            {
                // 看看宽高比是太大（隔得太远）还是太小（隔得非常近）
                if (armorplate_ratio < ideal_ratio) 
                {
                    RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和 %d 比例异常(太窄): 实际 %.2f, 理想 %.2f, 偏差 %.0f%%", i, j, armorplate_ratio, ideal_ratio, relative_deviation * 100);
                    distance_short_moderation = false;
                }
                else 
                {
                    RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和 %d 比例异常(太宽): 实际 %.2f, 理想 %.2f, 偏差 %.0f%%", i, j, armorplate_ratio, ideal_ratio, relative_deviation * 100);
                    distance_long_moderation = false;
                }
            }
            else
            {
                // 比例在合理范围内，计算这 25 分的得分
                // 偏差越小得分越高：0 偏差 = 25分，MAX_DEVIATION 偏差 = 0分
                // 由于非对称，所以得分情况讨论
                double score2 = 0.00;
                if(armorplate_ratio < ideal_ratio)
                {
                    // 两个都是负数，不用加 abs
                    score2 = (1.00 - (relative_deviation / MIN_DEVIATION_SHORT)) * 25.00;
                }
                else
                {
                    score2 = (1.00 - (relative_deviation / MAX_DEVIATION_LONG)) * 25.00;
                }
                
                temp_moderation += score2;

                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和 %d 比例合理: 实际 %.2f, 理想 %.2f, 获得分数: %.2f / 25.00", i, j, armorplate_ratio, ideal_ratio, score2);
            }




			// 2.3. 两个灯条 短:长 的比值不能太小

            const double MIN_TWO_STRIP_RATIO = 0.5 / 1; // 两个灯条 短:长 的比值不能太小 -> [灯条间的选取是否合理]

            double two_strip_ratio = strip_shortest / strip_longest;
            if(two_strip_ratio < MIN_TWO_STRIP_RATIO)
            {
                // 某个灯条太短，两个灯条 短:长 的比值不合理
                RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 短:长 的比【不】合理, = %.2f < %.2f", i, j, two_strip_ratio, MIN_TWO_STRIP_RATIO);
				
                length_moderation = false;
            }
            else
			{
                // 比值合理
                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 短:长 的比     合理, = %.2f > %.2f", i, j, two_strip_ratio, MIN_TWO_STRIP_RATIO);
				
                double score3 = ((two_strip_ratio - MIN_TWO_STRIP_RATIO) / (1.00 - MIN_TWO_STRIP_RATIO)) * 25.00; // 和最优的差距，最优时 比例接近 1
                temp_moderation += score3; 
                
                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 短:长 的比得分, = %.2f", i, j, score3);
			}
			



            // 2.4. 两个灯条的连接线角度和灯条平均角度应接近垂直（不能太小），避免选到共线的灯条

            const double MIN_CONNECTING_LINE_COMPARE_STRIP_ANGLE = 55.00; // 连接线和灯条的角度差不能太小 -> [避免选到共线的灯条]

            // 已知：dir1, dir2（cv::Point2f，方向从顶端到底端，dy>0），p1, p2（左、右灯条中心点）
            cv::Point2f dir1 = strip[i].lower - strip[i].upper; // 灯条 i 的方向向量
            cv::Point2f dir2 = strip[j].lower - strip[j].upper; // 灯条 j 的方向向量

            // 1. 平均方向向量（和向量归一化）
            cv::Point2f avg_dir = dir1 + dir2;
            float norm_avg = sqrt(avg_dir.x*avg_dir.x + avg_dir.y*avg_dir.y);
            if (norm_avg > 1e-6) avg_dir /= norm_avg;   // 归一化，否则保持原样（但夹角会无效）

            // 2. 连线向量（左→右）
            cv::Point2f line_vec = strip[j].center - strip[i].center;
            float len_line = sqrt(line_vec.x*line_vec.x + line_vec.y*line_vec.y);

            // 3. 锐角夹角（度）
            float dot = avg_dir.x*line_vec.x + avg_dir.y*line_vec.y;
            float cos_angle = fabs(dot) / len_line;     // 取绝对值保证锐角
            cos_angle = std::min(1.0f, std::max(-1.0f, cos_angle));
            float angle_deg = acos(cos_angle) * 180.0f / CV_PI;

            if (angle_deg < MIN_CONNECTING_LINE_COMPARE_STRIP_ANGLE)
            {
                // 连接线和灯条的角度差太小，可能共线了
                RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 连接线和灯条的角度差【不】合理 可能共线, = %.2f < %.2f", i, j, angle_deg, MIN_CONNECTING_LINE_COMPARE_STRIP_ANGLE);
                
                connecting_line_moderation = false;
            }
            else
            {
                // 连接线和灯条的角度差合理
                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 连接线和灯条的角度差     合理, = %.2f > %.2f", i, j, angle_deg, MIN_CONNECTING_LINE_COMPARE_STRIP_ANGLE);
                
                double score4 = ((angle_deg -  MIN_CONNECTING_LINE_COMPARE_STRIP_ANGLE) / (90.00 - MIN_CONNECTING_LINE_COMPARE_STRIP_ANGLE)) * 25; 
                temp_moderation += score4;

                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 连接线和灯条的角度差得分, = %.2f", i, j, score4);
            }



            // 2.5. 两个灯条构成的装甲板内不应该有其他灯条（必要条件）

            for(int k = i + 1; k < j; k++) 
            {
                // 判断灯条 k 的中心点是否在灯条 i 和 j 构成的矩形区域内
                // 这里简化为判断是否在以 i 和 j 的中心点为对角线的矩形内（只看高低就好）
                double min_y = std::min(strip[i].center.y, strip[j].center.y);
                double max_y = std::max(strip[i].center.y, strip[j].center.y);

                if (strip[k].center.y > min_y && strip[k].center.y < max_y)
                {
                    // 灯条 k 的中心点在 i 和 j 构成的矩形内，说明有内灯条，不合理
                    RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 构成的装甲板内有其他灯条 %d, 绝对不合理！", i, j, k);
                    
                    no_inner_strip_moderation = false;
                    break; // 只要找到一个就够了，不需要继续检查了
                }
            }


			// 2.6. 评判
            int conditions_met = angle_moderation + length_moderation;
            bool necessary_conditions_met = no_inner_strip_moderation && connecting_line_moderation && distance_short_moderation && distance_long_moderation; // 中间不能有灯条，连接线合理、距离不能太短和太长 是必须的
			if (conditions_met >= 1 && necessary_conditions_met) 
			{
                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "除必要的四个条件外，条件满足数: %d / 2", conditions_met);
				
                // 更新 i 对 j 的最大合理性
				moderation[i][j] = std::max(temp_moderation, moderation[i][j]);
                moderation[j][i] = moderation[i][j]; // i -> j 的合理性 和 j -> i 的合理性 是一样的
			}

		}

	}


	// 3. 整理简化数据，明确每个灯条 最合理的配对对象 是谁，放入 moderation_number 数组中
    double MAX_DISTANCE = 200.00; // 距离上一次最佳装甲板中心的位置不能超过这个阈值，才算合理

	for (int i = 0; i < strip.size(); i++)
	{
		double max_moderation = 0.00; // 对于当前灯条，最大合理性
		int number = -1; // 对于当前灯条，最合理的是谁

		for (int j = 0; j < strip.size(); j++) 
		{
			if (i == j) continue; // 跳过 自己

            // 计算当前两个灯条构成的装甲板，和之前最佳装甲板中心的距离
            double center_x = (strip[i].center.x + strip[j].center.x) / 2.00;
            double center_y = (strip[i].center.y + strip[j].center.y) / 2.00;
            double dx = center_x - this->last_best_center.x;
            double dy = center_y - this->last_best_center.y;
            double distance_to_last_best = std::sqrt(dx * dx + dy * dy);

 
            // 根据上一次最佳装甲板中心位置确定合理性阈值，去掉合理性不高的
            // 如果上一帧有装甲板而且在附近，那就跟着就好了
            if(this->last_best_center != cv::Point2f(-1.00, -1.00) && distance_to_last_best <= MAX_DISTANCE)
            {
                if(moderation[i][j] <= 35.00) continue;
            }
            else
            {
                if(moderation[i][j] <= 50.00) continue;
            }


            // 引入综合得分机制
            double current_score = moderation[i][j];

            if (this->last_best_center != cv::Point2f(-1.00, -1.00)) 
            {
                // 必须在这里就加上距离惩罚，和 Step 5 保持同频！
                current_score -= distance_to_last_best * 0.1; 
            }

            // 比较的是 current_score 而不是原始的 moderation
            if (current_score > max_moderation) 
            {
                max_moderation = current_score; 
                number = j;
            }

            RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 与 灯条 %d 的 moderation 是 %.2f", i, j, moderation[i][j]);
		}

		moderation_number[i] = number;

        RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "\n--------------------------");  
        RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 的 max_moderation 是 %.2f", i, max_moderation);
        RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 的最佳配对是 灯条 %d", i, moderation_number[i]);
	}


	// 4. 放入装甲板集合中
    std::vector<bool> is_used(strip.size(), false); // 记录灯条是否已经被配对过了，避免重复配对，有多少个灯条就有多少个 bool

	for (int i = 0; i < strip.size(); i++)
	{
		//如果双向奔赴，那就是他俩了。当然不能同时奔赴 -1
		if (!is_used[i] && moderation_number[i] != -1 && moderation_number[moderation_number[i]] == i) // 最佳灯条是存在的灯条
		{
			armorplate.push_back(ArmorPlate(strip[i], strip[moderation_number[i]], moderation[i][moderation_number[i]], this->CAMERA_NAME, this->ARMOR_TYPE)); // 传入两个灯条 + 置信度 + 相机名称，构造成装甲板
            
            is_used[i] = true; // 记录灯条 i 已经被配对过了
            is_used[moderation_number[i]] = true; // 记录灯条 moderation_number[i] 已经被配对过了
		}
	}


    RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "\n--------------------------");  
    RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "共检测到 %d 个装甲板", armorplate.size());


    // 5. 对于每一个装甲板，不仅按照置信度排序，还按照距离排列得到跟踪目标
    std::sort(armorplate.begin(), armorplate.end(), 
    [this](const ArmorPlate& a, const ArmorPlate& b) 
    {
        // 如果系统刚刚启动，没有历史中心，按 moderation 排序
        if (this->last_best_center == cv::Point2f(-1.00, -1.00)) 
        {
            return a.moderation > b.moderation;
        }

        // 计算综合得分 (Score = 置信度 - 距离惩罚)
        
        // 算 A 的距离惩罚
        double dist_A = cv::norm(a.center - this->last_best_center);
        // 距离越远，扣分越多，控制最多扣 15 分。惩罚系数可以调，比如 0.1 意味着每偏离 10 像素扣 1 分
        double penalty_A = std::min(dist_A * 0.1, 15.0);
        double score_A = a.moderation - penalty_A; 

        // 算 B 的距离惩罚
        double dist_B = cv::norm(b.center - this->last_best_center);
        double penalty_B = std::min(dist_B * 0.1, 15.0);
        double score_B = b.moderation - penalty_B; 

        // 【可选：锁定奖励】如果你想极其死板地咬住老目标，甚至可以给距离近的额外加分
        // if (dist_A < 100) score_A += 20.0;
        // if (dist_B < 100) score_B += 20.0;

        return score_A > score_B; // 按综合得分降序排列
    }
);


    if(armorplate.size() == 0)
    {
        this->last_best_center = cv::Point2f(-1.00, -1.00);
    }
    else
    {
        this->last_best_center = armorplate[0].center; // 更新最佳装甲板中心位置为当前帧第一个装甲板（置信度最高的装甲板）的中心位置
    }

    return armorplate; 
}





/////////////////////////////////////////////  判断灯条函数  /////////////////////////////////////////////


/**
* @brief	寻找灯带 + 判断灯带颜色 + 存储灯带
* @return   std::vector<Strip> 返回灯条集合
*/
std::vector<Strip> Prepare::findAndJudgeLightStrip()
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(this->mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	std::vector<Strip> strip; // 灯条集合

    int sum = 0; // 目前检测到的候选灯带数量

	for (int i = 0; i < contours.size(); i++)
	{
		int area = cv::contourArea(contours[i]);
		if (area > 15) // 面积去除噪点
		{
			/*
				所以旋转矩形的angle和width height是这样定义的
				首先 angle范围理论上为(0, 90]
				然后，过旋转矩形的中心点作一条水平直线，现在顺时针旋转这条直线，当直线和矩形某一边平行时停止旋转，此时这条矩形平行边记为width，水平直线顺时针转过的角度就是angle，
				当然，归一化到(0, 90]
				但是0度 和 90度没法区分倒是
			*/

            // ------- 1. 获取参数 + 亚像素优化-------

			cv::RotatedRect temp_rotatedRect = cv::minAreaRect(contours[i]); // 先获得旋转矩形

            // 获取四个顶点（整数精度）
            cv::Point2f vertices[4];
            temp_rotatedRect.points(vertices);

            // 将顶点放入 vector 以便 cornerSubPix 处理
            std::vector<cv::Point2f> vec_vertices(vertices, vertices + 4);

			double angle = temp_rotatedRect.angle; // 获取角度

			cv::Size2d size = temp_rotatedRect.size;  // 尺寸
			double width = size.width; // 宽
			double height = size.height; // 高

            int corners_num = contours[i].size(); // 当前灯条角点数



			// ------- 2. 角度处理 -------

            // 确保 width 是短边，height 是长边
			if (width > height) 
			{
				angle -= 90.00; // 如果矩形是通过逆时针旋转得到的，那么让 angle 变成负的，这样 angle 范围 (-90, 90)
				std::swap(width, height); 
			}
			if (angle == 90.00) angle = 0.00; // 90度视为0度，确保 angle 连续



            // ------- 3. 高宽比处理 -------

			double ratio = height / width; 
            
            if (ratio < 1.50) continue; 
            
            // 统计：灯条 + 1
            ++sum;
           


            // ------- 4.1 区分颜色之边缘部分，计算边缘 BGR 和 saturation --------

            int saturation_edge_total = 0; //  总边缘饱和度
            int blue_edge_total = 0; //  总边缘蓝值
            int green_edge_total = 0; //  总边缘绿值
            int red_edge_total = 0; //  总边缘红值

            for(int j = 0; j < corners_num; j++)
            {
                // 注意行在前，列在后
                if (contours[i][j].x >= 0 && contours[i][j].x < img.cols && contours[i][j].y >= 0 && contours[i][j].y < img.rows)
                {
                    // bgr
                    cv::Vec3b bgr_value = this->img.at<cv::Vec3b>(contours[i][j].y, contours[i][j].x);  
                    blue_edge_total += bgr_value[0];
                    green_edge_total += bgr_value[1];
                    red_edge_total += bgr_value[2];

                    // saturation
                    cv::Vec3b hsv_value = this->img_hsv.at<cv::Vec3b>(contours[i][j].y, contours[i][j].x);
                    saturation_edge_total += hsv_value[1];
                }
            }
            
            double blue_edge_average = blue_edge_total / corners_num; // 边缘平均蓝值
            double green_edge_average = green_edge_total / corners_num; // 边缘平均绿值
            double red_edge_average = red_edge_total / corners_num; // 边缘平均红值
            double saturation_edge_average = saturation_edge_total / corners_num; // 边缘平均饱和度
            

            // ------- 4.3 区分灯条与判断颜色-------
            // 不如大道至简：边缘饱和度高 红蓝有一定差值

            std::string color;
            
            const double COLOR_DIFF_THRESHOLD = 50.00; // 红蓝值相减阈值
            const double SATURATION_EDGE_THRESHOLD = 50.00; // 边缘饱和度阈值
            
            // 边缘饱和度高，一定是灯条
            if(std::abs(red_edge_average - blue_edge_average) > COLOR_DIFF_THRESHOLD && saturation_edge_average > SATURATION_EDGE_THRESHOLD)
            {
                if(red_edge_average > blue_edge_average) 
                {
                    color = "red"; // red
                }
                else 
                {
                    color = "blue"; // blue
                }
            }

            else // 边缘饱和度低，或者红蓝相差太小，说明是白炽灯
            {
                // 万一边缘饱和度低，但是红蓝差值很大，就当做灯条
                if(std::abs(red_edge_average - blue_edge_average) > COLOR_DIFF_THRESHOLD * 2.5)
                {
                    if(red_edge_average > blue_edge_average) 
                    {
                        color = "red"; // red
                    }
                    else 
                    {
                        color = "blue"; // blue
                    }
                }
                else
                {
                    color = "white";
                }
            }


            // ------- 4,4 画画部分 -------

            // 展示不是白色灯条的信息
            if (this->SHOW_LOGGER && color != "white")
            {
                cv::putText(
                img_show, 
                "strip " + std::to_string((int)sum) + 
                " corners = " + std::to_string((int)corners_num) + 
                " ratio = " + std::to_string((double)ratio).substr(0, 4) +
                " b = " + std::to_string((int)blue_edge_average) + 
                " g = " + std::to_string((int)green_edge_average) + 
                " r = " + std::to_string((int)red_edge_average) + 
                " s_edge = " + std::to_string((int)saturation_edge_average) + 
                " area = " + std::to_string((int)area) +
                " color = " + color,
                cv::Point2f(0, sum * 50), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(230, 255, 0), 1);
            
                // 灯条编号
                cv::putText(img_show, "L" + std::to_string((int)sum), cv::Point2f(temp_rotatedRect.center.x, temp_rotatedRect.center.y), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 180, 255), 1);

                // 获取角点，根据color画出不同颜色的旋转矩形
                cv::Scalar color_scalar; // 临时标注灯条边框的颜色
                if(color == "red") color_scalar = cv::Scalar(0, 255, 255); // 红色灯条 -> 黄色
                else if(color == "blue") color_scalar = cv::Scalar(255, 255, 0); // 蓝色灯条 -> 青色
                else color_scalar = cv::Scalar(255, 0, 255); // 白炽灯 -> 紫色
                
                cv::Point2f corners[4]; // 四个顶点
                temp_rotatedRect.points(corners); // 获取顶点

                for (int j = 0; j < 4; j++)
                {
                    cv::putText(img_show, "[" + std::to_string((int)j + 1) + "][" + std::to_string((int)corners[j].x) + ", " + std::to_string((int)corners[j].y) + "]", cv::Point2f(corners[j].x, corners[j].y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 0.5);
                    cv::line(img_show, corners[j], corners[(j + 1) % 4], color_scalar, 3); //画线
                }

            }
            

			// ------- 5. 颜色和所选颜色相同，则确认是目标灯带，入队 -------
            if(color == this->CHOSEN_COLOR) 
            {
                // 检查所有顶点是否在图像范围内（防止越界）
                bool all_in_range = true;
                for (const auto& pt : vec_vertices) 
                {
                    if (pt.x < 3 || pt.x >= img.cols - 3 || pt.y < 3 || pt.y >= img.rows - 3) 
                    {
                        all_in_range = false;
                        break;
                    }
                }


                // 只有当所有顶点都在范围内时才进行亚像素优化，否则跳过以避免越界访问
                if(all_in_range)
                {
                    // 设置亚像素细化的参数
                    cv::Size winSize(5, 5);          // 搜索窗口半宽为5，实际窗口 11x11
                    cv::Size zeroZone(-1, -1);       // 不使用死区
                    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01);

                    // 执行亚像素细化（必须在灰度图上进行）
                    cv::cornerSubPix(this->img_gray, vec_vertices, winSize, zeroZone, criteria);


                    // 【可选】用细化后的顶点重新生成一个更精确的旋转矩形（推荐）
                    cv::RotatedRect refined_rect = cv::minAreaRect(vec_vertices);

                    // 重新计算 angle, width, height
                    cv::Size2d refined_size = refined_rect.size;
                    double refined_width = refined_size.width;
                    double refined_height = refined_size.height;
                    double refined_angle = refined_rect.angle;
                    if (refined_width > refined_height) 
                    {
                        refined_angle -= 90.0;
                        std::swap(refined_width, refined_height);
                    }
                    if (refined_angle == 90.0)
                    {
                        refined_angle = 0.0;
                    }

                    // 使用细化后的参数构造 Strip
                    strip.push_back(Strip(refined_rect, refined_angle, refined_width, refined_height, color));
                    RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 经过亚像素优化并入选", sum);

                }

                else 
                {
                    // 边界不安全，直接使用原矩形
                    strip.push_back(Strip(temp_rotatedRect, angle, width, height, color));
                }

            }
		}
	}


    // ------- 6. 存储灯带，返回灯带集合 -------
	this->strip = strip;
    if(strip.size() == 0)
    {
        this->last_best_center = cv::Point2f(-1.00, -1.00); // 没有灯条了，重置 last_best_center
    }
    return strip;

}




/*

    灯条特征：边缘红蓝突出，中间基本全白。边缘色彩饱和度高，中间可能低

    白炽灯特征：全白，边缘和中间饱和度相差小，饱和度低

    目前问题：稍远距离（3m）识别不到，怀疑是曝光太低了

*/



/////////////////////////////////////////////  预处理函数  /////////////////////////////////////////////

/**
* @brief	预处理函数
* @param	cv::Mat& img 要处理的图
* @return   无返回值
*/
void Prepare::preProcessing(cv::Mat& img)
{   

    const int COLOR_HIGH_THERESHOLD_VALUE = 180; // 灯条颜色的二值化阈值
    const int COLOR_DIFF_VALUE = 50; // 红蓝差值阈值

    std::vector<cv::Mat> channels_bgr;
    std::vector<cv::Mat> channels_hsv;
    cv::Mat B, G, R; // bgr三通道值
    cv::Mat high_color_binary; // 选择的颜色的灯条的高阈值二值化图
    cv::Mat color_diff; // 红蓝差值图
    cv::Mat color_diff_binary; // 红蓝差值二值化图
    cv::Mat img_hsv; // hsv
    cv::Mat img_gray; // 灰度图

    this->img_show = img.clone(); // 复制得到绘图图像，对 img_show 的操作直接在 findcontours 等等函数里

    // // 1. 亮度调整 同时调整对比度和亮度，但目前不是必须
    // cv::Mat bright;
    // img.convertTo(bright, -1, 1.2, 30);

    // 2. 得到 hsv，尝试区分灯条和白炽灯。得到 img_gray，用于亚像素优化
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    // 3. 分离bgr三通道的值
    split(img, channels_bgr);

    B = channels_bgr[0];
    G = channels_bgr[1];
    R = channels_bgr[2];

    // 根据颜色只计算一个种类的图
    if(this->CHOSEN_COLOR == "red")
    {
        // 4. 高阈值二值化    红图: 红色灯条 + 白炽灯灯条 
        cv::threshold(R, high_color_binary, COLOR_HIGH_THERESHOLD_VALUE, 255, cv::THRESH_BINARY);

        // 5. 差分二值化    红减蓝：只剩下红色的边缘光晕
        cv::subtract(R, B, color_diff);

        cv::threshold(color_diff, color_diff_binary, COLOR_DIFF_VALUE, 255, cv::THRESH_BINARY);

    }
    else
    {
        // 4. 高阈值二值化    蓝图: 蓝色灯条 + 白炽灯灯条
        cv::threshold(B, high_color_binary, COLOR_HIGH_THERESHOLD_VALUE, 255, cv::THRESH_BINARY);

        // 5. 差分二值化    蓝减红：只剩下蓝色的边缘光晕
        cv::subtract(B, R, color_diff);

        cv::threshold(color_diff, color_diff_binary, COLOR_DIFF_VALUE, 255, cv::THRESH_BINARY);

    }

    // 6. 填补差分图中间的空缺
    cv::Mat kernel_fill = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(color_diff_binary, color_diff_binary, kernel_fill);

    // 7. AND 一下
    cv::bitwise_and(high_color_binary, color_diff_binary, this->mask);


    // 8. 解决“物理断裂”：形态学闭运算 (Closing = 先膨胀再腐蚀)
    // 【极其关键】：因为灯条是竖直生长的，我们绝对不能用正方形的核，那样会让灯条变宽，导致 PnP 距离测不准！
    // 我们定义一个【宽 3，高 7】或者【宽 1，高 9】的“竖直核”。
    // 它只在上下方向进行桥接缝合，绝不改变灯条的真实物理宽度。
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(5, 11));
    cv::morphologyEx(this->mask, this->mask, cv::MORPH_CLOSE, kernel_close);


    this->img = img; // 原图
    this->img_hsv = img_hsv; // hsv图
    this->img_gray = img_gray; // 灰度图

    cv::imshow("img_mask", this->mask);

}