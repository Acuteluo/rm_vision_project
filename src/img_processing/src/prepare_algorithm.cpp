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
	double moderation[101][101] = { 0.00 }; // [编号][编号]，合理性
	std::vector<int> moderation_number(strip.size(), -1); // 对于当前灯条，最合理的是谁（ -1 表示没找到目标 ）



    // ------- 0. 合理性评判参数设置 -------

	double MAX_ANGLE = 25.00; // 两个灯条最大差角 -> [两个灯条是否平行]

    // 两个灯条构成的装甲板 宽高比例是否合理 - > [灯条间距离是否合理] 容许的最大比例系数 1.5
    double ARMORPLATE_RATIO_THRESHOLD_MAX = 1.75;
    double ARMORPLATE_RATIO_THRESHOLD_MIN = 0.5;
	double MAX_HERO_ARMORPLATE_RATIO = (225.00 / 55.00) * ARMORPLATE_RATIO_THRESHOLD_MAX; // 英雄
    double MAX_NORMAL_ARMORPLATE_RATIO = (135.00 / 55.00) * ARMORPLATE_RATIO_THRESHOLD_MAX; // 步兵

	double MIN_TWO_STRIP_RATIO = 0.3 / 1; // 两个灯条 短:长 的比值不能太小 -> [灯条间的选取是否合理]

    double MIN_CONNECTING_LINE_COMPARE_STRIP_ANGLE = 30.00; // 连接线和灯条的角度差不能太大 -> [避免选到共线的灯条]



    // ------- 1. 灯条从左到右排序 -------  
	std::sort(strip.begin(), strip.end(), sortStripByX); // 从左到右排序



	// ------- 2. 灯条遍历，两两配对，计算合理性 moderation -------
	for (int i = 0; i < strip.size(); i++)
	{
		for (int j = i; j < strip.size(); j++) // 算过的就不用算了，也确保 i 灯条在左边，j 灯条在右边
		{
			if (i == j || moderation[j][i] != 0) continue; // 跳过自己 和 i -> j = j -> i

			bool angle_moderation = true; // [两个灯条是否平行]
			bool distance_long_moderation = true; // [灯条间距离是否合理，不能太长]
            bool distance_short_moderation = true; // [灯条间距离是否合理，不能太短]（必要）
            bool length_moderation = true; // [灯条间的选取是否合理]
            bool connecting_line_moderation = true; // [避免选到共线的灯条]（必要）

			double temp_moderation = 0.00; // 当前灯条 i 与 灯条 j 间的合理性


			// 2.1. 两个灯条是否平行
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
				
                double score0 = (MAX_ANGLE - angle_diff) / MAX_ANGLE * 25.00; // 和最优的差距，最优时差距为0，得分为25
                temp_moderation += score0;

                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 角度差得分, = %.2f", i, j, score0);
            }   



            // 2.2.1 & 2.2.2 两个灯条构成的装甲板 宽高比例是否合理

            double dx = strip[i].center.x - strip[j].center.x;
            double dy = strip[i].center.y - strip[j].center.y;
            double distance = std::sqrt(dx * dx + dy * dy); // 两个灯条中心点距离

            double strip_longest = std::max(strip[i].height, strip[j].height); // 两个灯条里长度 长 的那个
            double strip_shortest = std::min(strip[i].height, strip[j].height); // 两个灯条里长度 短 的那个
            double strip_average = (strip_longest + strip_shortest) / 2.00; // 两个灯条长度的平均值

            double armorplate_ratio = distance / strip_average; // 灯条间距离和灯条长度的比值
			
            // 对于英雄装甲板
            if(this->ARMOR_TYPE == "hero")
            {
                if(armorplate_ratio < MAX_HERO_ARMORPLATE_RATIO / ARMORPLATE_RATIO_THRESHOLD_MAX * ARMORPLATE_RATIO_THRESHOLD_MIN)
                {
                    // 距离太近，构成的装甲板的比值不合理
                    RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 旋转矩形的宽高比【不】合理, = %.2f < %.2f", i, j, armorplate_ratio, MAX_HERO_ARMORPLATE_RATIO / ARMORPLATE_RATIO_THRESHOLD_MAX * ARMORPLATE_RATIO_THRESHOLD_MIN);
                    
                    distance_short_moderation = false;
                }

                if(armorplate_ratio > MAX_HERO_ARMORPLATE_RATIO)
                {
                    // 距离太远，构成的装甲板的比值不合理
                    RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 旋转矩形的宽高比【不】合理, = %.2f > %.2f", i, j, armorplate_ratio, MAX_HERO_ARMORPLATE_RATIO);
                    
                    distance_long_moderation = false;
                }
                else 
                {
                    // 比值合理
                    RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 旋转矩形的宽高比     合理, = %.2f < %.2f", i, j, armorplate_ratio, MAX_NORMAL_ARMORPLATE_RATIO);
                    
                    double score1 = (MAX_HERO_ARMORPLATE_RATIO - armorplate_ratio) / (MAX_HERO_ARMORPLATE_RATIO - MAX_HERO_ARMORPLATE_RATIO / ARMORPLATE_RATIO_THRESHOLD_MAX) * 25.00; // 和英雄最优的差距（倍数），0.8倍和1.2倍得分一致
                    temp_moderation += score1; 
                    
                    RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 旋转矩形的宽高比得分, score1 = %.2f", i, j, score1);
                }
            }

            // 对于步兵装甲板
            else
            {
                if(armorplate_ratio < MAX_NORMAL_ARMORPLATE_RATIO / ARMORPLATE_RATIO_THRESHOLD_MAX * ARMORPLATE_RATIO_THRESHOLD_MIN)
                {
                    // 距离太近，构成的装甲板的比值不合理
                    RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 旋转矩形的宽高比【不】合理, = %.2f < %.2f", i, j, armorplate_ratio, MAX_NORMAL_ARMORPLATE_RATIO / ARMORPLATE_RATIO_THRESHOLD_MAX * ARMORPLATE_RATIO_THRESHOLD_MIN);
                    
                    distance_short_moderation = false;
                }

                if(armorplate_ratio > MAX_NORMAL_ARMORPLATE_RATIO)
                {
                    // 距离太远，构成的装甲板的比值不合理
                    RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 旋转矩形的宽高比【不】合理, = %.2f > %.2f", i, j, armorplate_ratio, MAX_NORMAL_ARMORPLATE_RATIO);
                    
                    distance_long_moderation = false;
                }
                else 
                {
                    // 比值合理
                    RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 旋转矩形的宽高比     合理, = %.2f < %.2f", i, j, armorplate_ratio, MAX_NORMAL_ARMORPLATE_RATIO);
                    
                    double score2 = (MAX_NORMAL_ARMORPLATE_RATIO - armorplate_ratio) / (MAX_NORMAL_ARMORPLATE_RATIO - MAX_NORMAL_ARMORPLATE_RATIO / ARMORPLATE_RATIO_THRESHOLD_MAX) * 25.00; // 和步兵最优的差距（倍数），0.8倍和1.2倍得分一致
                    temp_moderation += score2; 

                    RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 旋转矩形的宽高比得分, score2 = %.2f", i, j, score2);
                }
            }




			// 2.3. 两个灯条 短:长 的比值不能太小

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
				
                temp_moderation += two_strip_ratio * 25; // 和最优的差距，最优时 比例接近 1
                
                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 短:长 的比得分, = %.2f", i, j, two_strip_ratio * 25);
			}
			



            // 2.4. 两个灯条的连接线角度和灯条平均角度应接近垂直（不能太小），避免选到共线的灯条

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
                RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 连接线和灯条的角度差【不】合理, = %.2f < %.2f", i, j, angle_deg, MIN_CONNECTING_LINE_COMPARE_STRIP_ANGLE);
                
                connecting_line_moderation = false;
            }
            else
            {
                // 连接线和灯条的角度差合理
                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 连接线和灯条的角度差     合理, = %.2f > %.2f", i, j, angle_deg, MIN_CONNECTING_LINE_COMPARE_STRIP_ANGLE);
                
                double score3 = ((angle_deg -  MIN_CONNECTING_LINE_COMPARE_STRIP_ANGLE) / (90.00 - MIN_CONNECTING_LINE_COMPARE_STRIP_ANGLE)) * 25; 
                temp_moderation += score3;

                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 和灯条 %d 连接线和灯条的角度差得分, = %.2f", i, j, score3);
            }



			// 2.5. 评判：如果 2.2.short 和 2.4 满足，且 2.1-2.3 三个条件里 至少有 两个条件满足
            int conditions_met = angle_moderation + distance_long_moderation + length_moderation;
			if (conditions_met >= 2 && connecting_line_moderation && distance_short_moderation) // 连接线合理、距离不能太短是必须的
			{
                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "除必要的两个条件外，条件满足数: %d / 3", conditions_met);
				
                //更新 i 对 j 的最大合理性
				moderation[i][j] = std::max(temp_moderation, moderation[i][j]);
                moderation[j][i] = moderation[i][j]; // i -> j 的合理性 和 j -> i 的合理性 是一样的
			}

		}

	}


	// 3. 整理简化数据，明确每个灯条 最合理的配对对象 是谁，放入 moderation_number 数组中
	for (int i = 0; i < strip.size(); i++)
	{
		double max_moderation = 0.00; // 对于当前灯条，最大合理性
		int number = -1; // 对于当前灯条，最合理的是谁

		for (int j = 0; j < strip.size(); j++)
		{
			if (i == j || moderation[i][j] <= 25.00) continue; // 跳过 自己 和 没有合理性的，以及合理性不高的
			if (moderation[i][j] > max_moderation) // 合理性更高，更新合理性和最合理的是谁
			{
				max_moderation = moderation[i][j];
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


    // 5. 对于每一个装甲板，按照置信度排序
    for(int i = 0; i < this->armorplate.size(); i++)
    {
        for(int j = i; j < this->armorplate.size(); j++)
        {
            if(this->armorplate[j].moderation > this->armorplate[i].moderation)
            {
                // 交换装甲板 i 和装甲板 j
                ArmorPlate temp = this->armorplate[i];
                this->armorplate[i] = this->armorplate[j];
                this->armorplate[j] = temp;
            }
        }
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

	int sum = 0; // 目前检测到的候选灯带数量 Strip Count（需要吗？）
    int corners_num = 0; // 当前灯条角点数

	for (int i = 0; i < contours.size(); i++)
	{
        corners_num = 0;
		int area = cv::contourArea(contours[i]);
		if (area > 20) // 面积去除噪点（目前暂时不以面积作为筛选，因为只要够远，面积会非常小，但是饱和度可以保证）
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

                // 将细化后的顶点拷贝回 vertices
                for (int i = 0; i < 4; ++i) 
                {
                    vertices[i] = vec_vertices[i];
                }

                // 【可选】用细化后的顶点重新生成一个更精确的旋转矩形（推荐）
                cv::RotatedRect refined_rect = cv::minAreaRect(vec_vertices);

                temp_rotatedRect = refined_rect; // 这样 temp_rotatedRect 就是亚像素优化后的旋转矩形了

                RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("info"), this->SHOW_LOGGER, "灯条 %d 的旋转矩形经过亚像素优化了！", i);
            }


			double angle = temp_rotatedRect.angle; // 获取角度

			cv::Size2d size = temp_rotatedRect.size;  // 尺寸
			double width = size.width; // 宽
			double height = size.height; // 高

            corners_num = contours[i].size(); // 角点数量



			// ------- 2. 角度处理 -------

            // 确保 width 是短边，height 是长边
			if (width > height) 
			{
				angle -= 90.00; // 如果矩形是通过逆时针旋转得到的，那么让 angle 变成负的，这样 angle 范围 (-90, 90)
				std::swap(width, height); 
			}
			if (angle == 90.00) angle = 0.00; // 90度视为0度，确保 angle 连续



            // ------- 3. 高宽比处理 -------

			double ratio = height / width; // [高宽比]不能太小，确保像灯带

            if (area > 100 && ratio < 2.50) continue; // 面积不小，高宽比又不小，不是灯条，不要

            
            // 统计：灯条 + 1
            ++sum;
           


            // ------- 4.1 区分颜色之边缘部分，计算边缘 BGR 和 saturation --------


            // 生成一个只有当前灯条边缘及其内部是 255 的掩码图
            cv::Mat strip_mask = cv::Mat::zeros(this->mask.size(), CV_8UC1);
            cv::drawContours(strip_mask, std::vector<std::vector<cv::Point>>{contours[i]}, 0, 255, -1);

            int saturation_edge_total = 0; //  总边缘饱和度
            int blue_edge_total = 0; //  总边缘蓝值
            int green_edge_total = 0; //  总边缘绿值
            int red_edge_total = 0; //  总边缘红值

            double blue_edge_average = 0.00; // 边缘平均蓝值
            double green_edge_average = 0.00; // 边缘平均绿值
            double red_edge_average = 0.00; // 边缘平均红值
            double saturation_edge_average = 0.00; // 边缘平均饱和度

            for(int j = 0; j < corners_num; j++)
            {
                // 注意行在前，列在后
                if (contours[i][j].x >= 0 && contours[i][j].x < img.cols && contours[i][j].y >= 0 && contours[i][j].y < img.rows)
                {
                    // 在该点的 strip_mask 值
                    uchar mask_value = strip_mask.at<uchar>(contours[i][j].y, contours[i][j].x);

                    // 掩码图上是有这个点的 -> 这个点是这个灯条上的 -> bgr和hsv有效
                    if(mask_value) 
                    {
                        // bgr
                        cv::Vec3b bgr_value = img.at<cv::Vec3b>(contours[i][j].y, contours[i][j].x);  
                        uchar blue = bgr_value[0];
                        uchar green = bgr_value[1];   
                        uchar red = bgr_value[2];
                        blue_edge_total += blue;
                        green_edge_total += green;
                        red_edge_total += red;

                        // saturation
                        cv::Vec3b hsv_value = this->img_hsv.at<cv::Vec3b>(contours[i][j].y, contours[i][j].x);
                        uchar saturation = hsv_value[1]; // S 范围 0~255
                        saturation_edge_total += saturation;
                
                    }
                }
            }
            
            blue_edge_average = blue_edge_total / corners_num; // 边缘平均蓝值
            green_edge_average = green_edge_total / corners_num; // 边缘平均绿值
            red_edge_average = red_edge_total / corners_num; // 边缘平均红值
            saturation_edge_average = saturation_edge_total / corners_num; // 边缘平均饱和度
            



            // ------- 4.2 区分颜色 之 总体部分 注意是针对 strip_mask，与 contours 无关！ -------

            int saturation_total = 0; //  总饱和度
            double saturation_average = 0.00; // 平均饱和度
            int pixels_num = 0; // 灯条像素点总数

            for(int j = 0; j < strip_mask.rows; j++) // 行y
            {
                for(int k = 0; k < strip_mask.cols; k++) // 列x
                {
                    // 在该点的 strip_mask 值
                    uchar mask_value = strip_mask.at<uchar>(j, k);
                    if(mask_value) 
                    {
                        cv::Vec3b hsv_value = this->img_hsv.at<cv::Vec3b>(j, k);
                        uchar saturation = hsv_value[1]; // S 范围 0~255
                        saturation_total += saturation;

                        ++pixels_num;
                    }
                }
            }

            saturation_average = saturation_total / pixels_num; // 总体平均饱和度
            
            

            // ------- 4.3 区分灯条与判断颜色（还需完善）-------
            std::string color;
            double times = std::max((double)red_edge_average / (double)blue_edge_average, (double)blue_edge_average / (double)red_edge_average); // 红蓝值相差倍数，至少要 4 以上才比较合理
            
            // 边缘饱和度高，一定是灯条
            if(saturation_edge_average > 175.00 || saturation_average > 175.00)
            {
                if(red_edge_average > blue_edge_average) // 红值偏高，一般都会相差 100 以上的
                {
                    color = "red"; // red
                }
                
                else 
                {
                    color = "blue"; // blue
                }
            }

            else if(red_edge_average > 180.00 && blue_edge_average > 180.00) // 红蓝值都高，一定是白炽灯
            {
                color = "white";
            }

            else if(saturation_average > 50.00 && saturation_edge_average > 50.00 && green_edge_average < 150.00) // 总体饱和度高，边缘饱和度也高，绿值低，说明是灯条
            {
                if(std::fabs(red_edge_average - blue_edge_average) < 100.00) // 红蓝值相差不大，说明是白炽灯
                {
                    color = "white";
                }

                else if(times < 4.00) // 红蓝值相差倍数不大，说明是白炽灯
                {
                    color = "white";
                }

                else if(std::min(red_edge_average, blue_edge_average) > 75.00) // 红蓝值的最低值不低，说明是白炽灯
                {
                    color = "white";
                }

                
                else
                {
                    if(red_edge_average > blue_edge_average) // 红值偏高，一般都会相差 100 以上的
                    {
                        color = "red"; // red
                    }
                
                    else 
                    {
                        color = "blue"; // blue
                    }
                }
                
            }

            else // 总体饱和度低，边缘饱和度也低，说明是白炽灯
            {
                color = "white";
            }


            // ------- 4,4 画画部分 -------

            // 信息
            cv::putText(
                img_show, 
                "strip " + std::to_string((int)sum) + 
                " corners = " + std::to_string((int)corners_num) + 
                " b = " + std::to_string((int)blue_edge_average) + 
                " g = " + std::to_string((int)green_edge_average) + 
                " r = " + std::to_string((int)red_edge_average) + 
                " s_edge = " + std::to_string((int)saturation_edge_average) + 
                " s_total = " + std::to_string((int)saturation_average) +
                " area = " + std::to_string((int)area) +
                " color = " + color,
                cv::Point2f(0, sum * 50), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255), 2);
            
            // 灯条编号
            cv::putText(img_show, "L" + std::to_string((int)sum), cv::Point2f(temp_rotatedRect.center.x, temp_rotatedRect.center.y), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 180, 255), 1.25);

            // 获取角点，根据color画出不同颜色的旋转矩形
            cv::Scalar color_scalar; // 临时标注灯条边框的颜色
            if(color == "red") color_scalar = cv::Scalar(0, 255, 255); // 红色灯条 -> 黄色
            else if(color == "blue") color_scalar = cv::Scalar(255, 255, 0); // 蓝色灯条 -> 青色
            else color_scalar = cv::Scalar(255, 0, 255); // 白炽灯 -> 紫色
            
            cv::Point2f corners[4]; // 四个顶点
			temp_rotatedRect.points(corners); // 获取顶点

            for (int j = 0; j < 4; j++)
	        {
		        cv::putText(img_show, "[" + std::to_string((int)j + 1) + "][" + std::to_string((int)corners[j].x) + ", " + std::to_string((int)corners[j].y) + "]", cv::Point2f(corners[j].x, corners[j].y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 0.65);
		        cv::line(img_show, corners[j], corners[(j + 1) % 4], color_scalar, 3); //画线
	        }
            

			// ------- 5. 颜色和所选颜色相同，则确认是目标灯带，入队 -------
            if(color == this->CHOSEN_COLOR) 
            {
			    strip.push_back(Strip(temp_rotatedRect, angle, width, height, color));
            }
		}
	}


    // ------- 6. 存储灯带，返回灯带集合 -------
	this->strip = strip;
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

    if(this->CAMERA_NAME == "mind_vision")
    {
        int THERESHOLD_VALUE = 220; // 二值化阈值


        std::vector<cv::Mat> channels_bgr;
        std::vector<cv::Mat> channels_hsv;
        cv::Mat B, G, R; // bgr三通道值
        cv::Mat red_differ, blue_differ, mask; // 红色和蓝色灯条 BGR最高最低相差值掩码（需要吗？）
        cv::Mat img_red, img_blue;
        cv::Mat high_red, high_blue;
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

        // 4. 高阈值二值化   红图: 红色灯条 + 白炽灯灯条  蓝图: 蓝色灯条 + 白炽灯灯条
        cv::threshold(R, high_red, THERESHOLD_VALUE, 255, cv::THRESH_BINARY);
        cv::threshold(B, high_blue, THERESHOLD_VALUE, 255, cv::THRESH_BINARY);

        // // 5. 垂直方向模糊 重新变为二值化
        /*
            但实际效果需要验证

            建议你用实际图像测试一下，观察二值化后的灯条掩码是否有以下情况：

            断裂：如果灯条中心仍有空洞，导致掩码分成上下两段，则需要垂直模糊来连接。
            小缺口：边缘有小缺口，但整体连通，可以不处理。
            横向散光：如果仍有轻微横向扩散，垂直模糊也能帮助抑制（因为它只在垂直方向平滑）。
        */
        // cv::blur(high_red, high_red, cv::Size(1, 3));
        // cv::blur(high_blue, high_blue, cv::Size(1, 3));
        // cv::threshold(high_red, high_red, 1, 255, cv::THRESH_BINARY);
        // cv::threshold(high_blue, high_blue, 1, 255, cv::THRESH_BINARY);  
        

        // 根据要求的颜色选择最终的掩码图
        if(this->CHOSEN_COLOR == "red") this->mask = high_red; // 最终掩码图
        else this->mask = high_blue; 


        this->img = img; // 原图
        this->img_hsv = img_hsv; // hsv图
        this->img_gray = img_gray; // 灰度图


        //cv::imshow("high_blue", high_blue);
        //cv::imshow("high_red", high_red);
    }
    

    else
    {
        int THERESHOLD_VALUE = 220; // 二值化阈值


        std::vector<cv::Mat> channels_bgr;
        std::vector<cv::Mat> channels_hsv;
        cv::Mat B, G, R; // bgr三通道值
        cv::Mat red_differ, blue_differ, mask; // 红色和蓝色灯条 BGR最高最低相差值掩码（需要吗？）
        cv::Mat img_red, img_blue;
        cv::Mat high_red, high_blue;
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

        // 4. 高阈值二值化   红图: 红色灯条 + 白炽灯灯条  蓝图: 蓝色灯条 + 白炽灯灯条
        cv::threshold(R, high_red, THERESHOLD_VALUE, 255, cv::THRESH_BINARY);
        cv::threshold(B, high_blue, THERESHOLD_VALUE, 255, cv::THRESH_BINARY);

        // // 5. 垂直方向模糊 重新变为二值化
        /*
            但实际效果需要验证

            建议你用实际图像测试一下，观察二值化后的灯条掩码是否有以下情况：

            断裂：如果灯条中心仍有空洞，导致掩码分成上下两段，则需要垂直模糊来连接。
            小缺口：边缘有小缺口，但整体连通，可以不处理。
            横向散光：如果仍有轻微横向扩散，垂直模糊也能帮助抑制（因为它只在垂直方向平滑）。
        */
        // cv::blur(high_red, high_red, cv::Size(1, 3));
        // cv::blur(high_blue, high_blue, cv::Size(1, 3));
        // cv::threshold(high_red, high_red, 1, 255, cv::THRESH_BINARY);
        // cv::threshold(high_blue, high_blue, 1, 255, cv::THRESH_BINARY);  
        

        // 根据要求的颜色选择最终的掩码图
        if(this->CHOSEN_COLOR == "red") this->mask = high_red; // 最终掩码图
        else this->mask = high_blue; 


        this->img = img; // 原图
        this->img_hsv = img_hsv; // hsv图
        this->img_gray = img_gray; // 灰度图


        // cv::imshow("high_blue", high_blue);
        // cv::imshow("high_red", high_red);
    }

    //cv::imshow("img_mask", this->mask);

    // const int IMAGE_BRIGHT = 30;       // 全局亮度增益（过暗就加大）
    // const int THRESHOLD_VALUE = 220;   // 二值化阈值（噪点多就加大）
    // //const int KERNEL_SIZE = 2;       // 核大小

    // cv::Mat dst_BR;
    // cv::Mat dst_blue; // 最终的蓝色灯条掩码
    // cv::Mat dst_red; // 最终的红色灯条掩码

    // std::vector<cv::Mat> channels;

    // // 全局亮度调整
    // {
    //     cv::Mat BrightnessLut(1, 256, CV_8UC1); 
    //     for (int i = 0; i < 256; i++) {
    //         BrightnessLut.at<uchar>(i) = cv::saturate_cast<uchar>(i + IMAGE_BRIGHT);
    //     }
    //     cv::LUT(img, BrightnessLut, dst_BR);
    // }

    // this->img_show = dst_BR.clone(); // 复制得到绘图图像，对 img_show 的操作直接在 findcontours 等等函数里
    // cv::Mat img_hsv; // hsv
    // cv::cvtColor(dst_BR, img_hsv, cv::COLOR_BGR2HSV);

    // // 颜色通道差分
    // cv::split(dst_BR, channels);

    // // 二值化
    // cv::threshold(channels[0], dst_blue, THRESHOLD_VALUE, 255, cv::THRESH_BINARY);
    // cv::threshold(channels[2], dst_red, THRESHOLD_VALUE, 255, cv::THRESH_BINARY);

    // // 垂直方向模糊
    // cv::blur(dst_blue, dst_blue, cv::Size(1, 3)); // 只模糊垂直方向，保留灯条形状
    // cv::blur(dst_red, dst_red, cv::Size(1, 3)); // 只模糊垂直方向，保留灯条形状


    // // 形态学膨胀
    // // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(KERNEL_SIZE, KERNEL_SIZE));
    // // cv::dilate(dst, dst, kernel, cv::Point(-1, -1), 1);
    // //cv::erode(dst, dst, kernel, cv::Point(-1, -1), 1);
    // //cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    // //调试显示（
    // cv::imshow("最终掩码 blue", dst_blue);
    // //cv::imshow("最终掩码 red", dst_red);
        

    // this->CHOSEN_COLOR = "blue"; // 选择检测的颜色 red / blue

    // if(this->CHOSEN_COLOR == "red") this->mask = dst_red; // 最终掩码图
    // else this->mask = dst_blue; 

    // this->img = img; // 原图
    // this->img_hsv = img_hsv; // hsv图
    

}