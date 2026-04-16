std_msgs/Header header
float64[9] matrix_r   # 右手系 旋转矩阵
float64[3] tvec   # 右手系 平移向量，单位：mm

int32 continuous_count  # 连续有效帧id（已经丢弃了前五帧）