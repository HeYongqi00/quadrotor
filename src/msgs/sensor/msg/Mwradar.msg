uint8               index               # 目标ID，根据Track信息获得
float64             rcs                 # 出厂测试保留值，不做输出
float64             range               # 雷达输出的原始数据单位为cm，转换后目标距离的单位为米
uint8               rollcount           # 计数位
float64             verl                # 目标速度，单位m/s
int8                snr                 # 信噪比，出厂测试保留值，不做输出