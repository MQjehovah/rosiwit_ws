# 0 空闲, 1 预约, 2 离桩
uint8 order_status

# 预约充电桩的id
uint16 order_station_id

# 对接状态
uint8 dock_status

# 开启虚电压
bool request_charge

# 充电机输入继电器
bool relay_charge_input_ctrl

# 充电机输出继电器
bool relay_charge_output_ctrl

# 充电请求电流
uint16 charge_current

# 充电请求电压
uint16 charge_voltage

# 清水泵（百分比）
uint8 relay_clean_water_duty

# 污水泵（百分比）
uint8 relay_sewage_water_duty

# 清洁剂泵（百分比）
uint8 relay_cleanser_duty
