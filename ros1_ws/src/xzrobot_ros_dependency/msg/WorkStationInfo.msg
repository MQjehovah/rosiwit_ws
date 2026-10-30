# 预约状态：1，已经被预约；0，未被预约
uint8 order_status

# 与充电机的通信状态，0正常 ，1故障
uint8 comm_status

# 充电状态
uint8 charge_status

# 水电站的状态：01正常、02、故障
uint8 station_status

# 急停按钮状态，0未按下，1按下
uint8 emergency_status

# 重启按钮状态，0未按下，1按下
uint8 restart_status

# 清洁剂的状态：1，不足，0，充足
uint8 cleanser_status

# 清水泵状态：1开启，0未开启
uint8 relay_clean_water_status

# 清洁剂泵状态;1,开启，0，未开启
uint8 relay_cleanser_status

# 排污阀状态：1开启，0未开启
uint8 relay_sewage_water_status

# 充电机输入继电器状态（AC）
uint8 relay_charge_input_status

# 充电机输出继电器状态（DC）
uint8 relay_charge_output_status

# led的模式
uint8 led_mode

# 工作站环温
uint8 env_temp

# 充电极片温度
uint8 pole_temp

# 目标充电电流
uint16 target_charge_current

# 目标充电电压
uint16 target_charge_voltage

# 当前充电电流
uint16 cur_charge_current

# 当前充电电压
uint16 cur_charge_voltage

# 当前充电温度
uint16 cur_charge_temp

# 当前充电时间（min）
uint16 cur_charge_time

# 错误码
uint16 error_code