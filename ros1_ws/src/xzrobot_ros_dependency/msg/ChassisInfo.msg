# 左电机下发速度
int32 drv_wheel_speed_left

# 右电机下发速度
int32 drv_wheel_speed_right

# 左电机编码器值
int32 drv_wheel_encoder_left

# 右电机编码器值
int32 drv_wheel_encoder_right

# 驱动使能
bool driver_enable

# 急停信号
bool enmergency

# 碰撞信号
bool crash

# 碰撞位置
uint32 crash_position

# 跌落信号
bool fall

# 在桩信号
bool dock

# 手动充电信号
bool manual_charge

# 钥匙电信号
bool acc

# 雷达信号
bool laser

# 超声信号
bool sonar

# 防碰撞侧板开合信号
bool crash_gate

# 低电量关机保护信号
bool low_battery_protection

# 负压信号
int16 neg_press

# 左轮速度
int32 mcu_send_speedL

# 右轮速度
int32 mcu_send_speedR

# 预留位
uint32 csv

# 主控板温度
uint16 master_board_temp
# 自动充电MOS温度
uint16 auto_charge_mos_temp
# 手动充电MOS温度
uint16 manual_charge_mos_temp
# 左电机负载率
uint16 drv_load_left
# 右电机负载率
uint16 drv_load_right
# 左电机温度
uint16 drv_temp_left
# 右电机温度
uint16 drv_temp_right

# 错误码
uint32 drv_error