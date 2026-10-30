# 清水箱液位
float32 clean_height
# 污水箱液位
float32 dirty_height

# 盘刷电机速度
int16 motor_speed_brush
# 盘刷电机电流
float32 motor_current_brush
# 吸风电机速度
int16 motor_speed_suction
# 吸风电机电流
float32 motor_current_suction

# 喷水电机速度
int16 motor_speed_spray
# 过滤电机速度
int16 motor_speed_filter

# 刷盘电机升降位置
int8 motor_position_brush
# 水扒电机升降位置
int8 motor_position_scratcher

#消毒开关
bool disinfect

# 机器人清水阀开关
bool clean_drain_relay

#机器人排污开关
bool dirty_drain_relay

# 充电桩加水阀开关
bool station_clean_relay

uint8 motor_status

uint16 motor_position_brush_encoder

# 左盘刷电流
uint16 motor_current_left_brush
# 右盘刷电流
uint16 motor_current_right_brush
# 盘刷上下推杆电机电流
uint16 motor_current_brush_push
# 吸水趴推杆电机电流
uint16 motor_current_suction_push
# 清水泵电流
uint16 motor_current_clean_pump
# 污水泵电流
uint16 motor_current_dirty_pump

# 盘刷推杆电机编码器值
uint32 motor_position_brush_push_encoder
# 吸水趴推杆电机编码器值
uint32 motor_position_suction_push_encoder

# 错误码
uint32 clean_error