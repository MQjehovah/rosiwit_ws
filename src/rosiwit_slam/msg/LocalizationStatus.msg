# LocalizationStatus.msg
# 定位状态消息

# 定位状态常量
uint8 UNINITIALIZED = 0   # 未初始化
uint8 LOCALIZING = 1       # 正在定位
uint8 LOCALIZED = 2       # 已定位
uint8 LOST = 3            # 丢失定位

# 状态信息
uint8 state                      # 当前定位状态
geometry_msgs/Pose current_pose # 当前位姿

# 置信度
float64 fitness_score           # 配准得分
float64 confidence              # 置信度

# 匹配信息
int64 matched_keyframe_id       # 匹配的关键帧ID
float64 position_error          # 位置误差（米）
float64 rotation_error          # 旋转误差（弧度）

# 时间信息
builtin_interfaces/Time last_localization_time  # 最后定位时间
float64 time_elapsed                               # 耗时（秒）

# 状态消息
string message