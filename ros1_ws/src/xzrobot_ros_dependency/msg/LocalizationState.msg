# LocalizationServer 对外契约
#
# 一个话题承载"位姿 + 不确定度 + 健康等级 + 匹配度 + 地图标识",
# 取代消费者各自拼 /lio_odom + /slam/info + /map 的做法.
#
# 语义:
#   pose        map -> base_link 的位姿 (与 TF 同源)
#   covariance  估计器给出的 6x6 行优先协方差; 退化方向 (如长走廊的沿墙方向)
#               会显著大于其它方向. LOST/INIT 时置大值, 表示不可用.
#   health      状态机当前状态:
#                 INIT      刚启动/重定位中, 位姿不可信
#                 TRACKING  正常跟踪
#                 DEGRADED  仍在跟踪但质量下降 (匹配度低/退化), 消费端应降速
#                 LOST      丢失, 消费端必须停车
#   matching    当前帧与地图的匹配度 (0~1), INIT/LOST 时为 0
#   map_id      当前地图标识 (map_name), 用于消费端判断地图是否切换
#   map_valid   当前是否有可用地图
#   relocalize_count  累计重定位次数, 单调递增; 变化说明位姿发生了跳变

uint8 HEALTH_INIT=0
uint8 HEALTH_TRACKING=1
uint8 HEALTH_DEGRADED=2
uint8 HEALTH_LOST=3

std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
uint8 health
float32 matching
string map_id
bool map_valid
uint32 relocalize_count
