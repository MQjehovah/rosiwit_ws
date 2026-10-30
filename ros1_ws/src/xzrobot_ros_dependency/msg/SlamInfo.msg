int8 SUCCESS=0 # 定位正常
int8 RECOVERING=1 # 定位中
int8 FAILED=2 # 定位失败
int8 MAPPING = 0 # 建图状态
int8 LOCALIZATION = 1 # 定位状态
int8 ERROR = 2 # 错误状态
int8 slam_state # slam状态
int8 location_status # 定位状态
int8 update_request # 地图⾃更新状态 1-地图需要⾃更新
float32 matching_degree # 定位匹配度
string map_name # 地图名称
geometry_msgs/Pose2D robot_pose # 机器⼈位置
float32 map_area # 建图⾯积，仅建图模式有效