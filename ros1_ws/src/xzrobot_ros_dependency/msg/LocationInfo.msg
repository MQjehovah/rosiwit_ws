int8 SUCCESS=0
int8 RECOVERING=1
int8 FAILED=2

int8 MAPPING = 0
int8 LOCALIZATION = 1
int8 ERROR = 2

string map_name
geometry_msgs/Pose2D robot_pose
int8 location_status
float32 matching_degree
int8 slam_state
float32 map_area

