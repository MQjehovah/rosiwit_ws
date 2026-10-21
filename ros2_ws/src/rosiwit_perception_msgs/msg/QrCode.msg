# 单个二维码检测结果
string text

# 图像像素角点，固定顺序 TL, TR, BR, BL，展开为 [u0,v0, u1,v1, u2,v2, u3,v3]
float32[8] corners

# 图像像素中心 [u, v]
float32[2] center

# 二维码物理边长（米，来自参数 qr_size）
float32 size

# 二维码中心到相机光心的距离（米），0 表示无位姿
float32 distance

# 位姿。header.frame_id 为目标坐标系（默认 map），无 TF 时可能为空。
geometry_msgs/PoseStamped pose
