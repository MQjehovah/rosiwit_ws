#!/usr/bin/env python3
"""
生成大型测试场景 SDF 世界文件，包含：
- 大型矩形回环走廊 (外圈 60x40m, 走廊宽 4m)
- 长直走廊延伸段 (30m 死胡同)
- 4 个房间 (含家具)
- 走廊中的立柱 (回环检测特征)
- 多光源照明

用法:
  python3 scripts/generate_large_loop_world.py > world/large_loop.world
"""

# ============ 场景参数 ============
OUTER_X = 60.0   # 外圈 X 方向长度
OUTER_Y = 40.0   # 外圈 Y 方向长度
CORRIDOR_W = 4.0 # 走廊宽度
WALL_THICK = 0.2 # 墙厚度
WALL_H = 2.8     # 墙高度

EXT_CORRIDOR_LEN = 30.0  # 延伸走廊长度
PILLAR_SPACING = 8.0      # 立柱间距
NUM_ROOMS = 4

# 计算内圈尺寸
INNER_X = OUTER_X - 2 * CORRIDOR_W
INNER_Y = OUTER_Y - 2 * CORRIDOR_W
HALF_OUT_X = OUTER_X / 2
HALF_OUT_Y = OUTER_Y / 2
HALF_IN_X = INNER_X / 2
HALF_IN_Y = INNER_Y / 2


def wall_visual(name, sx, sy, sz, r=0.7, g=0.7, b=0.7):
    return f"""    <visual name='{name}_Visual'>
      <pose>0 0 {sz/2} 0 -0 0</pose>
      <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      <material>
        <ambient>{r} {g} {b} 1</ambient>
        <diffuse>{r} {g} {b} 1</diffuse>
        <specular>0.1 0.1 0.1 1</specular>
      </material>
    </visual>"""


def wall_collision(name, sx, sy, sz):
    return f"""    <collision name='{name}_Collision'>
      <pose>0 0 {sz/2} 0 -0 0</pose>
      <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      <surface><contact><ode/><ode/></contact><bounce/></surface>
    </collision>"""


def wall_link(name, sx, sy, sz, px, py, pz, ry=0, r=0.7, g=0.7, b=0.7):
    ry_str = f"0 -0 {ry}" if ry else "0 -0 0"
    return f"""  <link name='{name}'>
    <pose>{px} {py} {pz} {ry_str}</pose>
    {wall_collision(name, sx, sy, sz)}
    {wall_visual(name, sx, sy, sz, r, g, b)}
    <self_collide>0</self_collide>
    <kinematic>0</kinematic>
  </link>"""


def box_model(name, sx, sy, sz, px, py, pz, ry=0, r=0.7, g=0.7, b=0.7, static=True):
    """通用盒子模型"""
    static_str = "1" if static else "0"
    ry_str = f"0 -0 {ry}" if ry else "0 -0 0"
    return f"""  <model name='{name}'>
    <static>{static_str}</static>
    <pose>{px} {py} {pz} {ry_str}</pose>
    <link name='{name}_link'>
      <inertial><mass>1</mass></inertial>
      <collision name='collision'>
        <pose>0 0 {sz/2} 0 -0 0</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface><contact><ode/></contact><bounce/></surface>
      </collision>
      <visual name='visual'>
        <pose>0 0 {sz/2} 0 -0 0</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
    </link>
  </model>"""


def include_model(name, px, py, pz, ry=0):
    """引用 models/ 中的现有家具模型"""
    ry_str = f"0 -0 {ry}" if ry else "0 -0 0"
    return f"""  <include>
    <uri>model://{name}</uri>
    <pose>{px} {py} {pz} {ry_str}</pose>
  </include>"""


def generate_world():
    lines = []
    lines.append('<?xml version="1.0" ?>')
    lines.append('<sdf version="1.6">')
    lines.append('  <world name="large_loop_test">')

    # --- 光照 ---
    lines.append("""    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.3 -0.8</direction>
      <attenuation><range>200</range></attenuation>
    </light>""")

    # 走廊内 4 个点光源
    for i, (lx, ly) in enumerate([(0, 0), (20, 0), (-20, 0), (0, 12)]):
        lines.append(f"""    <light name='point_light_{i}' type='point'>
      <pose>{lx} {ly} 3 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>25</range><constant>0.5</constant><linear>0.1</linear><quadratic>0.01</quadratic></attenuation>
    </light>""")

    # --- 物理/场景 ---
    lines.append("""    <gravity>0 0 -9.8</gravity>
    <physics name='physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>0</shadows>
      <grid>0</grid>
    </scene>""")

    # --- 地面 ---
    lines.append("""    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><contact><ode/></contact><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name='visual'>
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>""")

    # ============ 外墙 ============
    outer_walls = [
        # 底墙 (长边)
        ("outer_bottom", OUTER_X, WALL_THICK, WALL_H, 0, -HALF_OUT_Y, 0, 0, 0.65, 0.65, 0.65),
        # 顶墙 (长边)
        ("outer_top", OUTER_X, WALL_THICK, WALL_H, 0, HALF_OUT_Y, 0, 0, 0.65, 0.65, 0.65),
        # 左墙 (短边)
        ("outer_left", WALL_THICK, OUTER_Y, WALL_H, -HALF_OUT_X, 0, 0, 0, 0.65, 0.65, 0.65),
        # 右墙 (短边)
        ("outer_right", WALL_THICK, OUTER_Y, WALL_H, HALF_OUT_X, 0, 0, 0, 0.65, 0.65, 0.65),
    ]
    # 内墙 (形成走廊)
    inner_walls = [
        ("inner_bottom", INNER_X, WALL_THICK, WALL_H, 0, -HALF_IN_Y, 0, 0, 0.8, 0.75, 0.7),
        ("inner_top", INNER_X, WALL_THICK, WALL_H, 0, HALF_IN_Y, 0, 0, 0.8, 0.75, 0.7),
        ("inner_left", WALL_THICK, INNER_Y, WALL_H, -HALF_IN_X, 0, 0, 0, 0.8, 0.75, 0.7),
        ("inner_right", WALL_THICK, INNER_Y, WALL_H, HALF_IN_X, 0, 0, 0, 0.8, 0.75, 0.7),
    ]

    # 组装外墙和内墙到同一个 model 中 (便于管理)
    all_wall_links = ""
    for name, sx, sy, sz, px, py, pz, ry, r, g, b in outer_walls + inner_walls:
        all_wall_links += wall_link(name, sx, sy, sz, px, py, pz, ry, r, g, b) + "\n"

    lines.append(f"""  <model name='corridor_walls'>
    <static>1</static>
    <pose>0 0 0 0 -0 0</pose>
{all_wall_links}  </model>""")

    # ============ 延伸走廊 (30m 死胡同, 从北侧伸出) ============
    EXT_X = EXT_CORRIDOR_LEN
    ext_wall_links = ""
    # 延伸走廊的左墙
    ext_wall_links += wall_link("ext_left", WALL_THICK, EXT_X, WALL_H,
                                -CORRIDOR_W/2, HALF_IN_Y + EXT_X/2, 0, 0, 0.7, 0.7, 0.8)
    # 延伸走廊的右墙
    ext_wall_links += wall_link("ext_right", WALL_THICK, EXT_X, WALL_H,
                                CORRIDOR_W/2, HALF_IN_Y + EXT_X/2, 0, 0, 0.7, 0.7, 0.8)
    # 延伸走廊的底端 (开口处封一半, 形成门洞)
    # 在延伸走廊与主走廊连接处, 留出 2m 门洞
    # 左门柱
    ext_wall_links += wall_link("ext_gate_l", 0.2, 1.0, WALL_H,
                                -1.0, HALF_IN_Y, 0, 0, 0.7, 0.7, 0.8)
    # 右门柱
    ext_wall_links += wall_link("ext_gate_r", 0.2, 1.0, WALL_H,
                                1.0, HALF_IN_Y, 0, 0, 0.7, 0.7, 0.8)

    lines.append(f"""  <model name='ext_corridor'>
    <static>1</static>
    <pose>0 0 0 0 -0 0</pose>
{ext_wall_links}  </model>""")

    # 延伸走廊末端墙 (死胡同尽头)
    lines.append(box_model("ext_end_wall", CORRIDOR_W, WALL_THICK, WALL_H,
                           0, HALF_IN_Y + EXT_X, 0, 0, 0.7, 0.7, 0.8))

    # ============ 走廊立柱 (每 8m 一个) ============
    pillar_r = 0.4  # 立柱半径
    pillar_h = 2.5
    pillar_color = (0.6, 0.4, 0.2)  # 棕色木柱

    def pillar(name, px, py, r=pillar_r, h=pillar_h, cr=0.6, cg=0.4, cb=0.2):
        return f"""  <model name='{name}'>
    <static>1</static>
    <pose>{px} {py} 0 0 -0 0</pose>
    <link name='link'>
      <inertial><mass>10</mass></inertial>
      <collision name='collision'>
        <pose>0 0 {h/2} 0 -0 0</pose>
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
      </collision>
      <visual name='visual'>
        <pose>0 0 {h/2} 0 -0 0</pose>
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        <material>
          <ambient>{cr} {cg} {cb} 1</ambient>
          <diffuse>{cr} {cg} {cb} 1</diffuse>
        </material>
      </visual>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
    </link>
  </model>"""

    # 在走廊中放置立柱
    # 底部走廊: y = -HALF_OUT_Y + CORRIDOR_W/2 = -20 + 2 = -18
    # 顶部走廊: y = HALF_OUT_Y - CORRIDOR_W/2 = 18
    # 左侧走廊: x = -HALF_OUT_X + CORRIDOR_W/2 = -28
    # 右侧走廊: x = HALF_OUT_X - CORRIDOR_W/2 = 28

    pillar_positions = []
    # 底部走廊
    for x in range(int(-HALF_IN_X + 4), int(HALF_IN_X - 2), int(PILLAR_SPACING)):
        pillar_positions.append((x, -HALF_OUT_Y + CORRIDOR_W/2))
    # 顶部走廊
    for x in range(int(-HALF_IN_X + 4), int(HALF_IN_X - 2), int(PILLAR_SPACING)):
        pillar_positions.append((x, HALF_OUT_Y - CORRIDOR_W/2))
    # 左侧走廊
    for y in range(int(-HALF_IN_Y + 4), int(HALF_IN_Y - 2), int(PILLAR_SPACING)):
        pillar_positions.append((-HALF_OUT_X + CORRIDOR_W/2, y))
    # 右侧走廊
    for y in range(int(-HALF_IN_Y + 4), int(HALF_IN_Y - 2), int(PILLAR_SPACING)):
        pillar_positions.append((HALF_OUT_X - CORRIDOR_W/2, y))
    # 延伸走廊
    for y in range(int(HALF_IN_Y + 3), int(HALF_IN_Y + EXT_X - 2), int(PILLAR_SPACING)):
        pillar_positions.append((0, y))

    for i, (px, py) in enumerate(pillar_positions):
        lines.append(pillar(f"pillar_{i:03d}", px, py))

    # ============ 房间 (沿外墙布置, 5mx5m) ============
    ROOM_SIZE = 5.0
    room_configs = [
        # (name, px, py, 门朝向)
        ("room_a", HALF_OUT_X + ROOM_SIZE/2, -HALF_OUT_Y + ROOM_SIZE/2 + 2, 0),   # 右墙外, 底部
        ("room_b", HALF_OUT_X + ROOM_SIZE/2, HALF_OUT_Y - ROOM_SIZE/2 - 2, 0),    # 右墙外, 顶部
        ("room_c", -HALF_OUT_X - ROOM_SIZE/2, 0, 0),                               # 左墙外, 中间
        ("room_d", -5, -HALF_OUT_Y - ROOM_SIZE/2 - 8, 0),                          # 底下
    ]

    for name, px, py, _ in room_configs:
        # 房间底座 (地板抬高标记)
        lines.append(box_model(f"{name}_floor", ROOM_SIZE, ROOM_SIZE, 0.1,
                               px, py, 0.01, 0, 0.9, 0.9, 0.85))
        # 房间三面墙 (朝向走廊的一面开口)
        # 后墙
        dx = px + ROOM_SIZE/2 + 0.5 if px > 0 else px - ROOM_SIZE/2 - 0.5
        dy = py + ROOM_SIZE/2 + 0.5 if py > 0 else py - ROOM_SIZE/2 - 0.5

        # 用 box 简单表示房间 3 面墙
        if abs(px) > abs(py):  # 左右侧房间
            # 后墙 (远离走廊)
            back_x = px + ROOM_SIZE/2 + WALL_THICK/2 if px > 0 else px - ROOM_SIZE/2 - WALL_THICK/2
            lines.append(box_model(f"{name}_wall_back", WALL_THICK, ROOM_SIZE, WALL_H,
                                     back_x, py, 0, 0, 0.75, 0.7, 0.65))
            # 上下侧墙
            lines.append(box_model(f"{name}_wall_top", ROOM_SIZE, WALL_THICK, WALL_H,
                                     px, py + ROOM_SIZE/2 + WALL_THICK/2, 0, 0, 0.75, 0.7, 0.65))
            lines.append(box_model(f"{name}_wall_bottom", ROOM_SIZE, WALL_THICK, WALL_H,
                                     px, py - ROOM_SIZE/2 - WALL_THICK/2, 0, 0, 0.75, 0.7, 0.65))
        else:  # 上下侧房间
            back_y = py + ROOM_SIZE/2 + WALL_THICK/2 if py > 0 else py - ROOM_SIZE/2 - WALL_THICK/2
            lines.append(box_model(f"{name}_wall_back", ROOM_SIZE, WALL_THICK, WALL_H,
                                     px, back_y, 0, 0, 0.75, 0.7, 0.65))
            lines.append(box_model(f"{name}_wall_left", WALL_THICK, ROOM_SIZE, WALL_H,
                                     px - ROOM_SIZE/2 - WALL_THICK/2, py, 0, 0, 0.75, 0.7, 0.65))
            lines.append(box_model(f"{name}_wall_right", WALL_THICK, ROOM_SIZE, WALL_H,
                                     px + ROOM_SIZE/2 + WALL_THICK/2, py, 0, 0, 0.75, 0.7, 0.65))

    # ============ 房间内家具 ============
    furniture_rooms = [
        ("room_a", "Sofa_01", 0, 0, 0),
        ("room_b", "Tablet_01", 0, 0, 0),
        ("room_c", "ChairA_01", 0, 0, 0),
        ("room_d", "Trash_01", 0, 0, 0),
    ]
    for room_name, model_name, dx, dy, ry in furniture_rooms:
        # 从 room_configs 中找到坐标
        for cfg_name, px, py, _ in room_configs:
            if cfg_name == room_name:
                lines.append(include_model(model_name, px + dx, py + dy, 0, ry))
                break

    # ============ 走廊内放置一些简单障碍物 (用于增加特征) ============
    # 在拐角处放一些箱子
    obstacle_positions = [
        (-HALF_IN_X, -HALF_IN_Y, 0, 0.3, 0.3, 0.3),  # 左下拐角
        (HALF_IN_X, -HALF_IN_Y, 0, 0.3, 0.2, 0.1),    # 右下拐角
        (-HALF_IN_X, HALF_IN_Y, 0, 0.2, 0.3, 0.1),    # 左上拐角
        (HALF_IN_X, HALF_IN_Y, 0, 0.1, 0.2, 0.3),     # 右上拐角
    ]
    for i, (ox, oy, oz, cr, cg, cb) in enumerate(obstacle_positions):
        lines.append(box_model(f"obstacle_{i}", 0.6, 0.6, 0.6,
                                ox, oy, 0.3, 0, cr, cg, cb))

    # ============ 场景中额外放置几个大特征物体 ============
    # 在走廊十字路口中间放一个大花盆/柱子
    for i, (mx, my) in enumerate([(0, 0), (15, 8), (-12, -10), (8, -12)]):
        lines.append(box_model(f"feature_pillar_{i}", 1.0, 1.0, 2.0,
                                mx, my, 0, 0, 0.4, 0.6, 0.2))

    # ============ 添加一些现有家具模型 ============
    extra_furniture = [
        ("Vase_01", -8, -18, 0, 0),
        ("Vase_01", 8, -18, 0, 0),
        ("ChairA_01", -15, 18, 0, 0),
        ("ChairA_01", 15, 18, 0, 0),
        ("Trash_01", 26, -5, 0, 0),
        ("Trash_01", -26, 5, 0, 0),
        ("Dumbbell_01", 0, -18, 0, 0),
        ("Board_01", 0, 18, 0, 0),
    ]
    for model_name, mx, my, mz, mry in extra_furniture:
        lines.append(include_model(model_name, mx, my, mz, mry))

    # ============ Gui 相机 ============
    lines.append("""    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>0 -35 30 0 1.2 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>""")

    lines.append('  </world>')
    lines.append('</sdf>')
    return '\n'.join(lines)


if __name__ == '__main__':
    print(generate_world())
