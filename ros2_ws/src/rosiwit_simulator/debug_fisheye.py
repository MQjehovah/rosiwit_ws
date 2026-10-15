#!/usr/bin/env python3
import os, sys
from ament_index_python import get_package_share_directory

d = get_package_share_directory('rosiwit_simulator')
print("share dir:", d)

xacro_file = os.path.join(d, 'urdf', 'xacro', 'gazebo', 'mbot_with_fisheye_gazebo.xacro')
print("xacro file exists:", os.path.exists(xacro_file))

# Check if directory structure exists
urdf_dir = os.path.join(d, 'urdf')
print("urdf dir exists:", os.path.isdir(urdf_dir))
if os.path.isdir(urdf_dir):
    for root, dirs, files in os.walk(urdf_dir):
        for f in files:
            if f.endswith('.xacro'):
                print("  ", os.path.relpath(os.path.join(root, f), urdf_dir))
else:
    # Maybe the urdf is at source level
    src_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(d)))), 'src', 'rosiwit_simulator')
    print("src dir:", src_dir)
    xacro_src = os.path.join(src_dir, 'urdf', 'xacro', 'gazebo', 'mbot_with_fisheye_gazebo.xacro')
    print("xacro at src:", xacro_src, "exists:", os.path.exists(xacro_src))

# Try processing xacro
import xacro
try:
    doc = xacro.process_file(xacro_file)
    xml = doc.toxml()
    print("xacro OK: %d links, %d joints, %d sensors" % (xml.count('<link '), xml.count('<joint '), xml.count('<sensor')))
except Exception as e:
    print("xacro error:", e)
    # Try with source path
    src_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(d)))), 'src', 'rosiwit_simulator')
    xacro_src = os.path.join(src_dir, 'urdf', 'xacro', 'gazebo', 'mbot_with_fisheye_gazebo.xacro')
    if os.path.exists(xacro_src):
        print("Trying src path...")
        try:
            doc = xacro.process_file(xacro_src)
            xml = doc.toxml()
            print("xacro from src OK: %d links, %d joints, %d sensors" % (xml.count('<link '), xml.count('<joint '), xml.count('<sensor')))
        except Exception as e2:
            print("xacro from src error:", e2)
