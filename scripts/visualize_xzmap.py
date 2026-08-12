#!/usr/bin/env python3
import json
import math
import os
import struct
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion

XZMAP = os.path.join(os.path.dirname(__file__), "..", "docs", "1776074080054", "map.xzmap")
PCD = os.path.join(os.path.dirname(__file__), "..", "docs", "1776074080054", "map.pcd")
FRAME = "map"


def read_pcd_points(path):
    with open(path, "rb") as f:
        line = f.readline()
        assert line.startswith(b"#"), line
        fields = {}
        while True:
            line = f.readline()
            if line.startswith(b"DATA"):
                fields["DATA"] = line.split()[1].strip().decode()
                break
            k, v = line.split(None, 1)
            fields[k.decode()] = v.strip()
        n = int(fields["POINTS"])
        blob = f.read(n * 12)
    pts = []
    for i in range(n):
        x, y, z = struct.unpack_from("<fff", blob, i * 12)
        pts.append((x, y, z))
    return pts


def rpy_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return Quaternion(
        x=round(sr * cp * cy - cr * sp * sy, 6),
        y=round(cr * sp * cy + sr * cp * sy, 6),
        z=round(cr * cp * sy - sr * sp * cy, 6),
        w=round(cr * cp * cy + sr * sp * sy, 6),
    )


class MapVisualizer(Node):
    def __init__(self):
        super().__init__("map_visualizer")
        qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self.pub_cloud = self.create_publisher(PointCloud2, "/map_pointcloud", qos)
        self.pub_path = self.create_publisher(Path, "/kf_trajectory", qos)

        self.points = read_pcd_points(PCD)
        with open(XZMAP) as f:
            data = json.load(f)
        self.poses = data["kf_frame_poses"]
        self.get_logger().info(f"loaded {len(self.points)} pts, {len(self.poses)} kf poses")

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = FRAME

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(self.points)
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        self.get_logger().info("packing point data...")
        cloud.data = struct.pack("<%df" % (len(self.points) * 3), *[v for p in self.points for v in p])
        self.cloud = cloud

        path = Path()
        path.header = header
        for pose in self.poses:
            ps = PoseStamped()
            ps.header = header
            ps.pose.position = Point(x=pose["x"] * 0.001, y=pose["y"] * 0.001, z=pose["z"] * 0.001)
            ps.pose.orientation = rpy_to_quat(
                pose["roll"] * 0.001, pose["pitch"] * 0.001, pose["yaw"] * 0.001
            )
            path.poses.append(ps)
        self.path = path
        self.publish_once()
        self.get_logger().info("publishing /map_pointcloud and /kf_trajectory")

    def publish_once(self):
        try:
            self.pub_cloud.publish(self.cloud)
            self.pub_path.publish(self.path)
            self.get_logger().info("published (transient_local latched)")
        except Exception as e:
            self.get_logger().error(f"publish failed: {e}")


def main():
    rclpy.init()
    node = MapVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
