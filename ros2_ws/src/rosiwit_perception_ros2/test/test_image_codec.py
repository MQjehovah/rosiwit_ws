"""Tests for sensor_msgs/Image <-> numpy conversion (ROS 2)."""

import numpy as np

from rosiwit_perception_ros2.image_codec import bgr_to_imgmsg, imgmsg_to_bgr


def test_bgr8_roundtrip():
    image = np.arange(4 * 5 * 3, dtype=np.uint8).reshape(4, 5, 3)
    msg = bgr_to_imgmsg(image)
    assert msg.encoding == 'bgr8'
    assert msg.step == 15
    out = imgmsg_to_bgr(msg)
    assert np.array_equal(out, image)


def test_rgb8_to_bgr():
    msg = bgr_to_imgmsg(np.zeros((1, 1, 3), np.uint8))
    msg.encoding = 'rgb8'
    msg.data = bytes([10, 20, 30])
    assert np.array_equal(imgmsg_to_bgr(msg), [[[30, 20, 10]]])


def test_mono8_to_bgr():
    msg = bgr_to_imgmsg(np.zeros((2, 2, 3), np.uint8))
    msg.encoding = 'mono8'
    msg.step = 2
    msg.data = bytes([0, 127, 128, 255])
    out = imgmsg_to_bgr(msg)
    assert out.shape == (2, 2, 3)
    assert np.array_equal(out[0, 0], [0, 0, 0])


def test_step_padding_stripped():
    msg = bgr_to_imgmsg(np.full((2, 2, 3), 7, np.uint8))
    # 每行末尾补 2 字节 padding
    msg.step = 8
    msg.data = bytes([7] * 6 + [0, 0] + [7] * 6 + [0, 0])
    out = imgmsg_to_bgr(msg)
    assert out.shape == (2, 2, 3)
    assert np.all(out == 7)
