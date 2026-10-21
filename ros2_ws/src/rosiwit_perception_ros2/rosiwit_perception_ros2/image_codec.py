"""sensor_msgs/Image <-> numpy 转换小工具.

直接用 numpy 解码 ROS Image，避免 cv_bridge 对 numpy ABI 的强耦合
（Humble 的 cv_bridge 与 numpy>=2 不兼容时会直接抛 ``_ARRAY_API not found``）。
仅支持感知节点实际会用到的常见编码。
"""

import numpy as np
from sensor_msgs.msg import Image

_MONO8 = ('mono8', '8uc1')
_MONO16 = ('mono16', '16uc1')


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """ROS Image -> BGR uint8 (H, W, 3)。"""
    enc = msg.encoding.lower()
    h, w = int(msg.height), int(msg.width)
    step = int(msg.step) if msg.step else 0

    if enc in ('bgr8', '8uc3'):
        arr = _rows(msg.data, h, step, w * 3)
        return arr.reshape(h, w, 3).copy()

    if enc == 'rgb8':
        arr = _rows(msg.data, h, step, w * 3)
        rgb = arr.reshape(h, w, 3)
        return rgb[:, :, ::-1].copy()

    if enc == 'bgra8':
        arr = _rows(msg.data, h, step, w * 4)
        bgra = arr.reshape(h, w, 4)
        return bgra[:, :, :3].copy()

    if enc == 'rgba8':
        arr = _rows(msg.data, h, step, w * 4)
        rgba = arr.reshape(h, w, 4)
        return rgba[:, :, [2, 1, 0]].copy()

    if enc in _MONO8:
        arr = _rows(msg.data, h, step, w)
        return np.repeat(arr.reshape(h, w, 1), 3, axis=2)

    if enc in _MONO16:
        arr = _rows(msg.data, h, step, w * 2)
        gray16 = np.ascontiguousarray(
            arr.reshape(h, w, 2)).view(np.uint16)[:, :, 0]
        gray8 = (gray16 >> 8).astype(np.uint8)
        return np.repeat(gray8.reshape(h, w, 1), 3, axis=2)

    raise ValueError(f'unsupported image encoding: {msg.encoding}')


def bgr_to_imgmsg(image: np.ndarray, header=None, encoding: str = 'bgr8') -> Image:
    """BGR uint8 (H, W, 3) -> ROS Image。"""
    arr = np.ascontiguousarray(image, dtype=np.uint8)
    if arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(f'expected (H, W, 3) BGR image, got {arr.shape}')

    msg = Image()
    if header is not None:
        msg.header = header
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(arr.shape[1] * 3)
    msg.data = arr.tobytes()
    return msg


def _rows(data: bytes, height: int, step: int, row_bytes: int) -> np.ndarray:
    """按 step 去掉每行 padding，返回 (height, row_bytes) uint8。"""
    buf = np.frombuffer(data, dtype=np.uint8)
    if step <= row_bytes:
        expected = height * row_bytes
        if buf.size < expected:
            raise ValueError(
                f'image data too short: {buf.size} < {expected}')
        return buf[:expected].reshape(height, row_bytes)

    expected = (height - 1) * step + row_bytes
    if buf.size < expected:
        raise ValueError(f'image data too short: {buf.size} < {expected}')
    full = height * step
    if buf.size < full:
        # 最后一行可能省略 padding
        buf = np.pad(buf, (0, full - buf.size))
    return buf[:full].reshape(height, step)[:, :row_bytes]
