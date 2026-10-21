"""Ground Texture Mapping (ROS 无关核心).

Stitches AVM bird's-eye-view images into a growing global texture map using
alpha blending, with optional local template-match pose refinement.

仅依赖 numpy / cv2，不依赖任何 ROS 接口。
"""

import json
import math
import os

import numpy as np
import cv2


class GroundTextureMap:
    """Growing global ground-texture map built from AVM BEV frames."""

    def __init__(self,
                 bev_resolution=0.005,
                 bev_size=800,
                 initial_canvas_size=2000,
                 canvas_expand_step=1000,
                 ground_z=-0.11):
        self.bev_resolution = float(bev_resolution)
        self.bev_size = int(bev_size)
        self.bev_coverage = self.bev_size * self.bev_resolution
        self.bev_half = self.bev_coverage / 2.0
        self.canvas_expand_step = int(canvas_expand_step)
        self.ground_z = float(ground_z)

        self.canvas_size = int(initial_canvas_size)
        self.canvas_center_x = self.canvas_size // 2
        self.canvas_center_y = self.canvas_size // 2
        self.texture_canvas = np.zeros(
            (self.canvas_size, self.canvas_size, 3), dtype=np.uint8)
        self.weight_canvas = np.zeros(
            (self.canvas_size, self.canvas_size), dtype=np.float32)
        self.frame_count = 0

    # ------------------------------------------------------------------ #
    # 位姿精化 / 重定位 (模板匹配对齐已有纹理)
    # ------------------------------------------------------------------ #
    def refine_pose(self, bev, x, y, yaw):
        if self.frame_count < 5:
            return x, y, yaw  # not enough canvas yet
        x, y, yaw, _ = self._match_pose(bev, x, y, yaw, min_score=0.3)
        return x, y, yaw

    def localize(self, bev, x, y, yaw,
                 search_radius=0.5, search_angle=0.3, patch_r=150,
                 min_score=0.35):
        """在已有纹理地图中重定位。

        Returns ``(success, x, y, yaw, score)``。与 ``refine_pose`` 不同，
        这里不做 ``frame_count`` 限制，也不会修改地图，供重定位服务使用。
        """
        search_radius = float(np.clip(search_radius, 0.0, 2.0))
        search_angle = float(np.clip(search_angle, 0.0, math.pi))
        x, y, yaw, score = self._match_pose(
            bev, x, y, yaw, search_radius, search_angle, patch_r,
            min_score=0.0)
        return score >= min_score, x, y, yaw, score

    def _match_pose(self, bev, x, y, yaw,
                    search_radius_m=0.15, search_angle_rad=0.05,
                    patch_r=150, min_score=0.3):
        """在当前纹理画布中搜索最佳 (x, y, yaw)，返回 ``(x, y, yaw, score)``。"""
        res = self.bev_resolution
        search_radius = max(1, int(search_radius_m / res))
        search_angles = np.linspace(-search_angle_rad, search_angle_rad, 7)
        bcx_val = self.bev_size / 2.0

        bc = self.bev_size // 2
        patch_r = min(int(patch_r), bc)
        if patch_r <= 0:
            return x, y, yaw, -1.0

        best_score = -1.0
        best_dx = best_dy = 0
        best_dyaw = 0.0

        for d_yaw in search_angles:
            test_yaw = yaw + d_yaw
            sy = math.sin(test_yaw)
            cy = math.cos(test_yaw)
            cx_canvas = float(self.canvas_center_x)
            cy_canvas = float(self.canvas_center_y)

            T_x = bcx_val * (cy - sy) + x / res + cx_canvas
            T_y = -bcx_val * (sy + cy) - y / res + cy_canvas
            M = np.array([[sy, -cy, T_x], [cy, sy, T_y]], dtype=np.float32)

            warped = cv2.warpAffine(
                bev, M, (self.canvas_size, self.canvas_size),
                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                borderValue=0)

            px = int(x / res + cx_canvas)
            py = int(-y / res + cy_canvas)

            # 模板必须从 warp 后的 BEV 取：BEV 的 x 轴朝上，画布 y 轴朝下，
            # 两者相差 90°，直接拿原 BEV 的 patch 去匹配会永远对不上。
            tx0, ty0 = px - patch_r, py - patch_r
            tx1, ty1 = px + patch_r, py + patch_r
            if tx0 < 0 or ty0 < 0 or tx1 > self.canvas_size or ty1 > self.canvas_size:
                continue
            template = cv2.cvtColor(
                warped[ty0:ty1, tx0:tx1], cv2.COLOR_BGR2GRAY)

            search_r = patch_r + search_radius
            x0 = max(0, px - search_r)
            y0 = max(0, py - search_r)
            x1 = min(self.canvas_size, px + search_r)
            y1 = min(self.canvas_size, py + search_r)
            if x1 - x0 < patch_r * 2 or y1 - y0 < patch_r * 2:
                continue

            search_region = cv2.cvtColor(
                self.texture_canvas[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)

            if np.count_nonzero(search_region > 10) < patch_r * patch_r:
                continue

            try:
                result = cv2.matchTemplate(
                    search_region, template, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(result)
            except cv2.error:
                continue

            if max_val > best_score:
                best_score = max_val
                best_dx = max_loc[0] - search_radius
                best_dy = max_loc[1] - search_radius
                best_dyaw = d_yaw

        if best_score >= min_score:
            x += best_dx * res
            y += -best_dy * res   # flip Y
            yaw += best_dyaw

        return x, y, yaw, best_score

    # ------------------------------------------------------------------ #
    # 持久化
    # ------------------------------------------------------------------ #
    def save(self, path):
        """保存纹理地图（.npz，原子写入），返回实际文件路径。"""
        path = str(path)
        if not path.endswith('.npz'):
            path += '.npz'
        parent = os.path.dirname(os.path.abspath(path))
        os.makedirs(parent, exist_ok=True)

        meta = {
            'bev_resolution': self.bev_resolution,
            'bev_size': self.bev_size,
            'canvas_size': self.canvas_size,
            'canvas_center_x': self.canvas_center_x,
            'canvas_center_y': self.canvas_center_y,
            'canvas_expand_step': self.canvas_expand_step,
            'ground_z': self.ground_z,
            'frame_count': self.frame_count,
        }
        tmp = path + '.tmp.npz'
        np.savez_compressed(
            tmp,
            texture=self.texture_canvas,
            weight=self.weight_canvas,
            meta=np.array(json.dumps(meta)),
        )
        os.replace(tmp, path)
        return path

    @classmethod
    def load(cls, path):
        """从 ``save()`` 生成的 .npz 恢复纹理地图。"""
        with np.load(str(path), allow_pickle=False) as data:
            meta = json.loads(str(data['meta'].item()))
            obj = cls(
                bev_resolution=meta['bev_resolution'],
                bev_size=meta['bev_size'],
                initial_canvas_size=meta['canvas_size'],
                canvas_expand_step=meta['canvas_expand_step'],
                ground_z=meta['ground_z'],
            )
            texture = data['texture']
            weight = data['weight']
            if texture.shape != (meta['canvas_size'], meta['canvas_size'], 3):
                raise ValueError(f'texture shape mismatch: {texture.shape}')
            if weight.shape != (meta['canvas_size'], meta['canvas_size']):
                raise ValueError(f'weight shape mismatch: {weight.shape}')
            obj.texture_canvas = texture.astype(np.uint8, copy=True)
            obj.weight_canvas = weight.astype(np.float32, copy=True)
            obj.canvas_center_x = int(meta['canvas_center_x'])
            obj.canvas_center_y = int(meta['canvas_center_y'])
            obj.frame_count = int(meta['frame_count'])
            return obj

    # ------------------------------------------------------------------ #
    # 画布管理
    # ------------------------------------------------------------------ #
    def _build_weight_mask(self, size):
        cx = cy = size / 2.0
        y_coords, x_coords = np.mgrid[0:size, 0:size]
        dist = np.sqrt((x_coords - cx) ** 2 + (y_coords - cy) ** 2)
        max_dist = size / 2.0
        weight = 1.0 - np.clip(dist / max_dist, 0, 1) ** 2
        return weight.astype(np.float32)

    def _expand_canvas(self):
        new_size = self.canvas_size + self.canvas_expand_step * 2
        offset = self.canvas_expand_step
        new_texture = np.zeros((new_size, new_size, 3), dtype=np.uint8)
        new_weight = np.zeros((new_size, new_size), dtype=np.float32)
        new_texture[offset:offset + self.canvas_size,
                    offset:offset + self.canvas_size] = self.texture_canvas
        new_weight[offset:offset + self.canvas_size,
                   offset:offset + self.canvas_size] = self.weight_canvas
        self.texture_canvas = new_texture
        self.weight_canvas = new_weight
        self.canvas_size = new_size
        self.canvas_center_x += offset
        self.canvas_center_y += offset

    # ------------------------------------------------------------------ #
    # 主流程: 用近似位姿把一帧 BEV 融合进全局纹理
    # ------------------------------------------------------------------ #
    def integrate(self, bev, x, y, yaw, refine=True):
        """Stitch one BEV frame given an approximate pose.

        Returns the (possibly corrected) pose ``(x, y, yaw)``.
        """
        if refine:
            x, y, yaw = self.refine_pose(bev, x, y, yaw)

        res = self.bev_resolution
        bcx = self.bev_size / 2.0

        px = self.canvas_center_x + int(x / res)
        py = self.canvas_center_y - int(y / res)
        margin = int(self.bev_half / res) + 10

        if (px - margin < 0 or px + margin >= self.canvas_size or
                py - margin < 0 or py + margin >= self.canvas_size):
            self._expand_canvas()
            px = self.canvas_center_x + int(x / res)
            py = self.canvas_center_y - int(y / res)

        sy = math.sin(yaw)
        cy = math.cos(yaw)
        cx = float(self.canvas_center_x)
        cy_canvas = float(self.canvas_center_y)

        T_x = bcx * (cy - sy) + x / res + cx
        T_y = -bcx * (sy + cy) - y / res + cy_canvas
        M = np.array([[sy, -cy, T_x], [cy, sy, T_y]], dtype=np.float32)

        warped = cv2.warpAffine(
            bev, M, (self.canvas_size, self.canvas_size),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
            borderValue=0)

        weight_full = self._build_weight_mask(self.bev_size)
        warped_weight = cv2.warpAffine(
            weight_full, M, (self.canvas_size, self.canvas_size),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
            borderValue=0)

        valid = warped_weight > 0.01
        w_new = warped_weight[valid]
        w_old = self.weight_canvas[valid]
        total_w = w_old + w_new
        safe = np.maximum(total_w, 1e-6)

        old_c = self.texture_canvas[valid].astype(np.float32)
        new_c = warped[valid].astype(np.float32)
        blended = (old_c * w_old[:, None] + new_c * w_new[:, None]) / safe[:, None]

        self.texture_canvas[valid] = blended.astype(np.uint8)
        self.weight_canvas[valid] = np.minimum(total_w, 10.0)

        self.frame_count += 1
        return x, y, yaw

    # ------------------------------------------------------------------ #
    # 输出 (返回纯 numpy / 数值, 由接口层转 ROS 消息)
    # ------------------------------------------------------------------ #
    def bounds(self, pad=50):
        ys, xs = np.where(self.weight_canvas > 0.01)
        if len(ys) == 0:
            return None
        y0 = max(0, int(ys.min()) - pad)
        y1 = min(self.canvas_size, int(ys.max()) + pad)
        x0 = max(0, int(xs.min()) - pad)
        x1 = min(self.canvas_size, int(xs.max()) + pad)
        return (x0, y0, x1, y1)

    def crop_texture(self, pad=50):
        b = self.bounds(pad)
        if b is None:
            return None, None
        x0, y0, x1, y1 = b
        return self.texture_canvas[y0:y1, x0:x1].copy(), b

    def occupancy_grid(self, pad=50):
        """Return ``(data, origin_x, origin_y, width, height)`` or ``None``.

        ``data`` is a flat ``np.int8`` array: 0 = observed(free), -1 = unknown.
        """
        b = self.bounds(pad)
        if b is None:
            return None
        x0, y0, x1, y1 = b
        sub_w = self.weight_canvas[y0:y1, x0:x1]
        sub_t = self.texture_canvas[y0:y1, x0:x1]

        gray = cv2.cvtColor(sub_t, cv2.COLOR_BGR2GRAY)
        data = np.full_like(gray, -1, dtype=np.int8)
        data[sub_w > 0.01] = 0

        res = self.bev_resolution
        origin_x = (x0 - self.canvas_center_x) * res
        origin_y = -(y1 - self.canvas_center_y) * res  # flip Y
        return data.flatten(), float(origin_x), float(origin_y), \
            int(x1 - x0), int(y1 - y0)
