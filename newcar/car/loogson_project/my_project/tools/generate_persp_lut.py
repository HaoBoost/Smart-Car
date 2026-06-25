"""
generate_persp_lut.py
离线生成「去畸变 + 透视变换」合并查找表，输出 persp_lut.h。
在 PC 上运行一次，把生成的 persp_lut.h 放到 smartCar/code/ 目录。

使用步骤：
  1. 用 Matlab/OpenCV 标定相机，填入下方 K 和 D
  2. 在赛道上标定透视变换 4 个点，填入 SRC_PTS / DST_PTS
  3. python generate_persp_lut.py
  4. 将生成的 persp_lut.h 复制到 smartCar/code/
"""

import cv2
import numpy as np

# ============================================================
#  1. 相机内参（Matlab/OpenCV 标定结果）
#     如果没有畸变（普通镜头），保持 D 全零即可
# ============================================================
# 示例：640×480 分辨率下的内参，需替换为实际标定值
K = np.array([
    [320.0,   0.0, 320.0],
    [  0.0, 320.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)

# 畸变系数 [k1, k2, p1, p2, k3]（针孔模型）
# 鱼眼模型请改用 cv2.fisheye.undistortPoints，并修改下方 undistort_pts()
D = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

# ============================================================
#  2. 透视变换标定点（基于 IMG_W × IMG_H 分辨率）
# ============================================================
IMG_W, IMG_H = 320, 120

SRC_PTS = np.float32([[102.5, 91.75], [220.75, 90.75], [195.0, 42.5], [127.5, 42.0]])
DST_PTS = np.float32([
    [145.14999389648438, 104.8499984741211],
    [174.85000610351562, 104.8499984741211],
    [174.85000610351562, 75.1500015258789],
    [145.14999389648438, 75.1500015258789],
])

# ============================================================
#  3. 生成 LUT
# ============================================================
M = cv2.getPerspectiveTransform(SRC_PTS, DST_PTS)

# 生成所有像素坐标网格
xs, ys = np.meshgrid(np.arange(IMG_W, dtype=np.float32),
                     np.arange(IMG_H, dtype=np.float32))
pts = np.stack([xs, ys], axis=-1).reshape(-1, 1, 2)  # (N,1,2)

# 去畸变（针孔模型）
# 如果是鱼眼，改为 cv2.fisheye.undistortPoints(pts, K, D, P=K)
undist = cv2.undistortPoints(pts, K, D, P=K).reshape(IMG_H, IMG_W, 2)

# 透视变换
ones = np.ones((IMG_H, IMG_W, 1), dtype=np.float32)
uvw  = np.concatenate([undist, ones], axis=-1)          # (H,W,3)
out  = (M.astype(np.float32) @ uvw.reshape(-1, 3).T).T  # (H*W, 3)
w    = out[:, 2:3]
out_xy = (out[:, :2] / w).reshape(IMG_H, IMG_W, 2)

lut_x = out_xy[:, :, 0]  # (H, W) float32
lut_y = out_xy[:, :, 1]

# ============================================================
#  4. 输出 C 头文件
# ============================================================
OUT = "persp_lut.h"

def arr2c(name, arr):
    h, w = arr.shape
    lines = [f"static const float {name}[{h}][{w}] = {{"]
    for row in arr:
        lines.append("    {" + ",".join(f"{v:.3f}f" for v in row) + "},")
    lines.append("};")
    return "\n".join(lines)

with open(OUT, "w") as f:
    f.write(f"""\
/**
 * persp_lut.h  —  自动生成，勿手动修改
 * 去畸变 + 透视变换合并查找表
 * IMG_W={IMG_W}, IMG_H={IMG_H}
 */
#pragma once
#define PERSP_LUT_W {IMG_W}
#define PERSP_LUT_H {IMG_H}

""")
    f.write(arr2c("g_lut_x", lut_x))
    f.write("\n\n")
    f.write(arr2c("g_lut_y", lut_y))
    f.write("\n")

print(f"已生成 {OUT}，请复制到 smartCar/code/")
