"""
calib_persp.py
增强版交互式透视变换标定工具

功能：
  - 鼠标点击标定 4 个源点（SRC_PTS）
  - 滑动条调整平移和缩放
  - 实时预览鸟瞰图效果
  - 保存标定结果

操作：
  - 鼠标左键：点击标定点（按顺序：左下→右下→右上→左上）
  - 滑动条：调整平移 X/Y 和缩放
  - 's' 键：保存标定结果到 calib_result.txt
  - 'r' 键：重置标定点
  - 'q' 键：退出
"""

import argparse

import cv2
import numpy as np

# ============================================================
#  配置参数
# ============================================================
IMG_W, IMG_H = 160, 120
SCALE = 4  # 显示放大倍数（320×2=640, 120×2=240）

# 默认标定点（适配 320×120 分辨率）
DEFAULT_SRC_PTS = [[10, 110], [150, 110], [110, 20], [50, 20]]
DEFAULT_DST_PTS = [[40, 110], [120, 110], [120, 20], [40, 20]]
DEFAULT_IMAGE_PATH = "你的图片.png"

# ============================================================
#  全局变量
# ============================================================
src_pts = [list(p) for p in DEFAULT_SRC_PTS]  # 源点（可编辑）
dst_pts_base = np.float32(DEFAULT_DST_PTS)    # 基准目标点

click_idx = 0  # 当前正在设置第几个点（0-3）

# 平移和缩放参数
translate_x = 0
translate_y = 0
scale_factor = 1.0

image_frame = None  # 当前图像帧

# ============================================================
#  辅助函数
# ============================================================

def get_adjusted_dst_pts():
    """获取调整后的目标点（应用平移和缩放）"""
    # 计算中心点
    center = dst_pts_base.mean(axis=0)

    # 先缩放（相对于中心），再平移
    dst_adjusted = (dst_pts_base - center) * scale_factor + center
    dst_adjusted += np.array([translate_x, translate_y])

    return dst_adjusted.astype(np.float32)

def get_M():
    """计算透视变换矩阵"""
    if len(src_pts) < 4:
        return None
    try:
        dst_adjusted = get_adjusted_dst_pts()
        return cv2.getPerspectiveTransform(np.float32(src_pts), dst_adjusted)
    except:
        return None

def draw_frame(gray):
    """绘制源图像视图（带标定点）"""
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    vis = cv2.resize(vis, (IMG_W * SCALE, IMG_H * SCALE))

    # 绘制标定点
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]
    labels = ['LB', 'RB', 'RT', 'LT']

    for i, (px, py) in enumerate(src_pts):
        dx = int(px * SCALE)
        dy = int(py * SCALE)

        # 绘制圆点
        cv2.circle(vis, (dx, dy), 8, colors[i], -1, cv2.LINE_AA)
        cv2.circle(vis, (dx, dy), 10, (255, 255, 255), 2, cv2.LINE_AA)

        # 绘制标签
        cv2.putText(vis, labels[i], (dx + 12, dy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[i], 2, cv2.LINE_AA)

    # 连线显示标定区域
    if len(src_pts) >= 2:
        pts = np.array([[int(p[0] * SCALE), int(p[1] * SCALE)] for p in src_pts], np.int32)
        if len(src_pts) == 4:
            cv2.polylines(vis, [pts[[0, 1, 2, 3, 0]]], False, (0, 255, 255), 2, cv2.LINE_AA)
        else:
            cv2.polylines(vis, [pts], False, (0, 255, 255), 2, cv2.LINE_AA)

    # 显示提示信息
    info = f"Click point {click_idx + 1}/4: {labels[click_idx] if click_idx < 4 else 'Done'}"
    cv2.putText(vis, info, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

    return vis

def draw_bird(gray):
    """绘制鸟瞰图视图"""
    M = get_M()
    small = cv2.resize(gray, (IMG_W, IMG_H))

    if M is not None:
        # 应用透视变换
        bird = cv2.warpPerspective(small, M, (IMG_W, IMG_H))
    else:
        bird = small

    # 转为彩色并放大
    vis = cv2.cvtColor(bird, cv2.COLOR_GRAY2BGR)
    vis = cv2.resize(vis, (IMG_W * SCALE, IMG_H * SCALE))

    # 绘制目标矩形区域
    dst_adjusted = get_adjusted_dst_pts()
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]
    labels = ['LB', 'RB', 'RT', 'LT']

    for i, (px, py) in enumerate(dst_adjusted):
        dx = int(px * SCALE)
        dy = int(py * SCALE)
        cv2.circle(vis, (dx, dy), 6, colors[i], -1, cv2.LINE_AA)
        cv2.putText(vis, labels[i], (dx + 8, dy - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[i], 1, cv2.LINE_AA)

    # 连线显示目标区域
    pts = np.array([[int(p[0] * SCALE), int(p[1] * SCALE)] for p in dst_adjusted], np.int32)
    cv2.polylines(vis, [pts[[0, 1, 2, 3, 0]]], False, (0, 255, 255), 2, cv2.LINE_AA)

    # 显示参数信息
    info = f"TX:{translate_x:+3d} TY:{translate_y:+3d} Scale:{scale_factor:.2f}"
    cv2.putText(vis, info, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

    return vis

def on_mouse(event, x, y, flags, param):
    """鼠标回调函数"""
    global click_idx

    if event == cv2.EVENT_LBUTTONDOWN and click_idx < 4:
        # 转换坐标（从显示分辨率到基准分辨率）
        px = x / SCALE
        py = y / SCALE

        # 限制在图像范围内
        px = max(0, min(IMG_W - 1, px))
        py = max(0, min(IMG_H - 1, py))

        # 更新标定点
        src_pts[click_idx] = [px, py]
        click_idx += 1

        print(f"设置点 {click_idx}: ({px:.1f}, {py:.1f})")

def trackbar_callback(val):
    """滑动条回调函数"""
    global translate_x, translate_y, scale_factor

    translate_x = cv2.getTrackbarPos('Translate X', 'Control') - 50
    translate_y = cv2.getTrackbarPos('Translate Y', 'Control') - 50
    scale_factor = cv2.getTrackbarPos('Scale', 'Control') / 100.0

def reset_calibration():
    """重置标定"""
    global click_idx, src_pts, translate_x, translate_y, scale_factor

    src_pts = [list(p) for p in DEFAULT_SRC_PTS]
    click_idx = 0
    translate_x = 0
    translate_y = 0
    scale_factor = 1.0

    # 重置滑动条
    cv2.setTrackbarPos('Translate X', 'Control', 50)
    cv2.setTrackbarPos('Translate Y', 'Control', 50)
    cv2.setTrackbarPos('Scale', 'Control', 100)

    print("已重置标定")

def save_calibration():
    """保存标定结果"""
    dst_adjusted = get_adjusted_dst_pts()

    with open('calib_result.txt', 'w') as f:
        f.write("# 透视变换标定结果\n")
        f.write("# 复制以下内容到 generate_persp_lut.py\n\n")
        f.write(f"CALIB_SIZE = ({IMG_W}, {IMG_H})\n")
        f.write(f"SRC_PTS = np.float32({[list(p) for p in src_pts]})\n")
        f.write(f"DST_PTS = np.float32({dst_adjusted.tolist()})\n\n")
        f.write("# 调整参数\n")
        f.write(f"translate_x = {translate_x}\n")
        f.write(f"translate_y = {translate_y}\n")
        f.write(f"scale_factor = {scale_factor}\n")

    print("=" * 60)
    print("标定结果已保存到 calib_result.txt")
    print("=" * 60)
    print(f"SRC_PTS = {[list(p) for p in src_pts]}")
    print(f"DST_PTS = {dst_adjusted.tolist()}")
    print(f"平移: ({translate_x}, {translate_y})")
    print(f"缩放: {scale_factor}")
    print("=" * 60)

# ============================================================
#  主程序
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(description="Perspective calibration tool")
    parser.add_argument("image", nargs="?", default=None, help="image path")
    parser.add_argument("--image", dest="image_opt", default=None, help="image path")
    args = parser.parse_args()
    return args.image_opt or args.image or DEFAULT_IMAGE_PATH

def main(image_path=None):
    global image_frame

    # 加载图片
    if image_path is None:
        image_path = parse_args()
    img = cv2.imread(image_path)

    if img is None:
        print(f"错误：无法加载图片 {image_path}")
        print("请确保图片文件存在")
        return

    # 转为灰度图并调整到基准分辨率
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    image_frame = cv2.resize(gray, (IMG_W, IMG_H))

    # 创建窗口
    cv2.namedWindow('Source (Click to set points)')
    cv2.namedWindow('Bird View (Adjusted)')
    cv2.namedWindow('Control')

    # 设置鼠标回调
    cv2.setMouseCallback('Source (Click to set points)', on_mouse)

    # 创建滑动条
    cv2.createTrackbar('Translate X', 'Control', 50, 100, trackbar_callback)
    cv2.createTrackbar('Translate Y', 'Control', 50, 100, trackbar_callback)
    cv2.createTrackbar('Scale', 'Control', 100, 200, trackbar_callback)

    print("=" * 60)
    print("增强版透视变换标定工具")
    print("=" * 60)
    print("操作说明：")
    print("  1. 鼠标左键点击标定 4 个点（按顺序：LB→RB→RT→LT）")
    print("  2. 使用滑动条调整平移和缩放")
    print("  3. 按 's' 保存标定结果")
    print("  4. 按 'r' 重置标定")
    print("  5. 按 'q' 退出")
    print("=" * 60)

    # 主循环
    while True:
        # 绘制显示窗口
        cv2.imshow('Source (Click to set points)', draw_frame(image_frame))
        cv2.imshow('Bird View (Adjusted)', draw_bird(image_frame))

        # 按键处理
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("退出程序")
            break
        elif key == ord('s'):
            save_calibration()
        elif key == ord('r'):
            reset_calibration()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
