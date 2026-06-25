/**
 * imgproc_port.h
 * ============================================================
 * 上交视觉算法移植层（龙芯 Linux 平台）
 *
 * 原始来源：上海交通大学 AuTop 战队 imgproc.c
 * 移植改动：
 *   - 去掉 AT_ITCM_SECTION_INIT / AT_DTCM_SECTION_ALIGN_INIT 宏
 *     （这两个宏是 RT1064 嵌入式专用，用于把函数/数据放到高速内存，
 *      龙芯 Linux 不需要，直接展开为变量本身）
 *   - 去掉 common.h 依赖，改用标准 C 头文件
 *   - 所有函数改为 static inline，直接 #include 即可使用
 *
 * 包含算法：
 *   1. findline_lefthand_adaptive   左手迷宫爬线
 *   2. findline_righthand_adaptive  右手迷宫爬线
 *   3. blur_points                  点集三角滤波
 *   4. resample_points              点集等距采样
 *   5. local_angle_points           局部角度变化率
 *   6. nms_angle                    角度非极大抑制
 *   7. track_leftline               左边线法向偏移→中线
 *   8. track_rightline              右边线法向偏移→中线
 * ============================================================
 */
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

// ---- 嵌入式内存段宏（龙芯平台直接展开，不做任何操作）----
// RT1064 上这两个宏把函数放到 ITCM（指令紧耦合内存，零等待周期）
// 龙芯 Linux 有 MMU 和 Cache，不需要手动指定内存段
#define AT_ITCM_SECTION_INIT(x)         x
#define AT_DTCM_SECTION_ALIGN_INIT(x,a) x

// ============================================================
//  图像结构体（与上交原版完全相同，保持兼容）
// ============================================================
/**
 * image_t — 灰度图像（uint8，每像素1字节）
 *   data   : 像素数据指针（行优先存储）
 *   width  : 图像宽度（像素）
 *   height : 图像高度（像素）
 *   step   : 每行字节数（通常等于 width，但可能有对齐填充）
 */
typedef struct image {
    uint8_t  *data;
    uint32_t  width;
    uint32_t  height;
    uint32_t  step;
} image_t;

/**
 * fimage_t — 浮点图像（float，每像素4字节）
 *   用于存储中间计算结果（如梯度图）
 */
typedef struct fimage {
    float    *data;
    uint32_t  width;
    uint32_t  height;
    uint32_t  step;
} fimage_t;

// AT_IMAGE(img, x, y) — 访问图像 (x,y) 处的像素值
// 注意：x 是列（水平），y 是行（垂直，向下为正）
#define AT_IMAGE(img, x, y)      ((img)->data[(y)*(img)->step+(x)])

// DEF_IMAGE — 用指针和尺寸快速初始化 image_t（step=width，无填充）
#define DEF_IMAGE(ptr, w, h)     {(ptr), (w), (h), (w)}

// ============================================================
//  工具函数
// ============================================================
// clip — 整数截断到 [low, up]
static inline int clip(int x, int low, int up) {
    return x < low ? low : (x > up ? up : x);
}

// clipf — 浮点截断到 [low, up]
static inline float clipf(float x, float low, float up) {
    return x < low ? low : (x > up ? up : x);
}

#ifndef PI
#define PI 3.14159265358979f
#endif

// ============================================================
//  图像清零
// ============================================================
static inline void clear_image(image_t *img) {
    memset(img->data, 0, img->width * img->height);
}

// ============================================================
//  迷宫爬线算法（上交核心算法）
// ============================================================
/**
 * findline_lefthand_adaptive() — 左手法则爬线
 *
 * 原理：
 *   模拟"左手贴墙"走迷宫。从起始点出发，优先向左前方走，
 *   遇到黑色（低于阈值）则右转，始终沿白线左边缘爬行。
 *   用于追踪赛道左边线。
 *
 * 自适应阈值：
 *   每步计算当前位置 block_size×block_size 邻域的均值，
 *   减去 clip_value（clip_value 为负数时，阈值 = 均值 + |clip_value|，
 *   更容易判断为白色，适合白线较暗的情况）。
 *
 * 方向编码：0=上, 1=右, 2=下, 3=左
 *
 * @param img        灰度图像
 * @param block_size 自适应阈值块大小（奇数，建议11）
 * @param clip_value 阈值偏移（建议-8，即阈值=均值+8）
 * @param x, y       起始点坐标（应在白线上）
 * @param pts        输出点集 [N][2]（x,y）
 * @param num        输入：pts 数组容量；输出：实际点数
 */
static inline void findline_lefthand_adaptive(
    image_t *img, int block_size, int clip_value,
    int x, int y, int pts[][2], int *num)
{
    // 方向向量：前方 和 左前方（左手法则）
    static const int dir_front[4][2]     = {{0,-1},{1,0},{0,1},{-1,0}};
    static const int dir_frontleft[4][2] = {{-1,-1},{1,-1},{1,1},{-1,1}};
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while (step < *num &&
           half < x && x < (int)img->width  - half - 1 &&
           half < y && y < (int)img->height - half - 1 &&
           turn < 4)  // turn>=4 说明被困住了，退出
    {
        // 计算局部自适应阈值（邻域均值 - clip_value）
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++)
            for (int dx = -half; dx <= half; dx++)
                local_thres += AT_IMAGE(img, x+dx, y+dy);
        local_thres /= block_size * block_size;
        local_thres -= clip_value;  // clip_value=-8 → thres = mean+8

        int front_val     = AT_IMAGE(img, x+dir_front[dir][0],     y+dir_front[dir][1]);
        int frontleft_val = AT_IMAGE(img, x+dir_frontleft[dir][0], y+dir_frontleft[dir][1]);

        if (front_val < local_thres) {
            // 前方是黑色 → 右转（左手法则：贴左墙右转）
            dir = (dir + 1) % 4; turn++;
        } else if (frontleft_val < local_thres) {
            // 左前方是黑色，前方是白色 → 直走
            x += dir_front[dir][0]; y += dir_front[dir][1];
            pts[step][0] = x; pts[step][1] = y; step++; turn = 0;
        } else {
            // 左前方也是白色 → 左转并走（贴左墙）
            x += dir_frontleft[dir][0]; y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;  // 左转 = (dir-1+4)%4
            pts[step][0] = x; pts[step][1] = y; step++; turn = 0;
        }
    }
    *num = step;
}

/**
 * findline_righthand_adaptive() — 右手法则爬线
 *
 * 与左手法则对称，用于追踪赛道右边线。
 * 优先向右前方走，遇到黑色则左转。
 */
static inline void findline_righthand_adaptive(
    image_t *img, int block_size, int clip_value,
    int x, int y, int pts[][2], int *num)
{
    static const int dir_front[4][2]      = {{0,-1},{1,0},{0,1},{-1,0}};
    static const int dir_frontright[4][2] = {{1,-1},{1,1},{-1,1},{-1,-1}};
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while (step < *num &&
           half < x && x < (int)img->width  - half - 1 &&
           half < y && y < (int)img->height - half - 1 &&
           turn < 4)
    {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++)
            for (int dx = -half; dx <= half; dx++)
                local_thres += AT_IMAGE(img, x+dx, y+dy);
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_val      = AT_IMAGE(img, x+dir_front[dir][0],      y+dir_front[dir][1]);
        int frontright_val = AT_IMAGE(img, x+dir_frontright[dir][0], y+dir_frontright[dir][1]);

        if (front_val < local_thres) {
            // 前方是黑色 → 左转（右手法则：贴右墙左转）
            dir = (dir + 3) % 4; turn++;
        } else if (frontright_val < local_thres) {
            // 右前方是黑色，前方是白色 → 直走
            x += dir_front[dir][0]; y += dir_front[dir][1];
            pts[step][0] = x; pts[step][1] = y; step++; turn = 0;
        } else {
            // 右前方也是白色 → 右转并走（贴右墙）
            x += dir_frontright[dir][0]; y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;  // 右转
            pts[step][0] = x; pts[step][1] = y; step++; turn = 0;
        }
    }
    *num = step;
}

// ============================================================
//  点集处理
// ============================================================
/**
 * blur_points() — 点集三角滤波
 *
 * 对点集做加权平均平滑，权重为三角形（中心权重最大，两端最小）。
 * 效果：消除爬线时的锯齿噪声，使边线更平滑。
 *
 * @param pts_in   输入点集 [num][2]
 * @param num      点数
 * @param pts_out  输出点集 [num][2]
 * @param kernel   滤波核大小（奇数，建议5）
 */
static inline void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) {
    int half = kernel / 2;
    // 三角形权重归一化系数：sum(1,2,...,half+1,...,2,1) = (half+1)^2
    float norm = (float)((2*half+2)*(half+1)) / 2.0f;
    for (int i = 0; i < num; i++) {
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) {
            int k = clip(i+j, 0, num-1);
            float w = (float)(half + 1 - abs(j));  // 三角形权重
            pts_out[i][0] += pts_in[k][0] * w;
            pts_out[i][1] += pts_in[k][1] * w;
        }
        pts_out[i][0] /= norm;
        pts_out[i][1] /= norm;
    }
}

/**
 * resample_points() — 点集等距采样
 *
 * 将不均匀间距的点集重采样为等间距点集。
 * 效果：使相邻点间距固定为 dist，方便用索引直接定位预瞄点。
 *
 * @param pts_in   输入点集
 * @param num1     输入点数
 * @param pts_out  输出点集
 * @param num2     输入：输出数组容量；输出：实际采样点数
 * @param dist     采样间距（像素）
 */
static inline void resample_points(
    float pts_in[][2], int num1,
    float pts_out[][2], int *num2, float dist)
{
    float remain = 0.f;
    int len = 0;
    for (int i = 0; i < num1-1 && len < *num2; i++) {
        float x0 = pts_in[i][0], y0 = pts_in[i][1];
        float dx = pts_in[i+1][0] - x0;
        float dy = pts_in[i+1][1] - y0;
        float dn = sqrtf(dx*dx + dy*dy);
        if (dn < 1e-6f) continue;
        dx /= dn; dy /= dn;  // 单位方向向量
        // 沿方向向量每隔 dist 采一个点
        while (remain < dn && len < *num2) {
            x0 += dx * remain; y0 += dy * remain;
            pts_out[len][0] = x0; pts_out[len][1] = y0;
            len++; dn -= remain; remain = dist;
        }
        remain -= dn;
    }
    *num2 = len;
}

/**
 * local_angle_points() — 局部角度变化率
 *
 * 计算点集中每个点处的曲线转角（弧度）。
 * 用拉格朗日中值定理近似：取前后各 dist 个点的连线方向，
 * 计算两方向的夹角。
 *
 * 用途：检测 L 型角点（赛道转角处角度变化率突变）。
 *
 * @param pts_in    输入点集（等距采样后）
 * @param num       点数
 * @param angle_out 输出角度数组（弧度，正=左转，负=右转）
 * @param dist      前后取点距离（采样点数，建议8）
 */
static inline void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist) {
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num-1) { angle_out[i] = 0; continue; }
        // 前向向量（当前点 → 后方 dist 点）
        float dx1 = pts_in[i][0] - pts_in[clip(i-dist,0,num-1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i-dist,0,num-1)][1];
        float dn1 = sqrtf(dx1*dx1 + dy1*dy1);
        // 后向向量（当前点 → 前方 dist 点）
        float dx2 = pts_in[clip(i+dist,0,num-1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i+dist,0,num-1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2*dx2 + dy2*dy2);
        if (dn1 < 1e-6f || dn2 < 1e-6f) { angle_out[i] = 0; continue; }
        float c1 = dx1/dn1, s1 = dy1/dn1;
        float c2 = dx2/dn2, s2 = dy2/dn2;
        // 叉积/点积 → atan2 得到有符号夹角
        angle_out[i] = atan2f(c1*s2 - c2*s1, c2*c1 + s2*s1);
    }
}

/**
 * nms_angle() — 角度变化率非极大抑制（NMS）
 *
 * 在 kernel 范围内，只保留局部最大的角度变化点，其余置零。
 * 效果：消除角点附近的多余响应，每个角点只保留一个最强点。
 *
 * @param angle_in   输入角度数组
 * @param num        点数
 * @param angle_out  输出角度数组（非极大点置零）
 * @param kernel     NMS 窗口大小（建议 ANGLE_DIST_N*2+1）
 */
static inline void nms_angle(float angle_in[], int num, float angle_out[], int kernel) {
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) {
            // 若邻域内有更大的角度变化，当前点置零
            if (fabsf(angle_in[clip(i+j,0,num-1)]) > fabsf(angle_out[i])) {
                angle_out[i] = 0; break;
            }
        }
    }
}

// ============================================================
//  边线偏移取中线
// ============================================================
/**
 * track_leftline() — 左边线法向偏移→中线
 *
 * 对左边线上每个点，计算该点的切线方向，
 * 然后沿切线法向向右偏移 dist 像素，得到中线上的对应点。
 *
 * 切线方向用拉格朗日中值定理近似（前后各 approx_num 点连线）。
 *
 * @param pts_in     左边线点集（等距采样后）
 * @param num        点数
 * @param pts_out    输出中线点集
 * @param approx_num 切线近似距离（采样点数，建议 ANGLE_DIST_N=8）
 * @param dist       偏移距离（像素，= 赛道宽度/2）
 */
static inline void track_leftline(
    float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist)
{
    for (int i = 0; i < num; i++) {
        // 切线方向向量（前后点连线）
        float dx = pts_in[clip(i+approx_num,0,num-1)][0] - pts_in[clip(i-approx_num,0,num-1)][0];
        float dy = pts_in[clip(i+approx_num,0,num-1)][1] - pts_in[clip(i-approx_num,0,num-1)][1];
        float dn = sqrtf(dx*dx + dy*dy);
        if (dn < 1e-6f) { pts_out[i][0] = pts_in[i][0]; pts_out[i][1] = pts_in[i][1]; continue; }
        dx /= dn; dy /= dn;
        // 法向量（切线旋转90°向右）：(-dy, dx) 旋转后为 (-dy, dx)
        // 向右偏移：x -= dy*dist, y += dx*dist
        pts_out[i][0] = pts_in[i][0] - dy * dist;
        pts_out[i][1] = pts_in[i][1] + dx * dist;
    }
}

/**
 * track_rightline() — 右边线法向偏移→中线
 *
 * 与 track_leftline 对称，向左偏移 dist 像素。
 * 向左偏移：x += dy*dist, y -= dx*dist
 */
static inline void track_rightline(
    float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist)
{
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i+approx_num,0,num-1)][0] - pts_in[clip(i-approx_num,0,num-1)][0];
        float dy = pts_in[clip(i+approx_num,0,num-1)][1] - pts_in[clip(i-approx_num,0,num-1)][1];
        float dn = sqrtf(dx*dx + dy*dy);
        if (dn < 1e-6f) { pts_out[i][0] = pts_in[i][0]; pts_out[i][1] = pts_in[i][1]; continue; }
        dx /= dn; dy /= dn;
        pts_out[i][0] = pts_in[i][0] + dy * dist;
        pts_out[i][1] = pts_in[i][1] - dx * dist;
    }
}

// ============================================================
//  大津法（OTSU）— 自动计算最佳二值化阈值
// ============================================================
/**
 * otsu_threshold()
 * 对图像指定区域计算大津阈值（最大类间方差法）。
 * 隔行隔列采样以加速，计算量极小。
 *
 * @param img  灰度图像
 * @param x0,y0,x1,y1  区域范围 [x0,x1) × [y0,y1)
 * @return 最佳阈值 (0~255)
 */
static inline int otsu_threshold(image_t *img, int x0, int x1, int y0, int y1) {
    int histogram[256] = {0};
    // 隔行隔列采样
    for (int y = y0; y < y1; y += 2)
        for (int x = x0; x < x1; x += 2)
            histogram[AT_IMAGE(img, x, y)]++;

    // 找灰度范围
    int min_v = 0, max_v = 255;
    while (min_v < 256 && histogram[min_v] == 0) min_v++;
    while (max_v > min_v && histogram[max_v] == 0) max_v--;
    if (max_v <= min_v + 1) return min_v;

    // 像素总数、灰度积分
    int total = 0;
    long integral = 0;
    for (int j = min_v; j <= max_v; j++) {
        total += histogram[j];
        integral += (long)histogram[j] * j;
    }

    // 遍历找最大类间方差
    int best_thres = min_v;
    float best_sigma = -1;
    int bg_count = 0;
    long bg_integral = 0;
    for (int j = min_v; j < max_v; j++) {
        bg_count += histogram[j];
        bg_integral += (long)histogram[j] * j;
        int fg_count = total - bg_count;
        float wb = (float)bg_count / total;
        float wf = (float)fg_count / total;
        float ub = (float)bg_integral / bg_count;
        float uf = (float)(integral - bg_integral) / fg_count;
        float sigma = wb * wf * (ub - uf) * (ub - uf);
        if (sigma > best_sigma) {
            best_sigma = sigma;
            best_thres = j;
        }
    }
    return best_thres;
}



