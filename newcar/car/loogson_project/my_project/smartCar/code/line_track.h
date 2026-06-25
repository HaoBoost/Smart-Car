/**
 * line_track.h
 * ============================================================
 * 视觉巡线核心算法（单头文件，直接 #include 即可使用）
 *
 * 算法流程（每帧执行一次）：
 *   1. process_image()  — 爬线 → 透视变换 → 三角滤波 → 等距采样
 *                          → 角点检测 → 中线生成 → 中线归一化
 *   2. calc_pure_pursuit() — 纯跟踪算法，输出航向角误差（度）
 *
 * 透视变换说明：
 *   优先使用离线生成的查找表 persp_lut.h（去畸变+透视合并，速度最快）。
 *   若 persp_lut.h 不存在，退回运行时矩阵乘法（无去畸变）。
 *   生成方法：PC 上运行 tools/generate_persp_lut.py，
 *   填入相机内参 K/D 和透视标定点，复制生成的头文件到本目录。
 *
 * 巡左/右中线切换：
 *   修改 g_track_side = 0（巡右中线，默认）或 1（巡左中线）
 *   在元素识别时可动态切换，实现灵活的路径跟踪。
 * ============================================================
 */
#pragma once

#include "imgproc_port.h"
#include "pid.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
// 透视变换模式：
//   USE_PERSP_LUT=1 使用离线查找表 persp_lut.h（需重新生成 LUT）
//   USE_PERSP_LUT=0 使用运行时透视矩阵（直接使用下方 SRC_PTS / DST_PTS）
#define USE_PERSP_LUT 1

#if USE_PERSP_LUT
#include "persp_lut.h"
#endif

// ============================================================
//  可调参数（根据实际赛道和摄像头修改）
// ============================================================
#define IMG_W           160     // 摄像头图像宽度（像素）
#define IMG_H           120   // 摄像头图像高度（像素）
#define POINTS_MAX      512     // 单侧边线最大点数（防止数组越界）
#define BLOCK_SIZE      7       // 自适应阈值块大小（必须为奇数）
#define CLIP_VALUE      (-8)    // 自适应阈值偏移（负值=更容易判白线）
#define SAMPLE_DIST     3.0f    // 等距采样间距（像素），越小中线越密
#define ANGLE_DIST_N    8       // 角度变化率计算距离（采样点数）
#define BLUR_KERNEL     5       // 边线三角滤波核大小（必须为奇数）
// 当前标定使用的 DST 宽度为 90.8 - 69.2 = 21.6 像素
#define ROAD_WIDTH_PX   21.6f   // 赛道宽度（透视变换后的像素数）
// 轴距控制参数保留现有调参值，可继续实车微调
#define WHEELBASE_PX    16.0f   // 纯跟踪轴距参数

// ============================================================
//  全局状态变量（供 main.cc 读取）
// ============================================================

#define  LOOK_AHEAD_POINT_EN 10
extern int look_ahead_point;  // 前瞻点数


// 原图坐标的左右边线（爬线直接输出）
extern int   g_ipts0[POINTS_MAX][2];    // 左边线（原图坐标）
extern int   g_ipts1[POINTS_MAX][2];    // 右边线（原图坐标）
extern int   g_ipts0_num;
extern int   g_ipts1_num;

// 等距采样后的左右边线点集
extern float g_rpts0s[POINTS_MAX][2];   // 左边线（透视变换后）
extern float g_rpts1s[POINTS_MAX][2];   // 右边线（透视变换后）
extern int   g_rpts0s_num;              // 左边线点数
extern int   g_rpts1s_num;              // 右边线点数

// 中线点集（由边线法向偏移得到）
extern float g_rptsc[POINTS_MAX][2];
extern int   g_rptsc_num;

// 等距重采样后的中线（用于纯跟踪）
extern float g_rptsn[POINTS_MAX][2];
extern int   g_rptsn_num;

// L 型角点（约90°，用于识别十字/圆环/车库，并截断边线）
extern bool  g_Lpt0_found;  // 左边线是否检测到 L 角点
extern bool  g_Lpt1_found;  // 右边线是否检测到 L 角点
extern int   g_Lpt0_id;     // 左边线 L 角点在 g_rpts0s 中的索引
extern int   g_Lpt1_id;     // 右边线 L 角点在 g_rpts1s 中的索引


extern float rptsc0[POINTS_MAX][2];  // 由左边线得到的中线
extern float rptsc1[POINTS_MAX][2];  // 由右边线得到的中线

// 纯跟踪输出：航向角误差（度）
//   正值 = 需左转（预瞄点在车身左侧）
//   负值 = 需右转（预瞄点在车身右侧）
extern float g_pure_angle;

// 车身在图像中的参考坐标（图像底部中心附近）
extern float g_cx;  // 车身 X（图像水平中心）
extern float g_cy;  // 车身 Y（图像底部偏上一点）

// 巡线方向选择：0=巡右中线（默认），1=巡左中线
// 在元素识别时可动态切换，例如：
//   g_track_side = 0;  // 正常跑圈，跟右侧边线
//   g_track_side = 1;  // 通过特殊元素，跟左侧边线
extern int g_track_side;

// 中线偏移系数：1.0=正常中线，>1.0=偏向外侧，<1.0=偏向内侧
// 在环岛入环时可以调大（如1.3），让中线离内线远一点，避免压内线
extern float g_centerline_offset;

// ============================================================
//  透视变换（OpenCV 矩阵，仅在 USE_PERSP_LUT=0 时使用）
// ============================================================
extern cv::Mat g_persp_M;
extern bool    g_persp_ready;

/**
 * camera_param_init()
 * 初始化透视变换矩阵。
 *
 * 使用方法：
 *   1. 运行 tools/calib_persp.py，鼠标点击赛道4个角点
 *   2. 将打印的 SRC_PTS 坐标填入下方 src[] 数组
 *   3. DST_PTS（dst[]）通常不需要修改
 *
 * 点的顺序：左下 → 右下 → 右上 → 左上
 */

// SRC_PTS = [[48.75, 73.25], [114.5, 74.25], [97.75, 44.75], [64.5, 44.5]]
// DST_PTS = [[69.19999694824219, 103.1500015258789], [90.80000305175781, 103.1500015258789], [90.80000305175781, 78.8499984741211], [69.19999694824219, 78.8499984741211]]


extern int line_point;
static inline void camera_param_init() {
    // ---- 根据实际摄像头标定后修改这4个点 ----
    // 原图坐标（像素）：赛道梯形区域的4个角
    // 点的顺序：左下 → 右下 → 右上 → 左上
    cv::Point2f src[4] = {
        { 48.75f, 73.25f },    // 左下角
        { 114.5f, 74.25f },    // 右下角
        { 97.75f, 44.75f },    // 右上角
        { 64.5f, 44.5f },      // 左上角
    };
    // 俯视图坐标（像素）：变换后的矩形区域
    cv::Point2f dst[4] = {
        { 69.19999694824219f, 103.1500015258789f },  // 左下角
        { 90.80000305175781f, 103.1500015258789f },  // 右下角
        { 90.80000305175781f, 78.8499984741211f },   // 右上角
        { 69.19999694824219f, 78.8499984741211f },   // 左上角
    };
    g_persp_M     = cv::getPerspectiveTransform(src, dst);
    g_persp_ready = true;
}

/**
 * persp_transform()
 * 将单个像素坐标做透视变换（原图坐标 → 鸟瞰图坐标）
 *
 * USE_PERSP_LUT=1：查表法（去畸变+透视合并，O(1)，最快）
 * USE_PERSP_LUT=0：运行时矩阵乘法（无去畸变，稍慢）
 */
static inline void persp_transform(float px, float py, float &ox, float &oy) {
#if USE_PERSP_LUT
    // 查表：直接用像素坐标索引 LUT 数组
    int ix = (int)(px + 0.5f);
    int iy = (int)(py + 0.5f);
    if (ix < 0 || ix >= PERSP_LUT_W || iy < 0 || iy >= PERSP_LUT_H) {
        ox = px; oy = py; return;  // 越界时原样返回
    }
    ox = g_lut_x[iy][ix];
    oy = g_lut_y[iy][ix];
#else
    // 矩阵乘法：w = m[6]*x + m[7]*y + m[8]，然后除以 w 归一化
    if (!g_persp_ready) { ox = px; oy = py; return; }
    double *m = (double*)g_persp_M.data;
    double w  = m[6]*px + m[7]*py + m[8];
    ox = (float)((m[0]*px + m[1]*py + m[2]) / w);
    oy = (float)((m[3]*px + m[4]*py + m[5]) / w);
#endif
}

// ============================================================
//  中线起始点归一化（上交算法关键步骤）
// ============================================================
/**
 * normalize_centerline()
 * 将中线的起始点固定到车身位置附近，避免左右中线切换时预瞄点抖动。
 *
 * 原理：
 *   - 固定起始点 = (g_cx, g_cy)，即图像底部中心（车身位置）
 *   - 若中线最近点比起始点更远（Y 更小）：在前面插入起始点
 *   - 若中线最近点比起始点更近（Y 更大）：截断超出部分
 *
 * 效果：无论从左边线还是右边线生成中线，起始点都相同，
 *       切换时预瞄点不会突变，舵机不会抖动。
 */
static inline void normalize_centerline() {
    if (g_rptsn_num < 2) return;

    float start_y = g_cy;   // 固定起始 Y = 车身位置
    float start_x = g_cx;   // 固定起始 X = 图像水平中心

    float bottom_y = g_rptsn[0][1];  // 中线第0个点的 Y（应最靠近车身）

    if (bottom_y < start_y - SAMPLE_DIST) {
        // 中线没有延伸到车身位置 → 在最前面插入起始点
        if (g_rptsn_num < POINTS_MAX) {
            memmove(&g_rptsn[1], &g_rptsn[0], sizeof(float) * 2 * g_rptsn_num);
            g_rptsn[0][0] = start_x;
            g_rptsn[0][1] = start_y;
            g_rptsn_num++;
        }
    } else if (bottom_y > start_y + SAMPLE_DIST) {
        // 中线延伸超过车身位置 → 找到第一个 Y <= start_y 的点，截断前面的
        int cut = 0;
        for (int i = 0; i < g_rptsn_num; i++) {
            if (g_rptsn[i][1] <= start_y) { cut = i; break; }
        }
        if (cut > 0) {
            memmove(&g_rptsn[0], &g_rptsn[cut],
                    sizeof(float) * 2 * (g_rptsn_num - cut));
            g_rptsn_num -= cut;
        }
    }
}

// ============================================================
//  图像处理主流程
// ============================================================
/**
 * process_image()
 * 输入：OpenCV 灰度图 frame（IMG_W × IMG_H，CV_8UC1）
 * 输出：更新全局 g_rpts0s / g_rpts1s / g_rptsc / g_rptsn 及角点标志
 *
 * 调用频率：与摄像头帧率相同（约 120Hz）
 */
static inline void process_image(const cv::Mat &frame) {
    // assert(frame.cols == IMG_W && frame.rows == IMG_H);
    // assert(frame.type() == CV_8UC1);

    // ---- 构造 image_t 包装（零拷贝，直接指向 OpenCV 内存）----
    image_t img;
    img.data   = frame.data;
    img.width  = IMG_W;
    img.height = IMG_H;
    img.step   = frame.step[0];


    // ---- 爬线起始点（图像底部中心附近）----
    // begin_y：从图像 80% 高度处开始，避免车头遮挡
    int begin_y = (int)(IMG_H * 0.70f) + begin_y_offset_l;
    int begin_x = IMG_W / 2;
    int half = BLOCK_SIZE / 2;

    // 用大津法(OTSU)自动计算起始行附近的白/黑分割阈值
    // OTSU找最大类间方差，天然就是白色赛道与黑色背景的最佳分界线
    // 隔行隔列采样，160×21区域仅~840次读取，计算量极小
    int start_thres = otsu_threshold(&img, 0, IMG_W,
                          clip(begin_y - 10, 0, IMG_H - 1),
                          clip(begin_y + 10, 0, IMG_H - 1) + 1);

    // 左边线起始点：从中心向左找到白色区域的左边缘
    int x0 = begin_x + begin_x_offset_l;
    while (x0 > half && AT_IMAGE(&img, x0-1, begin_y) > start_thres) x0--;

    // 右边线起始点：从中心向右找到白色区域的右边缘
    int x1 = begin_x + begin_x_offset_r;
    while (x1 < IMG_W-half-1 && AT_IMAGE(&img, x1+1, begin_y) > start_thres) x1++;

    // ---- 爬线（迷宫左手/右手法则）----
    // 只有起始点在白线上才爬，否则置0
    int ipts0_num = POINTS_MAX;
    if (AT_IMAGE(&img, x0, begin_y) > start_thres)
        findline_lefthand_adaptive(&img, BLOCK_SIZE, CLIP_VALUE, x0, begin_y, g_ipts0, &ipts0_num);
    else
        ipts0_num = 0;
    // 截断左边线：允许小幅回头（弯道/角点），大幅回头才截断（乱爬）
    if (ipts0_num > 1) {
        int min_y = g_ipts0[0][1];
        for (int i = 1; i < ipts0_num; i++) {
            if (g_ipts0[i][1] < min_y) min_y = g_ipts0[i][1];
            if (g_ipts0[i][1] > min_y + 3) { ipts0_num = i; break; }
        }
    }
    g_ipts0_num = ipts0_num;

    int ipts1_num = POINTS_MAX;
    if (AT_IMAGE(&img, x1, begin_y) > start_thres)
        findline_righthand_adaptive(&img, BLOCK_SIZE, CLIP_VALUE, x1, begin_y, g_ipts1, &ipts1_num);
    else
        ipts1_num = 0;
    // 截断右边线：同理
    if (ipts1_num > 1) {
        int min_y = g_ipts1[0][1];
        for (int i = 1; i < ipts1_num; i++) {
            if (g_ipts1[i][1] < min_y) min_y = g_ipts1[i][1];
            if (g_ipts1[i][1] > min_y + 3) { ipts1_num = i; break; }
        }
    }
    g_ipts1_num = ipts1_num;

    // ---- 透视变换：原图像素坐标 → 鸟瞰图坐标 ----
    // 变换后的坐标才能用于法向偏移取中线（赛道宽度在鸟瞰图中是固定像素数）
    static float rpts0[POINTS_MAX][2], rpts1[POINTS_MAX][2];
    for (int i = 0; i < ipts0_num; i++)
        persp_transform(g_ipts0[i][0], g_ipts0[i][1], rpts0[i][0], rpts0[i][1]);
    for (int i = 0; i < ipts1_num; i++)
        persp_transform(g_ipts1[i][0], g_ipts1[i][1], rpts1[i][0], rpts1[i][1]);

    // ---- 三角滤波：平滑边线，降低噪声干扰 ----
    static float rpts0b[POINTS_MAX][2], rpts1b[POINTS_MAX][2];
    blur_points(rpts0, ipts0_num, rpts0b, BLUR_KERNEL);
    blur_points(rpts1, ipts1_num, rpts1b, BLUR_KERNEL);

    // ---- 等距采样：使相邻点间距相等，方便后续角度计算和预瞄点索引 ----
    g_rpts0s_num = POINTS_MAX;
    resample_points(rpts0b, ipts0_num, g_rpts0s, &g_rpts0s_num, SAMPLE_DIST);
    g_rpts1s_num = POINTS_MAX;
    resample_points(rpts1b, ipts1_num, g_rpts1s, &g_rpts1s_num, SAMPLE_DIST);

    // ---- 角点检测（L型角点，70°~140° 角度变化率）----
    // 用于截断边线，避免跟到错误的线（如交叉路口）
    static float ang0[POINTS_MAX], ang1[POINTS_MAX];
    static float ang0n[POINTS_MAX], ang1n[POINTS_MAX];
    local_angle_points(g_rpts0s, g_rpts0s_num, ang0, ANGLE_DIST_N);
    local_angle_points(g_rpts1s, g_rpts1s_num, ang1, ANGLE_DIST_N);
    // NMS：非极大抑制，只保留局部最大角度变化点
    nms_angle(ang0, g_rpts0s_num, ang0n, ANGLE_DIST_N*2+1);
    nms_angle(ang1, g_rpts1s_num, ang1n, ANGLE_DIST_N*2+1);

    g_Lpt0_found = g_Lpt1_found = false;
    for (int i = 0; i < g_rpts0s_num; i++) {
        if (ang0n[i] == 0) continue;
        int im1 = clip(i - ANGLE_DIST_N, 0, g_rpts0s_num-1);
        int ip1 = clip(i + ANGLE_DIST_N, 0, g_rpts0s_num-1);
        // conf = 当前角度变化率 - 邻近点平均（突出程度）
        float conf = fabsf(ang0[i]) - (fabsf(ang0[im1]) + fabsf(ang0[ip1])) / 2.0f;
        // L 角点：70°~140°（约90°直角，十字/圆环/车库）
        if (!g_Lpt0_found && conf > 70.0f/180.0f*PI && conf < 140.0f/180.0f*PI) {
            g_Lpt0_id = i; g_Lpt0_found = true;
        }
    }
    for (int i = 0; i < g_rpts1s_num; i++) {
        if (ang1n[i] == 0) continue;
        int im1 = clip(i - ANGLE_DIST_N, 0, g_rpts1s_num-1);
        int ip1 = clip(i + ANGLE_DIST_N, 0, g_rpts1s_num-1);
        float conf = fabsf(ang1[i]) - (fabsf(ang1[im1]) + fabsf(ang1[ip1])) / 2.0f;
        if (!g_Lpt1_found && conf > 70.0f/180.0f*PI && conf < 140.0f/180.0f*PI) {
            g_Lpt1_id = i; g_Lpt1_found = true;
        }
    }

    // track_leftline(g_rpts0s, g_rpts0s_num, rptsc0, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f * g_centerline_offset);
    // track_rightline(g_rpts1s, g_rpts1s_num, rptsc1, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f * g_centerline_offset);
    
}


// ============================================================
//  纯跟踪算法（Pure Pursuit）
// ============================================================
/**
 * calc_pure_pursuit()
 * 从 g_rptsn 计算航向角误差，结果写入 g_pure_angle（度）
 *
 * 原理：
 *   1. 找中线上距车身最近的点作为"当前位置"
 *   2. 从当前位置向前数 AIM_DISTANCE_N 个点作为"预瞄点"
 *   3. 用纯跟踪公式计算需要的转向角：
 *      angle = atan(2 * L * sin(α) / d)
 *      其中 L=轴距，α=车身朝向与预瞄点方向的夹角，d=预瞄距离
 *
 * 调用频率：与 process_image 相同（约 120Hz）
 */
static inline void calc_pure_pursuit() {

    if (g_rptsn_num < 3) { g_pure_angle = 0; return; }

    // 找中线上距车身最近的点（用距离平方比较，避免开方）
    float min_dist2 = 1e10f;
    int begin_id = 0;
    for (int i = 0; i < g_rptsn_num; i++) {
        float dx = g_rptsn[i][0] - g_cx;
        float dy = g_rptsn[i][1] - g_cy;
        float d2 = dx*dx + dy*dy;
        if (d2 < min_dist2) { min_dist2 = d2; begin_id = i; }
    }

    // 检查预瞄点是否有效
    if (g_rptsn_num - begin_id < 3) { g_pure_angle = 0; return; }

    // 预瞄点索引（从最近点向前数 AIM_DISTANCE_N 步）
    int aim_idx = clip(begin_id + look_ahead_point, 0, g_rptsn_num - 1);

    // 预瞄点相对车身的偏移
    float dx = g_rptsn[aim_idx][0] - g_cx;
    // 图像 Y 轴向下为正，纯跟踪需要向前为正，所以取反；加轴距偏移
    float dy = g_cy - g_rptsn[aim_idx][1] + WHEELBASE_PX;
    float dn = sqrtf(dx*dx + dy*dy);
    if (dn < 1e-3f) { g_pure_angle = 0; return; }

    // 纯跟踪公式：angle = atan(2 * L * dx / dn²)
    // 负号：dx>0（预瞄点在右）→ 需右转 → 输出负值
    g_pure_angle = -atanf(2.0f * WHEELBASE_PX * dx / (dn * dn)) / PI * 180.0f;
}




static inline float calc_pure_pursuit_preview(int look_ahead, float fallback_angle)
{
    float pure_angle = fallback_angle;
    if (g_rptsn_num < 3) 
    {
        pure_angle = fallback_angle;
    }
    else
    {
        // 找中线上距车身最近的点（用距离平方比较，避免开方）
        float min_dist2 = 1e10f;
        int begin_id = 0;
        for (int i = 0; i < g_rptsn_num; i++) 
        {
            float dx = g_rptsn[i][0] - g_cx;
            float dy = g_rptsn[i][1] - g_cy;
            float d2 = dx*dx + dy*dy;
            if (d2 < min_dist2) { min_dist2 = d2; begin_id = i; }
        }
            // 检查预瞄点是否有效
            if (g_rptsn_num - begin_id < 3) 
            { 
                pure_angle = fallback_angle;
            }
            else
            {
                // 预瞄点索引（从最近点向前数 AIM_DISTANCE_N 步）
                int aim_idx = clip(begin_id + look_ahead, 0, g_rptsn_num - 1);
                // 预瞄点相对车身的偏移
                float dx = g_rptsn[aim_idx][0] - g_cx;
                // 图像 Y 轴向下为正，纯跟踪需要向前为正，所以取反；加轴距偏移
                float dy = g_cy - g_rptsn[aim_idx][1] + WHEELBASE_PX;
                float dn = sqrtf(dx*dx + dy*dy);
                if (dn < 1e-3f) 
                {
                    pure_angle = fallback_angle;
                }
                else
                {
                    // 纯跟踪公式：angle = atan(2 * L * dx / dn²)
                    // 负号：dx>0（预瞄点在右）→ 需右转 → 输出负值
                    pure_angle = -atanf(2.0f * WHEELBASE_PX * dx / (dn * dn)) / PI * 180.0f;
                }
            }

        }
    return pure_angle;
}

static inline float calc_pure_pursuit_better(int look_ahead) 
{
    static float last_pure_angle = 0.0f;
    last_pure_angle = calc_pure_pursuit_preview(look_ahead, last_pure_angle);
    return last_pure_angle;
}


/*-------------------------------------------------------------------------------------------------------------------
 * @brief  曲率自适应调速
 *
 * 原理：
 *   1. 取中线前方一段区间，计算每个点的局部转角（相邻向量夹角）
 *   2. 取最大转角作为"最急弯曲率"
 *   3. 曲率越大 → 速度系数越低；直道 → 系数接近 1.0
 *
 * @param  base_speed  基础目标速度（如 1000）
 * @param  min_ratio   最低速度比例（如 0.3，即最慢不低于 30%）
 * @param  max_ratio   最高速度比例（如 1.0，即直道满速）
 * @return 调整后的目标速度
 *
 * 用法：在 motor_task 里替代固定系数
 *   int adj_speed = speed_by_curvature(speed_target_l, 0.3f, 1.0f);
 *-------------------------------------------------------------------------------------------------------------------*/
extern float g_max_curvature;  // 当前帧最大曲率（度），可用于调试显示

static inline int speed_by_curvature(int base_speed, float min_ratio, float max_ratio)
{
    // 中线点数不够，保守减速
    if (g_rptsn_num < 5) return (int)(base_speed * min_ratio);

    // ---- 1. 计算中线前方区间的局部转角 ----
    // 从 begin（靠近车身）到 end（前方），取前瞻范围内的点
    int begin = 1;
    int end = g_rptsn_num - 1;
    if (end > 20) end = 20;  // 最多看前方30个采样点

    float max_angle = 0.0f;
    for (int i = begin; i < end; i++) {
        // 向量 A = P[i] - P[i-1]
        float ax = g_rptsn[i][0] - g_rptsn[i-1][0];
        float ay = g_rptsn[i][1] - g_rptsn[i-1][1];
        // 向量 B = P[i+1] - P[i]
        float bx = g_rptsn[i+1][0] - g_rptsn[i][0];
        float by = g_rptsn[i+1][1] - g_rptsn[i][1];

        float dot   = ax*bx + ay*by;
        float cross = ax*by - ay*bx;
        float angle = fabsf(atan2f(cross, dot));  // 0~PI，单位弧度

        if (angle > max_angle) max_angle = angle;
    }

    g_max_curvature = max_angle / PI * 180.0f;  // 转换为度，方便调试

    // ---- 2. 映射：曲率 → 速度系数 ----
    // angle_low  以下视为直道，满速
    // angle_high 以上视为急弯，最低速
    const float angle_low  = 5.0f  / 180.0f * PI;   // 5°
    const float angle_high = 40.0f / 180.0f * PI;    // 40°

    float ratio;
    if (max_angle <= angle_low) {
        ratio = max_ratio;               // 直道：满速
    } else if (max_angle >= angle_high) {
        ratio = min_ratio;               // 急弯：最低速
    } else {
        // 线性插值
        float t = (max_angle - angle_low) / (angle_high - angle_low);
        ratio = max_ratio - t * (max_ratio - min_ratio);
    }

    return (int)(base_speed * ratio);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左补线
  @param     补线的起点，终点
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的,不要乱补
-------------------------------------------------------------------------------------------------------------------*/
static inline void Left_Add_Line(float line_point[][2],int start_point,int x1,int y1,int x2,int y2)//左补线,补的是边界
{
    int i;
    int step;
    int hx;

    if (start_point < 0 || start_point >= POINTS_MAX) return;

    if(x1>=IMG_W-1)//起始点位置校正，排除数组越界的可能
       x1=IMG_W-1;
    else if(x1<=0)
        x1=0;
    if(y1>=IMG_H-1)
        y1=IMG_H-1;
    else if(y1<=0)
        y1=0;
    if(x2>=IMG_W-1)
        x2=IMG_W-1;
    else if(x2<=0)
        x2=0;
    if(y2>=IMG_H-1)
        y2=IMG_H-1;
    else if(y2<=0)
        y2=0;

    if (y1 == y2) {
        line_point[start_point][0] = x1;
        line_point[start_point][1] = y1;
        return;
    }

    step = (y2 > y1) ? 1 : -1;
    for(i = y1; ; i += step)//根据斜率补线即可
    {
        if (start_point >= POINTS_MAX) break;

        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=IMG_W-1)
            hx=IMG_W-1;
        else if(hx<=0)
            hx=0;

        line_point[start_point][0] = hx;
        line_point[start_point][1] = i;
        start_point++;

        if (i == y2) break;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右补线
  @param     补线的起点，终点
  @return    null
  Sample     Right_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的，不要乱补
-------------------------------------------------------------------------------------------------------------------*/
static inline void Right_Add_Line(float line_point[][2],int start_point,int x1,int y1,int x2,int y2)//右补线,补的是边界
{
    int i;
    int step;
    int hx;

    if (start_point < 0 || start_point >= POINTS_MAX) return;

    if(x1>=IMG_W-1)//起始点位置校正，排除数组越界的可能
       x1=IMG_W-1;
    else if(x1<=0)
        x1=0;
    if(y1>=IMG_H-1)
        y1=IMG_H-1;
    else if(y1<=0)
        y1=0;
    if(x2>=IMG_W-1)
        x2=IMG_W-1;
    else if(x2<=0)
        x2=0;
    if(y2>=IMG_H-1)
        y2=IMG_H-1;
    else if(y2<=0)
         y2=0;

    if (y1 == y2) {
        line_point[start_point][0] = x1;
        line_point[start_point][1] = y1;
        return;
    }

    step = (y2 > y1) ? 1 : -1;
    for(i = y1; ; i += step)//根据斜率补线即可
    {
        if (start_point >= POINTS_MAX) break;

        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=IMG_W-1)
            hx=IMG_W-1;
        else if(hx<=0)
            hx=0;

        line_point[start_point][0] = hx;
        line_point[start_point][1] = i;
        start_point++;

        if (i == y2) break;

        // frame.ptr<uint8_t>(i)[hx] = 0;
    }
}


static inline float SquareRootFloat(float number)
{
    long i;
    float x, y;
    const float f = 1.5F;

    x = number * 0.5F;
    y  = number;
    i  = * ( long * ) &y;
    i  = 0x5f3759df - ( i >> 1 );
    y  = * ( float * ) &i;
    y  = y * ( f - ( x * y * y ) );
    y  = y * ( f - ( x * y * y ) );
    return number * y;
}


/**
 * @brief 三点二次贝塞尔曲线生成函数（环岛补线核心算法）
 *
 * @param location1 起点坐标 [x, y]
 * @param location2 控制点坐标 [x, y]（决定曲线的弯曲程度）
 * @param location3 终点坐标 [x, y]
 * @param pts 输出数组，存储生成的曲线点
 * @param add_location 从pts的哪个位置开始存储（用于拼接）
 * @param dist 采样距离（相邻点之间的最小间隔，单位：像素）
 * @return 生成的点数
 *
 * 功能说明：
 * 1. 根据三个点生成平滑的二次贝塞尔曲线
 * 2. 按照固定距离采样点（避免点过密或过疏）
 * 3. 用于环岛处理中补全丢失的边界
 *
 * 数学原理：
 * 二次贝塞尔曲线公式：B(t) = (1-t)² * P0 + 2(1-t)t * P1 + t² * P2
 * 其中：P0=起点，P1=控制点，P2=终点，t∈[0,1]
 *
 * 应用场景：
 * - 环岛内部：补外侧边界（起点 → 控制点 → 拐点）
 * - 环岛出口：连接拐点和出口边界
 */
static inline int add_three_point_bezier(float location1[2], float location2[2], float location3[2],
                             float pts[][2], int add_location, int dist)
{
    // 边界检查：起始位置不能超出数组范围
    if (add_location >= IMG_W) {
        return 0;
    }

    // ========== 步骤1: 估算曲线长度 ==========
    // 计算起点到控制点的距离（len1）
    float dx1 = location2[0] - location1[0];
    float dy1 = location2[1] - location1[1];
    float len1 = SquareRootFloat(dx1 * dx1 + dy1 * dy1);

    // 计算控制点到终点的距离（len2）
    float dx2 = location3[0] - location2[0];
    float dy2 = location3[1] - location2[1];
    float len2 = SquareRootFloat(dx2 * dx2 + dy2 * dy2);

    // 估算曲线长度（实际曲线长度约为两段直线长度之和的80%）
    float curve_length = (len1 + len2) * 0.8f;

    // ========== 步骤2: 计算需要生成的点数 ==========
    // 根据曲线长度和采样距离计算点数
    int num_points = curve_length / dist + 1;
    // 限制点数不超过剩余空间
    if (num_points > (IMG_W - add_location)) {
        num_points = IMG_W - add_location;
    }

    // ========== 步骤3: 初始化采样参数 ==========
    float step = 1.0f / (num_points - 1);  // t的步进值（将[0,1]区间均分）
    float t = 0.0f;                        // 贝塞尔曲线参数，范围[0,1]
    int len = 0;                         // 已生成的点数
    float x, y, prev_x = 0, prev_y = 0;    // 当前点和前一个点的坐标
    float current_dist = 0;                // 当前点到前一个点的距离

    // ========== 步骤4: 添加起点 ==========
    pts[add_location + len][0] = location1[0];
    pts[add_location + len][1] = location1[1];
    prev_x = location1[0];
    prev_y = location1[1];
    len++;

    // ========== 步骤5: 生成曲线上的点 ==========
    t = step;  // 从第二个点开始（第一个点已经是起点）
    while (t <= 1.0f && len < (IMG_W - add_location)) {
        // 计算贝塞尔曲线的系数
        float t_inv = 1.0f - t;              // (1-t)
        float t_inv_squared = t_inv * t_inv; // (1-t)²
        float t_squared = t * t;             // t²

        // 二次贝塞尔曲线公式：B(t) = (1-t)² * P0 + 2(1-t)t * P1 + t² * P2
        x = t_inv_squared * location1[0] + 2 * t_inv * t * location2[0] + t_squared * location3[0];
        y = t_inv_squared * location1[1] + 2 * t_inv * t * location2[1] + t_squared * location3[1];

        // 计算当前点到前一个点的距离
        float dx = x - prev_x;
        float dy = y - prev_y;
        current_dist = SquareRootFloat(dx * dx + dy * dy);

        // 只有当距离大于等于采样距离时，才添加这个点（避免点过密）
        if (current_dist >= dist) {
            // 边界检查：点必须在图像范围内
            if (x < 0 || x >= IMG_W || y < 0 || y >= IMG_W) break;

            // 添加点到数组
            pts[add_location + len][0] = x;
            pts[add_location + len][1] = y;
            len++;

            // 更新前一个点的坐标
            prev_x = x;
            prev_y = y;
            current_dist = 0;  // 重置距离
        }

        t += step;  // 增加t，继续采样下一个点
    }

    // ========== 步骤6: 添加终点 ==========
    // 确保终点一定被添加（如果终点在图像范围内）
    if (len < (IMG_W - add_location) &&
        !(location3[0] < 0 || location3[0] >= IMG_W || location3[1] < 0 || location3[1] >= IMG_H)) {
        pts[add_location + len][0] = location3[0];
        pts[add_location + len][1] = location3[1];
        len++;
    }

    return len;  // 返回生成的点数
}


static inline int Arry_roll(float pts_in[][2], float pts_out[][2], int location1, int location2, int step, int step_Max)
{
    int j = location1;
    for(int i = location2; i < step; i++)
    {
        pts_out[j][0] = pts_in[i][0];
        pts_out[j++][1] = pts_in[i][1];
        if(j >= step_Max)break;
    }
    return j;
}

// 元素识别与状态机（依赖上面的变量和函数定义）
#include "cross.h"
extern int no_line_flag;
// ---- 根据 g_track_side 选择中线来源 ----
// g_track_side=0：优先用右边线（巡右中线，适合正常跑圈）
// g_track_side=1：优先用左边线（巡左中线，适合特殊元素）
static inline void track_task()
{
    float (*rpts_src)[2] = 0;
    int rpts_src_num = 0;

    auto clear_centerline = [&]() {
        g_rptsc_num = 0;
        g_rptsn_num = 0;
        normalize_centerline();
    };

    auto commit_centerline = [&](float sample_dist) -> bool {
        if (rpts_src == 0 || rpts_src_num < 3) {
            clear_centerline();
            return false;
        }

        g_rptsc_num = rpts_src_num;
        memcpy(g_rptsc, rpts_src, sizeof(g_rptsc[0]) * rpts_src_num);
        g_rptsn_num = POINTS_MAX;
        resample_points(g_rptsc, g_rptsc_num, g_rptsn, &g_rptsn_num, sample_dist);
        return true;
    };

    // 十字寻远线
    if (g_elem_type == ELEM_CROSS && g_cross_state == CROSS_BEGIN)
    {
        if (g_far_Lpt1_found && g_Lpt1_found && g_Lpt1_id < 30 && !no_line_flag) {
            rpts_src = rptsc1;
            rpts_src_num = g_far_rpts1s_num - g_far_Lpt1_id + g_rpts1s_num - g_Lpt1_id;
        }
        else if (g_far_Lpt0_found && g_Lpt0_found && g_Lpt0_id < 30 && !no_line_flag) {
            rpts_src = rptsc0;
            rpts_src_num = g_far_rpts0s_num - g_far_Lpt0_id;
        }
        else {
            clear_centerline();
            return;
        }

        if (!commit_centerline(SAMPLE_DIST)) return;
    }
    // 十字穿越中
    else if (g_elem_type == ELEM_CROSS && g_cross_state == CROSS_RUNNING)
    {
        if (g_far_Lpt0_found && !g_far_Lpt1_found) 
        { //左角点找到
            rpts_src = rptsc0;
            rpts_src_num = g_far_rpts0s_num - g_far_Lpt0_id;
        }
        else if (g_far_Lpt1_found && !g_far_Lpt0_found) 
        {//右角点找到
            rpts_src = rptsc1;
            rpts_src_num = g_far_rpts1s_num - g_far_Lpt1_id;
        }
        else if (g_far_Lpt0_found && g_far_Lpt1_found) //左右角点都找到
        {
            if (g_far_rpts0s_num > g_far_rpts1s_num) 
            {
                rpts_src = rptsc0;
                rpts_src_num = g_far_rpts0s_num - g_far_Lpt0_id;
            }
            else 
            {
                rpts_src = rptsc1;
                rpts_src_num = g_far_rpts1s_num - g_far_Lpt1_id;
            }
        }
        else 
        {
            clear_centerline();
            return;
        }

        if (!commit_centerline(SAMPLE_DIST)) return;
    }
    // 圆环 BEGIN：跟对侧直线
    else if (g_circle_state == CIRCLE_BEGIN && g_elem_type == ELEM_CIRCLE)
    {
        if (g_circle_type) { // 右圆环：跟左线
            g_track_side = 1;
            rpts_src = rptsc0;
            rpts_src_num = g_rpts0s_num;
        }
        else { // 左圆环：跟右线
            g_track_side = 0;
            rpts_src = rptsc1;
            rpts_src_num = g_rpts1s_num;
        }

        if (!commit_centerline(SAMPLE_DIST)) return;
    }
    // 圆环 APPROACH：跟圆环侧弧线，在角点处截断
    else if (g_circle_state == CIRCLE_APPROACH && g_elem_type == ELEM_CIRCLE)
    {
        if (g_circle_type == 0) { // 左圆环：跟左线入口弧
            g_track_side = 1;
            rpts_src = rptsc0;
            rpts_src_num = g_rpts0s_num;
            if (g_Lpt0_found && g_Lpt0_id >= 3)
                rpts_src_num = g_Lpt0_id;
        }
        else { // 右圆环：跟右线入口弧
            g_track_side = 0;
            rpts_src = rptsc1;
            rpts_src_num = g_rpts1s_num;
            if (g_Lpt1_found && g_Lpt1_id >= 3)
                rpts_src_num = g_Lpt1_id;
        }

        if (!commit_centerline(2)) return;
    }
    else if (g_circle_state == CIRCLE_RUNNING && g_elem_type == ELEM_CIRCLE)
    {
        if (g_circle_type == 0) { // 左圆环：跟右线跑
            g_track_side = 0;
            rpts_src = rptsc1;
            rpts_src_num = g_rpts1s_num;
        }
        else { // 右圆环：跟左线跑
            g_track_side = 1;
            rpts_src = rptsc0;
            rpts_src_num = g_rpts0s_num;
        }

        if (!commit_centerline(2)) return;
    }
    else if (g_elem_type == ELEM_CIRCLE && g_circle_state == CIRCLE_OUT)
    {
        if (g_circle_type == 0) { // 左圆环：跟左线出环
            g_track_side = 1;
            rpts_src = rptsc0;
            rpts_src_num = g_rpts0s_num;
        }
        else { // 右圆环：跟右线出环
            g_track_side = 0;
            rpts_src = rptsc1;
            rpts_src_num = g_rpts1s_num;
        }

        if (!commit_centerline(2)) return;
    }
    // 普通寻线
    else if (g_elem_type == ELEM_NONE)
    {
        if (g_track_side == 0) {
            if (g_rpts1s_num >= 3) {
                rpts_src = rptsc1;
                rpts_src_num = g_rpts1s_num;
            }
            else {
                rpts_src = rptsc0;
                rpts_src_num = g_rpts0s_num;
            }

            if (rpts_src == rptsc1 && g_rpts1s_num >= 10 && g_Lpt1_found)
                rpts_src_num = g_Lpt1_id;
        }
        else {
            if (g_rpts0s_num >= 3) {
                rpts_src = rptsc0;
                rpts_src_num = g_rpts0s_num;
            }
            else {
                rpts_src = rptsc1;
                rpts_src_num = g_rpts1s_num;
            }

            if (rpts_src == rptsc0 && g_rpts0s_num >= 10 && g_Lpt0_found)
                rpts_src_num = g_Lpt0_id;
        }

        if (rpts_src_num < 3) {
            if (rpts_src == rptsc1 && g_rpts0s_num >= 3) {
                rpts_src = rptsc0;
                rpts_src_num = g_rpts0s_num;
            }
            else if (rpts_src == rptsc0 && g_rpts1s_num >= 3) {
                rpts_src = rptsc1;
                rpts_src_num = g_rpts1s_num;
            }
        }

        if (!commit_centerline(SAMPLE_DIST)) return;
    }
    else if (g_elem_type == ELEM_CIRCLE &&
             (g_circle_state == CIRCLE_NONE || g_circle_state == CIRCLE_END))
    {
        if (g_circle_type == 0) { // 左圆环：跟右线
            track_rightline(g_rpts1s, g_rpts1s_num, rptsc1, ANGLE_DIST_N, ROAD_WIDTH_PX / 2.0f);
            g_track_side = 0;
            rpts_src = rptsc1;
            rpts_src_num = g_rpts1s_num;
        }
        else { // 右圆环：跟左线
            track_leftline(g_rpts0s, g_rpts0s_num, rptsc0, ANGLE_DIST_N, ROAD_WIDTH_PX / 2.0f);
            g_track_side = 1;
            rpts_src = rptsc0;
            rpts_src_num = g_rpts0s_num;
        }

        if (!commit_centerline(2)) return;
    }
    line_point = rpts_src_num;
    // ---- 中线起始点归一化（上交算法关键步骤）----
    normalize_centerline();
}



// float Deviation_calculation(void)//偏差计算(后轮)  通过增大偏差来提高p
// {
//    static float Deviation_all_last;
//    static float Deviation, //主偏差，总体偏差。
//     Deviation_2,           // 近处偏差，更偏向保持当前车身居中。
//     Deviation_3,           //提前量偏差，作用是提前转向。
//     Deviation_4,           //更远处的提前量，属于“超级提前量”
//     Deviation_all;         //最终合成后的总偏差

//    Deviation_2=get_center_error(77,97);//保持中间位置

// //   Deviation=get_center_error(0,97);//总体计算
// //   Deviation_3=get_center_error(40,63);//给定提前量

//    //动态提前量
//    if(g_elem_type == ELEM_NONE)//无元素
//    {
//      Deviation   = get_center_error(0,97);//总体计算
//      Deviation_3 = get_center_error(20,43);//给定提前量
//      Deviation_4 = get_center_error(3,23);//给定提前量的提前量（超级提前量）
//    }
//    else//可能有元素
//    {
//       Deviation = get_center_error(40,97);//总体计算
//       Deviation_3 = get_center_error(40,63);//给定提前量
//       Deviation_4 = 0;
//    }


// //   }
// /*********************************************///极限修正
//    if(fabsf(Deviation)>=10)
//      {
//         Deviation=Deviation*1.02;
//      }
//    if(fabsf(Deviation)>=15)
//      {
//         Deviation=Deviation*1.04;
//      }
//    if(fabsf(Deviation)>=20)
//      {
//         Deviation=Deviation*1.04;
//      }
//    if(fabsf(Deviation)>=25)
//      {
//        Deviation=Deviation*1.06;
//      }
//    if(fabsf(Deviation)>=30)
//       {
//        Deviation=Deviation*1.06;
//       }
//    if(fabsf(Deviation)>=32)
//      {
//      Deviation=Deviation*1.08;
//      }
//  /******************************************/
//  if(Longest_Column[0]<=65)//说明不为直道
//  {
//    if(fabsf(Deviation_2)>=10)//极限修正
//       {
//          Deviation_2=Deviation_2*1.0002;
//       }
//    if(fabsf(Deviation_2)>=15)//极限修正
//       {
//          Deviation_2=Deviation_2*1.0002;
//       }
//    if(fabsf(Deviation_2)>=20)//极限修正
//       {
//          Deviation_2=Deviation_2*1.0006;
//       }
//    if(fabsf(Deviation_2)>=25)//极限修正
//       {
//          Deviation_2=Deviation_2*1.0006;
//        }
//    if(fabsf(Deviation_2)>=30)//极限修正
//        {
//            Deviation_2=Deviation_2*1.0008;
//        }
// /**********************************************/
//    if(fabsf(Deviation_3)>=6)//极限修正
//        {
//             Deviation_3=Deviation_3*1.02;
//        }
//    if(fabsf(Deviation_3)>=10)//极限修正
//        {
//              Deviation_3=Deviation_3*1.06;
//        }
//    if(fabsf(Deviation_3)>=15)//极限修正
//        {
//              Deviation_3=Deviation_3*1.1;
//         }
//    if(fabsf(Deviation_3)>=20)//极限修正
//         {
//                 Deviation_3=Deviation_3*1.14;
//         }
//    if(fabsf(Deviation_3)>=25)//极限修正
//         {
//                 Deviation_3=Deviation_3*1.18;
//         }
//    if(fabsf(Deviation_3)>=30)//极限修正
//         {
//              Deviation_3=Deviation_3*1.08;
//         }
//    if(fabsf(Deviation_3)>=40)//极限修正
//         {
//               Deviation_3=Deviation_3*1.1;
//         }
//    }
// /*********************************************/

//    Deviation_all=(float)Deviation+(float)Deviation_2*0.86+(float)Deviation_3*9.98+(float)Deviation_4*2.6;//计算最终偏差
//    //偏差平滑滤波
//    Deviation_all=(0.2)*Deviation_all+(0.8)*Deviation_all_last;
//    Deviation_all_last=Deviation_all;//保存上次偏差

// //    zx_Deviation_speed=Deviation_all;
//    return Deviation_all;
// }




