#pragma once
#include "headfile.h"
#include "buzzer_sound.h"
#include "line_track.h"



// ============================================================
//  元素检测专用参数（通用参数在 line_track.h 中定义）
// ============================================================
#define CROSS_DIST_MAX        120.0f
#define STRAIGHT_MIN_PTS      30
#define STRAIGHT_ANGLE_MAX    (5.0f/180.0f*PI)
#define CIRCLE_LPOINT_MAX_ID  15
#define CIRCLE_LPOINT_MIN_Y   45.0f
#define CURVE_MIN_PTS         15
#define CURVE_ANGLE_MIN       (8.0f/180.0f*PI)
#define ZEBRA_TRANS_MIN       4


// ============================================================
//  元素类型与状态
// ============================================================
typedef enum {
    ELEM_NONE     = 0,  // 正常循线
    ELEM_CROSS    = 1,  // 十字路口
    ELEM_CIRCLE   = 2,  // 环岛
    ELEM_OBSTACLE = 3,  // 路障（红砖）
    ELEM_RAMP     = 4,  // 坡道
    ELEM_ZEBRA    = 5,  // 斑马线（发车区/终点）
    ELEM_CONE     = 6,  // 锥筒识别
    ELEM_NCNN     = 7,  // NCNN识别
} elem_type_t;

typedef enum {
    CIRCLE_NONE = 0,    // 未进入圆环
    CIRCLE_BEGIN,       // 检测到圆环，跟外圆直道等待入口出现
    CIRCLE_APPROACH,    // 内圆弧线出现，跟内圆弧线引导进环
    CIRCLE_RUNNING,     // 进入圆环，跟外圆弧线跑圈
    CIRCLE_OUT,         // 检测到出环角点，切内线出环
    CIRCLE_END,          // 等待外线恢复直道，退出
} circle_state_t;

typedef enum {
    CROSS_NONE = 0,
    CROSS_BEGIN,
    CROSS_RUNNING,
    CROSS_END
} cross_state_t;

typedef enum {
    NCNN_ACT_IDLE = 0,
    NCNN_ACT_WAIT_TRIGGER,
    NCNN_ACT_SLOW_CLASSIFY,
    NCNN_ACT_LOCKED,
    NCNN_ACT_BYPASS,
    NCNN_ACT_RECOVER,
    NCNN_ACT_DONE,
} ncnn_action_state_t;

typedef enum {
    NCNN_CMD_NONE = 0,
    NCNN_CMD_LEFT,
    NCNN_CMD_STRAIGHT,
    NCNN_CMD_RIGHT,
} ncnn_action_cmd_t;

// ============================================================
//  全局状态（供 main.cc 读取）
// ============================================================
extern elem_type_t    g_elem_type;
extern cross_state_t  g_cross_state;
extern circle_state_t g_circle_state;
// static int            g_circle_dir   = -1;
extern bool           g_zebra_found;
extern int            g_circle_confirm_cnt;
extern int            g_cross_confirm_cnt;
extern int            g_circle_stage_cnt;
extern int            g_circle_type;
#define CIRCLE_CONFIRM_FRAMES  10
#define CROSS_CONFIRM_FRAMES  6

// 元素超时退出机制
extern int g_cross_timeout_cnt;
extern int g_circle_timeout_cnt;
extern bool g_cross_timeout_flag;
extern bool g_circle_timeout_flag;


#define CROSS_TIMEOUT_FRAMES    150   // 十字超时帧数（约3秒@50fps）
#define CIRCLE_TIMEOUT_FRAMES   300   // 圆环超时帧数（约6秒@50fps）
#define CIRCLE_BEGIN_TIMEOUT    1
#define CIRCLE_APPROACH_TIMEOUT 20000
#define CIRCLE_RUNING_TIMEOUT 1500
#define CIRCLE_OUT_TIMEOUT 1500


extern int start_flag;
extern int g_run_elapsed_ms;
// gray 图像（定义在 main.cc）
extern cv::Mat frame;
extern cv::Mat frame_full;
extern cv::Mat gray;
extern cv::Mat g_red_model_input_40;
extern cv::Rect g_red_box_full;
extern cv::Rect g_red_model_box_full;
extern bool g_red_model_input_ready;
extern bool g_red_model_classify_enabled;
extern bool g_red_detected_waiting_distance;
extern bool g_red_model_cycle_done;
extern bool g_red_detected_this_frame;
extern float g_red_model_estimated_distance_cm;
extern std::string g_red_model_last_label;
extern float g_red_model_last_score;
extern long g_red_detect_total_us;
extern long g_red_hsv_local_us;
extern long g_red_hsv_full_ref_us;
extern long g_red_candidate_us;
extern long g_red_confirm_us;
extern long g_red_fallback_us;
extern long g_red_confirm_mask_us;
extern long g_red_confirm_morph_us;
extern long g_red_confirm_cc_us;
extern long g_red_confirm_bottom_scan_us;
extern int g_red_perf_sample_count;
extern ncnn_action_state_t g_ncnn_action_state;
extern ncnn_action_cmd_t g_ncnn_locked_cmd;
extern std::string g_ncnn_locked_label;
extern float g_ncnn_locked_score;
extern int g_ncnn_classify_stable_count;
extern std::string g_ncnn_last_label;
extern int g_ncnn_trigger_seen_count;
extern int g_ncnn_action_encoder_l;
extern int g_ncnn_action_encoder_r;
extern float g_ncnn_bypass_bias_deg;
extern bool g_ncnn_high_level_frozen;
extern int g_ncnn_red_center_y;
extern bool g_ncnn_red_valid_this_frame;
extern int g_ncnn_trigger_center_y;
extern int g_ncnn_trigger_confirm_frames;
extern int g_ncnn_classify_stable_frames;
extern float g_ncnn_classify_min_score;
extern float g_ncnn_bypass_offset_scale;
extern float g_ncnn_recover_offset_scale;
extern float g_ncnn_recover_bias_deg;
extern int g_ncnn_bypass_pass_counts;
extern int g_ncnn_recover_counts;
void stop_quick();
bool run_red_model_classification_if_needed(void);

extern int g_Lpt0_offset_x;   // 左角点偏移量—x
extern int g_Lpt0_offset_y;   // 左角点偏移量—y

extern int g_Lpt1_offset_x;   // 右角点偏移量—x
extern int g_Lpt1_offset_y;   // 右角点偏移量—y


extern int far_start_thres;     // 爬远线图像阈值
extern int cross_encoder_L;
extern int cross_encoder_R;
extern int circle_encoder_L;
extern int circle_encoder_R;
extern float circle_yaw_angle;
extern int circle_yaw_angle_flag;

// 补线出环：保存角点原图坐标，下一帧画到 gray 上
extern int   g_circle_line_x;
extern int   g_circle_line_y;
extern bool  g_circle_line_active;

// 原图坐标的远处的左右边线（爬线直接输出）
extern int   g_far_ipts0[POINTS_MAX][2];    // 远处左边线（原图坐标）
extern int   g_far_ipts1[POINTS_MAX][2];    // 远处右边线（原图坐标）
extern int   g_far_ipts0_num;
extern int   g_far_ipts1_num;

// 等距采样后的远处左右边线点集
extern float g_far_rpts0s[POINTS_MAX][2];   // 左边线（透视变换后）
extern float g_far_rpts1s[POINTS_MAX][2];   // 右边线（透视变换后）
extern int   g_far_rpts0s_num;              // 左边线点数
extern int   g_far_rpts1s_num;              // 右边线点数

extern bool  g_far_Lpt0_found;  // 左边线是否检测到 L 角点
extern bool  g_far_Lpt1_found;  // 右边线是否检测到 L 角点
extern int   g_far_Lpt0_id;     // 左边线 L 角点在 g_rpts0s 中的索引
extern int   g_far_Lpt1_id;     // 右边线 L 角点在 g_rpts1s 中的索引
extern int   g_circle_last_high;



extern int   g_far_begin_last[2][2];
extern int   g_far_begin_last_flag[2];  // 0为左，1为右


//反向透视变换（缓存逆矩阵避免重复计算）
extern cv::Mat g_inv_persp_M;  // 缓存逆矩阵
extern bool g_inv_persp_M_initialized;

static inline bool is_valid_line_point_index(int idx, int num);

static inline void inv_persp_transform(float bx, float by, float &ox, float &oy) {
    // 首次调用时计算并缓存逆矩阵
    if (!g_inv_persp_M_initialized) {
        g_inv_persp_M = g_persp_M.inv();
        g_inv_persp_M_initialized = true;
    }

    double *m = (double*)g_inv_persp_M.data;
    double w  = m[6]*bx + m[7]*by + m[8];
    ox = (float)((m[0]*bx + m[1]*by + m[2]) / w);
    oy = (float)((m[3]*bx + m[4]*by + m[5]) / w);
}
// ============================================================
//  辅助函数
// ============================================================

// 判断点集是否为长直道（连续 STRAIGHT_MIN_PTS 点角度 < 5°）
static inline bool is_long_straight(float pts[][2], int num) {
    if (num < STRAIGHT_MIN_PTS) return false;
    static float ang[POINTS_MAX];
    local_angle_points(pts, num, ang, ANGLE_DIST_N);
    int cnt = 0;
    for (int i = ANGLE_DIST_N; i < num - ANGLE_DIST_N; i++) {
        cnt = (fabsf(ang[i]) < STRAIGHT_ANGLE_MAX) ? cnt + 1 : 0;
        if (cnt >= STRAIGHT_MIN_PTS) return true;
    }
    return false;
}

// 判断点集是否为弧线（连续 CURVE_MIN_PTS 点角度同向且 > 阈值）
static inline bool is_curve(float pts[][2], int num) {
    if (num < CURVE_MIN_PTS + ANGLE_DIST_N * 2) return false;
    static float ang[POINTS_MAX];
    local_angle_points(pts, num, ang, ANGLE_DIST_N);
    int cnt = 0;
    for (int i = ANGLE_DIST_N; i < num - ANGLE_DIST_N; i++) {
        if (fabsf(ang[i]) > CURVE_ANGLE_MIN)
            cnt++;
        else
            cnt = 0;
        if (cnt >= CURVE_MIN_PTS) return true;
    }
    return false;
}

static inline bool detect_zebra(const cv::Mat &gray) {
    if (gray.empty()) return false;
    int h = gray.rows, w = gray.cols;
    int scan_y = (int)(h * 0.75f);
    int trans = 0;
    bool last_white = gray.at<uint8_t>(scan_y, w/4) > 128;
    for (int x = w/4; x < 3*w/4; x++) {
        bool cur_white = gray.at<uint8_t>(scan_y, x) > 128;
        if (cur_white != last_white) { trans++; last_white = cur_white; }
    }
    return trans >= ZEBRA_TRANS_MIN;
}

// 两个 L 角点之间的欧拉距离
static inline float lpoint_dist() {
    if (!g_Lpt0_found || !g_Lpt1_found) return 1e6f;
    if (!is_valid_line_point_index(g_Lpt0_id, g_rpts0s_num)
        || !is_valid_line_point_index(g_Lpt1_id, g_rpts1s_num)) {
        return 1e6f;
    }
    float dx = g_rpts0s[g_Lpt0_id][0] - g_rpts1s[g_Lpt1_id][0];
    float dy = g_rpts0s[g_Lpt0_id][1] - g_rpts1s[g_Lpt1_id][1];
    return sqrtf(dx*dx + dy*dy);
}

static inline void far_fine_line(cv::Mat &frame ,int line_type ,int begin_x , int begin_y);
static inline void far_find_corners(int line_type);
static inline void circle_out();   // 环岛出环
static inline void Lengthen_Boundry(float line_point[][2],int line_type,int start_point,int end);

static inline int cross_clamp_int(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline bool is_valid_line_point_index(int idx, int num)
{
    return idx >= 0 && idx < num && idx < POINTS_MAX;
}

static inline bool rect_inside_image(const cv::Rect &rect, int cols, int rows)
{
    return rect.x >= 0 && rect.y >= 0 && rect.width > 0 && rect.height > 0
        && rect.x + rect.width <= cols && rect.y + rect.height <= rows;
}

static inline cv::Rect scale_rect_160_to_320(const cv::Rect &rect160)
{
    if (rect160.width <= 0 || rect160.height <= 0) return cv::Rect();

    const int x0 = cross_clamp_int(rect160.x * 2, 0, 319);
    const int y0 = cross_clamp_int(rect160.y * 2, 0, 239);
    const int x1 = cross_clamp_int((rect160.x + rect160.width) * 2, 0, 320);
    const int y1 = cross_clamp_int((rect160.y + rect160.height) * 2, 0, 240);
    if (x1 <= x0 || y1 <= y0) return cv::Rect();
    return cv::Rect(x0, y0, x1 - x0, y1 - y0);
}

static inline cv::Rect build_fixed_40_model_box_on_320(const cv::Rect &red_box_320)
{
    if (red_box_320.width <= 0 || red_box_320.height <= 0) return cv::Rect();

    const int cx = red_box_320.x + red_box_320.width / 2;
    const int cy = red_box_320.y + red_box_320.height / 2;
    const int up_bias = 6;
    const int x0 = cross_clamp_int(cx - 20, 0, 320 - 40);
    const int y0 = cross_clamp_int(cy - 20 - up_bias, 0, 240 - 40);
    return cv::Rect(x0, y0, 40, 40);
}

// 在某一行上估计赛道左右边界，并得到赛道中心列。
// 这里只沿灰度图水平扫描，不修改任何循线结果。
static inline bool detect_lane_center_on_row(const cv::Mat &gray_img,
                                             int y,
                                             int lane_thres,
                                             int &left_edge,
                                             int &right_edge,
                                             int &center_x)
{
    if (gray_img.empty() || gray_img.type() != CV_8UC1
        || gray_img.cols != IMG_W || gray_img.rows != IMG_H) {
        return false;
    }

    y = cross_clamp_int(y, 0, IMG_H - 1);
    const uchar *row = gray_img.ptr<uchar>(y);
    int seed_x = IMG_W / 2;

    // 中心点不在白赛道上时，向左右就近找回赛道区域。
    if (row[seed_x] <= lane_thres) {
        bool found = false;
        for (int d = 1; d < IMG_W / 2; ++d) {
            const int xl = seed_x - d;
            const int xr = seed_x + d;
            if (xl >= 0 && row[xl] > lane_thres) {
                seed_x = xl;
                found = true;
                break;
            }
            if (xr < IMG_W && row[xr] > lane_thres) {
                seed_x = xr;
                found = true;
                break;
            }
        }
        if (!found) return false;
    }

    left_edge = seed_x;
    while (left_edge > 1 && row[left_edge - 1] > lane_thres) --left_edge;

    right_edge = seed_x;
    while (right_edge < IMG_W - 2 && row[right_edge + 1] > lane_thres) ++right_edge;

    if (right_edge - left_edge < 12) return false;

    center_x = (left_edge + right_edge) / 2;
    return true;
}

static inline bool is_red_hsv_pixel(const cv::Vec3b &hsv)
{
    // 红色跨越 HSV 色环两端，因此要同时判断低 H 和高 H 两段。
    // S/V 下限用于滤掉白色反光和过暗噪声。
    const int h = hsv[0];
    const int s = hsv[1];
    const int v = hsv[2];
    const bool hue_ok = (h <= 10) || (h >= 170);
    return hue_ok && s >= 80 && v >= 60;
}

static inline bool convert_bgr_roi_to_hsv(const cv::Mat &bgr_img,
                                          const cv::Rect &roi_rect,
                                          cv::Mat &hsv_roi)
{
    if (bgr_img.empty() || bgr_img.type() != CV_8UC3) return false;
    if (!rect_inside_image(roi_rect, bgr_img.cols, bgr_img.rows)) return false;

    cv::cvtColor(bgr_img(roi_rect), hsv_roi, cv::COLOR_BGR2HSV);
    return !hsv_roi.empty();
}

static inline bool is_red_box_above_vehicle_head(const cv::Rect &red_box)
{
    if (red_box.width <= 0 || red_box.height <= 0) return false;

    // 图像下方是车头附近区域，红色目标必须整体位于车头上方才允许触发。
    const int vehicle_head_row = static_cast<int>(IMG_H * 0.72f);
    const int red_bottom_y = red_box.y + red_box.height - 1;
    return red_bottom_y < vehicle_head_row;
}

static inline int collect_centerline_orig_points(cv::Point pts[], int max_pts)
{
    if (pts == 0 || max_pts <= 0 || g_rptsn_num <= 0) return 0;

    // 红色目标位于赛道前方，取中线前 100 个点已经足够覆盖搜索范围。
    const int sample_num = PID_MIN(g_rptsn_num, 100);
    int out_num = 0;
    int last_x = -1000;
    int last_y = -1000;

    for (int i = 0; i < sample_num && out_num < max_pts; ++i) 
    {
        float ox = 0.0f;
        float oy = 0.0f;
        inv_persp_transform(g_rptsn[i][0], g_rptsn[i][1], ox, oy);

        const int ix = cross_clamp_int(static_cast<int>(lroundf(ox)), 0, IMG_W - 1);
        const int iy = cross_clamp_int(static_cast<int>(lroundf(oy)), 0, IMG_H - 1);

        // 逆透视后相邻点可能挤到同一像素，去重后更适合沿线逐点扫描。
        if (out_num > 0 && std::abs(ix - last_x) <= 1 && std::abs(iy - last_y) <= 1) 
        {
            continue;
        }

        pts[out_num++] = cv::Point(ix, iy);
        last_x = ix;
        last_y = iy;
    }

    return out_num;
}

static inline float calc_red_ratio_on_hsv_band_roi(const cv::Mat &hsv_roi,
                                                   int cx,
                                                   int cy,
                                                   int half_w,
                                                   int half_h)
{
    if (hsv_roi.empty() || hsv_roi.type() != CV_8UC3) return 0.0f;

    const int x0 = cross_clamp_int(cx - half_w, 0, hsv_roi.cols - 1);
    const int x1 = cross_clamp_int(cx + half_w, 0, hsv_roi.cols - 1);
    const int y0 = cross_clamp_int(cy - half_h, 0, hsv_roi.rows - 1);
    const int y1 = cross_clamp_int(cy + half_h, 0, hsv_roi.rows - 1);

    int red_cnt = 0;
    int total_cnt = 0;
    for (int y = y0; y <= y1; ++y) {
        const cv::Vec3b *row = hsv_roi.ptr<cv::Vec3b>(y);
        for (int x = x0; x <= x1; ++x) {
            if (is_red_hsv_pixel(row[x])) {
                ++red_cnt;
            }
            ++total_cnt;
        }
    }

    if (total_cnt <= 0) return 0.0f;
    return static_cast<float>(red_cnt) / static_cast<float>(total_cnt);
}

static inline bool locate_red_candidate_on_centerline(const cv::Mat &hsv_roi,
                                                      const cv::Rect &roi_rect,
                                                      const cv::Point centerline_pts[],
                                                      int centerline_num,
                                                      cv::Point &candidate_pt,
                                                      int &candidate_row)
{
    candidate_pt = cv::Point(-1, -1);
    candidate_row = -1;

    if (hsv_roi.empty() || hsv_roi.type() != CV_8UC3) {
        return false;
    }
    if (centerline_num < 6) return false;

    float prev_ratio = 0.0f;
    float best_ratio = 0.0f;
    int best_idx = -1;
    int fallback_idx = -1;

    for (int i = 0; i < centerline_num; ++i) {
        const int local_x = centerline_pts[i].x - roi_rect.x;
        const int local_y = centerline_pts[i].y - roi_rect.y;
        if (local_x < 0 || local_x >= hsv_roi.cols || local_y < 0 || local_y >= hsv_roi.rows) {
            continue;
        }

        // 每个中线点都在局部横向小带上做一次红色响应统计。
        // 命中红块底边时，响应会从低值突然升高。
        const float ratio = calc_red_ratio_on_hsv_band_roi(hsv_roi, local_x, local_y, 7, 2);

        if (ratio > best_ratio) {
            best_ratio = ratio;
            fallback_idx = i;
        }

        const float rise = ratio - prev_ratio;
        if (ratio >= 0.28f && rise >= 0.10f) {
            best_idx = i;
            break;
        }
        prev_ratio = ratio;
    }

    // 若没有明显跳变，就退化为响应最高点，避免近距离或模糊场景直接丢失。
    if (best_idx < 0 && fallback_idx >= 0 && best_ratio >= 0.18f) {
        best_idx = fallback_idx;
    }
    if (best_idx < 0) return false;

    candidate_pt = centerline_pts[best_idx];
    candidate_row = candidate_pt.y;
    return true;
}

static inline bool build_red_mask_in_roi(const cv::Mat &hsv_img,
                                         const cv::Rect &roi_rect,
                                         cv::Mat &red_mask,
                                         long *mask_fill_us,
                                         long *morph_us,
                                         bool apply_morphology)
{
    if (hsv_img.empty() || hsv_img.type() != CV_8UC3) return false;
    if (!rect_inside_image(roi_rect, hsv_img.cols, hsv_img.rows)) return false;

    using clock = std::chrono::high_resolution_clock;
    red_mask.create(roi_rect.height, roi_rect.width, CV_8UC1);
    const auto mask_start = clock::now();
    for (int y = 0; y < roi_rect.height; ++y) {
        const cv::Vec3b *src_row = hsv_img.ptr<cv::Vec3b>(roi_rect.y + y);
        uchar *dst_row = red_mask.ptr<uchar>(y);
        for (int x = 0; x < roi_rect.width; ++x) {
            dst_row[x] = is_red_hsv_pixel(src_row[roi_rect.x + x]) ? 255 : 0;
        }
    }
    if (mask_fill_us != 0) {
        *mask_fill_us = std::chrono::duration_cast<std::chrono::microseconds>(
            clock::now() - mask_start).count();
    }

    if (apply_morphology) {
        // 闭运算把 JPEG 压缩噪声和细小裂缝补上，便于后面提底边。
        static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        const auto morph_start = clock::now();
        cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
        if (morph_us != 0) {
            *morph_us = std::chrono::duration_cast<std::chrono::microseconds>(
                clock::now() - morph_start).count();
        }
    } else if (morph_us != 0) {
        *morph_us = 0;
    }
    return true;
}

static inline bool finalize_red_component_from_labels(const cv::Mat &label_img,
                                                      const cv::Mat &stats,
                                                      int label_count,
                                                      const cv::Rect &hsv_roi_rect,
                                                      const cv::Point &candidate_pt,
                                                      cv::Rect &red_box,
                                                      cv::Point &red_center,
                                                      cv::Point &bottom_left,
                                                      cv::Point &bottom_right,
                                                      cv::Rect &model_box,
                                                      long *bottom_scan_us,
                                                      int *best_label_out,
                                                      double *best_score_out)
{
    using clock = std::chrono::high_resolution_clock;
    double best_score = -1.0;
    cv::Rect best_bbox_local;
    int best_label = -1;
    for (int label = 1; label < label_count; ++label) {
        const int area_i = stats.at<int>(label, cv::CC_STAT_AREA);
        const double area = static_cast<double>(area_i);
        if (area < 12.0) continue;

        const cv::Rect bbox(stats.at<int>(label, cv::CC_STAT_LEFT),
                            stats.at<int>(label, cv::CC_STAT_TOP),
                            stats.at<int>(label, cv::CC_STAT_WIDTH),
                            stats.at<int>(label, cv::CC_STAT_HEIGHT));
        if (bbox.width < 4 || bbox.height < 3) continue;

        const int bbox_cx = hsv_roi_rect.x + bbox.x + bbox.width / 2;
        const int center_penalty = std::abs(bbox_cx - candidate_pt.x);
        const double score = area - center_penalty * 1.5;
        if (score > best_score) {
            best_score = score;
            best_bbox_local = bbox;
            best_label = label;
        }
    }

    if (best_label_out != 0) *best_label_out = best_label;
    if (best_score_out != 0) *best_score_out = best_score;
    if (best_score < 0.0 || best_label < 0) return false;

    bool bottom_found = false;
    const auto bottom_start = clock::now();
    for (int y = best_bbox_local.y + best_bbox_local.height - 1;
         y >= PID_MAX(best_bbox_local.y, best_bbox_local.y + best_bbox_local.height - 5);
         --y) {
        const int *row = label_img.ptr<int>(y);
        int lx = -1;
        int rx = -1;
        for (int x = best_bbox_local.x; x < best_bbox_local.x + best_bbox_local.width; ++x) {
            if (row[x] == best_label) {
                if (lx < 0) lx = x;
                rx = x;
            }
        }

        if (lx >= 0 && rx >= lx && (rx - lx + 1) >= PID_MAX(4, best_bbox_local.width / 2)) {
            bottom_left = cv::Point(hsv_roi_rect.x + lx, hsv_roi_rect.y + y);
            bottom_right = cv::Point(hsv_roi_rect.x + rx, hsv_roi_rect.y + y);
            bottom_found = true;
            break;
        }
    }
    if (bottom_scan_us != 0) {
        *bottom_scan_us = std::chrono::duration_cast<std::chrono::microseconds>(
            clock::now() - bottom_start).count();
    }

    if (!bottom_found) {
        bottom_left = cv::Point(hsv_roi_rect.x + best_bbox_local.x,
                                hsv_roi_rect.y + best_bbox_local.y + best_bbox_local.height - 1);
        bottom_right = cv::Point(hsv_roi_rect.x + best_bbox_local.x + best_bbox_local.width - 1,
                                 hsv_roi_rect.y + best_bbox_local.y + best_bbox_local.height - 1);
    }

    red_box = cv::Rect(hsv_roi_rect.x + best_bbox_local.x,
                       hsv_roi_rect.y + best_bbox_local.y,
                       best_bbox_local.width,
                       best_bbox_local.height);
    red_center = cv::Point(red_box.x + red_box.width / 2, red_box.y + red_box.height / 2);

    const int bottom_width = PID_MAX(bottom_right.x - bottom_left.x + 1, red_box.width);
    const int side_pad = PID_MAX(6, bottom_width / 3);
    const int bottom_pad = PID_MAX(4, red_box.height / 2);
    const int roi_height = cross_clamp_int(static_cast<int>(lroundf(bottom_width * 2.4f)),
                                           30, IMG_H - 1);

    const int model_x0 = cross_clamp_int(bottom_left.x - side_pad, 0, IMG_W - 1);
    const int model_x1 = cross_clamp_int(bottom_right.x + side_pad + 1, 0, IMG_W);
    const int model_y1 = cross_clamp_int(PID_MAX(bottom_left.y, bottom_right.y) + bottom_pad + 1,
                                         0, IMG_H);
    const int model_y0 = cross_clamp_int(model_y1 - roi_height, 0, IMG_H - 1);
    model_box = cv::Rect(model_x0, model_y0, model_x1 - model_x0, model_y1 - model_y0);

    return red_box.width > 0 && red_box.height > 0 && model_box.width > 0 && model_box.height > 0;
}

static inline bool confirm_red_rect_and_build_model_roi(const cv::Mat &hsv_roi,
                                                        const cv::Rect &hsv_roi_rect,
                                                        const cv::Point &candidate_pt,
                                                        cv::Rect &red_box,
                                                        cv::Point &red_center,
                                                        cv::Point &bottom_left,
                                                        cv::Point &bottom_right,
                                                        cv::Rect &model_box,
                                                        long *mask_us,
                                                        long *morph_us,
                                                        long *cc_us,
                                                        long *bottom_scan_us)
{
    using clock = std::chrono::high_resolution_clock;
    red_box = cv::Rect();
    red_center = cv::Point(-1, -1);
    bottom_left = cv::Point(-1, -1);
    bottom_right = cv::Point(-1, -1);
    model_box = cv::Rect();

    // 候选点更接近红块下边缘，因此确认 ROI 应“向上多看、向下少看”。
    static thread_local cv::Mat red_mask;
    static thread_local cv::Mat label_img;
    static thread_local cv::Mat stats;
    static thread_local cv::Mat centroids;
    if (hsv_roi.empty() || hsv_roi.type() != CV_8UC3) return false;
    if (!rect_inside_image(hsv_roi_rect, IMG_W, IMG_H)) return false;
    long mask_fill_cost = 0;
    long morph_cost = 0;
    if (!build_red_mask_in_roi(hsv_roi, cv::Rect(0, 0, hsv_roi.cols, hsv_roi.rows),
                               red_mask, &mask_fill_cost, &morph_cost, false)) {
        return false;
    }
    long raw_bottom_cost = 0;

    const auto cc_start = clock::now();
    const int label_count = cv::connectedComponentsWithStats(red_mask, label_img, stats,
                                                             centroids, 8, CV_32S);
    long raw_cc_cost = std::chrono::duration_cast<std::chrono::microseconds>(
        clock::now() - cc_start).count();
    if (mask_us != 0) *mask_us = mask_fill_cost;
    if (morph_us != 0) *morph_us = 0;

    int raw_best_label = -1;
    double raw_best_score = -1.0;
    if (label_count > 1 && finalize_red_component_from_labels(label_img, stats, label_count,
                                                              hsv_roi_rect, candidate_pt,
                                                              red_box, red_center, bottom_left,
                                                              bottom_right, model_box,
                                                              &raw_bottom_cost,
                                                              &raw_best_label,
                                                              &raw_best_score)) {
        if (cc_us != 0) *cc_us = raw_cc_cost;
        if (bottom_scan_us != 0) *bottom_scan_us = raw_bottom_cost;
        return true;
    }

    long morph_mask_fill_cost = 0;
    long morph_only_cost = 0;
    if (!build_red_mask_in_roi(hsv_roi, cv::Rect(0, 0, hsv_roi.cols, hsv_roi.rows),
                               red_mask, &morph_mask_fill_cost, &morph_only_cost, true)) {
        return false;
    }

    const auto morph_cc_start = clock::now();
    const int morph_label_count = cv::connectedComponentsWithStats(red_mask, label_img, stats,
                                                                   centroids, 8, CV_32S);
    const long morph_cc_cost = std::chrono::duration_cast<std::chrono::microseconds>(
        clock::now() - morph_cc_start).count();
    long morph_bottom_cost = 0;
    if (morph_label_count <= 1
        || !finalize_red_component_from_labels(label_img, stats, morph_label_count,
                                               hsv_roi_rect, candidate_pt,
                                               red_box, red_center, bottom_left,
                                               bottom_right, model_box,
                                               &morph_bottom_cost, 0, 0)) {
        if (cc_us != 0) *cc_us = raw_cc_cost + morph_cc_cost;
        if (bottom_scan_us != 0) *bottom_scan_us = raw_bottom_cost + morph_bottom_cost;
        if (morph_us != 0) *morph_us = morph_only_cost;
        return false;
    }

    if (cc_us != 0) *cc_us = raw_cc_cost + morph_cc_cost;
    if (bottom_scan_us != 0) *bottom_scan_us = raw_bottom_cost + morph_bottom_cost;
    if (morph_us != 0) *morph_us = morph_only_cost;
    return true;
}

static inline bool detect_red_model_roi_by_centerline(const cv::Mat &gray_img,
                                                      const cv::Mat &bgr_img,
                                                      cv::Rect &red_box,
                                                      cv::Rect &model_box,
                                                      cv::Point &red_center,
                                                      cv::Point &bottom_left,
                                                      cv::Point &bottom_right,
                                                      int &candidate_row)
{
    using clock = std::chrono::high_resolution_clock;
    const auto detect_start = clock::now();
    red_box = cv::Rect();
    model_box = cv::Rect();
    red_center = cv::Point(-1, -1);
    bottom_left = cv::Point(-1, -1);
    bottom_right = cv::Point(-1, -1);
    candidate_row = -1;

    if (gray_img.empty() || bgr_img.empty()) return false;
    if (gray_img.type() != CV_8UC1 || bgr_img.type() != CV_8UC3) return false;
    if (gray_img.cols != IMG_W || gray_img.rows != IMG_H) return false;
    if (bgr_img.cols != IMG_W || bgr_img.rows != IMG_H) return false;

    long local_hsv_us = 0;
    long candidate_us = 0;
    long confirm_us = 0;
    long confirm_mask_us = 0;
    long confirm_morph_us = 0;
    long confirm_cc_us = 0;
    long confirm_bottom_scan_us = 0;
    bool detected = false;

    // 这一版实现的核心链路：
    // 中线逆透视 -> 沿中线找红色跳变候选 -> 小 ROI 做 HSV 确认 -> 提底边 -> 生成模型框。
    cv::Point centerline_pts[50];
    const int centerline_num = collect_centerline_orig_points(centerline_pts, 50);

    if (centerline_num >= 6) {
        int min_x = IMG_W - 1;
        int max_x = 0;
        int min_y = IMG_H - 1;
        int max_y = 0;
        for (int i = 0; i < centerline_num; ++i) {
            min_x = PID_MIN(min_x, centerline_pts[i].x);
            max_x = PID_MAX(max_x, centerline_pts[i].x);
            min_y = PID_MIN(min_y, centerline_pts[i].y);
            max_y = PID_MAX(max_y, centerline_pts[i].y);
        }

        const int band_pad_x = 8;
        const int band_pad_y = 3;
        const int band_x0 = cross_clamp_int(min_x - band_pad_x, 0, IMG_W - 1);
        const int band_y0 = cross_clamp_int(min_y - band_pad_y, 0, IMG_H - 1);
        const int band_x1 = cross_clamp_int(max_x + band_pad_x + 1, 0, IMG_W);
        const int band_y1 = cross_clamp_int(max_y + band_pad_y + 1, 0, IMG_H);
        const cv::Rect band_rect(band_x0, band_y0, band_x1 - band_x0, band_y1 - band_y0);

        cv::Mat centerline_hsv_roi;
        const auto local_hsv_start = clock::now();
        const bool band_hsv_ok = band_rect.width >= 8 && band_rect.height >= 6
            && convert_bgr_roi_to_hsv(bgr_img, band_rect, centerline_hsv_roi);
        local_hsv_us += std::chrono::duration_cast<std::chrono::microseconds>(
            clock::now() - local_hsv_start).count();

        if (band_hsv_ok) {
            cv::Point candidate_pt;
            const auto candidate_start = clock::now();
            const bool candidate_ok = locate_red_candidate_on_centerline(centerline_hsv_roi,
                                                                         band_rect,
                                                                         centerline_pts,
                                                                         centerline_num,
                                                                         candidate_pt,
                                                                         candidate_row);
            candidate_us += std::chrono::duration_cast<std::chrono::microseconds>(
                clock::now() - candidate_start).count();

            if (candidate_ok) {
                const int half_w = 20;
                const int up_h = 24;
                const int down_h = 10;
                const int x0 = cross_clamp_int(candidate_pt.x - half_w, 0, IMG_W - 1);
                const int y0 = cross_clamp_int(candidate_pt.y - up_h, 0, IMG_H - 1);
                const int x1 = cross_clamp_int(candidate_pt.x + half_w + 1, 0, IMG_W);
                const int y1 = cross_clamp_int(candidate_pt.y + down_h + 1, 0, IMG_H);
                const cv::Rect confirm_roi(x0, y0, x1 - x0, y1 - y0);

                cv::Mat confirm_hsv_roi;
                const auto confirm_hsv_start = clock::now();
                const bool confirm_hsv_ok = confirm_roi.width >= 8 && confirm_roi.height >= 8
                    && convert_bgr_roi_to_hsv(bgr_img, confirm_roi, confirm_hsv_roi);
                local_hsv_us += std::chrono::duration_cast<std::chrono::microseconds>(
                    clock::now() - confirm_hsv_start).count();

                const auto confirm_start = clock::now();
                const bool confirm_ok = confirm_hsv_ok
                    && confirm_red_rect_and_build_model_roi(confirm_hsv_roi, confirm_roi, candidate_pt,
                                                            red_box, red_center, bottom_left,
                                                            bottom_right, model_box,
                                                            &confirm_mask_us,
                                                            &confirm_morph_us,
                                                            &confirm_cc_us,
                                                            &confirm_bottom_scan_us);
                confirm_us += std::chrono::duration_cast<std::chrono::microseconds>(
                    clock::now() - confirm_start).count();

                if (confirm_ok && is_red_box_above_vehicle_head(red_box)) {
                    detected = true;
                }
            }
        }
    }

    if (!detected) {
        g_red_hsv_local_us = local_hsv_us;
        g_red_hsv_full_ref_us = -1;
        g_red_candidate_us = candidate_us;
        g_red_confirm_us = confirm_us;
        g_red_fallback_us = 0;
        g_red_confirm_mask_us = confirm_mask_us;
        g_red_confirm_morph_us = confirm_morph_us;
        g_red_confirm_cc_us = confirm_cc_us;
        g_red_confirm_bottom_scan_us = confirm_bottom_scan_us;
        g_red_detect_total_us = std::chrono::duration_cast<std::chrono::microseconds>(
            clock::now() - detect_start).count();
        g_red_perf_sample_count = 0;
        return false;
    }

    g_red_hsv_local_us = local_hsv_us;
    g_red_hsv_full_ref_us = -1;
    g_red_candidate_us = candidate_us;
    g_red_confirm_us = confirm_us;
    g_red_fallback_us = 0;
    g_red_confirm_mask_us = confirm_mask_us;
    g_red_confirm_morph_us = confirm_morph_us;
    g_red_confirm_cc_us = confirm_cc_us;
    g_red_confirm_bottom_scan_us = confirm_bottom_scan_us;
    g_red_detect_total_us = std::chrono::duration_cast<std::chrono::microseconds>(
        clock::now() - detect_start).count();
    g_red_perf_sample_count = 0;
    return true;
}

static inline ncnn_action_cmd_t ncnn_cmd_from_label(const std::string &label)
{
    if (label == "weapon") return NCNN_CMD_LEFT;
    if (label == "vehicle") return NCNN_CMD_STRAIGHT;
    if (label == "supplies") return NCNN_CMD_RIGHT;
    return NCNN_CMD_NONE;
}

static inline void reset_ncnn_detection_runtime(void)
{
    g_red_model_classify_enabled = false;
    g_red_model_cycle_done = false;
    g_red_detected_waiting_distance = false;
    g_red_detected_this_frame = false;
    g_ncnn_red_center_y = -1;
    g_ncnn_red_valid_this_frame = false;
}

static inline int *get_line_point_count(float line_point[][2])
{
    if (line_point == g_rpts0s) return &g_rpts0s_num;
    if (line_point == g_rpts1s) return &g_rpts1s_num;
    if (line_point == g_far_rpts0s) return &g_far_rpts0s_num;
    if (line_point == g_far_rpts1s) return &g_far_rpts1s_num;
    return 0;
}



static inline void element_task()
{
    reset_ncnn_detection_runtime();
    const bool allow_high_level_switch = !g_ncnn_high_level_frozen;
    g_far_Lpt1_found = g_far_Lpt0_found = false;
    // ---- 环岛和十字的判断逻辑 ----
    if (allow_high_level_switch && (g_Lpt0_found || g_Lpt1_found))
    {
        //找到左边近处角点 
        if(g_Lpt0_found && is_valid_line_point_index(g_Lpt0_id, g_rpts0s_num)
           && g_Lpt0_id < 30 && g_rpts0s[g_Lpt0_id][1] > 70)
        {
            //计算左角点的原图像坐标
            float bird_x_0 = g_rpts0s[g_Lpt0_id][0];
            float bird_y_0 = g_rpts0s[g_Lpt0_id][1];
            float orig_x, orig_y;
            inv_persp_transform(bird_x_0, bird_y_0, orig_x, orig_y);
            int begin_x_0 = (orig_x - g_Lpt0_offset_x) > 0 ? (orig_x - g_Lpt0_offset_x) : orig_x ; // 寻线起始点偏移 x
            int begin_y_0 = (orig_y - g_Lpt0_offset_y) > 0 ? (orig_y - g_Lpt0_offset_y) : orig_y; // 寻线起始点偏移 y
            
            far_fine_line(gray,0,begin_x_0,begin_y_0);//爬远处线
            far_find_corners(0);  //寻找远处角点

            // ---- 十字：近点和远点的角点都被找到，说明是十字 ----
            if(g_far_Lpt0_found && g_elem_type == ELEM_NONE) 
            {
                if(++g_cross_confirm_cnt == CROSS_CONFIRM_FRAMES)
                {
                    g_cross_confirm_cnt = 0;
                    g_elem_type = ELEM_CROSS;
                    g_cross_state = CROSS_BEGIN;
                       printf("CROSS_BEGIN");
                    buzzer_trigger(SOUND_BEEP_LONG);
                    g_far_begin_last[0][0]= begin_x_0;
                    g_far_begin_last[0][1]= begin_y_0;
                    g_far_begin_last_flag[0] = 1;
                }
                
            }
             // ---- 环岛：单侧 L 角点 + 另一侧长直道 + 角点不能太远 ----
            else if(!g_far_Lpt0_found  && g_elem_type == ELEM_NONE && is_long_straight(g_rpts1s, g_rpts1s_num ) && g_circle_state == CIRCLE_NONE)  //右边线是直道
            {
                if(++g_cross_confirm_cnt == CIRCLE_CONFIRM_FRAMES)
                {
                    g_cross_confirm_cnt = 0;
                    g_elem_type = ELEM_CIRCLE;
                    g_circle_type = 0;
                }
            }
            
        }
        //找右边远处角点
        if(g_Lpt1_found && is_valid_line_point_index(g_Lpt1_id, g_rpts1s_num)
           && g_Lpt1_id < 30 && g_rpts1s[g_Lpt1_id][1] > 70 )
        {
            //计算右角点的原图像坐标
            float bird_x_1 = g_rpts1s[g_Lpt1_id][0];
            float bird_y_1 = g_rpts1s[g_Lpt1_id][1];
            float orig_x_1, orig_y_1;
            inv_persp_transform(bird_x_1, bird_y_1, orig_x_1, orig_y_1);
            int begin_x_1 = (orig_x_1 + g_Lpt1_offset_x) < IMG_W ? (orig_x_1 + g_Lpt1_offset_x) : orig_x_1; // 寻线起始点偏移 x
            int begin_y_1 = (orig_y_1 - g_Lpt1_offset_y) > 0 ? (orig_y_1 - g_Lpt1_offset_y) : orig_y_1; // 寻线起始点偏移 y
            
            far_fine_line(gray,1,begin_x_1,begin_y_1);  //爬远处线
            far_find_corners(1); //寻找远处角点

            // ---- 十字：近点和远点的角点都被找到，说明是十字 ----
            if(g_far_Lpt1_found && g_elem_type == ELEM_NONE) 
            {
                if(++g_cross_confirm_cnt == CROSS_CONFIRM_FRAMES)
                {
                    g_cross_confirm_cnt = 0;
                    g_elem_type = ELEM_CROSS;
                    g_cross_state = CROSS_BEGIN;
                    g_far_begin_last[1][0]= begin_x_1;
                    g_far_begin_last[1][1]= begin_y_1;
                    g_far_begin_last_flag[1] = 1;
                    printf("CROSS_BEGIN");
                    buzzer_trigger(SOUND_BEEP_LONG);
                }
                

            }
             // ---- 环岛：单侧 L 角点 + 另一侧长直道 + 角点不能太远 ----
            else if(!g_far_Lpt1_found && g_elem_type == ELEM_NONE && is_long_straight(g_rpts0s, g_rpts0s_num ) && g_circle_state == CIRCLE_NONE)  //左边线是直道
            {
                if(++g_cross_confirm_cnt == CIRCLE_CONFIRM_FRAMES)
                {
                    g_cross_confirm_cnt = 0;
                    g_elem_type = ELEM_CIRCLE;
                    g_circle_type = 1;
                    g_circle_timeout_flag = 1;
                }
                
                // g_circle_state = CIRCLE_BEGIN;
                

            }
        }
        if(g_circle_state == CIRCLE_NONE && g_elem_type == ELEM_CIRCLE)
        {
            if(g_circle_type && is_valid_line_point_index(g_Lpt1_id, g_rpts1s_num)
               && g_rpts1s[g_Lpt1_id][1] > 80)
            {
                if(++g_cross_confirm_cnt == CROSS_CONFIRM_FRAMES)
                {
                    g_cross_confirm_cnt = 0;
                    g_circle_timeout_cnt = 0;
                    g_circle_state = CIRCLE_BEGIN;
                    g_circle_timeout_flag = 1;
                    buzzer_trigger(SOUND_BEEP_LONG);
                    printf("CIRCLE_BEGIN\r\n");
                }
                    

            }
            if(!g_circle_type && is_valid_line_point_index(g_Lpt0_id, g_rpts0s_num)
               && g_rpts0s[g_Lpt0_id][1] > 80)
            {
                if(++g_cross_confirm_cnt == CROSS_CONFIRM_FRAMES)
                {
                    g_cross_confirm_cnt = 0;
                    g_circle_timeout_cnt = 0;
                    g_circle_state = CIRCLE_BEGIN;
                    g_circle_timeout_flag = 1;
                    buzzer_trigger(SOUND_BEEP_LONG);
                    printf("CIRCLE_BEGIN\r\n");
                }
            }
        }

    
    }



    // ---- 模型识别的判断逻辑 ---- // 1 先识别红色方块 2 以红色方块裁减出要识别的图像进行模型识别
    {


        //对透视变换的中线进行逆变换 得到中线在原图里面的坐标  有最大限制 例如 100个点


        //从中线往上爬 找到红色跳变点 得到跳变点坐标， 然后进行hsv红色识别 

        cv::Rect red_box;
        cv::Rect model_box;
        cv::Point red_center;
        cv::Point bottom_left;
        cv::Point bottom_right;
        int candidate_row = -1;
        g_red_box_full = cv::Rect();
        g_red_model_box_full = cv::Rect();
        g_red_model_input_ready = false;

        if (detect_red_model_roi_by_centerline(gray, frame, red_box, model_box, red_center,
                                               bottom_left, bottom_right, candidate_row)) {
            g_red_detected_this_frame = true;
            g_ncnn_red_valid_this_frame = true;
            g_ncnn_red_center_y = red_center.y;
            g_red_box_full = scale_rect_160_to_320(red_box);
            g_red_model_box_full = build_fixed_40_model_box_on_320(g_red_box_full);
            if (!frame_full.empty() && frame_full.type() == CV_8UC3
                && rect_inside_image(g_red_model_box_full, frame_full.cols, frame_full.rows)) {
                cv::resize(frame_full(g_red_model_box_full), g_red_model_input_40,
                           cv::Size(40, 40), 0, 0, cv::INTER_LINEAR);
                g_red_model_input_ready = true;
            }
            // // 黄线表示沿逆透视中线命中的候选行，便于看搜索是否跟着赛道中心走。
            // cv::line(frame,
            //          cv::Point(0, candidate_row),
            //          cv::Point(IMG_W - 1, candidate_row),
            //          cv::Scalar(0, 255, 255), 1);

            // // 红框是当前确认到的红色矩形本体。
            // cv::rectangle(frame, red_box, cv::Scalar(0, 0, 255), 1);
            // cv::circle(frame, red_center, 3, cv::Scalar(0, 255, 0), -1);

            // // 蓝点是红块底边左右端点，它们是后续模型裁剪框的锚点。
            // cv::circle(frame, bottom_left, 3, cv::Scalar(255, 0, 0), -1);
            // cv::circle(frame, bottom_right, 3, cv::Scalar(255, 0, 0), -1);

            // // 青框是 160x120 小图上的调试框。
            // cv::rectangle(frame, model_box, cv::Scalar(255, 255, 0), 1);
            // if (!frame_full.empty() && frame_full.type() == CV_8UC3) {
            //     if (rect_inside_image(g_red_box_full, frame_full.cols, frame_full.rows)) {
            //         cv::rectangle(frame_full, g_red_box_full, cv::Scalar(0, 0, 255), 2);
            //     }
            //     if (rect_inside_image(g_red_model_box_full, frame_full.cols, frame_full.rows)) {
            //         cv::rectangle(frame_full, g_red_model_box_full, cv::Scalar(255, 255, 0), 2);
            //     }
            // }
        }
    }

    if (g_elem_type == ELEM_NONE) {
        switch (g_ncnn_action_state) {
            case NCNN_ACT_IDLE:
                if (g_ncnn_red_valid_this_frame) {
                    g_ncnn_action_state = NCNN_ACT_WAIT_TRIGGER;
                    g_ncnn_trigger_seen_count = 0;
                    g_ncnn_classify_stable_count = 0;
                    g_ncnn_last_label.clear();
                    printf("NCNN_ACT_WAIT_TRIGGER 等待触发距离\r\n");
                }
                break;
            case NCNN_ACT_WAIT_TRIGGER:
                if (!g_ncnn_red_valid_this_frame) {
                    g_ncnn_action_state = NCNN_ACT_IDLE;
                    printf("NCNN_ACT_IDLE 回到空闲\r\n");
                    g_ncnn_trigger_seen_count = 0;
                    break;
                }
                g_red_detected_waiting_distance = true;
                if (g_ncnn_red_center_y >= g_ncnn_trigger_center_y) {
                    if (g_ncnn_trigger_seen_count < g_ncnn_trigger_confirm_frames) {
                        ++g_ncnn_trigger_seen_count;
                    }
                } else {
                    g_ncnn_trigger_seen_count = 0;
                }
                if (g_ncnn_trigger_seen_count >= g_ncnn_trigger_confirm_frames
                    && g_red_model_input_ready) {
                    g_ncnn_action_state = NCNN_ACT_SLOW_CLASSIFY;
                    printf("NCNN_ACT_SLOW_CLASSIFY 开启模型识别\r\n");
                    g_ncnn_classify_stable_count = 0;
                    g_ncnn_last_label.clear();
                    g_red_model_classify_enabled = true;
                }
                break;
            case NCNN_ACT_SLOW_CLASSIFY:
                if (!g_ncnn_red_valid_this_frame || !g_red_model_input_ready) {
                    g_ncnn_action_state = NCNN_ACT_IDLE;
                    g_red_model_classify_enabled = false;
                    g_ncnn_classify_stable_count = 0;
                    g_ncnn_last_label.clear();
                    break;
                }
                g_red_model_classify_enabled = true;
                if (run_red_model_classification_if_needed()
                    && g_red_model_last_score >= g_ncnn_classify_min_score) 
                {
                    if (g_red_model_last_label == g_ncnn_last_label) 
                    {
                        ++g_ncnn_classify_stable_count;
                    } 
                    else 
                    {
                        g_ncnn_last_label = g_red_model_last_label;
                        g_ncnn_classify_stable_count = 1;
                    }
                    if (g_ncnn_classify_stable_count >= g_ncnn_classify_stable_frames) 
                    {
                        g_ncnn_locked_label = g_red_model_last_label;
                        g_ncnn_locked_score = g_red_model_last_score;
                        g_ncnn_locked_cmd = ncnn_cmd_from_label(g_red_model_last_label);
                        if (g_ncnn_locked_cmd != NCNN_CMD_NONE) 
                        {
                            g_ncnn_action_state = NCNN_ACT_LOCKED;
                            g_red_model_classify_enabled = false;
                            g_ncnn_action_encoder_l = 0;
                            g_ncnn_action_encoder_r = 0;
                            printf("NCNN_ACT_LOCKED 开始绕行\r\n");
                        } 
                        else 
                        {
                            g_ncnn_action_state = NCNN_ACT_IDLE;
                            g_red_model_classify_enabled = false;
                        }
                    }
                } 
                else 
                {
                    g_ncnn_classify_stable_count = 0;
                    g_ncnn_last_label.clear();
                }
                break;
            default:
                break;
        }
    }


    // ---- 斑马线的判断逻辑 ----
    if (g_elem_type == ELEM_NONE) {
        g_zebra_found = false;
        if (start_flag && g_run_elapsed_ms >= g_zebra_enable_after_ms) {
            g_zebra_found = detect_zebra(gray);
            if (g_zebra_found) {
                g_elem_type = ELEM_ZEBRA;
                start_flag = 0;
                stop_quick();
                printf("ZEBRA_STOP\r\n");
                return;
            }
        }
    } else if (g_elem_type == ELEM_ZEBRA) {
        g_zebra_found = true;
    }



    // ---- 红色砖块的判断逻辑 ----



    // ---- 锥筒的判断逻辑 ---- //不知道写不写
}

extern int no_line_flag;

static inline void run_cross()
{
    static int no_line_cnt = 0;
    static int find_line_cnt = 0;
    switch(g_cross_state)
    {
        case CROSS_BEGIN: { // 进入十字：寻远线
            if(g_far_Lpt1_found && g_Lpt1_found
               && is_valid_line_point_index(g_Lpt1_id, g_rpts1s_num)
               && is_valid_line_point_index(g_far_Lpt1_id, g_far_rpts1s_num)
               && g_Lpt1_id < 15 && g_far_Lpt1_id < 15 && g_rpts1s[g_Lpt1_id][1] > 40) {
                track_rightline(g_rpts1s, g_Lpt1_id, rptsc1, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f);
                track_rightline(g_far_rpts1s+g_far_Lpt1_id, g_far_rpts1s_num - g_far_Lpt1_id, rptsc1 +  g_Lpt1_id, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f);
                
                g_track_side = 0; // 寻右线
            }
            else if(g_far_Lpt0_found && g_Lpt0_found
                    && is_valid_line_point_index(g_Lpt0_id, g_rpts0s_num)
                    && is_valid_line_point_index(g_far_Lpt0_id, g_far_rpts0s_num)
                    && g_Lpt0_id < 15 && g_far_Lpt0_id < 15 && g_rpts0s[g_Lpt0_id][1] > 40) {

                track_rightline(g_rpts0s, g_Lpt0_id, rptsc0, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f);
                track_leftline(g_far_rpts0s+g_far_Lpt0_id, g_far_rpts0s_num - g_far_Lpt0_id, rptsc0 + g_Lpt0_id, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f);
                g_track_side = 1; // 寻左线
            }
            else if(g_rpts0s_num < 5 && g_rpts1s_num < 5 && no_line_flag == 0)
            {
                    if(++no_line_cnt == 3)
                    {
                        no_line_flag = 1;
                        no_line_cnt = 0;
                        g_cross_state = CROSS_RUNNING;
                    }
            }
            else if(!g_far_Lpt1_found && !g_far_Lpt0_found)
            {
                if(g_Lpt1_found && is_valid_line_point_index(g_Lpt1_id, g_rpts1s_num))
                {
                    // Lengthen_Boundry(g_rpts1s,1,g_Lpt1_id,  80);
                    track_rightline(g_rpts1s,g_Lpt1_id , rptsc1, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f);
                }
                if(g_Lpt0_found && is_valid_line_point_index(g_Lpt0_id, g_rpts0s_num))
                {
                    // Lengthen_Boundry(g_rpts0s,0,g_Lpt0_id,80);
                    track_rightline(g_rpts0s,g_Lpt0_id , rptsc0, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f);
                }
                
                }
            }
            break;
        
        case CROSS_RUNNING: { // 十字中央，保持直行
            far_fine_line(gray,0,40,90);//爬远处线
            far_find_corners(0);  //寻找远处角点
            if(g_far_Lpt0_found)
            {
                // Lengthen_Boundry(g_far_rpts0s,0,g_far_Lpt0_id,100);
                track_leftline(g_far_rpts0s+g_far_Lpt0_id, g_far_rpts0s_num - g_far_Lpt0_id, rptsc0, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f);         
            }

            far_fine_line(gray,1,120,90);//爬远处线
            far_find_corners(1);  //寻找远处角点
            if(g_far_Lpt1_found)
            {
                // Lengthen_Boundry(g_far_rpts1s,1,g_far_Lpt1_id,100);
                track_rightline(g_far_rpts1s+g_far_Lpt1_id, g_far_rpts1s_num - g_far_Lpt1_id, rptsc1, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f);
            }
            if(g_rpts0s_num > 5 && g_rpts1s_num > 5)
            {
                if(++find_line_cnt == 5)
                {
                    g_cross_state = CROSS_NONE;
                    g_elem_type = ELEM_NONE;
                    no_line_flag = 0;
                    find_line_cnt = 0;
                    // look_ahead_point = 8;
                    printf("出十字\r\n");
                }
            }
            break;
        }
            default:
            
            break;
    }
}










static inline void run_circle()
{

    static int g_circle_end_cnt = 0;
    track_leftline(g_rpts0s, g_rpts0s_num, rptsc0, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f * g_centerline_offset);
    track_rightline(g_rpts1s, g_rpts1s_num, rptsc1, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f * g_centerline_offset);
    switch(g_circle_state)
    {
        case CIRCLE_BEGIN:
        if(g_circle_type == 0) // 识别到左圆环 寻右边线走一段距离 直到左有点 且编码器数值满足
        {
            g_track_side = 0;  //跑右直线
            if((g_rpts0s_num > 5 && circle_encoder_L > 7500 && circle_encoder_R > 7500)) //直到左有点 且编码器数值满足
            {
                g_circle_state = CIRCLE_APPROACH;
                g_circle_timeout_cnt = 0;
                printf("CIRCLE_APPROACH\r\n");
            }
        }
        else //右圆环 识别到右圆环 寻左边线走一段距离 直到右边线有点 且编码器数值满足
        {
            g_track_side = 1;  //跑左直线
            if((g_rpts1s_num > 5 && circle_encoder_L > 7500 && circle_encoder_R > 7500)) //直到左有点 且编码器数值满足
            {
                g_track_side = 0;//跑右线进圆环
                g_circle_state = CIRCLE_APPROACH;
                g_circle_timeout_cnt = 0;
                printf("CIRCLE_APPROACH\r\n");
            }

        }
        break;
        //引导小车进环 
        case CIRCLE_APPROACH:
        if(g_circle_type == 0) //左圆环
        {
            if((circle_yaw_angle > 10500 && g_rpts1s_num > 15))  //imu 积分满足 进入CIRCLE_RUNNING
            {
                
                g_circle_state = CIRCLE_RUNNING;
                g_circle_timeout_cnt = 0;
                printf("CIRCLE_RUNNING\r\n");
                buzzer_trigger(SOUND_BEEP_LONG);
            }
        }
        else //右圆环
        {
            if(((circle_yaw_angle < -10500) && g_rpts0s_num > 15)) //imu 积分满足 进入CIRCLE_RUNNING
            {
                g_circle_timeout_cnt = 0;
                g_circle_state = CIRCLE_RUNNING;
                printf("CIRCLE_RUNNING\r\n");
                buzzer_trigger(SOUND_BEEP_LONG);
            }

        }
        break;
        //寻外环线跑 识别到角点 变成=> CIRCLE_OUT
        case CIRCLE_RUNNING:
        if(g_circle_type == 0)//左圆环
        { 
            
            if((g_Lpt1_found && is_valid_line_point_index(g_Lpt1_id, g_rpts1s_num)
                && g_rpts1s[g_Lpt1_id][1] > 75) ) //识别到右角点 且角点较近
            {
                g_circle_timeout_cnt = 0;
                g_circle_state = CIRCLE_OUT;
                printf("CIRCLE_OUT\r\n");
                buzzer_trigger(SOUND_BEEP_LONG);
            }
           
            
        }
        else //右圆环
        {
            if((g_Lpt0_found && is_valid_line_point_index(g_Lpt0_id, g_rpts0s_num)
                && g_rpts0s[g_Lpt0_id][1] > 75 )) //识别到左角点 且角点较近
            {
                g_circle_timeout_cnt = 0;
                g_circle_state = CIRCLE_OUT;
                printf("CIRCLE_OUT\r\n");
                buzzer_trigger(SOUND_BEEP_LONG);

            }  
        }
        break;
        //补线出环：保存角点坐标，激活补线引导出环
        case CIRCLE_OUT:
        {
            if (g_circle_type == 0) // 左圆环出环：补右边线
            {
                g_track_side = 1; // 跟左线出环
                // circle_out();
                // 角点消失且右线足够长 → 出环完成
                if (((!g_Lpt1_found || g_Lpt1_id > 40) && (g_rpts1s_num > 30))) {
                    if(++g_circle_end_cnt == 3)
                    {
                        g_circle_timeout_cnt = 0;
                        g_circle_state = CIRCLE_END;
                        buzzer_trigger(SOUND_BEEP_LONG);
                        g_circle_end_cnt = 0;
                        printf("CIRCLE_END\r\n");
                    }
                }
            }
            else // 右圆环出环：补左边线
            {
                g_track_side = 0; // 跟右线出环
                // 角点消失且左线足够长 → 出环完成
                if (((!g_Lpt0_found || g_Lpt0_id > 40) && (g_rpts0s_num > 30)) ) {
                    if(++g_circle_end_cnt == 3)
                    {
                        g_circle_state = CIRCLE_END;
                        g_circle_timeout_cnt = 0;
                        printf("CIRCLE_END\r\n");
                        buzzer_trigger(SOUND_BEEP_LONG);
                        g_circle_end_cnt = 0;
                    }
                }
            }
            break;
        }
        // 出环完成，恢复正常
        case CIRCLE_END:
        {
            g_centerline_offset = 1.0f;  // 恢复正常偏移
            g_circle_line_active = false;
            if(g_circle_type == 0)
            g_track_side = 0;
            else
            g_track_side = 1;
            g_elem_type = ELEM_NONE;
            g_circle_state = CIRCLE_NONE;
            g_circle_timeout_cnt = 0;  // 重置超时计数器
            g_circle_timeout_flag = 0;
            break;
        }
    }



}




static inline void run_none()
{
// ---- 中线生成：将边线沿法向偏移半个赛道宽度 ----
    // track_leftline：左边线向右偏移 → 中线
    // track_rightline：右边线向左偏移 → 中线
    // 使用 g_centerline_offset 调整偏移量（>1.0 偏向外侧）
    //如果没有识别到模型，正常寻线偏执
    
    //识别到模型的状态机 根据识别出的种类 选择绕行方式 此时不会有元素处理，
    {
        const int avg_cnt =
            (std::abs(g_ncnn_action_encoder_l) + std::abs(g_ncnn_action_encoder_r)) / 2;
        switch (g_ncnn_action_state) {
            case NCNN_ACT_LOCKED:
                g_ncnn_high_level_frozen = true;
                g_ncnn_bypass_bias_deg = 0.0f;
                g_centerline_offset = g_ncnn_bypass_offset_scale;
                g_ncnn_action_state = NCNN_ACT_BYPASS;
                printf("NCNN_ACT_BYPASS 开始绕行\r\n");
                break;
            case NCNN_ACT_BYPASS:
            
                if (g_ncnn_locked_cmd == NCNN_CMD_LEFT) 
                {
                    g_track_side = 1; // 跟左线左绕

                    if(g_ipts0_num == 0) //没有线的话
                    {
                        begin_x_offset_l = 30;
                        begin_y_offset_l = 5; 
                        lost_line_flag = 1;
                        // g_centerline_offset = -0.2f;
                    }
                    if(g_ncnn_action_encoder_r > 6500)
                    {
                        g_ncnn_action_state = NCNN_ACT_RECOVER;
                        printf("NCNN_ACT_RECOVER 开始恢复正常\r\n");
                        lost_line_flag = 0;
                        begin_x_offset_l = 0;
                        // begin_x_offset_r = 30;
                        begin_y_offset_l = 5;
                        g_ncnn_action_encoder_l = 0;
                        g_ncnn_action_encoder_r = 0;
                    }
                    // g_centerline_offset = g_ncnn_bypass_offset_scale;
                     g_centerline_offset = -0.9f;
                } 
                else if (g_ncnn_locked_cmd == NCNN_CMD_RIGHT) 
                {
                    g_track_side = 0; 

                    if(g_ipts1_num == 0) //没有线的话
                    {
                        begin_x_offset_r = -30;
                        begin_y_offset_l = 5; 
                        lost_line_flag = 1;
                    }
                    if(g_ncnn_action_encoder_l > 6500)
                    {
                        g_ncnn_action_state = NCNN_ACT_RECOVER;
                        printf("NCNN_ACT_RECOVER 开始恢复正常\r\n");
                        lost_line_flag = 0;
                        begin_x_offset_r = 0;
                        // begin_x_offset_l = -20;
                        begin_y_offset_l = 0;
                        g_ncnn_action_encoder_l = 0;
                        g_ncnn_action_encoder_r = 0;
                    }
                    g_centerline_offset = -0.9f;
                } 
                else 
                {
                    g_centerline_offset = 1.0f;
                    if(g_ncnn_action_encoder_l > 3000 && g_ncnn_action_encoder_r > 3000)
                    {
                        g_ncnn_action_encoder_l = 0;
                        g_ncnn_action_encoder_r = 0;
                        g_ncnn_action_state = NCNN_ACT_RECOVER;
                        printf("NCNN_ACT_RECOVER 开始恢复正常\r\n");
                    }
                }
                break;
            case NCNN_ACT_RECOVER:
                if (g_ncnn_locked_cmd == NCNN_CMD_LEFT) 
                {
                    g_track_side = 1;  //依旧跟左线 
                    if(g_ipts0_num == 0) //没有线的话
                    {
                        begin_x_offset_l = 30;
                        begin_y_offset_l = 5; 
                        lost_line_flag = 1;
                        // g_centerline_offset = -0.2f;
                    }
                    // g_centerline_offset = g_ncnn_recover_offset_scale;
                    g_centerline_offset = 1.0f;
                    // g_ncnn_bypass_bias_deg = -g_ncnn_recover_bias_deg;
                } 
                else if (g_ncnn_locked_cmd == NCNN_CMD_RIGHT) 
                {
                    g_track_side = 0;
                    if(g_ipts1_num == 0) //没有线的话
                    {
                        begin_x_offset_r = -30;
                        begin_y_offset_l = 5; 
                        lost_line_flag = 1;
                        // g_centerline_offset = -0.2f;
                    }
                    // g_centerline_offset = g_ncnn_recover_offset_scale;
                    g_centerline_offset = 1.0f;
                    // g_ncnn_bypass_bias_deg = g_ncnn_recover_bias_deg;
                } 
                else 
                {
                    g_centerline_offset = 1.0f;
                    g_ncnn_bypass_bias_deg = 0.0f;
                }
                // if (avg_cnt >= g_ncnn_bypass_pass_counts + g_ncnn_recover_counts) {
                //     g_ncnn_action_state = NCNN_ACT_DONE;
                //     printf("NCNN_ACT_DONE 绕行结束\r\n");
                // }
                if(g_ncnn_action_encoder_l > 1000 && g_ncnn_action_encoder_r > 1000)
                    {
                        g_ncnn_action_state = NCNN_ACT_DONE;
                        begin_x_offset_r = 0;
                        begin_x_offset_l = 0;
                        begin_y_offset_l = 0;
                        printf("NCNN_ACT_RECOVER 开始恢复正常\r\n");
                    }
                break;
            case NCNN_ACT_DONE:
                g_track_side = 0;
                g_centerline_offset = 1.0f;
                g_ncnn_bypass_bias_deg = 0.0f;
                g_ncnn_high_level_frozen = false;
                g_ncnn_action_state = NCNN_ACT_IDLE;
                g_ncnn_locked_cmd = NCNN_CMD_NONE;
                g_ncnn_locked_label.clear();
                g_ncnn_locked_score = 0.0f;
                g_ncnn_classify_stable_count = 0;
                g_ncnn_last_label.clear();
                g_ncnn_trigger_seen_count = 0;
                g_ncnn_action_encoder_l = 0;
                g_ncnn_action_encoder_r = 0;
                printf("NCNN_ACT_IDLE 恢复正常\r\n");
                break;
            default:
                break;
        }
    }
    {
        track_leftline(g_rpts0s, g_rpts0s_num, rptsc0, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f * g_centerline_offset);
        track_rightline(g_rpts1s, g_rpts1s_num, rptsc1, ANGLE_DIST_N, ROAD_WIDTH_PX/2.0f * g_centerline_offset);

    }

}


static inline void far_fine_line(cv::Mat &frame ,int line_type ,int begin_x , int begin_y)
{
    // ---- 构造 image_t 包装（零拷贝，直接指向 OpenCV 内存）----
    image_t img;
    img.data   = frame.data;
    img.width  = IMG_W;
    img.height = IMG_H;
    img.step   = frame.step[0];
    // 左边线起始点：从中心向左找到白色区域的左边缘
    // int y0 = begin_y;
    // while (y0 > 20 && AT_IMAGE(&img, begin_x, y0 - 1) > start_thres) y0--;
    if(line_type == 0)
    {
        int far_x0 = begin_x;
        int far_y0 = begin_y;

        // 边界检查
        if (far_x0 < 0 || far_x0 >= IMG_W || far_y0 < 0 || far_y0 >= IMG_H) {
            g_far_ipts0_num = 0;
            g_far_rpts0s_num = 0;
            return;
        }

        bool white_found = false;
        for (; far_y0 > 0; far_y0--) {
            //先黑后白，先找white
            if (AT_IMAGE(&img, far_x0, far_y0) >= far_start_thres) { white_found = true; }
            if (AT_IMAGE(&img, far_x0, far_y0) < far_start_thres && white_found) {
                break;
            }
        }
        // ---- 爬线（迷宫左手法则）----
        // 只有起始点在白线上才爬，否则置0
        int far_ipts0_num = POINTS_MAX;
        if (far_y0 < IMG_H-1 && AT_IMAGE(&img, far_x0, far_y0+1) > far_start_thres)
            findline_lefthand_adaptive(&img, BLOCK_SIZE, CLIP_VALUE, far_x0, far_y0, g_far_ipts0, &far_ipts0_num);
        else
            far_ipts0_num = 0;
            
        // 截断左边线：只保留从底部向上爬的部分，到最高点(y最小)为止
        if (far_ipts0_num > 1) {
            int min_y_idx = 0;
            for (int i = 1; i < far_ipts0_num; i++)
                if (g_far_ipts0[i][1] < g_far_ipts0[min_y_idx][1]) min_y_idx = i;
            far_ipts0_num = min_y_idx + 1;
        }
        g_far_ipts0_num = far_ipts0_num;


    // ---- 透视变换：原图像素坐标 → 鸟瞰图坐标 ----
    // 变换后的坐标才能用于法向偏移取中线（赛道宽度在鸟瞰图中是固定像素数）
    static float far_rpts0[POINTS_MAX][2];
    for (int i = 0; i < far_ipts0_num; i++)
        persp_transform(g_far_ipts0[i][0], g_far_ipts0[i][1], far_rpts0[i][0], far_rpts0[i][1]);

    // ---- 三角滤波：平滑边线，降低噪声干扰 ----
    static float far_rpts0b[POINTS_MAX][2];
    blur_points(far_rpts0, far_ipts0_num, far_rpts0b, BLUR_KERNEL);
    // ---- 等距采样：使相邻点间距相等，方便后续角度计算和预瞄点索引 ----
    g_far_rpts0s_num = POINTS_MAX;
    resample_points(far_rpts0b, far_ipts0_num, g_far_rpts0s, &g_far_rpts0s_num, SAMPLE_DIST);
    }
    else
    {
        int far_x1 = begin_x;
        int far_y1 = begin_y;

        // 边界检查
        if (far_x1 < 0 || far_x1 >= IMG_W || far_y1 < 0 || far_y1 >= IMG_H) {
            g_far_ipts1_num = 0;
            g_far_rpts1s_num = 0;
            return;
        }

        bool white_found = false;
        for (; far_y1 > 0; far_y1--) {
            //先黑后白，先找white
            if (AT_IMAGE(&img, far_x1, far_y1) >= far_start_thres) { white_found = true; }
            if (AT_IMAGE(&img, far_x1, far_y1) < far_start_thres && white_found) {
                break;
            }
        }
        // ---- 爬线（迷宫左手法则）----
        // 只有起始点在白线上才爬，否则置0
        int far_ipts1_num = POINTS_MAX;
        if (far_y1 < IMG_H-1 && AT_IMAGE(&img, far_x1, far_y1+1) > far_start_thres)
            findline_righthand_adaptive(&img, BLOCK_SIZE, CLIP_VALUE, far_x1, far_y1, g_far_ipts1, &far_ipts1_num);
        else
            far_ipts1_num = 0;
        // 截断左边线：只保留从底部向上爬的部分，到最高点(y最小)为止
        if (far_ipts1_num > 1) {
            int min_y_idx = 0;
            for (int i = 1; i < far_ipts1_num; i++)
                if (g_far_ipts1[i][1] < g_far_ipts1[min_y_idx][1]) min_y_idx = i;
            far_ipts1_num = min_y_idx + 1;
        }
        g_far_ipts1_num = far_ipts1_num;


    // ---- 透视变换：原图像素坐标 → 鸟瞰图坐标 ----
    // 变换后的坐标才能用于法向偏移取中线（赛道宽度在鸟瞰图中是固定像素数）
    static float far_rpts1[POINTS_MAX][2];
    for (int i = 0; i < far_ipts1_num; i++)
        persp_transform(g_far_ipts1[i][0], g_far_ipts1[i][1], far_rpts1[i][0], far_rpts1[i][1]);

    // ---- 三角滤波：平滑边线，降低噪声干扰 ----
    static float far_rpts1b[POINTS_MAX][2];
    blur_points(far_rpts1, far_ipts1_num, far_rpts1b, BLUR_KERNEL);
    // ---- 等距采样：使相邻点间距相等，方便后续角度计算和预瞄点索引 ----
    g_far_rpts1s_num = POINTS_MAX;
    resample_points(far_rpts1b, far_ipts1_num, g_far_rpts1s, &g_far_rpts1s_num, SAMPLE_DIST);
    }
}




//寻找远线角点 0为左 1为右
static inline void far_find_corners(int line_type) {

    //寻找左边线角点
    if(line_type == 0)
    {
    // 用于截断边线，避免跟到错误的线（如交叉路口）
        static float ang0[POINTS_MAX];
        static float ang0n[POINTS_MAX];
        local_angle_points(g_far_rpts0s, g_far_rpts0s_num, ang0, 5);
        // NMS：非极大抑制，只保留局部最大角度变化点605
        nms_angle(ang0, g_far_rpts0s_num, ang0n, 5*2+1);

        g_far_Lpt0_found = false;
        for (int i = 0; i < g_far_rpts0s_num; i++) {
            if (ang0n[i] == 0) continue;
            int im1 = clip(i - 5, 0, g_far_rpts0s_num-1);
            int ip1 = clip(i + 5, 0, g_far_rpts0s_num-1);
            // conf = 当前角度变化率 - 邻近点平均（突出程度）
            float conf = fabsf(ang0[i]) - (fabsf(ang0[im1]) + fabsf(ang0[ip1])) / 2.0f;
            // L 角点：70°~140°（约90°直角，十字/圆环/车库）
            if (!g_far_Lpt0_found && conf > 70.0f/180.0f*PI && conf < 140.0f/180.0f*PI) {
                g_far_Lpt0_id = i;
                g_far_Lpt0_found = true;
            }
        }
    }
    else  //寻找右边线角点
    {
        // 用于截断边线，避免跟到错误的线（如交叉路口）
        static float ang1[POINTS_MAX];
        static float ang1n[POINTS_MAX];
        local_angle_points(g_far_rpts1s, g_far_rpts1s_num, ang1, 5);
        // local_angle_points(g_far_rpts1s, g_far_rpts1s_num, ang1, ANGLE_DIST_N);
        // NMS：非极大抑制，只保留局部最大角度变化点
        nms_angle(ang1, g_far_rpts1s_num, ang1n, 5*2+1);
        g_far_Lpt1_found = false;
        for (int i = 0; i < g_far_rpts1s_num; i++) {
            if (ang1n[i] == 0) continue;
            int im1 = clip(i - 5, 0, g_far_rpts1s_num-1);
            int ip1 = clip(i + 5, 0, g_far_rpts1s_num-1);
            // conf = 当前角度变化率 - 邻近点平均（突出程度）
            float conf = fabsf(ang1[i]) - (fabsf(ang1[im1]) + fabsf(ang1[ip1])) / 2.0f;
            // L 角点：70°~140°（约90°直角，十字/圆环/车库）
            if (!g_far_Lpt1_found && conf > 70.0f/180.0f*PI && conf < 140.0f/180.0f*PI) {
                g_far_Lpt1_id = i;
                g_far_Lpt1_found = true;
            }
        }


    }
   
}
