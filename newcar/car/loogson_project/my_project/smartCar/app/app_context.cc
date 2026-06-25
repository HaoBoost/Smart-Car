#include "app_context.h"

// 本文件只定义“共享运行状态”和“设备对象”。
// 这样任务文件里只拿 extern，不会再出现多文件重复定义问题。

// 菜单相关全局状态：按键驱动会直接 extern 这些变量。
Menu* g_menu = nullptr;
bool g_menu_mode = true;

// 硬件与通信对象统一在这里定义，避免多文件拆分后出现重复定义。
lq_camera_ex camera(320, 240, 120, LQ_CAMERA_0CPU_MJPG);
CameraStreamServer camera_server;
cv::Mat frame;
cv::Mat frame_full;
cv::Mat perspective_frame;
cv::Mat gray;
cv::Mat g_red_model_input_40;
cv::Rect g_red_box_full;
cv::Rect g_red_model_box_full;
bool g_red_model_input_ready = false;
bool g_red_model_classify_enabled = true;
bool g_red_detected_waiting_distance = false;
bool g_red_model_cycle_done = false;
bool g_red_detected_this_frame = false;
float g_red_model_trigger_distance_cm = 20.0f;
float g_red_model_pixels_per_cm = 3.0f;
float g_red_model_estimated_distance_cm = 0.0f;
std::string g_red_model_last_label;
float g_red_model_last_score = 0.0f;
std::vector<float> g_red_model_last_probs;
std::string g_red_model_last_probs_text;
CameraTelemetry camera_telemetry;
ICM42688 imu;
VL53L0X tof;
Key key;
LCD lcd;
Motor motor;
Buzzer buzzer_1;
Brushless brush;
VofaClient vofa_tcp_client;
SeekfreeTcpClient seekfree_client;

// 运行控制参数。
// start_flag 表示当前是否真正进入运行态；
// start__1000ms_flag / __1000ms 负责延时发车计时。
int start_flag = 0;
int __1000ms = 0;
int start__1000ms_flag = 0;
int g_launch_ramp_elapsed_ms = 0;
int g_run_elapsed_ms = 0;

// 调试量：实时写入上位机和图传页面，便于看控制链路是否正常。
float g_target_gyro_dbg = 0.0f;
float g_actual_gyro_dbg = 0.0f;
float g_speed_l_dbg = 0.0f;
float g_speed_r_dbg = 0.0f;
float g_pwm_l_dbg = 0.0f;
float g_pwm_r_dbg = 0.0f;
float g_target_speed_l_dbg = 0.0f;
float g_target_speed_r_dbg = 0.0f;
float g_pure_angle_near = 0.0f;
float g_pure_angle_far = 0.0f;
float g_pure_angle_ff_delta = 0.0f;
float g_target_gyro_ff_dbg = 0.0f;
long g_photo_algo_us = 0;
long g_photo_debug_us = 0;
long g_photo_pre_us = 0;
long g_photo_line_us = 0;
long g_photo_elem_us = 0;
long g_photo_track_us = 0;
long g_photo_pure_us = 0;
long g_red_detect_total_us = 0;
long g_red_hsv_local_us = 0;
long g_red_hsv_full_ref_us = 0;
long g_red_candidate_us = 0;
long g_red_confirm_us = 0;
long g_red_fallback_us = 0;
long g_red_confirm_mask_us = 0;
long g_red_confirm_morph_us = 0;
long g_red_confirm_cc_us = 0;
long g_red_confirm_bottom_scan_us = 0;
int g_red_perf_sample_count = 0;
ncnn_action_state_t g_ncnn_action_state = NCNN_ACT_IDLE;
ncnn_action_cmd_t g_ncnn_locked_cmd = NCNN_CMD_NONE;
std::string g_ncnn_locked_label;
float g_ncnn_locked_score = 0.0f;
int g_ncnn_classify_stable_count = 0;
std::string g_ncnn_last_label;
int g_ncnn_trigger_seen_count = 0;
int g_ncnn_action_encoder_l = 0;
int g_ncnn_action_encoder_r = 0;
float g_ncnn_bypass_bias_deg = 0.0f;
bool g_ncnn_high_level_frozen = false;
int g_ncnn_red_center_y = -1;
bool g_ncnn_red_valid_this_frame = false;

// line_track.h 中原本定义在头文件里的共享状态，集中移动到这里。
// 这些变量本质上是“每一帧视觉处理结果”。
int look_ahead_point = LOOK_AHEAD_POINT_EN;
int g_ipts0[POINTS_MAX][2] = {};
int g_ipts1[POINTS_MAX][2] = {};
int g_ipts0_num = 0;
int g_ipts1_num = 0;
float g_rpts0s[POINTS_MAX][2] = {};
float g_rpts1s[POINTS_MAX][2] = {};
int g_rpts0s_num = 0;
int g_rpts1s_num = 0;
float g_rptsc[POINTS_MAX][2] = {};
int g_rptsc_num = 0;
float g_rptsn[POINTS_MAX][2] = {};
int g_rptsn_num = 0;
bool g_Lpt0_found = false;
bool g_Lpt1_found = false;
int g_Lpt0_id = 0;
int g_Lpt1_id = 0;
float rptsc0[POINTS_MAX][2] = {};
float rptsc1[POINTS_MAX][2] = {};
float g_pure_angle = 0.0f;
float g_cx = IMG_W / 2.0f;
float g_cy = IMG_H * 0.85f;
int g_track_side = 0;
float g_centerline_offset = 1.0f;
cv::Mat g_persp_M;
bool g_persp_ready = false;
float g_max_curvature = 0.0f;

// cross.h 中的共享状态。
// 这些变量本质上是“元素识别状态机”和“补线中间结果”。
elem_type_t g_elem_type = ELEM_NONE;
cross_state_t g_cross_state = CROSS_NONE;
circle_state_t g_circle_state = CIRCLE_NONE;
bool g_zebra_found = false;
int g_circle_confirm_cnt = 0;
int g_cross_confirm_cnt = 0;
int g_circle_stage_cnt = 0;
int g_circle_type = 0;
int g_cross_timeout_cnt = 0;
int g_circle_timeout_cnt = 0;
bool g_cross_timeout_flag = false;
bool g_circle_timeout_flag = false;
int cross_encoder_L = 0;
int cross_encoder_R = 0;
int circle_encoder_L = 0;
int circle_encoder_R = 0;
float circle_yaw_angle = 0.0f;
int circle_yaw_angle_flag = 0;
int g_circle_line_x = 0;
int g_circle_line_y = 0;
bool g_circle_line_active = false;
int g_far_ipts0[POINTS_MAX][2] = {};
int g_far_ipts1[POINTS_MAX][2] = {};
int g_far_ipts0_num = 0;
int g_far_ipts1_num = 0;
float g_far_rpts0s[POINTS_MAX][2] = {};
float g_far_rpts1s[POINTS_MAX][2] = {};
int g_far_rpts0s_num = 0;
int g_far_rpts1s_num = 0;
bool g_far_Lpt0_found = false;
bool g_far_Lpt1_found = false;
int g_far_Lpt0_id = 0;
int g_far_Lpt1_id = 0;
int g_circle_last_high = 0;
int g_far_begin_last[2][2] = {};
int g_far_begin_last_flag[2] = {};
cv::Mat g_inv_persp_M;
bool g_inv_persp_M_initialized = false;
int no_line_flag = 0;
float g_circle_corner_last_l[2] = {};
float g_circle_corner_last_r[2] = {};
int line_point = 0;

int begin_x_offset_l;
int begin_y_offset_l;

int begin_x_offset_r;
int begin_y_offset_r;


int lost_line_flag = 0;







const char* ncnn_action_state_name(ncnn_action_state_t state)
{
    switch (state) {
        case NCNN_ACT_WAIT_TRIGGER: return "WAIT_TRIGGER";
        case NCNN_ACT_SLOW_CLASSIFY: return "SLOW_CLASSIFY";
        case NCNN_ACT_LOCKED: return "LOCKED";
        case NCNN_ACT_BYPASS: return "BYPASS";
        case NCNN_ACT_RECOVER: return "RECOVER";
        case NCNN_ACT_DONE: return "DONE";
        default: return "IDLE";
    }
}

const char* ncnn_action_cmd_name(ncnn_action_cmd_t cmd)
{
    switch (cmd) {
        case NCNN_CMD_LEFT: return "LEFT";
        case NCNN_CMD_STRAIGHT: return "STRAIGHT";
        case NCNN_CMD_RIGHT: return "RIGHT";
        default: return "NONE";
    }
}

void reset_pid_state(pid_positional_t& pid)
{
    // 位置式 PID 会积累 P/I/D 中间项，停车或切模式时必须清零。
    pid.out_p = 0.0f;
    pid.out_i = 0.0f;
    pid.out_d = 0.0f;
}

void reset_pid_state(pid_incremental_t& pid)
{
    // 增量式 PID 的历史误差不清掉，重新起步时容易突然冲一下。
    pid.err_last = 0.0f;
    pid.err_last2 = 0.0f;
}

bool update_pid_params(const std::string& group, float kp, float ki, float kd, std::string& message)
{
    // 图传网页在线改参统一走这里，避免不同模块各自改各自的 PID。
    if (group == "speed_l") {
        pid_speed_1.kp = kp;
        pid_speed_1.ki = ki;
        pid_speed_1.kd = kd;
        reset_pid_state(pid_speed_1);
        message = "左轮速度 PID 已更新";
        return true;
    }
    if (group == "speed_r") {
        pid_speed_r.kp = kp;
        pid_speed_r.ki = ki;
        pid_speed_r.kd = kd;
        reset_pid_state(pid_speed_r);
        message = "右轮速度 PID 已更新";
        return true;
    }
    if (group == "gyro") {
        pid_angle_v.kp = kp;
        pid_angle_v.ki = ki;
        pid_angle_v.kd = kd;
        reset_pid_state(pid_angle_v);
        message = "角速度 PID 已更新";
        return true;
    }
    if (group == "angle") {
        pid_angle.kp = kp;
        pid_angle.ki = ki;
        pid_angle.kd = kd;
        reset_pid_state(pid_angle);
        message = "角度 PID 已更新";
        return true;
    }
    if (group == "circle") {
        pid_angle_circle.kp = kp;
        pid_angle_circle.ki = ki;
        pid_angle_circle.kd = kd;
        reset_pid_state(pid_angle_circle);
        message = "环岛角度 PID 已更新";
        return true;
    }

    message = "未知 PID 分组";
    return false;
}

void reset_element_state(void)
{
    // 手动复位或异常退出元素状态时，统一把状态机相关变量恢复到默认值。
    g_elem_type = ELEM_NONE;
    g_cross_state = CROSS_NONE;
    g_circle_state = CIRCLE_NONE;
    g_circle_type = 0;
    g_track_side = 0;
    g_centerline_offset = 1.0f;
    g_zebra_found = false;
    g_circle_confirm_cnt = 0;
    g_cross_confirm_cnt = 0;
    g_circle_stage_cnt = 0;
    g_cross_timeout_cnt = 0;
    g_circle_timeout_cnt = 0;
    g_cross_timeout_flag = false;
    g_circle_timeout_flag = false;
    cross_encoder_L = 0;
    cross_encoder_R = 0;
    circle_encoder_L = 0;
    circle_encoder_R = 0;
    circle_yaw_angle = 0.0f;
    circle_yaw_angle_flag = 0;
    g_circle_line_x = 0;
    g_circle_line_y = 0;
    g_circle_line_active = false;
    g_circle_last_high = 0;
    g_far_begin_last_flag[0] = 0;
    g_far_begin_last_flag[1] = 0;
    look_ahead_point = 10;
    g_ncnn_action_state = NCNN_ACT_IDLE;
    g_ncnn_locked_cmd = NCNN_CMD_NONE;
    g_ncnn_locked_label.clear();
    g_ncnn_locked_score = 0.0f;
    g_ncnn_classify_stable_count = 0;
    g_ncnn_last_label.clear();
    g_ncnn_trigger_seen_count = 0;
    g_ncnn_action_encoder_l = 0;
    g_ncnn_action_encoder_r = 0;
    g_ncnn_bypass_bias_deg = 0.0f;
    g_ncnn_high_level_frozen = false;
    g_ncnn_red_center_y = -1;
    g_ncnn_red_valid_this_frame = false;
}
