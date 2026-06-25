#pragma once

#include "headfile.h"
#include "app_config.h"
#include "line_track.h"
#include "seekfree_tcp_adapter.h"
#include "ww_menu.h"

// 本文件只放“共享运行状态”和“共享设备对象”的声明。
// 和 app_config.h 的区别：
// 1. app_config.h 放可调默认值；
// 2. app_context.h 放程序运行过程中会变化的状态与设备句柄。

// 菜单状态：被按键库直接引用，因此集中在公共上下文里统一定义。
extern Menu* g_menu;
extern bool g_menu_mode;

// 设备与通信对象：所有任务共享，统一在 app_context.cc 中定义。
extern lq_camera_ex camera;
extern CameraStreamServer camera_server;
extern cv::Mat frame;
extern cv::Mat frame_full;
extern cv::Mat perspective_frame;
extern cv::Mat gray;
extern cv::Mat g_red_model_input_40;
extern cv::Rect g_red_box_full;
extern cv::Rect g_red_model_box_full;
extern bool g_red_model_input_ready;
extern bool g_red_model_classify_enabled;
extern bool g_red_detected_waiting_distance;
extern bool g_red_model_cycle_done;
extern bool g_red_detected_this_frame;
extern float g_red_model_trigger_distance_cm;
extern float g_red_model_pixels_per_cm;
extern float g_red_model_estimated_distance_cm;
extern std::string g_red_model_last_label;
extern float g_red_model_last_score;
extern std::vector<float> g_red_model_last_probs;
extern std::string g_red_model_last_probs_text;
extern CameraTelemetry camera_telemetry;
extern ICM42688 imu;
extern VL53L0X tof;
extern Key key;
extern LCD lcd;
extern Motor motor;
extern Buzzer buzzer_1;
extern Brushless brush;
extern VofaClient vofa_tcp_client;
extern SeekfreeTcpClient seekfree_client;

// 运行控制参数：供任务和上位机同时访问。
// 这些值在运行过程中会被任务动态修改，因此不放到 config 中。
extern int start_flag;
extern int __1000ms;
extern int start__1000ms_flag;
extern int g_launch_ramp_elapsed_ms;
extern int g_run_elapsed_ms;

// 调试量：主要给图传和示波器看，不建议当成配置项频繁手改。
extern float g_target_gyro_dbg;
extern float g_actual_gyro_dbg;
extern float g_speed_l_dbg;
extern float g_speed_r_dbg;
extern float g_pwm_l_dbg;
extern float g_pwm_r_dbg;
extern float g_target_speed_l_dbg;
extern float g_target_speed_r_dbg;
extern float g_pure_angle_near;
extern float g_pure_angle_far;
extern float g_pure_angle_ff_delta;
extern float g_target_gyro_ff_dbg;
extern long g_photo_algo_us;
extern long g_photo_debug_us;
extern long g_photo_pre_us;
extern long g_photo_line_us;
extern long g_photo_elem_us;
extern long g_photo_track_us;
extern long g_photo_pure_us;
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
// 共享工具函数：既给任务用，也给图传服务用。
// reset_pid_state 用来在停车、重置元素或在线改参后清掉历史状态。
void reset_pid_state(pid_positional_t& pid);
void reset_pid_state(pid_incremental_t& pid);
bool update_pid_params(const std::string& group, float kp, float ki, float kd, std::string& message);
void reset_element_state(void);
int calc_speed_related_look_ahead_point(float vehicle_speed);
void stop_quick();
bool run_red_model_classification_if_needed(void);
const char* ncnn_action_state_name(ncnn_action_state_t state);
const char* ncnn_action_cmd_name(ncnn_action_cmd_t cmd);
