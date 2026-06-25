#pragma once

#include "pid.h"

// 本文件只放“需要频繁调”的参数声明。
// 原则：
// 1. 会被反复试车调的量放这里。
// 2. 运行时中间状态不要放这里，避免把“配置”和“状态”混在一起。
// 3. 以后调车优先看 app_config.cc，不必进任务代码里翻默认值。

// 速度目标：默认巡航速度，左右轮通常保持一致。
// 这是直道基准速度，弯道时 motor_task 会在此基础上做减速。
extern int base_speed_target_l;
extern int base_speed_target_r;
extern float target_v;

// 发车相关调参项：控制延时发车与起步爬升手感。
// delay_ms 决定按下发车后等多久真正起步；
// ramp_ms / start_ratio 决定起步时从多大比例逐渐爬到目标速度。
extern int g_launch_delay_ms;
extern int g_launch_ramp_ms;
extern int g_zebra_enable_after_ms;
extern float g_launch_start_ratio;
extern float g_launch_turn_ratio;
extern float g_launch_encoder_release_threshold;
extern float g_launch_delta_limit;
// 弯道前馈调参项：基于近/远双前瞻角差，提前给一点目标角速度。
// enable=0 时整条前馈链路关闭；
// far_offset 控制远前瞻比当前前瞻多看几个点；
// k_delta 控制角差映射到前馈角速度的强度；
// limit / low_pass / active_angle_min 分别负责限幅、滤波和弯道启用阈值。
extern int g_curve_ff_enable;
extern int g_curve_ff_far_offset;
extern float g_curve_ff_k_delta;
extern float g_curve_ff_limit;
extern float g_curve_ff_low_pass;
extern float g_curve_ff_active_angle_min;

// 前瞻点调参项：按弯道强弱选择不同前瞻距离。
// 值越小，车越愿意盯近处，转向更积极；
// 值越大，车越偏平顺，但急弯可能转不过去。
extern int g_line_lookahead_straight;
extern int g_line_lookahead_mid;
extern int g_line_lookahead_turn;
extern int g_line_lookahead_hairpin;
extern float g_line_lookahead_speed_gain;
extern float g_line_lookahead_dist_min;
extern float g_line_lookahead_dist_max;

// 输出限幅：统一管理差速、目标速度和 PWM 上下限。
// 这里都是保护项，调大之前要先确认不是算法方向错了。
extern float g_diff_output_limit;
extern float g_target_speed_limit;
extern float g_pwm_output_min_launch;
extern float g_pwm_output_min;
extern float g_pwm_output_max;

// 无刷占空比。
// 当前工程里无刷默认关闭，菜单修改该值后由 motor_task 周期写入电调。
extern int g_brushless_duty;

// 速度环 PID。
// 负责把目标轮速兑现成 PWM 增量。
extern pid_incremental_t pid_speed_1;
extern pid_incremental_t pid_speed_r;

// 角速度环 PID。
// 负责把目标角速度兑现成左右轮差速。
extern pid_positional_t pid_angle_v;
extern pid_positional_t pid_angle_v_circle;

// 角度环 PID。
// 负责把纯跟踪误差转换成目标角速度。
extern pid_positional_t pid_angle;
extern pid_positional_t pid_angle_circle;

// 元素识别与补线调参项。
// 主要给十字 / 圆环角点补线和远线起始阈值使用。
extern int g_Lpt0_offset_x;
extern int g_Lpt0_offset_y;
extern int g_Lpt1_offset_x;
extern int g_Lpt1_offset_y;
extern int far_start_thres;

// 直道加速调参项。
// 基础速度始终由 speed_target_l/r 决定；
// 只有长直道连续确认后，才额外叠加 straight_add_speed。
extern int g_straight_add_speed;
extern float g_straight_angle_limit;
extern int g_straight_confirm_frames;
extern int g_straight_min_centerline_pts;
extern int g_straight_min_side_pts;
extern int g_straight_corner_min_id;

// 普通寻线下 NCNN 识别绕行参数。
// 第一版先用红框 center_y 触发进入分类，不再依赖估距。
extern int g_ncnn_trigger_center_y;
extern int g_ncnn_trigger_confirm_frames;
extern int g_ncnn_classify_stable_frames;
extern float g_ncnn_classify_min_score;
extern float g_ncnn_slow_speed_ratio;
extern float g_ncnn_bypass_offset_scale;
extern float g_ncnn_recover_offset_scale;
extern float g_ncnn_recover_bias_deg;
extern int g_ncnn_bypass_pass_counts;
extern int g_ncnn_recover_counts;


extern int pesrsp_photo_flag;
extern int param_photo_flag;

extern bool straight_accel_active;

extern float lookahead_speed ;



extern int begin_x_offset_l;
extern int begin_y_offset_l;

extern int begin_x_offset_r;
extern int begin_y_offset_r;
extern int lost_line_flag;
