#include "app_config.h"

// 本文件只放默认调参值。
// 后续试车若要调 PID、限幅、前瞻点、发车手感，优先改这里。

// 默认巡航速度。
// 弯里仍会被 motor_task 按纯跟踪误差和元素状态进一步压低。
int base_speed_target_l = 2000;
int base_speed_target_r = 2000;

float target_v = -20;
// 发车调参项。
// 如果起步太冲，优先调大 ramp_ms 或调小 start_ratio。
int g_launch_delay_ms = 500;
int g_launch_ramp_ms = 800;
int g_zebra_enable_after_ms = 5000;
float g_launch_start_ratio = 0.01f;
float g_launch_turn_ratio = 0.4f;
float g_launch_encoder_release_threshold = 600.0f;
float g_launch_delta_limit = 180.0f;
// 弯道双前瞻前馈默认关闭，先保证原车行为不变。
// 建议试车顺序：
// 1. enable = 1 打开功能
// 2. far_offset 先保持 3
// 3. k_delta 从小往上加，优先看入弯是否提前
// 4. 若车头发飘，先增大 low_pass 或减小 k_delta
// 5. 若前馈过冲，再收紧 limit 或提高 active_angle_min
int g_curve_ff_enable = 0;  
int g_curve_ff_far_offset = 3;           // 远前瞻比当前前瞻额外多看的采样点数
float g_curve_ff_k_delta = 0.20f;        // 双前瞻角差 -> 前馈角速度 的比例系数
float g_curve_ff_limit = 20.0f;          // 前馈角速度绝对值上限，防止提前量过猛
float g_curve_ff_low_pass = 0.3f;        // 前馈一阶低通系数，越小越稳，越大越跟手
float g_curve_ff_active_angle_min = 10.0f;// 纯跟踪误差超过该阈值才认为进入弯道，单位：度

// 纯跟踪前瞻点。
// 当前普通道路已经改成“按目标速度换算前瞻距离”。
// 这里这几组点数现在主要留给特殊元素或手动兜底使用。
int g_line_lookahead_straight = 12;
int g_line_lookahead_mid = 8;
int g_line_lookahead_turn = 5;
int g_line_lookahead_hairpin = 6;
// 普通道路下，前瞻点现在是：
// 左右轮平均速度 -> 前瞻距离 -> 前瞻点数
//
// 输出限幅。
// 这些值直接决定“最多能打多少差速”和“PWM 最终允许到多大”。
float g_diff_output_limit = 9000.0f;
float g_target_speed_limit = 6000.0f;
float g_pwm_output_min_launch = -2000.0f;
float g_pwm_output_min = -7000.0f;
float g_pwm_output_max = 7000.0f;
int g_brushless_duty = 0;


int pesrsp_photo_flag = 0;
int param_photo_flag = 0;



bool straight_accel_active = 0;
float lookahead_speed = 0;


// 速度环 PID。
// 若目标轮速和实际轮速跟不紧，主要看这里。
pid_incremental_t pid_speed_1 =
{
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .out_max = 5000.0f,
};

pid_incremental_t pid_speed_r =
{
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .out_max = 5000.0f,
};

// 角速度环 PID。
// 若目标角速度已经出来，但车身转向响应慢，主要看这里。
pid_positional_t pid_angle_v =
{
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.000f,
    .p_max = 5000.0f,
    .i_max = 0.0f,
    .d_max = 1050.0f,
    .low_pass = 1.0f,
};



pid_positional_t pid_angle =
{
    .kp = 0.0f, //28
    .ki = 0.0f,
    .kd = 0.00f, //0.15
    .p_max = 2000.0f,
    .i_max = 0.0f,
    .d_max = 1000.0f,
    .low_pass = 1.0f,
    .kp_2 = 0.0f,
};
pid_positional_t pid_angle_circle =
{
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .p_max = 7000.0f,
    .i_max = 0.0f,
    .d_max = 1000.0f,
    .low_pass = 1.0f,
};

// 元素识别与补线参数。
// 角点偏移用于重新找线起点，far_start_thres 用s于远线二值判断。
int g_Lpt0_offset_x = 5;
int g_Lpt0_offset_y = 5;
int g_Lpt1_offset_x = 5;
int g_Lpt1_offset_y = 5;
int far_start_thres = 100;

// 直道加速参数。
// 目标是保留弯道基础速度，只在“看得远且误差小”的长直道上加速。
int g_straight_add_speed = 0;        // 直道额外加多少速度，最终效果是 base_speed + add_speed
float g_straight_angle_limit = 10.0f;  // 纯跟踪误差上限，车头偏得太多就不允许当直道
int g_straight_confirm_frames = 4;     // 连续满足多少个控制周期后，才真正进入直道加速
int g_straight_min_centerline_pts = 20;// 中线最少要看到多少个点，避免“看不远”时误加速
int g_straight_min_side_pts = 30;      // 单侧边线最少点数，至少要有一侧边界足够长
int g_straight_corner_min_id = 18;     // 若角点太靠近车身，说明快进弯/进元素了，不算直道

// 普通寻线下 NCNN 识别绕行参数。
// 第一版使用红框 center_y 触发识别，并通过“单边线主导”完成绕行。
int g_ncnn_trigger_center_y = 40;
int g_ncnn_trigger_confirm_frames = 2;
int g_ncnn_classify_stable_frames = 3;
float g_ncnn_classify_min_score = 0.50f;
float g_ncnn_slow_speed_ratio = 1.0f;
float g_ncnn_bypass_offset_scale = -0.2f;
float g_ncnn_recover_offset_scale = 0.92f;
float g_ncnn_recover_bias_deg = 2.0f;
int g_ncnn_bypass_pass_counts = 9000;
int g_ncnn_recover_counts = 2000;
