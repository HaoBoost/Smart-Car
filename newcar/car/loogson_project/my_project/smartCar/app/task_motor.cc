#include "app_context.h"
#include "app_tasks.h"



namespace {

// 直道加速不是一帧命中就立刻生效，而是做成一个简易的“连续确认状态”：
// 1. 当前帧像长直道 -> 计数加 1
// 2. 当前帧不像长直道 -> 计数减 1
// 3. 计数达到阈值 -> 允许在基础速度上叠加 straight_add_speed
//
// 这样做的目的，是避免出弯瞬间、边线抖动、角点误检时一帧误判直道，
// 导致车刚进弯或刚出弯就突然加速。
int g_straight_confirm_cnt_runtime = 0;
bool g_straight_accel_active_runtime = false;
float g_filtered_lookahead_speed_runtime = 0.0f;
float g_curve_ff_filtered_runtime = 0.0f;

int apply_launch_speed_ramp(int target_speed)
{
    // 起步阶段不要一步跳到目标速度，避免刚发车就大电流或甩尾。
    if (target_speed <= 0) return 0;
    if (g_launch_ramp_ms <= 0) return target_speed;

    float progress = static_cast<float>(g_launch_ramp_elapsed_ms) / static_cast<float>(g_launch_ramp_ms);
    progress = PID_CLAMP(progress, 0.0f, 1.0f);

    float start_ratio = PID_CLAMP(g_launch_start_ratio, 0.0f, 1.0f);
    float speed_ratio = start_ratio + (1.0f - start_ratio) * progress;
    return static_cast<int>(target_speed * speed_ratio);
}

float calc_forward_lookahead_speed(float spd_l, float spd_r)
{
    // 前瞻点现在跟“当前实际前进速度”走，不再直接跟目标速度走。
    //
    // 这里不用真实物理速度（m/s），而是直接用当前工程内部的轮速单位。
    // 只要后面的前瞻距离换算系数也基于同一套单位调参，控制逻辑就是自洽的。
    //
    // 对差速车来说，如果只想知道“车整体向前跑得有多快”，
    // 最简单也最合理的近似就是左右轮速度平均值：
    //
    //      v_forward = (v_l + v_r) / 2
    //
    // 这样做的好处是：
    // 1. 直道时，两轮接近，平均值就接近真实前进速度；
    // 2. 弯道时，一边快一边慢，平均值仍然代表“整体前进速度”；
    // 3. 不会像把角速度也算进去那样，弯里反而把前瞻错误拉远。
    //
    // 另外这里再加一个一阶低通滤波。
    // 原因是编码器速度天然会抖，若直接拿瞬时轮速去算前瞻点，
    // look_ahead_point 会在相邻周期来回跳，车头手感会发飘。
    float raw_speed = (spd_l + spd_r) * 0.5f;
    // 前瞻距离只关心“向前速度”，倒车或瞬时反号时不希望把前瞻拉成负数。
    raw_speed = PID_MAX(raw_speed, 0.0f);

    // alpha 越大，跟随越快；alpha 越小，前瞻点越稳但响应更慢。
    const float alpha = 0.3f;
    g_filtered_lookahead_speed_runtime =
        alpha * raw_speed + (1.0f - alpha) * g_filtered_lookahead_speed_runtime;
    return g_filtered_lookahead_speed_runtime;
}

float calc_launch_progress()
{
    if (g_launch_ramp_ms <= 0) return 1.0f;

    float progress = static_cast<float>(g_launch_ramp_elapsed_ms) / static_cast<float>(g_launch_ramp_ms);
    return PID_CLAMP(progress, 0.0f, 1.0f);
}

float calc_launch_diff_scale()
{
    const float start_ratio = PID_CLAMP(g_launch_turn_ratio, 0.0f, 1.0f);
    const float progress = calc_launch_progress();
    return start_ratio + (1.0f - start_ratio) * progress;
}

bool is_launch_low_speed(float spd_l, float spd_r)
{
    return fabsf(spd_l) + fabsf(spd_r) < g_launch_encoder_release_threshold;
}

float calc_curve_gyro_feedforward()
{
    if (!g_curve_ff_enable) return 0.0f;
    if (g_elem_type != ELEM_NONE || g_circle_state != CIRCLE_NONE || g_cross_state != CROSS_NONE) return 0.0f;
    if (g_launch_ramp_elapsed_ms < g_launch_ramp_ms) return 0.0f;
    if (g_rptsn_num < 6) return 0.0f;
    if (fabsf(g_pure_angle) < g_curve_ff_active_angle_min) return 0.0f;

    const float ff = g_curve_ff_k_delta * g_pure_angle_ff_delta;
    return PID_CLAMP(ff, -g_curve_ff_limit, g_curve_ff_limit);
}

}  // namespace

namespace {

float ncnn_action_bias_deg()
{
    if (g_elem_type != ELEM_NONE) return 0.0f;
    return g_ncnn_bypass_bias_deg;
}

bool ncnn_action_slow_active()
{
    if (g_elem_type != ELEM_NONE) return false;
    return g_ncnn_action_state == NCNN_ACT_SLOW_CLASSIFY
        || g_ncnn_action_state == NCNN_ACT_BYPASS
        || g_ncnn_action_state == NCNN_ACT_RECOVER;
}

}  // namespace

int calc_speed_related_look_ahead_point(float vehicle_speed)
{

    float lookahead_dist = vehicle_speed * g_line_lookahead_speed_gain;
    lookahead_dist = PID_CLAMP(lookahead_dist, g_line_lookahead_dist_min, g_line_lookahead_dist_max);

    int lookahead_points = static_cast<int>(lroundf(lookahead_dist / SAMPLE_DIST));
    lookahead_points = PID_CLAMP(lookahead_points, 3, POINTS_MAX - 1);
    return lookahead_points;
}

namespace {

bool is_side_straight_candidate(float pts[][2], int num, bool lpt_found, int lpt_id)
{
    // 单侧边线想被认为是“直道候选”，至少要满足三件事：
    // 1. 点数够长，说明这一侧确实看得比较远；
    // 2. 局部角度变化持续很小，说明这条边线整体够直；
    // 3. 没有很近的 L 角点，避免把“马上进十字/圆环/急弯”的画面当成直道。
    if (num < g_straight_min_side_pts) return false;
    if (!is_long_straight(pts, num)) return false;
    return !lpt_found || lpt_id >= g_straight_corner_min_id;
}

bool is_long_straight_candidate()
{
    // 这里对应你原来工程里的 long_state 候选阶段，但实现方式换成了
    // 当前仓库已有的视觉特征：
    // - 不在特殊元素状态里
    // - 中线/边线都要看得足够远
    // - 纯跟踪误差要小，说明车头基本摆正
    // - 左右边线至少一侧明显是长直线
    //
    // 注意：这里只是“这一帧像直道”，真正加速还要经过后面的连续确认。
    if (g_elem_type != ELEM_NONE || g_circle_state != CIRCLE_NONE || g_cross_state != CROSS_NONE) return false;
    if (g_rptsn_num < g_straight_min_centerline_pts) return false;
    if (fabsf(g_pure_angle) > g_straight_angle_limit) return false;

    const bool left_straight = is_side_straight_candidate(g_rpts0s, g_rpts0s_num, g_Lpt0_found, g_Lpt0_id);
    const bool right_straight = is_side_straight_candidate(g_rpts1s, g_rpts1s_num, g_Lpt1_found, g_Lpt1_id);
    return left_straight || right_straight;
}

bool update_straight_accel_state()
{
    // 这里相当于做一个带迟滞的状态确认：
    // 连续像直道时逐步“充能”，不像直道时逐步“放电”。
    // 比单纯 bool 开关稳定，实车上不容易来回抽动。
    if (is_long_straight_candidate()) {
        if (g_straight_confirm_cnt_runtime < g_straight_confirm_frames) {
            g_straight_confirm_cnt_runtime++;
        }
    } else if (g_straight_confirm_cnt_runtime > 0) {
        g_straight_confirm_cnt_runtime--;
    }

    g_straight_accel_active_runtime = g_straight_confirm_cnt_runtime >= g_straight_confirm_frames;
    return g_straight_accel_active_runtime;
}

}  // namespace

void stop_quick()
{

        // 停车态下把控制链路的历史量都清掉，避免再次起步时带着旧状态。
        g_launch_ramp_elapsed_ms = 0;
        g_run_elapsed_ms = 0;
        // 直道确认状态也要同步清零，否则重新发车时可能直接沿用上一次的加速态。
        g_straight_confirm_cnt_runtime = 0;
        g_straight_accel_active_runtime = false;
        straight_accel_active = false;
        // 前瞻速度滤波值也要清零，避免重新发车时沿用上一次停车前的速度记忆。
        g_filtered_lookahead_speed_runtime = 0.0f;
        g_curve_ff_filtered_runtime = 0.0f;
        lookahead_speed = 0.0f;
        g_target_gyro_dbg = 0.0f;
        g_target_gyro_ff_dbg = 0.0f;
        g_target_speed_l_dbg = 0.0f;
        g_target_speed_r_dbg = 0.0f;
        g_pwm_l_dbg = 0.0f;
        g_pwm_r_dbg = 0.0f;
        reset_pid_state(pid_angle);
        reset_pid_state(pid_angle_circle);
        reset_pid_state(pid_angle_v);
        reset_pid_state(pid_angle_v_circle);
        reset_pid_state(pid_speed_1);
        reset_pid_state(pid_speed_r);
        motor.set_motor1(0, 0);
        motor.set_motor2(0, 0);
        brush.set_duty(0);
}




void motor_task(void* arg)
{
    
    (void)arg;
    static int __10ms = 0;
    static int __5ms = 0;
    static int error_cnt = 0;
    int target_speed_l = 0;
    int target_speed_r = 0;
    // 正常循迹时如果双边都长时间丢线，直接停车保护。
    // if ((g_elem_type == ELEM_NONE && g_ipts0_num < 5 && g_ipts1_num < 5) ||
    //     (g_elem_type == ELEM_CIRCLE && g_ipts0_num < 5 && g_ipts1_num < 5)
    //     && g_ncnn_action_state != NCNN_ACT_BYPASS && 
    //     g_ncnn_action_state != NCNN_ACT_RECOVER) 
    //     {
    //     if (++error_cnt == 100) {
    //         motor.set_motor1(0, 0);
    //         motor.set_motor2(0, 0);
    //         brush.set_duty(0);
    //         start_flag = 0;
    //         start__1000ms_flag = 0;
    //         __1000ms = 0;
    //         error_cnt = 0;
    //     }
    // }
    // else
    // {
    //     error_cnt = 0;
    // }

    // 延时发车请求优先级最高：停车计时，到点后再切到速度斜坡起步。
    if (start__1000ms_flag)
    {
        stop_quick();
        if (++__1000ms < g_launch_delay_ms) {
            return;
        }
        start__1000ms_flag = 0;
        start_flag = 1;
        __1000ms = 0;
        __10ms = 0;
        __5ms = 0;
    }
    else
    {
        __1000ms = 0;
    }

    //紧急刹车
    if(!start_flag)
    {
        stop_quick();
        return ;
    }

    if (g_launch_ramp_elapsed_ms < g_launch_ramp_ms) {
        ++g_launch_ramp_elapsed_ms;
    }
    ++g_run_elapsed_ms;

    brush.set_duty(g_brushless_duty);
    // 每个周期都处理一次上位机参数，方便在线调 PID。
    seekfree_client.process_parameters();

    //PID计算部分
    if(++__10ms >= 10)
    {
        __10ms = 0;

        //误差增益
        const float pure_angle_with_bias = g_pure_angle + ncnn_action_bias_deg();
        const float gyro_fb = pid_positional_solve_Error2(&pid_angle, pure_angle_with_bias);
        const float gyro_ff_raw = calc_curve_gyro_feedforward();
        const float ff_alpha = PID_CLAMP(g_curve_ff_low_pass, 0.0f, 1.0f);
        g_curve_ff_filtered_runtime =
            ff_alpha * gyro_ff_raw + (1.0f - ff_alpha) * g_curve_ff_filtered_runtime;
        g_target_gyro_ff_dbg = g_curve_ff_filtered_runtime;
        g_target_gyro_dbg = gyro_fb + g_target_gyro_ff_dbg;
        
    }
    if(++__5ms >= 5)
    {
        __5ms = 0;

        /********角速度环********/
        //更新角速度
        float actual_gyro = imu.gyro_z;
        float gyro_error = g_target_gyro_dbg - actual_gyro;
        // diff_output 表示左右轮目标速度差，正负决定左右转向。
        float diff_output = 0.0f;
        diff_output = pid_positional_solve(&pid_angle_v, gyro_error);

        /********速度环********/
        //更新速度
        motor.update_encoders();
        motor.thread_syn();
        const int encoder1_counts = motor.syn_encoder1_counts;
        const int encoder2_counts = motor.syn_encoder2_counts;
        // 编码器计数换算成轮速，系数 12.2f 是当前车模标定值。
        float spd_l = -encoder1_counts * 12.2f;
        float spd_r = -encoder2_counts * 12.2f;
        const bool in_launch_ramp = g_launch_ramp_elapsed_ms < g_launch_ramp_ms;
        target_speed_l = base_speed_target_l;
        target_speed_r = base_speed_target_r;

        // if(g_track_side)
        // // printf("左边线\r\n");
        // else
        // // printf("右边线\r\n");
        //速度决策部分
        straight_accel_active = update_straight_accel_state(); //判断是否为直道
        if (straight_accel_active)  //是直道
        {
            target_speed_l += g_straight_add_speed;
            target_speed_r += g_straight_add_speed;
            // printf("是直道路\r\n");
        }
        else //不是直道
        {
           switch (g_elem_type) 
           {
            case ELEM_CROSS:

            
            break;
            case ELEM_NONE: {
                //计算平均速度
                lookahead_speed = calc_forward_lookahead_speed(spd_l, spd_r);
                if (in_launch_ramp) {
                    look_ahead_point = g_line_lookahead_straight;
                }
                break;
            }
            case ELEM_CIRCLE:
                switch (g_circle_state) 
                {
                    case CIRCLE_BEGIN:
                        target_speed_l  *= 0.7f;
                        target_speed_r  *= 0.7f;
                        g_centerline_offset = 1.0f;
                        g_track_side = g_circle_type ? 1 : 0;
                        break;
                    case CIRCLE_APPROACH:
                        target_speed_l  *= 0.9f;
                        target_speed_r  *= 0.9f;
                        g_centerline_offset = 1.0f;
                        g_track_side = g_circle_type ? 0 : 1;
                        break;
                    case CIRCLE_RUNNING:
                        target_speed_l  *= 0.9f;
                        target_speed_r  *= 0.9f;
                        g_centerline_offset = 1.0f;
                        g_track_side = g_circle_type ? 1 : 0;
                        break;
                    case CIRCLE_OUT:
                        target_speed_l  *= 0.9f;
                        target_speed_r  *= 0.9f;
                        g_centerline_offset = 1.0f;
                        g_track_side = g_circle_type ? 0 : 1;
                        break;
                    case CIRCLE_END:
                        target_speed_l  *= 0.9f;
                        target_speed_r  *= 0.9f;
                        g_centerline_offset = 1.0f;
                        g_track_side = g_circle_type ? 1 : 0;
                        break;
                    default:
                        g_centerline_offset = 1.0f;
                        g_track_side = g_circle_type ? 1 : 0;
                        break;
                }
                break;
                default:
                if (line_point < look_ahead_point) 
                {
                    look_ahead_point = line_point;
                }
                break;
            }
        }
        if (ncnn_action_slow_active()) {
            target_speed_l = static_cast<int>(target_speed_l * g_ncnn_slow_speed_ratio);
            target_speed_r = static_cast<int>(target_speed_r * g_ncnn_slow_speed_ratio);
        }

        target_speed_l = PID_CLAMP(target_speed_l, 0, static_cast<int>(g_target_speed_limit));
        target_speed_r = PID_CLAMP(target_speed_r, 0, static_cast<int>(g_target_speed_limit));
        if (in_launch_ramp) {
            target_speed_l = apply_launch_speed_ramp(target_speed_l);
            target_speed_r = apply_launch_speed_ramp(target_speed_r);
        }

        const float diff_output_limit_dynamic =
            g_diff_output_limit * (in_launch_ramp ? calc_launch_diff_scale() : 1.0f);
        diff_output = PID_CLAMP(diff_output, -diff_output_limit_dynamic, diff_output_limit_dynamic);

        //差速计算
        // float tgt_l = target_speed_l - diff_output;
        // float tgt_r = target_speed_r + diff_output;

        float tgt_l = 2000;
        float tgt_r = 2000;
        // if (diff_output >= 0) 
        // {
        //     tgt_l = target_speed_l - 1.5f *diff_output;
        //     tgt_r = target_speed_r + 0.8 * diff_output;
        // } 
        // else 
        // {
        //     tgt_l = target_speed_l -  0.8 * diff_output;
        //     tgt_r = target_speed_r + 1.5f * diff_output;
        // }

        if (diff_output >= 0) 
        {
            tgt_l = target_speed_l - 1 * diff_output;
            tgt_r = target_speed_r +  diff_output;
        } 
        else 
        {
            tgt_l = target_speed_l -   diff_output;
            tgt_r = target_speed_r +  1 * diff_output;
        }
        //计算速度环PID
        float delta_l = pid_incremental_solve(&pid_speed_1, tgt_l - spd_l);
        float delta_r = pid_incremental_solve(&pid_speed_r, tgt_r - spd_r);
        if (in_launch_ramp && is_launch_low_speed(spd_l, spd_r)) {
            delta_l = PID_CLAMP(delta_l, -g_launch_delta_limit, g_launch_delta_limit);
            delta_r = PID_CLAMP(delta_r, -g_launch_delta_limit, g_launch_delta_limit);
        }

        g_pwm_l_dbg += delta_l;
        g_pwm_r_dbg += delta_r;
        //pwm限幅
        const float pwm_output_min = in_launch_ramp ? g_pwm_output_min_launch : g_pwm_output_min;
        g_pwm_l_dbg = PID_CLAMP(g_pwm_l_dbg, pwm_output_min, g_pwm_output_max);
        g_pwm_r_dbg = PID_CLAMP(g_pwm_r_dbg, pwm_output_min, g_pwm_output_max);
        //PWM作用到电机
        motor.set_motor1(g_pwm_l_dbg > 0 ? 1 : 0, abs(g_pwm_l_dbg));
        motor.set_motor2(g_pwm_r_dbg > 0 ? 0 : 1, abs(g_pwm_r_dbg));
        //示波器数据更新
        seekfree_client.send_oscilloscope(tgt_l,tgt_r,spd_l, spd_r,g_target_gyro_dbg,actual_gyro,look_ahead_point,g_pure_angle);

        g_actual_gyro_dbg = actual_gyro;
        g_speed_l_dbg = spd_l;
        g_speed_r_dbg = spd_r;
        g_target_speed_l_dbg = tgt_l;
        g_target_speed_r_dbg = tgt_r;
    }
}
