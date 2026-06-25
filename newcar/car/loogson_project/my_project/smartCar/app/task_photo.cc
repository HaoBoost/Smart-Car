#include "app_context.h"
#include "app_tasks.h"

// photo_task 负责“每帧视觉链路”：
// 1. 图像预处理
// 2. 巡线和元素识别
// 3. 中线与纯跟踪误差计算
// 4. 调试图像 / telemetry 打包

namespace {

// 视觉主链路固定跑 160x120。
// 这里把小图缓冲固定下来，避免每帧 resize 时反复更换底层内存。
cv::Mat g_photo_frame_small(IMG_H, IMG_W, CV_8UC3);
cv::Mat g_photo_frame_full(240, 320, CV_8UC3);

inline bool is_valid_point_index(int idx, int num)
{
    return idx >= 0 && idx < num && idx < POINTS_MAX;
}

inline float point_x_or_zero(float pts[][2], int num, int idx)
{
    return is_valid_point_index(idx, num) ? pts[idx][0] : 0.0f;
}

inline float point_y_or_zero(float pts[][2], int num, int idx)
{
    return is_valid_point_index(idx, num) ? pts[idx][1] : 0.0f;
}

void ensure_photo_buffers_ready()
{
    if (g_photo_frame_full.rows != 240 || g_photo_frame_full.cols != 320
        || g_photo_frame_full.type() != CV_8UC3) {
        g_photo_frame_full.create(240, 320, CV_8UC3);
    }

    if (g_photo_frame_small.rows != IMG_H || g_photo_frame_small.cols != IMG_W
        || g_photo_frame_small.type() != CV_8UC3) {
        g_photo_frame_small.create(IMG_H, IMG_W, CV_8UC3);
    }

    // gray 是全局共享灰度图，create() 在尺寸和类型不变时不会重复分配。
    if (gray.rows != IMG_H || gray.cols != IMG_W || gray.type() != CV_8UC1) {
        gray.create(IMG_H, IMG_W, CV_8UC1);
    }

    if (g_red_model_input_40.rows != 40 || g_red_model_input_40.cols != 40
        || g_red_model_input_40.type() != CV_8UC3) {
        g_red_model_input_40.create(40, 40, CV_8UC3);
    }
}

const char* elem_name_of(int elem_type)
{
    switch (elem_type) {
        case ELEM_CROSS: return "CROSS";
        case ELEM_CIRCLE: return "CIRCLE";
        case ELEM_OBSTACLE: return "OBSTACLE";
        case ELEM_RAMP: return "RAMP";
        case ELEM_ZEBRA: return "ZEBRA";
        case ELEM_NCNN: return "NCNN";
        default: return "NONE";
    }
}

const char* cross_name_of(int cross_state)
{
    switch (cross_state) {
        case CROSS_BEGIN: return "BEGIN";
        case CROSS_RUNNING: return "RUN";
        case CROSS_END: return "END";
        default: return "NONE";
    }
}

const char* circle_name_of(int circle_state)
{
    switch (circle_state) {
        case CIRCLE_BEGIN: return "BEGIN";
        case CIRCLE_APPROACH: return "APPROACH";
        case CIRCLE_RUNNING: return "RUN";
        case CIRCLE_OUT: return "OUT";
        case CIRCLE_END: return "END";
        default: return "NONE";
    }
}

const char* track_name_of(int track_side)
{
    return track_side == 0 ? "RIGHT" : "LEFT";
}

const char* circle_type_name_of(int circle_type)
{
    return circle_type == 0 ? "RIGHT" : "LEFT";
}

void update_camera_telemetry()
{
    // 这里把运行时关键状态打包给图传页面，便于现场直接看控制链路。
    camera_telemetry.target_speed_l = g_target_speed_l_dbg;
    camera_telemetry.target_speed_r = g_target_speed_r_dbg;
    camera_telemetry.actual_speed_l = g_speed_l_dbg;
    camera_telemetry.actual_speed_r = g_speed_r_dbg;
    camera_telemetry.pwm_l = g_pwm_l_dbg;
    camera_telemetry.pwm_r = g_pwm_r_dbg;
    camera_telemetry.target_gyro = g_target_gyro_dbg;
    camera_telemetry.actual_gyro = g_actual_gyro_dbg;
    camera_telemetry.pure_angle = g_pure_angle;

    camera_telemetry.pid_speed_l_kp = pid_speed_1.kp;
    camera_telemetry.pid_speed_l_ki = pid_speed_1.ki;
    camera_telemetry.pid_speed_l_kd = pid_speed_1.kd;
    camera_telemetry.pid_speed_r_kp = pid_speed_r.kp;
    camera_telemetry.pid_speed_r_ki = pid_speed_r.ki;
    camera_telemetry.pid_speed_r_kd = pid_speed_r.kd;
    camera_telemetry.pid_gyro_kp = pid_angle_v.kp;
    camera_telemetry.pid_gyro_ki = pid_angle_v.ki;
    camera_telemetry.pid_gyro_kd = pid_angle_v.kd;
    camera_telemetry.pid_angle_kp = pid_angle.kp;
    camera_telemetry.pid_angle_ki = pid_angle.ki;
    camera_telemetry.pid_angle_kd = pid_angle.kd;
    camera_telemetry.pid_circle_kp = pid_angle_circle.kp;
    camera_telemetry.pid_circle_ki = pid_angle_circle.ki;
    camera_telemetry.pid_circle_kd = pid_angle_circle.kd;

    camera_telemetry.start_flag = start_flag;
    camera_telemetry.look_ahead_point = look_ahead_point;
    camera_telemetry.track_side = g_track_side;
    camera_telemetry.elem_type = g_elem_type;
    camera_telemetry.cross_state = g_cross_state;
    camera_telemetry.circle_state = g_circle_state;
    camera_telemetry.circle_type = g_circle_type;

    camera_telemetry.ipts0_num = g_ipts0_num;
    camera_telemetry.ipts1_num = g_ipts1_num;
    camera_telemetry.rptsn_num = g_rptsn_num;
    camera_telemetry.rpts0s_num = g_rpts0s_num;
    camera_telemetry.rpts1s_num = g_rpts1s_num;
    camera_telemetry.far_rpts0s_num = g_far_rpts0s_num;
    camera_telemetry.far_rpts1s_num = g_far_rpts1s_num;

    camera_telemetry.lpt0_found = g_Lpt0_found ? 1 : 0;
    camera_telemetry.lpt1_found = g_Lpt1_found ? 1 : 0;
    camera_telemetry.lpt0_id = g_Lpt0_id;
    camera_telemetry.lpt1_id = g_Lpt1_id;
    camera_telemetry.lpt0_x = point_x_or_zero(g_rpts0s, g_rpts0s_num, g_Lpt0_id);
    camera_telemetry.lpt0_y = point_y_or_zero(g_rpts0s, g_rpts0s_num, g_Lpt0_id);
    camera_telemetry.lpt1_x = point_x_or_zero(g_rpts1s, g_rpts1s_num, g_Lpt1_id);
    camera_telemetry.lpt1_y = point_y_or_zero(g_rpts1s, g_rpts1s_num, g_Lpt1_id);
    camera_telemetry.far_lpt0_found = g_far_Lpt0_found ? 1 : 0;
    camera_telemetry.far_lpt1_found = g_far_Lpt1_found ? 1 : 0;
    camera_telemetry.far_lpt0_id = g_far_Lpt0_id;
    camera_telemetry.far_lpt1_id = g_far_Lpt1_id;
    camera_telemetry.far_lpt0_x = point_x_or_zero(g_far_rpts0s, g_far_rpts0s_num, g_far_Lpt0_id);
    camera_telemetry.far_lpt0_y = point_y_or_zero(g_far_rpts0s, g_far_rpts0s_num, g_far_Lpt0_id);
    camera_telemetry.far_lpt1_x = point_x_or_zero(g_far_rpts1s, g_far_rpts1s_num, g_far_Lpt1_id);
    camera_telemetry.far_lpt1_y = point_y_or_zero(g_far_rpts1s, g_far_rpts1s_num, g_far_Lpt1_id);

    camera_telemetry.elem_name = elem_name_of(g_elem_type);
    camera_telemetry.cross_name = cross_name_of(g_cross_state);
    camera_telemetry.circle_name = circle_name_of(g_circle_state);
    camera_telemetry.track_name = track_name_of(g_track_side);
    camera_telemetry.circle_type_name = circle_type_name_of(g_circle_type);
    camera_telemetry.classify_label = g_red_model_last_label;
    camera_telemetry.classify_score = g_red_model_last_score;
    camera_telemetry.classify_probs_text = g_red_model_last_probs_text;
    camera_telemetry.ncnn_action_state_name = ncnn_action_state_name(g_ncnn_action_state);
    camera_telemetry.ncnn_action_cmd_name = ncnn_action_cmd_name(g_ncnn_locked_cmd);
    camera_telemetry.ncnn_classify_stable_count = g_ncnn_classify_stable_count;
    camera_telemetry.ncnn_red_center_y = g_ncnn_red_center_y;
    camera_telemetry.ncnn_action_encoder_l = g_ncnn_action_encoder_l;
    camera_telemetry.ncnn_action_encoder_r = g_ncnn_action_encoder_r;
    camera_telemetry.ncnn_high_level_frozen = g_ncnn_high_level_frozen ? 1 : 0;
    camera_telemetry.red_detected_this_frame = g_red_detected_this_frame ? 1 : 0;
    camera_telemetry.red_waiting_distance = g_red_detected_waiting_distance ? 1 : 0;
    camera_telemetry.red_cycle_done = g_red_model_cycle_done ? 1 : 0;
    camera_telemetry.red_classify_enabled = g_red_model_classify_enabled ? 1 : 0;
    camera_telemetry.red_model_input_ready = g_red_model_input_ready ? 1 : 0;
    camera_telemetry.red_estimated_distance_cm = g_red_model_estimated_distance_cm;
    camera_telemetry.red_trigger_distance_cm = g_red_model_trigger_distance_cm;
    camera_telemetry.red_pixels_per_cm = g_red_model_pixels_per_cm;
    camera_telemetry.red_box_x = g_red_box_full.x;
    camera_telemetry.red_box_y = g_red_box_full.y;
    camera_telemetry.red_box_w = g_red_box_full.width;
    camera_telemetry.red_box_h = g_red_box_full.height;
    camera_telemetry.red_model_box_x = g_red_model_box_full.x;
    camera_telemetry.red_model_box_y = g_red_model_box_full.y;
    camera_telemetry.red_model_box_w = g_red_model_box_full.width;
    camera_telemetry.red_model_box_h = g_red_model_box_full.height;
}

}  // namespace




void look_ahead_point_decision()
{

    if(start_flag && g_launch_ramp_elapsed_ms < g_launch_ramp_ms) // 起步斜坡期固定前瞻，避免低速抖动
    {
        look_ahead_point = g_line_lookahead_straight;
    }
    else if(straight_accel_active && g_ncnn_action_state == NCNN_ACT_IDLE) //是直道
    {
        look_ahead_point = 12;
    }
    else //不是直道
    {
        switch(g_elem_type)
        {
            case ELEM_CROSS:
                look_ahead_point = 8;
                break;
            case ELEM_NONE: 
            {
                //计算前瞻点
                // look_ahead_point = calc_speed_related_look_ahead_point(lookahead_speed);
                 
                 if(g_ncnn_action_state == NCNN_ACT_LOCKED || g_ncnn_action_state == NCNN_ACT_BYPASS || g_ncnn_action_state == NCNN_ACT_RECOVER)
                 {
                    look_ahead_point = 3;
                 }
                 else
                 look_ahead_point = 10;
                break;
            }
            case ELEM_CIRCLE:
                switch (g_circle_state) 
                {
                    case CIRCLE_BEGIN:
                        look_ahead_point = 8;
                        break;
                    case CIRCLE_APPROACH:
                        look_ahead_point = 6;
                        break;
                    case CIRCLE_RUNNING:
                        look_ahead_point = 6;
                        break;
                    case CIRCLE_OUT:
                        look_ahead_point = 6;
                        break;
                    case CIRCLE_END:
                        look_ahead_point = 6;
                        break;
                    default:
                        look_ahead_point = LOOK_AHEAD_POINT_EN;
                        break;
                }
            break;
    }
    }
    if (line_point < look_ahead_point) //基本上没有点了
    {
        look_ahead_point = PID_MIN(look_ahead_point, line_point);
    }
}







void photo_task()
{
    ensure_photo_buffers_ready();
    
    // 保留一份 320x240 翻转原图给后续模型裁剪，再缩到 160x120 跑主算法。
    cv::flip(frame, g_photo_frame_full, -1);
    frame_full = g_photo_frame_full;
    
    cv::resize(frame_full, g_photo_frame_small, cv::Size(IMG_W, IMG_H), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(g_photo_frame_small, gray, cv::COLOR_BGR2GRAY);
    
    // 巡线和元素识别继续使用 160x120 小图。
    frame = g_photo_frame_small;
    g_red_box_full = cv::Rect();
    g_red_model_box_full = cv::Rect();
    g_red_model_input_ready = false;

    // 视觉主链路顺序：
    // 先巡线，再识别元素并生成当前帧候选中线，
    // 然后统一提交中线，最后算纯跟踪误差。
    
    process_image(gray);
    element_task();
    if(g_elem_type == ELEM_NONE) run_none();
    else if (g_cross_state != CROSS_NONE && g_elem_type == ELEM_CROSS) run_cross();
    else if (g_circle_state != CIRCLE_NONE && g_elem_type == ELEM_CIRCLE) run_circle();
    track_task();
    look_ahead_point_decision();//前瞻决策
    g_pure_angle_near = calc_pure_pursuit_better(look_ahead_point);
    g_pure_angle = g_pure_angle_near;
    // 当前默认关闭调试绘图，但 telemetry 仍然要持续更新给图传页面。
    // 这样现场调 PID 时，即使不画叠加线，也能看到实时控制量。
    if(!pesrsp_photo_flag) return;
    g_photo_debug_us = 0;
    
    const auto debug_start = std::chrono::high_resolution_clock::now();
    update_camera_telemetry();
    perspective_frame.release();
    
    // 后面的代码是调试显示用，不参与实际控制。
    cv::Mat persp_view = cv::Mat::zeros(gray.size(), CV_8UC3);
    cv::Mat debug_view = frame_full.clone();
    if (start__1000ms_flag) {
        perspective_frame = persp_view.clone();
        frame = debug_view;
        update_camera_telemetry();
        const auto debug_end = std::chrono::high_resolution_clock::now();
        g_photo_debug_us =
            std::chrono::duration_cast<std::chrono::microseconds>(debug_end - debug_start).count();
        return;
    }

    for (int i = 0; i < g_rpts0s_num - 1; i++) {
        cv::line(persp_view,
                 cv::Point(static_cast<int>(g_rpts0s[i][0]), static_cast<int>(g_rpts0s[i][1])),
                 cv::Point(static_cast<int>(g_rpts0s[i + 1][0]), static_cast<int>(g_rpts0s[i + 1][1])),
                 cv::Scalar(0, 255, 0), 1);
    }
    for (int i = 0; i < g_rpts1s_num - 1; i++) {
        cv::line(persp_view,
                 cv::Point(static_cast<int>(g_rpts1s[i][0]), static_cast<int>(g_rpts1s[i][1])),
                 cv::Point(static_cast<int>(g_rpts1s[i + 1][0]), static_cast<int>(g_rpts1s[i + 1][1])),
                 cv::Scalar(255, 0, 0), 1);
    }
    if (g_rptsn_num > 0) {
        const int draw_look_ahead_idx = clip(look_ahead_point, 0, g_rptsn_num - 1);
        cv::circle(persp_view,
                   cv::Point(static_cast<int>(g_rptsn[draw_look_ahead_idx][0]), static_cast<int>(g_rptsn[draw_look_ahead_idx][1])),
                   4, cv::Scalar(0, 255, 255), 2);
    }

    for (int i = 0; i < g_far_rpts0s_num - 1; i++) {
        cv::line(persp_view,
                 cv::Point(static_cast<int>(g_far_rpts0s[i][0]), static_cast<int>(g_far_rpts0s[i][1])),
                 cv::Point(static_cast<int>(g_far_rpts0s[i + 1][0]), static_cast<int>(g_far_rpts0s[i + 1][1])),
                 cv::Scalar(0, 255, 0), 1);
    }
    for (int i = 0; i < g_far_ipts0_num - 1; i++) {
        cv::line(debug_view,
                 cv::Point(g_far_ipts0[i][0] * 2, g_far_ipts0[i][1] * 2),
                 cv::Point(g_far_ipts0[i + 1][0] * 2, g_far_ipts0[i + 1][1] * 2),
                 cv::Scalar(0, 255, 0), 1);
    }
    for (int i = 0; i < g_far_rpts1s_num - 1; i++) {
        cv::line(persp_view,
                 cv::Point(static_cast<int>(g_far_rpts1s[i][0]), static_cast<int>(g_far_rpts1s[i][1])),
                 cv::Point(static_cast<int>(g_far_rpts1s[i + 1][0]), static_cast<int>(g_far_rpts1s[i + 1][1])),
                 cv::Scalar(255, 0, 0), 1);
    }
    for (int i = 0; i < g_far_ipts1_num - 1; i++) {
        cv::line(debug_view,
                 cv::Point(g_far_ipts1[i][0] * 2, g_far_ipts1[i][1] * 2),
                 cv::Point(g_far_ipts1[i + 1][0] * 2, g_far_ipts1[i + 1][1] * 2),
                 cv::Scalar(255, 0, 0), 1);
    }

    for (int i = 0; i < g_rptsn_num - 1; i++) {
        cv::line(persp_view,
                 cv::Point(static_cast<int>(g_rptsn[i][0]), static_cast<int>(g_rptsn[i][1])),
                 cv::Point(static_cast<int>(g_rptsn[i + 1][0]), static_cast<int>(g_rptsn[i + 1][1])),
                 cv::Scalar(0, 0, 255), 2);
    }

    if (g_Lpt0_found && is_valid_point_index(g_Lpt0_id, g_rpts0s_num)) {
        cv::circle(persp_view,
                   cv::Point(static_cast<int>(g_rpts0s[g_Lpt0_id][0]), static_cast<int>(g_rpts0s[g_Lpt0_id][1])),
                   5, cv::Scalar(0, 255, 255), 2);
    }
    if (g_Lpt1_found && is_valid_point_index(g_Lpt1_id, g_rpts1s_num)) {
        cv::circle(persp_view,
                   cv::Point(static_cast<int>(g_rpts1s[g_Lpt1_id][0]), static_cast<int>(g_rpts1s[g_Lpt1_id][1])),
                   5, cv::Scalar(0, 255, 255), 2);
    }
    if (g_far_Lpt1_found && is_valid_point_index(g_far_Lpt1_id, g_far_rpts1s_num)) {
        cv::circle(persp_view,
                   cv::Point(static_cast<int>(g_far_rpts1s[g_far_Lpt1_id][0]), static_cast<int>(g_far_rpts1s[g_far_Lpt1_id][1])),
                   5, cv::Scalar(0, 255, 255), 2);
    }
    if (g_far_Lpt0_found && is_valid_point_index(g_far_Lpt0_id, g_far_rpts0s_num)) {
        cv::circle(persp_view,
                   cv::Point(static_cast<int>(g_far_rpts0s[g_far_Lpt0_id][0]), static_cast<int>(g_far_rpts0s[g_far_Lpt0_id][1])),
                   5, cv::Scalar(0, 255, 255), 2);
    }

    for (int i = 0; i < g_ipts0_num - 1; i++) {
        cv::line(debug_view,
                 cv::Point(g_ipts0[i][0] * 2, g_ipts0[i][1] * 2),
                 cv::Point(g_ipts0[i + 1][0] * 2, g_ipts0[i + 1][1] * 2),
                 cv::Scalar(0, 255, 0), 1);
    }

    for (int i = 0; i < g_ipts0_num - 1; i++) {
        cv::line(debug_view,
                 cv::Point(160, 0),
                 cv::Point(160, 240),
                 cv::Scalar(0, 255, 0), 1);
    }

    
    for (int i = 0; i < g_ipts1_num - 1; i++) {
        cv::line(debug_view,
                 cv::Point(g_ipts1[i][0] * 2, g_ipts1[i][1] * 2),
                 cv::Point(g_ipts1[i + 1][0] * 2, g_ipts1[i + 1][1] * 2),
                 cv::Scalar(255, 0, 0), 1);
    }

    const char* elem_names[] = {"NONE", "CROSS", "CIRCLE", "OBSTACLE", "RAMP", "ZEBRA"};
    std::string status = "Elem: ";
    status += elem_names[g_elem_type];

    if (g_elem_type == ELEM_CROSS) {
        const char* cross_states[] = {"NONE", "BEGIN", "RUN", "END"};
        status += " [" + std::string(cross_states[g_cross_state]) + "]";
    } else if (g_elem_type == ELEM_CIRCLE) {
        const char* circle_dirs[] = {"RIGHT", "LEFT"};
        const char* circle_states[] = {"NONE", "BEGIN", "APPROACH", "RUN", "OUT", "END"};
        if (g_circle_type >= 0 && g_circle_type <= 1) {
            status += " [" + std::string(circle_dirs[!g_circle_type]) + "-"
                   + std::string(circle_states[g_circle_state]) + "]";
        }
    }

    cv::putText(debug_view, status, cv::Point(5, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);

    std::string track_info = "Track: ";
    track_info += (g_track_side == 0) ? "RIGHT" : "LEFT";
    cv::putText(debug_view, track_info, cv::Point(5, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);

    frame = debug_view;
    perspective_frame = persp_view.clone();
    update_camera_telemetry();
    const auto debug_end = std::chrono::high_resolution_clock::now();
    g_photo_debug_us =
        std::chrono::duration_cast<std::chrono::microseconds>(debug_end - debug_start).count();
}
