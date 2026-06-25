/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — LCD 多级菜单系统实现
 * 版权所有 (c) 2025 wuwu
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 文件名称：ww_menu.cc
 * 功能描述：LCD 多级菜单系统实现
 ********************************************************************************************************************/

#include "ww_menu.h"
#include "app_config.h"
#include "app_context.h"
#include "cross.h"
#include <cstdio>

namespace {

// 这一段是“动作项”和“状态显示项”会复用到的辅助函数区。
//
// 维护建议：
// 1. 想新增“按一下就执行”的菜单项，就在这里加一个 menu_action_xxx()。
// 2. 想新增“自定义显示格式”的状态项，就在这里加一个 menu_format_xxx()。
// 3. 这里尽量只放轻量逻辑；复杂控制逻辑仍然放回业务文件。

const char* elem_name(int elem_type)
{
    switch (elem_type) {
        case ELEM_CROSS: return "CROSS";
        case ELEM_CIRCLE: return "CIRCLE";
        case ELEM_OBSTACLE: return "OBST";
        case ELEM_RAMP: return "RAMP";
        case ELEM_ZEBRA: return "ZEBRA";
        case ELEM_CONE: return "CONE";
        case ELEM_NCNN: return "NCNN";
        default: return "NONE";
    }
}

const char* cross_name(int state)
{
    switch (state) {
        case CROSS_BEGIN: return "BEGIN";
        case CROSS_RUNNING: return "RUN";
        case CROSS_END: return "END";
        default: return "NONE";
    }
}

const char* circle_name(int state)
{
    switch (state) {
        case CIRCLE_BEGIN: return "BEGIN";
        case CIRCLE_APPROACH: return "APPR";
        case CIRCLE_RUNNING: return "RUN";
        case CIRCLE_OUT: return "OUT";
        case CIRCLE_END: return "END";
        default: return "NONE";
    }
}

void menu_action_delay_launch()
{
    // 延时发车：先彻底清空当前控制状态，再进入延时等待。
    stop_quick();
    reset_element_state();
    __1000ms = 0;
    start__1000ms_flag = 1;
    start_flag = 0;
}

void menu_action_immediate_launch()
{
    // 立即发车：不走 delay，直接进入运行态，但仍先清 PID/输出历史量。
    stop_quick();
    reset_element_state();
    __1000ms = 0;
    start__1000ms_flag = 0;
    start_flag = 1;
}

void menu_action_stop()
{
    // 停车动作统一走 stop_quick()，避免不同地方停车行为不一致。
    start_flag = 0;
    start__1000ms_flag = 0;
    __1000ms = 0;
    stop_quick();
}

void menu_action_reset_element()
{
    // 只复位元素状态机，不直接改发车状态。
    reset_element_state();
}

void menu_format_start_state(char* out, size_t out_size)
{
    const char* state = "STOP";
    if (start__1000ms_flag) state = "DELAY";
    else if (start_flag) state = "RUN";
    snprintf(out, out_size, "%s", state);
}

void menu_format_launch_progress(char* out, size_t out_size)
{
    int percent = 0;
    if (g_launch_ramp_ms > 0) {
        percent = static_cast<int>(100.0f * g_launch_ramp_elapsed_ms / g_launch_ramp_ms);
        percent = PID_CLAMP(percent, 0, 100);
    } else if (start_flag) {
        percent = 100;
    }
    snprintf(out, out_size, "%d%%", percent);
}

void menu_format_elem(char* out, size_t out_size)
{
    snprintf(out, out_size, "%s", elem_name(g_elem_type));
}

void menu_format_cross(char* out, size_t out_size)
{
    snprintf(out, out_size, "%s", cross_name(g_cross_state));
}

void menu_format_circle(char* out, size_t out_size)
{
    snprintf(out, out_size, "%s", circle_name(g_circle_state));
}

void menu_format_track_side(char* out, size_t out_size)
{
    snprintf(out, out_size, "%s", g_track_side == 0 ? "RIGHT" : "LEFT");
}

void menu_format_circle_type(char* out, size_t out_size)
{
    snprintf(out, out_size, "%s", g_circle_type == 0 ? "RIGHT" : "LEFT");
}

void menu_format_brushless(char* out, size_t out_size)
{
    snprintf(out, out_size, "%d%%", g_brushless_duty);
}

void menu_format_lookahead(char* out, size_t out_size)
{
    snprintf(out, out_size, "%d", look_ahead_point);
}

void menu_format_line_pts(char* out, size_t out_size)
{
    snprintf(out, out_size, "L%d R%d", g_ipts0_num, g_ipts1_num);
}

void menu_format_center_pts(char* out, size_t out_size)
{
    snprintf(out, out_size, "%d", g_rptsn_num);
}

void menu_format_far_pts(char* out, size_t out_size)
{
    snprintf(out, out_size, "L%d R%d", g_far_rpts0s_num, g_far_rpts1s_num);
}

}  // namespace

Menu::Menu(LCD* lcd_ptr)
    : lcd(lcd_ptr)
    , current_menu_index(0)
    , selected_index(0)
    , display_offset(0)
    , dirty(true)
    , last_refresh_time(std::chrono::steady_clock::now())
{
    build_default_menu();
}

Menu::~Menu() {}

int Menu::add_submenu(int parent, const char* name)
{
    MenuNode node;
    node.name = name;
    node.node_type = MENU_NODE_SUBMENU;
    node.parent = parent;
    nodes.push_back(node);
    const int index = static_cast<int>(nodes.size()) - 1;
    if (parent >= 0) {
        nodes[parent].children.push_back(index);
    }
    return index;
}

int Menu::add_action(int parent, const char* name, MenuActionFunc action)
{
    MenuNode node;
    node.name = name;
    node.node_type = MENU_NODE_ACTION;
    node.parent = parent;
    node.action = action;
    nodes.push_back(node);
    const int index = static_cast<int>(nodes.size()) - 1;
    nodes[parent].children.push_back(index);
    return index;
}

int Menu::add_int_param(int parent, const char* name, int* value_ptr,
                        int min_val, int max_val, int step,
                        MenuFormatterFunc formatter)
{
    MenuNode node;
    node.name = name;
    node.node_type = MENU_NODE_PARAM;
    node.value_type = MENU_VALUE_INT;
    node.value_ptr = value_ptr;
    node.min_val = static_cast<float>(min_val);
    node.max_val = static_cast<float>(max_val);
    node.step = static_cast<float>(step);
    node.parent = parent;
    node.formatter = formatter;
    nodes.push_back(node);
    const int index = static_cast<int>(nodes.size()) - 1;
    nodes[parent].children.push_back(index);
    return index;
}

int Menu::add_float_param(int parent, const char* name, float* value_ptr,
                          float min_val, float max_val, float step,
                          unsigned char decimals,
                          MenuFormatterFunc formatter)
{
    MenuNode node;
    node.name = name;
    node.node_type = MENU_NODE_PARAM;
    node.value_type = MENU_VALUE_FLOAT;
    node.value_ptr = value_ptr;
    node.min_val = min_val;
    node.max_val = max_val;
    node.step = step;
    node.decimals = decimals;
    node.parent = parent;
    node.formatter = formatter;
    nodes.push_back(node);
    const int index = static_cast<int>(nodes.size()) - 1;
    nodes[parent].children.push_back(index);
    return index;
}

int Menu::add_readonly_int(int parent, const char* name, int* value_ptr,
                           MenuFormatterFunc formatter)
{
    MenuNode node;
    node.name = name;
    node.node_type = MENU_NODE_READONLY;
    node.value_type = MENU_VALUE_INT;
    node.value_ptr = value_ptr;
    node.parent = parent;
    node.formatter = formatter;
    nodes.push_back(node);
    const int index = static_cast<int>(nodes.size()) - 1;
    nodes[parent].children.push_back(index);
    return index;
}

int Menu::add_readonly_float(int parent, const char* name, float* value_ptr,
                             unsigned char decimals,
                             MenuFormatterFunc formatter)
{
    MenuNode node;
    node.name = name;
    node.node_type = MENU_NODE_READONLY;
    node.value_type = MENU_VALUE_FLOAT;
    node.value_ptr = value_ptr;
    node.parent = parent;
    node.decimals = decimals;
    node.formatter = formatter;
    nodes.push_back(node);
    const int index = static_cast<int>(nodes.size()) - 1;
    nodes[parent].children.push_back(index);
    return index;
}

int Menu::add_readonly_custom(int parent, const char* name, MenuFormatterFunc formatter)
{
    MenuNode node;
    node.name = name;
    node.node_type = MENU_NODE_READONLY;
    node.parent = parent;
    node.formatter = formatter;
    nodes.push_back(node);
    const int index = static_cast<int>(nodes.size()) - 1;
    nodes[parent].children.push_back(index);
    return index;
}

void Menu::build_default_menu()
{
    // 这里是整棵菜单树的“总装区”。
    //
    // 以后你自己维护菜单，主要就改这个函数。
    // 维护顺序建议固定为：
    // 1. 先 add_submenu() 建目录
    // 2. 再往目录里 add_action() / add_int_param() / add_float_param()
    // 3. 只读显示项用 add_readonly_xxx()
    //
    // 常用接口说明：
    // - add_submenu(parent, "Name")
    //     新建一个子菜单，返回该菜单节点索引，后续继续往里面挂子项。
    // - add_action(parent, "Name", func)
    //     新建一个动作项，按 KEY4 时执行 func。
    // - add_int_param(...) / add_float_param(...)
    //     新建一个可调参数项。菜单改值后，会直接改到你传入的变量。
    // - add_readonly_int/float/custom(...)
    //     新建一个只显示、不允许修改的状态项。
    //
    // 判断“参数改了会不会生效”的关键不在菜单，而在这个变量有没有被业务代码使用：
    // - 如果变量在 motor_task / photo_task / 其它任务里参与计算，菜单改完就会立刻生效。
    // - 如果变量只是挂进菜单但没人使用，那它只会显示变化，不会影响小车行为。

    nodes.clear();
    const int root = add_submenu(-1, "ROOT");
    current_menu_index = root;

    // -------------------------------
    // 1. 发车 / 停车 / 起步相关
    // -------------------------------
    // 放“和发车过程强相关”的参数：
    // - 延时发车
    // - 立即发车
    // - 停车
    // - 起步斜坡
    // - 起步转向抑制
    const int launch_menu = add_submenu(root, "Launch");
    add_action(launch_menu, "DelayGo", menu_action_delay_launch);
    add_action(launch_menu, "NowGo", menu_action_immediate_launch);
    add_action(launch_menu, "StopCar", menu_action_stop);
    add_action(launch_menu, "ResetElt", menu_action_reset_element);
    add_int_param(launch_menu, "DelayMs", &g_launch_delay_ms, 0, 5000, 50);
    add_int_param(launch_menu, "RampMs", &g_launch_ramp_ms, 0, 5000, 50);
    add_float_param(launch_menu, "StartRat", &g_launch_start_ratio, 0.0f, 1.0f, 0.01f, 2);
    add_float_param(launch_menu, "TurnRat", &g_launch_turn_ratio, 0.0f, 1.0f, 0.05f, 2);
    add_float_param(launch_menu, "RelThr", &g_launch_encoder_release_threshold, 0.0f, 3000.0f, 50.0f, 0);
    add_float_param(launch_menu, "DltLim", &g_launch_delta_limit, 0.0f, 1000.0f, 10.0f, 0);

    // -------------------------------
    // 2. 速度相关
    // -------------------------------
    // 放“整车整体跑多快”的参数：
    // - 左右基础速度
    // - 直道加速量
    // - 前瞻距离和速度映射
    // - 总速度上限 / 差速上限
    const int speed_menu = add_submenu(root, "Speed");
    add_int_param(speed_menu, "BaseL", &base_speed_target_l, 0, 5000, 50);
    add_int_param(speed_menu, "BaseR", &base_speed_target_r, 0, 5000, 50);
    add_int_param(speed_menu, "AddSpd", &g_straight_add_speed, 0, 3000, 50);
    add_float_param(speed_menu, "SpdGain", &g_line_lookahead_speed_gain, 0.000f, 0.100f, 0.001f, 3);
    add_float_param(speed_menu, "DistMin", &g_line_lookahead_dist_min, 3.0f, 60.0f, 1.0f, 1);
    add_float_param(speed_menu, "DistMax", &g_line_lookahead_dist_max, 5.0f, 80.0f, 1.0f, 1);
    add_float_param(speed_menu, "TgtLim", &g_target_speed_limit, 0.0f, 8000.0f, 100.0f, 0);
    add_float_param(speed_menu, "DiffLim", &g_diff_output_limit, 0.0f, 10000.0f, 100.0f, 0);

    // -------------------------------
    // 3. PID 菜单
    // -------------------------------
    // 这里再按控制环细分子菜单，后续调车时更清楚：
    // - SpeedL / SpeedR：左右轮速度环
    // - Gyro：角速度环
    // - Angle：普通循迹角度环
    // - Circle：环岛工况角度环
    const int pid_menu = add_submenu(root, "PID");
    const int speed_l_menu = add_submenu(pid_menu, "SpeedL");
    add_float_param(speed_l_menu, "Kp", &pid_speed_1.kp, 0.0f, 20.0f, 0.1f, 2);
    add_float_param(speed_l_menu, "Ki", &pid_speed_1.ki, 0.0f, 5.0f, 0.01f, 2);
    add_float_param(speed_l_menu, "Kd", &pid_speed_1.kd, 0.0f, 5.0f, 0.01f, 2);
    add_float_param(speed_l_menu, "OutMax", &pid_speed_1.out_max, 0.0f, 10000.0f, 100.0f, 0);

    const int speed_r_menu = add_submenu(pid_menu, "SpeedR");
    add_float_param(speed_r_menu, "Kp", &pid_speed_r.kp, 0.0f, 20.0f, 0.1f, 2);
    add_float_param(speed_r_menu, "Ki", &pid_speed_r.ki, 0.0f, 5.0f, 0.01f, 2);
    add_float_param(speed_r_menu, "Kd", &pid_speed_r.kd, 0.0f, 5.0f, 0.01f, 2);
    add_float_param(speed_r_menu, "OutMax", &pid_speed_r.out_max, 0.0f, 10000.0f, 100.0f, 0);

    const int gyro_menu = add_submenu(pid_menu, "Gyro");
    add_float_param(gyro_menu, "Kp", &pid_angle_v.kp, 0.0f, 20.0f, 0.1f, 2);
    add_float_param(gyro_menu, "Ki", &pid_angle_v.ki, 0.0f, 5.0f, 0.01f, 2);
    add_float_param(gyro_menu, "Kd", &pid_angle_v.kd, 0.0f, 5.0f, 0.001f, 3);
    add_float_param(gyro_menu, "Pmax", &pid_angle_v.p_max, 0.0f, 10000.0f, 100.0f, 0);
    add_float_param(gyro_menu, "Dmax", &pid_angle_v.d_max, 0.0f, 5000.0f, 10.0f, 0);

    const int angle_menu = add_submenu(pid_menu, "Angle");
    add_float_param(angle_menu, "Kp", &pid_angle.kp, 0.0f, 80.0f, 0.1f, 2);
    add_float_param(angle_menu, "Ki", &pid_angle.ki, 0.0f, 5.0f, 0.01f, 2);
    add_float_param(angle_menu, "Kd", &pid_angle.kd, 0.0f, 5.0f, 0.01f, 2);
    add_float_param(angle_menu, "Pmax", &pid_angle.p_max, 0.0f, 10000.0f, 100.0f, 0);
    add_float_param(angle_menu, "Dmax", &pid_angle.d_max, 0.0f, 5000.0f, 10.0f, 0);
    add_float_param(angle_menu, "Kp2", &pid_angle.kp_2, 0.0f, 5.0f, 0.01f, 2);

    const int circle_menu = add_submenu(pid_menu, "Circle");
    add_float_param(circle_menu, "Kp", &pid_angle_circle.kp, 0.0f, 50.0f, 0.1f, 2);
    add_float_param(circle_menu, "Ki", &pid_angle_circle.ki, 0.0f, 5.0f, 0.01f, 2);
    add_float_param(circle_menu, "Kd", &pid_angle_circle.kd, 0.0f, 5.0f, 0.01f, 2);
    add_float_param(circle_menu, "Pmax", &pid_angle_circle.p_max, 0.0f, 10000.0f, 100.0f, 0);
    add_float_param(circle_menu, "Dmax", &pid_angle_circle.d_max, 0.0f, 5000.0f, 10.0f, 0);

    // -------------------------------
    // 4. 无刷
    // -------------------------------
    // 当前工程里 motor_task 会周期调用 brush.set_duty(g_brushless_duty)，
    // 所以这里改 Duty 后会直接生效。
    const int brush_menu = add_submenu(root, "Brushless");
    add_int_param(brush_menu, "Duty", &g_brushless_duty, 0, 100, 5, menu_format_brushless);

    // -------------------------------
    // 5. 弯道前馈
    // -------------------------------
    // 这一组都是“前瞻前馈链”参数。
    // 如果后面你新增别的前馈项，建议也集中放这里，不要和 PID 混在一起。
    const int ff_menu = add_submenu(root, "CurveFF");
    add_int_param(ff_menu, "Enable", &g_curve_ff_enable, 0, 1, 1);
    add_int_param(ff_menu, "FarOff", &g_curve_ff_far_offset, 1, 20, 1);
    add_float_param(ff_menu, "Kdelta", &g_curve_ff_k_delta, 0.0f, 3.0f, 0.01f, 2);
    add_float_param(ff_menu, "Limit", &g_curve_ff_limit, 0.0f, 100.0f, 1.0f, 1);
    add_float_param(ff_menu, "LowPass", &g_curve_ff_low_pass, 0.0f, 1.0f, 0.05f, 2);
    add_float_param(ff_menu, "ActAng", &g_curve_ff_active_angle_min, 0.0f, 45.0f, 0.5f, 1);

    // -------------------------------
    // 6. 实时状态页
    // -------------------------------
    // 这里全是只读项，方便你现场看车状态。
    //
    // 三种加法：
    // - add_readonly_int / float：直接显示变量值
    // - add_readonly_custom：显示组合文本，适合状态枚举、双值拼接等
    //
    // 如果以后你想看“某个调试量是否真的在变”，最方便的做法就是在这里再挂一项。
    const int status_menu = add_submenu(root, "Status");
    add_readonly_custom(status_menu, "Start", menu_format_start_state);
    add_readonly_custom(status_menu, "RampPct", menu_format_launch_progress);
    add_readonly_custom(status_menu, "Elem", menu_format_elem);
    add_readonly_custom(status_menu, "Cross", menu_format_cross);
    add_readonly_custom(status_menu, "Circle", menu_format_circle);
    add_readonly_custom(status_menu, "Track", menu_format_track_side);
    add_readonly_custom(status_menu, "CirType", menu_format_circle_type);
    add_readonly_custom(status_menu, "LookPt", menu_format_lookahead);
    add_readonly_custom(status_menu, "LinePts", menu_format_line_pts);
    add_readonly_custom(status_menu, "CtrPts", menu_format_center_pts);
    add_readonly_custom(status_menu, "FarPts", menu_format_far_pts);
    add_readonly_float(status_menu, "PureAng", &g_pure_angle, 1);
    add_readonly_float(status_menu, "GyroT", &g_target_gyro_dbg, 1);
    add_readonly_float(status_menu, "GyroA", &g_actual_gyro_dbg, 1);
    add_readonly_float(status_menu, "SpdL", &g_speed_l_dbg, 0);
    add_readonly_float(status_menu, "SpdR", &g_speed_r_dbg, 0);
    add_readonly_float(status_menu, "TgtL", &g_target_speed_l_dbg, 0);
    add_readonly_float(status_menu, "TgtR", &g_target_speed_r_dbg, 0);
    add_readonly_float(status_menu, "PwmL", &g_pwm_l_dbg, 0);
    add_readonly_float(status_menu, "PwmR", &g_pwm_r_dbg, 0);
}

void Menu::mark_dirty_locked()
{
    dirty = true;
}

const MenuNode* Menu::current_selected_node_locked() const
{
    if (current_menu_index < 0 || current_menu_index >= static_cast<int>(nodes.size())) return nullptr;
    const MenuNode& menu = nodes[current_menu_index];
    if (menu.children.empty()) return nullptr;
    if (selected_index < 0 || selected_index >= static_cast<int>(menu.children.size())) return nullptr;
    return &nodes[menu.children[selected_index]];
}

void Menu::format_node_value_locked(const MenuNode& node, char* out, size_t out_size) const
{
    if (out_size == 0) return;
    out[0] = '\0';

    if (node.formatter) {
        node.formatter(out, out_size);
        return;
    }

    if (node.node_type == MENU_NODE_SUBMENU) {
        snprintf(out, out_size, ">");
        return;
    }
    if (node.node_type == MENU_NODE_ACTION) {
        snprintf(out, out_size, "GO");
        return;
    }
    if (node.value_type == MENU_VALUE_INT && node.value_ptr) {
        const int value = *static_cast<int*>(node.value_ptr);
        snprintf(out, out_size, "%d", value);
        return;
    }
    if (node.value_type == MENU_VALUE_FLOAT && node.value_ptr) {
        const double value = *static_cast<float*>(node.value_ptr);
        snprintf(out, out_size, "%.*f", node.decimals, value);
    }
}

void Menu::render_menu_locked()
{
    // 菜单模式：
    // 显示当前目录下的子项，支持上下滚动。
    lcd->clearScreen();

    const MenuNode& menu = nodes[current_menu_index];
    lcd->showString(0, TITLE_Y, menu.name ? menu.name : "MENU");
    lcd->showString(0, HINT_Y, "K3Back K4OK");

    const int total = static_cast<int>(menu.children.size());
    const int show_count = PID_MIN(total - display_offset, MAX_DISPLAY_ITEMS);
    for (int i = 0; i < show_count; ++i) {
        const int child_index = menu.children[display_offset + i];
        const MenuNode& node = nodes[child_index];
        const int y = START_Y + i * LINE_HEIGHT;
        char line[32] = {0};
        char value[20] = {0};
        format_node_value_locked(node, value, sizeof(value));
        snprintf(line, sizeof(line), "%c%-9s %-8s",
                 (display_offset + i == selected_index) ? '>' : ' ',
                 node.name ? node.name : "",
                 value);
        lcd->showString(0, y, line);
    }

    if (display_offset > 0) lcd->showString(118, TITLE_Y, "^");
    if (display_offset + MAX_DISPLAY_ITEMS < total) lcd->showString(118, 118, "v");
}

void Menu::render_runtime_locked()
{
    // 运行态概览页：
    // 给你快速看发车状态、元素状态、目标/实际速度、PWM 等核心信息。
    lcd->clearScreen();

    lcd->showString(0, 0, "Run State");

    char line[32] = {0};
    snprintf(line, sizeof(line), "S:%s E:%s",
             start__1000ms_flag ? "DELAY" : (start_flag ? "RUN" : "STOP"),
             elem_name(g_elem_type));
    lcd->showString(0, 16, line);

    snprintf(line, sizeof(line), "LA:%02d PA:%5.1f", look_ahead_point, g_pure_angle);
    lcd->showString(0, 32, line);

    snprintf(line, sizeof(line), "VL:%4.0f VR:%4.0f", g_speed_l_dbg, g_speed_r_dbg);
    lcd->showString(0, 48, line);

    snprintf(line, sizeof(line), "TL:%4.0f TR:%4.0f", g_target_speed_l_dbg, g_target_speed_r_dbg);
    lcd->showString(0, 64, line);

    snprintf(line, sizeof(line), "PL:%4.0f PR:%4.0f", g_pwm_l_dbg, g_pwm_r_dbg);
    lcd->showString(0, 80, line);

    snprintf(line, sizeof(line), "B:%02d C:%s", g_brushless_duty, circle_name(g_circle_state));
    lcd->showString(0, 96, line);

    lcd->showString(0, 112, "KEY5 Menu");
}

void Menu::render_locked()
{
    if (!lcd) return;
    if (g_menu_mode) render_menu_locked();
    else render_runtime_locked();
    last_refresh_time = std::chrono::steady_clock::now();
    dirty = false;
}

void Menu::adjust_value_locked(MenuNode& node, int direction)
{
    // direction:
    // -1 = 减小
    // +1 = 增大
    if (node.node_type != MENU_NODE_PARAM || !node.value_ptr) return;

    if (node.value_type == MENU_VALUE_INT) {
        int* value = static_cast<int*>(node.value_ptr);
        *value += static_cast<int>(node.step) * direction;
        *value = PID_CLAMP(*value, static_cast<int>(node.min_val), static_cast<int>(node.max_val));
    } else if (node.value_type == MENU_VALUE_FLOAT) {
        float* value = static_cast<float*>(node.value_ptr);
        *value += node.step * static_cast<float>(direction);
        *value = PID_CLAMP(*value, node.min_val, node.max_val);
    }

    mark_dirty_locked();
}

void Menu::enter_child_locked()
{
    // KEY4 的语义统一放在这里：
    // - 子菜单：进入
    // - 动作项：执行
    // - 参数项：增大
    const MenuNode* node = current_selected_node_locked();
    if (!node) return;

    if (node->node_type == MENU_NODE_SUBMENU) {
        current_menu_index = static_cast<int>(node - &nodes[0]);
        selected_index = 0;
        display_offset = 0;
        mark_dirty_locked();
        return;
    }

    if (node->node_type == MENU_NODE_ACTION && node->action) {
        node->action();
        mark_dirty_locked();
        return;
    }

    if (node->node_type == MENU_NODE_PARAM) {
        MenuNode& editable = nodes[static_cast<int>(node - &nodes[0])];
        adjust_value_locked(editable, +1);
    }
}

void Menu::back_or_decrease_locked()
{
    // KEY3 的语义统一放在这里：
    // - 参数项：减小
    // - 非参数项：返回上一级
    const MenuNode* node = current_selected_node_locked();
    if (node && node->node_type == MENU_NODE_PARAM) {
        MenuNode& editable = nodes[static_cast<int>(node - &nodes[0])];
        adjust_value_locked(editable, -1);
        return;
    }

    if (current_menu_index == 0) return;
    current_menu_index = nodes[current_menu_index].parent;
    selected_index = 0;
    display_offset = 0;
    mark_dirty_locked();
}

void Menu::increase_or_enter_locked()
{
    enter_child_locked();
}

void Menu::key_up()
{
    std::lock_guard<std::mutex> lock(menu_mutex);
    const int total = static_cast<int>(nodes[current_menu_index].children.size());
    if (total <= 0) return;
    if (selected_index > 0) {
        --selected_index;
        if (selected_index < display_offset) display_offset = selected_index;
        mark_dirty_locked();
        render_locked();
    }
}

void Menu::key_down()
{
    std::lock_guard<std::mutex> lock(menu_mutex);
    const int total = static_cast<int>(nodes[current_menu_index].children.size());
    if (total <= 0) return;
    if (selected_index < total - 1) {
        ++selected_index;
        if (selected_index >= display_offset + MAX_DISPLAY_ITEMS) {
            display_offset = selected_index - MAX_DISPLAY_ITEMS + 1;
        }
        mark_dirty_locked();
        render_locked();
    }
}

void Menu::key_decrease()
{
    std::lock_guard<std::mutex> lock(menu_mutex);
    back_or_decrease_locked();
    render_locked();
}

void Menu::key_increase()
{
    std::lock_guard<std::mutex> lock(menu_mutex);
    increase_or_enter_locked();
    render_locked();
}

void Menu::refresh()
{
    std::lock_guard<std::mutex> lock(menu_mutex);
    mark_dirty_locked();
    render_locked();
}

void Menu::tick()
{
    // 菜单刷新线程会周期调用这里。
    // 作用：
    // 1. 菜单值变化后自动刷新
    // 2. 运行状态页定期刷新
    //
    // 如果你后面觉得状态页刷新太快/太慢，可以改这里的时间阈值。
    std::lock_guard<std::mutex> lock(menu_mutex);
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_refresh_time).count();
    if (!dirty && elapsed_ms < 150) return;
    render_locked();
}
