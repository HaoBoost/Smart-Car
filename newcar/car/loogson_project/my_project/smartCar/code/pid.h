/**
 * pid.h
 * ============================================================
 * PID 控制器库（位置式 + 增量式）
 *
 * 包含两种 PID 实现：
 *   1. 位置式 PID (pid_positional_*)
 *      - 输出绝对值，直接控制执行器
 *      - 适用：角度环、角速度环、弯道减速
 *
 *   2. 增量式 PID (pid_incremental_*)
 *      - 输出增量值，需要累加到执行器
 *      - 适用：速度环（抗干扰、无积分饱和）
 *
 * 特点：
 *   - D 项带低通滤波，抑制高频噪声（陀螺仪/编码器噪声）
 *   - 积分限幅（位置式），防止积分饱和
 *   - 各项独立限幅（P/I/D 分别限幅）
 *
 * 使用方法：
 *   // 位置式 PID
 *   pid_positional_t angle_pid;
 *   pid_positional_init(&angle_pid, kp, ki, kd, low_pass, p_max, i_max, d_max);
 *   float out = pid_positional_solve(&angle_pid, error);
 *
 *   // 增量式 PID
 *   pid_incremental_t speed_pid;
 *   pid_incremental_init(&speed_pid, kp, ki, kd, out_max);
 *   float delta = pid_incremental_solve(&speed_pid, error);
 *   pwm += delta;  // 累加到执行器
 *
 * 兼容性：
 *   - 保留 pid_t, pid_init(), pid_solve(), pid_reset() 别名
 *   - 旧代码无需修改，可直接使用
 * ============================================================
 */
#pragma once

#include <cmath>
#include <float.h>

// ---- 工具宏 ----
#define PID_MIN(a,b)        ((a)<(b)?(a):(b))
#define PID_MAX(a,b)        ((a)>(b)?(a):(b))
#define PID_CLAMP(x,lo,hi)  PID_MIN(PID_MAX((x),(lo)),(hi))

/**
 * 位置式 PID 控制器结构体
 *
 * 参数说明：
 *   kp        比例系数，越大响应越快，过大会震荡
 *   ki        积分系数，消除稳态误差，过大会积分饱和
 *   kd        微分系数，抑制超调，过大会放大噪声
 *   low_pass  D 项低通滤波系数（0~1）
 *             越小滤波越强（噪声小但响应慢）
 *             越大滤波越弱（响应快但噪声大）
 *             建议值：0.6~0.9
 *   p_max     P 项输出限幅
 *   i_max     I 项输出限幅（防积分饱和）
 *   d_max     D 项输出限幅
 */
typedef struct {
    float kp,ki, kd;
    float p_max, i_max, d_max;
    float low_pass;     // D 项低通滤波系数
    float kp_2;
    // 内部状态（不要手动修改）
    float out_p;        // 上一次误差（用于计算 D 项）
    float out_i;        // 积分累加值
    float out_d;        // 滤波后的 D 项
} pid_positional_t;

// // 兼容旧代码的别名
// typedef pid_positional_t pid_t;

/**
 * pid_positional_init() — 初始化位置式 PID 参数
 *
 * 示例（角度环）：
 *   pid_positional_init(&angle_pid, 1.5f, 0.0f, 1.0f, 0.8f, 30.0f, 0.0f, 30.0f);
 *
 * 示例（速度环）：
 *   pid_positional_init(&speed_pid, 50.0f, 0.5f, 0.0f, 1.0f, 8000.0f, 8000.0f, 8000.0f);
 */
static inline void pid_positional_init(pid_positional_t *p,
    float kp, float ki, float kd, float low_pass,
    float p_max, float i_max, float d_max)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->low_pass = low_pass;
    p->p_max = p_max; p->i_max = i_max; p->d_max = d_max;
    p->out_p = p->out_i = p->out_d = 0;
}

// // 兼容旧代码的别名
// static inline void pid_init(pid_t *p,
//     float kp, float ki, float kd, float low_pass,
//     float p_max, float i_max, float d_max)
// {
//     pid_positional_init(p, kp, ki, kd, low_pass, p_max, i_max, d_max);
// }

/**
 * pid_positional_solve() — 位置式 PID 计算（每个控制周期调用一次）
 *
 * @param error  当前误差 = 目标值 - 实际值
 * @return       PID 输出（P/I/D 分项独立限幅后）
 *
 * 计算过程：
 *   D 项 = (error - prev_error) * low_pass + prev_D * (1 - low_pass)
 *   I 项 += error（积分限幅）
 *   输出 = kp * error + ki * I + kd * D
 */
static inline float pid_positional_solve(pid_positional_t *p, float error) {
    // D 项：(当前误差 - 上次误差) 做低通滤波
    p->out_d = (error - p->out_p);
    p->out_p = error;   // 保存本次误差供下次计算 D 项

    // I 项：累加误差，并限幅防止积分饱和
    p->out_i += error;
    if (p->ki != 0 && p->i_max > 0) {
        float ki_abs = p->ki > 0 ? p->ki : -p->ki;
        p->out_i = PID_CLAMP(p->out_i, -p->i_max / ki_abs, p->i_max / ki_abs);
    }

    float out_p = p->kp * p->out_p;
    float out_i = p->ki * p->out_i;
    float out_d = p->kd * p->out_d;

    if (p->p_max > 0)
        out_p = PID_CLAMP(out_p, -p->p_max, p->p_max);
    if (p->i_max > 0)
        out_i = PID_CLAMP(out_i, -p->i_max, p->i_max);
    if (p->d_max > 0)
        out_d = PID_CLAMP(out_d, -p->d_max, p->d_max);

    return out_p + out_i + out_d;
}



/**
 * pid_positional_solve() — 位置式 PID 计算（每个控制周期调用一次）
 *
 * @param error  当前误差 = 目标值 - 实际值
 * @return       PID 输出（P/I/D 分项独立限幅后）
 *
 * 计算过程：
 *   D 项 = (error - prev_error) * low_pass + prev_D * (1 - low_pass)
 *   I 项 += error（积分限幅）
 *   输出 = kp * error + ki * I + kd * D
 */
static inline float pid_positional_solve_Error2(pid_positional_t *p, float error) {
    // D 项：(当前误差 - 上次误差) 做低通滤波
    p->out_d = (error - p->out_p);
    p->out_p = error;   // 保存本次误差供下次计算 D 项

    // I 项：累加误差，并限幅防止积分饱和
    p->out_i += error;
    if (p->ki != 0 && p->i_max > 0) {
        float ki_abs = p->ki > 0 ? p->ki : -p->ki;
        p->out_i = PID_CLAMP(p->out_i, -p->i_max / ki_abs, p->i_max / ki_abs);
    }

    float out_p = p->kp * p->out_p;
    float out_i = p->ki * p->out_i;
    float out_d = p->kd * p->out_d;
    float out_2 = p->kp_2 * std::fabs(p->out_p) * p->out_p;
    if (p->p_max > 0)
        out_p = PID_CLAMP(out_p, -p->p_max, p->p_max);
    if (p->i_max > 0)
        out_i = PID_CLAMP(out_i, -p->i_max, p->i_max);
    if (p->d_max > 0)
        out_d = PID_CLAMP(out_d, -p->d_max, p->d_max);

    return out_p + out_i + out_d + out_2;
}
// // 兼容旧代码的别名
// static inline float pid_solve(pid_t *p, float error) {
//     return pid_positional_solve(p, error);
// }

/**
 * pid_positional_reset() — 清除积分和 D 项
 *
 * 在以下情况调用：
 *   - 模式切换（如从巡线切换到停车）
 *   - 长时间停止后重新启动
 *   - 防止积分饱和冲击
 */
static inline void pid_positional_reset(pid_positional_t *p) {
    p->out_i = 0;
    p->out_d = 0;
}

// // 兼容旧代码的别名
// static inline void pid_reset(pid_t *p) {
//     pid_positional_reset(p);
// }

// ============================================================
//  增量式 PID（用于速度环）
// ============================================================

/**
 * 增量式 PID 控制器结构体
 *
 * 与位置式的区别：
 *   - 输出是增量值（Δu），不是绝对值
 *   - 需要累加到执行器：pwm += pid_solve_incremental()
 *   - 不需要积分限幅（无累加项）
 *   - 误动作影响小（只影响一次）
 */
typedef struct {
    float kp, ki, kd;
    float out_max;          // 单次增量输出限幅

    // 历史误差（用于计算增量）
    float err_last;         // e[k-1]：上一次误差
    float err_last2;        // e[k-2]：上上次误差
} pid_incremental_t;

/**
 * pid_incremental_init() — 初始化增量式 PID
 *
 * @param kp        比例系数
 * @param ki        积分系数
 * @param kd        微分系数
 * @param out_max   单次增量输出限幅
 */
static inline void pid_incremental_init(pid_incremental_t *p,
    float kp, float ki, float kd, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_max = out_max;
    p->err_last = 0;
    p->err_last2 = 0;
}

/**
 * pid_incremental_solve() — 增量式 PID 计算
 *
 * @param error  当前误差 = 目标值 - 实际值
 * @return       增量输出 Δu（需要累加到执行器）
 *
 * 增量式 PID 公式：
 *   Δu = Kp × (e[k] - e[k-1])
 *      + Ki × e[k]
 *      + Kd × (e[k] - 2×e[k-1] + e[k-2])
 *
 * 使用方法：
 *   float delta = pid_incremental_solve(&pid, error);
 *   pwm += delta;  // 累加到当前输出
 */
static inline float pid_incremental_solve(pid_incremental_t *p, float error) {
    // 增量式 PID 公式
    float delta = p->kp * (error - p->err_last)                    // P 项增量
                + p->ki * error                                     // I 项
                + p->kd * (error - 2.0f * p->err_last + p->err_last2);  // D 项增量

    // 限幅（防止单次变化过大）
    delta = PID_CLAMP(delta, -p->out_max, p->out_max);

    // 更新历史误差
    p->err_last2 = p->err_last;
    p->err_last = error;

    return delta;
}

/**
 * pid_incremental_reset() — 清除历史误差
 */
static inline void pid_incremental_reset(pid_incremental_t *p) {
    p->err_last = 0;
    p->err_last2 = 0;
}
