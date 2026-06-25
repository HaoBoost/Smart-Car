/**
 * buzzer_sound.h
 * 非阻塞蜂鸣器声音管理器
 *
 * 4 种声音：
 *   SOUND_BEEP_SHORT   — 短促单响（滴）
 *   SOUND_BEEP_DOUBLE  — 双响（滴滴）
 *   SOUND_BEEP_LONG    — 长响（嘟——）
 *   SOUND_BEEP_RISING  — 上升音（低→高）
 *
 * 使用方法：
 *   1. 初始化蜂鸣器：buzzer_1.buzzer_init()
 *   2. 触发声音：buzzer_trigger(SOUND_BEEP_SHORT)
 *   3. 在 1ms 定时任务中调用：buzzer_update()
 */
#pragma once
#include "ww_buzzer.h"

// 声音类型
typedef enum {
    SOUND_NONE = 0,
    SOUND_BEEP_SHORT,    // 短促单响：1000Hz, 100ms
    SOUND_BEEP_DOUBLE,   // 双响：1500Hz, 80ms×2 + 间隔60ms
    SOUND_BEEP_LONG,     // 长响：800Hz, 400ms
    SOUND_BEEP_RISING,   // 上升音：500→1000→2000Hz, 各100ms
} buzzer_sound_t;

// 声音序列中的一个步骤
struct BuzzerStep {
    int freq;       // 频率（0=静音）
    int duration;   // 持续时间（ms）
};

// ---- 4 种声音的序列定义 ----
static const BuzzerStep seq_short[] = {
    {1000, 100},
    {0, 0},         // 结束标记
};

static const BuzzerStep seq_double[] = {
    {1500, 80},
    {0,    60},     // 间隔静音
    {1500, 80},
    {0, 0},
};

static const BuzzerStep seq_long[] = {
    {800, 400},
    {0, 0},
};

static const BuzzerStep seq_rising[] = {
    {500,  100},
    {1000, 100},
    {2000, 100},
    {0, 0},
};

// ---- 内部状态 ----
static const BuzzerStep* s_seq = nullptr;  // 当前播放的序列
static int s_step_idx  = 0;                // 当前步骤索引
static int s_tick_cnt  = 0;                // 当前步骤已计时（ms）
static bool s_playing  = false;

/**
 * buzzer_trigger() — 触发一种声音
 * 如果当前有声音在播放，会被新声音打断。
 */
static inline void buzzer_trigger(buzzer_sound_t sound) {
    switch (sound) {
    case SOUND_BEEP_SHORT:  s_seq = seq_short;  break;
    case SOUND_BEEP_DOUBLE: s_seq = seq_double; break;
    case SOUND_BEEP_LONG:   s_seq = seq_long;   break;
    case SOUND_BEEP_RISING: s_seq = seq_rising;  break;
    default: s_seq = nullptr; break;
    }
    s_step_idx = 0;
    s_tick_cnt = 0;
    s_playing  = (s_seq != nullptr);
}

/**
 * buzzer_update() — 每 1ms 调用一次（在定时任务中）
 * @param buzzer  蜂鸣器对象引用
 */
static inline void buzzer_update(Buzzer &buzzer) {
    if (!s_playing || s_seq == nullptr) return;

    const BuzzerStep &step = s_seq[s_step_idx];

    // 结束标记：freq=0 且 duration=0
    if (step.freq == 0 && step.duration == 0) {
        buzzer.set_duty(0);  // 关闭蜂鸣器
        s_playing = false;
        return;
    }

    // 步骤开始时设置频率
    if (s_tick_cnt == 0) {
        if (step.freq > 0) {
            buzzer.set_duty_freq(50, step.freq);
        } else {
            buzzer.set_duty(0);  // 静音间隔
        }
    }

    s_tick_cnt++;

    // 当前步骤结束，切换到下一步
    if (s_tick_cnt >= step.duration) {
        s_tick_cnt = 0;
        s_step_idx++;
    }
}
