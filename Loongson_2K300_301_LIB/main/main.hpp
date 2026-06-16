#ifndef __MAIN_HPP
#define __MAIN_HPP

#include <stdio.h>
#include <sys/time.h>
#include <thread>
#include <chrono>
#include <atomic>

// 包含所有底层驱动头文件
#include "lq_drv_inc.hpp"

// 包含所有工具头文件
#include "lq_common.hpp"

// 包含所有应用层头文件
#include "lq_app_inc.hpp"
#include "car_runtime.hpp"
#include "vofa.h"
// 包含所有测试程序头文件
#include "lq_all_demo.hpp"

// 全局目标速度（差速PID的基准速度，由速度决策设定）
extern float target_speed;

// 时间戳（微秒级），用于裸机多任务模型
extern timeval start_time, end_time;

// 系统运行状态标志（Ctrl+C 安全退出）
extern std::atomic<bool> ls_system_running;

// 实现根据x值进行的任务调度（微秒级）
// 用于在main循环中实现裸机多任务模型
#define PERIODIC(x)                                         \
    static uint64_t nxt = 0;                                \
    if (end_time.tv_sec * 1000000 + end_time.tv_usec < nxt) \
        return;                                             \
    nxt += (x);

#endif
