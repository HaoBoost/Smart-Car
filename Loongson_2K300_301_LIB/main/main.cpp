/********************************************************************************
 * @file            main.cpp
 * @brief           智能车主程序 —— 三环级联PID（OOP架构）
 * @copyright       版权所有 (C) 2025-2026 北京龙邱科技有限公司
 * @website         http://www.lqist.cn
 * @taobao          https://longqiu.taobao.com
 *
 * @description     龙邱科技 LS2K300/301 核心板驱动开源库
 *
 * 本文件遵循 GPL-3.0 开源协议发布.
 * 商业用途(包括单位使用)需提前联系作者获取授权
 *
 * @author          龙邱科技-012
 * @email           chiusir@163.com
 * @version         V2.1.0
 * @update          2026-06-13
 *
 * 三环级联PID架构:
 *   图像环(5ms) → 目标角速度 → 角速度环(2ms) → 差速修正 → 速度环(1ms) → PWM
 ********************************************************************************/

#include "main.hpp"
#include "image.hpp"

// ========================== 全局变量定义 ==========================

timeval start_time, end_time; // 时间戳（PERIODIC宏使用）
float target_speed = 0.0f;    // 全局目标速度

// ========================== 主函数 ==========================

int main()
{
    printf("========================================\n");
    printf("  智能车三环级联PID控制程序 (OOP架构)\n");
    printf("  速度环(1ms) → 角速度环(2ms) → 图像环(5ms)\n");
    printf("========================================\n");

    // ---- 创建小车运行时对象 ----
    CarRuntime car;

    // 步骤1: 集中初始化所有PID参数
    car.init_pid();

    // 步骤2: 初始化硬件（Motor、IMU、摄像头等）
    car.init_hardware(true, 1000);

    // ---- 打开摄像头 ----
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cerr << "无法打开摄像头" << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
    cap.set(cv::CAP_PROP_FPS, 120);

    sleep(2); // 等待摄像头稳定

    // ---- 定时器：调度三环 ----
    lq_timer timer_1ms, timer_2ms, timer_5ms, timer_10ms;

    // 1ms: 速度环（Motor PID + PWM输出）
    timer_1ms.set_seconds_ms(1, [&car]()
                             { car.on_timer_1ms(); });

    // 2ms: 角速度环（IMU读取 + 角速度PID）
    timer_2ms.set_seconds_ms(2, [&car]()
                             { car.on_timer_2ms(); });

    // 5ms: 图像环（图像处理 + 转向PID）
    timer_5ms.set_seconds_ms(5, [&car]()
                             { car.on_timer_5ms(); });

    // 10ms: VOFA+ JustFloat 发送目标速度 & 实时编码器速度
    timer_10ms.set_seconds_ms(10, [&car]()
                              {
        // float data[3];
        // data[0] = car.get_target_speed();   // ch0: 目标速度
        // data[1] = car.motor_->L_speed;     // ch1: 左轮实时速度（编码器）
        // data[2] = car.motor_->R_speed;    // ch2: 右轮实时速度（编码器）
        /* vofa_send_justfloat(data, 3);*/ });

    // ---- 主循环：持续捕获摄像头帧 ----
    printf("智能车启动，按 Ctrl+C 安全退出...\n");

    while (ls_system_running.load())
    {
        gettimeofday(&start_time, nullptr);

        cap >> First_image; // 捕获一帧
        if (First_image.empty())
        {
            std::cerr << "空帧，跳过..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        gettimeofday(&end_time, nullptr);
    }

    // ---- 安全退出 ----
    printf("\n正在安全退出...\n");

    timer_1ms.stop();
    timer_2ms.stop();
    timer_5ms.stop();
    timer_10ms.stop();

    cap.release();
    cleanup();

    printf("程序已退出.\n");
    return 0;
}