/********************************************************************************
 * @file            main.cpp
 * @brief           本文件是 LQ_2K300_301_LIB 软件开源库文件的一部分
 * @copyright       版权所有 (C) 2025-2026 北京龙邱科技有限公司
 * @website         http://www.lqist.cn
 * @taobao          https://longqiu.taobao.com
 *
 * @description     龙邱科技 LS2K300/301 核心板驱动开源库声明
 *
 * 本文件遵循 GPL-3.0 开源协议发布，旨在为龙芯 2K300/301 平台提供快速上手开发基于龙芯 2K300/301 平台的应用程序的参考实现.
 * 商业用途(包括单位使用)需提前联系作者获取授权
 *
 * GPL-3.0 许可证声明摘要:
 * 1. 允许自由使用、修改、分发本软件
 * 2. 分发修改后的版本时，必须以相同许可证发布
 * 3. 必须保留原始版权声明和许可证信息
 * 4. 不提供任何担保，使用风险自负
 * 5. 完整协议文本请参见项目根目录 LICENSE 文件
 *
 * @author          龙邱科技-012
 * @email           chiusir@163.com
 * @version         V2.1.0
 * @update          2026-04-21
 *
 * @note            使用本开源库前, 请确认板卡型号与 PMON 版本是否匹配.
 *                  龙邱 2K301 或已根据群文件升级系统的2K300可直接使用.
 *                  旧版本 2K300 需要修改时钟频率, 请到 lq_clock.hpp 文件修改 CONFIG_USE_PMON.
 *
 * @note            本库已重载 CTRL+C 信号, 按下 CTRL+C 时，会设置 ls_system_running 为 false，
 *                  从而退出所有正在运行的 demo 例程.
 * @note            如果用户在主程序写了自定义的循环，需要在循环中判断 ls_system_running.load() 是否为 true，
 *                  否则会导致程序无法退出.
 ********************************************************************************/

#include <opencv2/opencv.hpp>  // 万能头文件，包含所有常用模块
#include <opencv2/core.hpp>     // 核心模块（cv::命名空间基础）
#include <opencv2/videoio.hpp> // 视频IO模块（VideoCapture/VideoWriter）
#include <thread>
#include <chrono>
#include "main.hpp"
#include <image.hpp>
#include <motor.hpp>

////////////////////////////变量定义区///////////////////////////////////
void signalHandler(int signum);
std::atomic<bool> stopSignal1(false);
// 删除冗余变量 stopSignal2
// 在main.cpp的顶部添加声明，告诉编译器这个变量在别的文件里定义
extern cv::Mat First_image; 

////////////////////////////主函数区///////////////////////////////////////
int main() 
{
    signal(SIGINT, signalHandler);

    cv::VideoCapture cap(0); 
    if (!cap.isOpened()) 
    {
        std::cerr << "无法打开摄像头" << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
    cap.set(cv::CAP_PROP_FPS, 30);  // 修复：改为摄像头支持的FPS

    // 修复：1. 先加载参数，再初始化硬件
    Data_Settings(); 
    Motor_Init1(1000);
    Motor_Argument();  
    sleep(2);

    while (!stopSignal1) 
    {
        cap >> First_image; 
        if (First_image.empty()) 
        {
            std::cerr << "Error: Unable to grab frame." << std::endl;
            // 修复：空帧跳过，不执行控制
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }    

        ImageProcess();

        // 修复：2. 调用总控函数（内部执行差速PID+双电机PID）
        Motor_Control();

        // 修复：3. 循环延时10ms，降低CPU占用，固定控制周期
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        printf("ImageStatus.Det_True: %d \n", ImageStatus.Det_True);    
    }

    printf("startDisable\n");
    cap.release();
    Motor_Disable1();
    // 删除：退出后调用无效的 Motor_Diff_Pid1()

    printf("Done\n");
    return 0;
}

void signalHandler(int signum) 
{
    printf("\nCtrl+C安全退出...\n");
    stopSignal1 = true;
}

// 底部注释代码保留，不影响运行
        // beep.SetGpioValue(0);   // 关闭蜂鸣器
 
        // GpioOutputTest1(88);    // GPIO输出功能(设备文件)
        // GpioOutputTest2(88);    // GPIO输出功能(硬件)
        // GpioInputTest1();       // GPIO输入功能(设备文件)
        // GpioInputTest2();       // GPIO输入功能(硬件)
        // PwmDevTest();           // PWM 测试(设备文件)
        // PwmHWTest();            // PWM 测试(寄存器)
        // GtimPwmTest();          // Gtim PWM 测试(硬件)
        // EncoderTest();          // 编码器测试(寄存器)
        // CameraTest();           // 摄像头测试
        // AdcFunTest();           // ADC 功能测试
        // TFTTest();              // TFT屏幕测试
        // GetTimeTest();          // 时间戳打印测试
        // sleepTest();            // sleep()函数测试 -- 以秒为单位延时
        // usleepTest();           // usleep()函数测试 -- 以微秒为单位延时
        // nanosleepTest();        // nanosleep()函数测试 -- 以纳秒为单位延时
        // clock_nanosleepTest();  // clock_nanosleep()函数测试 -- 以纳秒为单位延时
        //MotorTest();            // 电机测试程序
        // ServoTest();舵机测试程序
        // GpioTest();             // 久久派22个GPIO翻转测试
        //MotorTestrun();
        //Servo_Control();
        // Motor_test_run(0);


