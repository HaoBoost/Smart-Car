// car_runtime.cpp —— 小车运行时总控实现
// 集中初始化PID → 初始化三个环的类 → 定时调度三环级联PID

#include "car_runtime.hpp"
#include "lq_common.hpp"
#include "pid.h"
#include "speeddecision.h"
#include "IMU.h"
#include "motor.h"
#include "image.hpp"

#ifdef LQ_HAVE_OPENCV
#include <opencv2/core/mat.hpp>
#endif

// ========================== 构造 & 析构 ==========================

CarRuntime::CarRuntime()
{
    running_ = false;
}

CarRuntime::~CarRuntime()
{
    running_ = false;
    // 智能指针自动释放Motor/IMU/ImageSteering对象
}

// ========================== 步骤1: 初始化PID参数 ==========================

void CarRuntime::init_pid()
{
    // 集中初始化所有PID参数
    // 参数顺序: 左电机PID, 右电机PID, 角速度环PID, 图像环PD前馈, 图像环PID
    PID_init(&Lmotor_PID,  // 左电机速度环PID
             &Rmotor_PID,  // 右电机速度环PID
             &Angle_PID,   // 角速度环PID（纯位置式，无前馈）
             &Photo_PID_F, // 图像环/转向环 PD+前馈
             &Photo_PID);  // 图像环PID
}

// ========================== 步骤2: 初始化硬件 ==========================

void CarRuntime::init_hardware(bool enable_motor, int motor_init_duty)
{
    // 加载图像处理全局参数（MiddleLine、阈值等）
    Data_Settings();

    motor_enabled_ = enable_motor;

    if (enable_motor)
    {
        // 创建Motor对象（拷贝全局PID参数）
        motor_ = std::make_unique<Motor>(&Lmotor_PID, &Rmotor_PID, motor_init_duty);
    }

    // 创建IMU对象（角速度环，纯PID，无前馈）
    imu_ = std::make_unique<IMU>(&Angle_PID);

    // 创建ImageSteering对象（图像环/转向环，PID + PD前馈）
    image_steering_ = std::make_unique<ImageSteering>(&Photo_PID, &Photo_PID_F);

    // 初始化速度决策
    target_speed_ = 300.0f; // 默认基础速度

    running_ = true;
}

// ========================== 定时回调 ==========================

void CarRuntime::on_timer_1ms()
{
    // 速度环：1ms周期
    if (motor_ && motor_->is_initialized())
    {
        motor_->Motor_proc(target_speed_, diff_speed_);
    }
}

void CarRuntime::on_timer_2ms()
{
    // 角速度环：2ms周期
    if (imu_ && imu_->is_initialized())
    {
        diff_speed_ = imu_->IMU_proc(target_angle_rate_);
    }
}

void CarRuntime::on_timer_5ms()
{
    // 图像环：5ms周期
    // 先执行图像处理（更新ImageStatus等全局状态）
    ImageProcess();

    // 再执行图像环PID（计算中线偏差 → 输出目标角速度）
    if (image_steering_)
    {
        target_angle_rate_ = image_steering_->proc();
    }
}

// ========================== 速度查询 ==========================

float CarRuntime::get_left_speed() const
{
    if (motor_) return motor_->L_speed;
    return 0.0f;
}

float CarRuntime::get_right_speed() const
{
    if (motor_) return motor_->R_speed;
    return 0.0f;
}

// ========================== 主循环 ==========================

bool CarRuntime::run_camera_loop(uint16_t width, uint16_t height, uint16_t fps)
{
#ifndef LQ_HAVE_OPENCV
    (void)width;
    (void)height;
    (void)fps;
    lq_log_error("OpenCV not enabled, car runtime is unavailable");
    return false;
#else
    // 打开摄像头
    lq_camera_ex cam(width, height, fps);
    if (!cam.is_cam_opened())
    {
        lq_log_error("Failed to open camera for car runtime");
        return false;
    }

    // 初始化PID + 硬件
    init_pid();
    init_hardware(true, 1000);

    // 定时器：调度三环
    lq_timer timer_1ms, timer_2ms, timer_5ms;

    timer_1ms.set_seconds_ms(1, [this]()
                             { on_timer_1ms(); });
    timer_2ms.set_seconds_ms(2, [this]()
                             { on_timer_2ms(); });
    timer_5ms.set_seconds_ms(5, [this]()
                             { on_timer_5ms(); });

    // 主循环：持续捕获摄像头帧
    while (running_ && ls_system_running.load())
    {
        cv::Mat frame = cam.get_frame_raw();
        if (frame.empty())
        {
            usleep(10 * 1000); // 空帧等待10ms
            continue;
        }
        First_image = frame; // 存入全局变量供ImageProcess使用
    }

    // 停止定时器
    timer_1ms.stop();
    timer_2ms.stop();
    timer_5ms.stop();

    // 失能电机
    if (motor_)
    {
        motor_->disable();
    }

    cleanup();
    return true;
#endif
}