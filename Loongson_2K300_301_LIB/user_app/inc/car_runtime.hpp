// car_runtime.hpp —— 小车运行时总控
// 负责：集中初始化PID参数 → 初始化三个环的类（Motor/IMU/ImageSteering）→ 定时调度

#ifndef CAR_RUNTIME_HPP
#define CAR_RUNTIME_HPP

#include "lq_drv_inc.hpp"

// 前向声明（避免循环包含，完整头文件在 car_runtime.cpp 中引入）
class Motor;
class IMU;
class ImageSteering;

#ifdef LQ_HAVE_OPENCV
#include <opencv2/core/mat.hpp>
#endif

// ========================== 小车运行时类 ==========================

class CarRuntime
{
public:
    CarRuntime();
    ~CarRuntime();

    // ---- 初始化 ----
    // 步骤1: 初始化所有PID参数（全局）
    void init_pid();
    // 步骤2: 初始化硬件（Motor、IMU、摄像头等）
    void init_hardware(bool enable_motor = true, int motor_init_duty = 1000);

    // ---- 核心循环 ----
    // 主循环：捕获摄像头 → 图像处理 → 三环PID级联控制
    bool run_camera_loop(uint16_t width = 160,
                         uint16_t height = 120,
                         uint16_t fps = 120);

    // ---- 定时回调（供lq_timer使用） ----
    void on_timer_1ms(); // 速度环
    void on_timer_2ms(); // 角速度环
    void on_timer_5ms(); // 图像环

    // ---- 状态查询 ----
    bool is_running() const { return running_; }
    float get_left_speed() const;
    float get_right_speed() const;

private:
    // ---- 三环控制对象（智能指针，延迟初始化） ----
    std::unique_ptr<Motor> motor_;
    std::unique_ptr<IMU> imu_;
    std::unique_ptr<ImageSteering> image_steering_;

    // ---- 级联PID中间变量 ----
    float target_speed_ = 0.0f;      // 基础目标速度（由速度决策设定）
    float diff_speed_ = 0.0f;        // 差速修正量（角速度环 → 速度环）
    float target_angle_rate_ = 0.0f; // 目标角速度（图像环 → 角速度环）

    bool running_ = false;
    bool motor_enabled_ = true;
};

#endif
