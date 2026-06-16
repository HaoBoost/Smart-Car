// motor.h —— 速度环（Motor）类，位置式PID，1ms周期
// 负责：编码器读取、左右轮速度PID控制、差速分配、PWM输出、平滑起步

#ifndef MOTOR_H
#define MOTOR_H

#include <memory>
#include "pid.h"
#include "main.hpp"

class Motor
{
public:
    // 构造函数：传入左右电机PID指针（拷贝参数），duty为初始化占空比
    Motor(PID *Lmotor_PID, PID *Rmotor_PID, int duty = 1000);
    ~Motor();

    // ---- 编码器数据结构 ----
    struct EncoderData
    {
        float left_count = 0.0f;
        float right_count = 0.0f;
    };

    // ---- PID 对象（从全局拷贝，独立运行） ----
    PID Lmotor_PID;
    PID Rmotor_PID;

    // ---- 实时速度 ----
    float L_speed = 0.0f;
    float R_speed = 0.0f;

    // ---- 核心接口 ----

    // 读取编码器（一次读取左右，避免重复调用导致数据不同步）
    EncoderData read_encoders();

    // 左电机速度PID：target = 目标速度（含差速修正）
    void PID_Lmotor(float target);
    // 右电机速度PID：target = 目标速度（含差速修正）
    void PID_Rmotor(float target);

    // 速度环主处理函数（1ms周期调用）
    // target_speed: 基础目标速度, diff_speed: 差速修正量（来自角速度环）
    void Motor_proc(float target_speed, float diff_speed);

    // 平滑起步：逐步放开PID输出限幅
    void PID_CarStart(float target, float now_value, int step,
                      PID *left_speed, PID *right_speed);

    // 直接PWM控制（调试用）
    void left_pwm_out(int duty, bool dir);
    void right_pwm_out(int duty, bool dir);

    // 失能电机（停止PWM输出）
    void disable();

    // 查询初始化状态
    bool is_initialized() const { return kMotorInitialized; }

private:
    // ---- 滤波器状态 ----
    float L_filter_speed = 0.0f; // 左轮一阶低通滤波上一次值
    float R_filter_speed = 0.0f; // 右轮一阶低通滤波上一次值

    // ---- 平滑起步限幅 ----
    float limit_p = 0.0f;

    // ---- 初始化标志 ----
    bool kMotorInitialized = false;

    // ---- 硬件引脚常量 ----
    static constexpr atim_pwm_pin_t kLeftMotorPwmPin = ATIM_PWM0_PIN81;
    static constexpr atim_pwm_pin_t kRightMotorPwmPin = ATIM_PWM1_PIN82;
    static constexpr gpio_pin_t kLeftMotorDirPin = PIN_21;
    static constexpr gpio_pin_t kRightMotorDirPin = PIN_22;
    static constexpr ls_enc_pwm_pin_t kLeftEncoderPin = ENC_PWM0_PIN64;
    static constexpr ls_enc_pwm_pin_t kRightEncoderPin = ENC_PWM1_PIN65;
    static constexpr gpio_pin_t kLeftEncoderDirPin = PIN_72;
    static constexpr gpio_pin_t kRightEncoderDirPin = PIN_73;
    static constexpr uint32_t kMotorPwmFreqHz = 10000;
    static constexpr bool kLeftForwardDir = false;
    static constexpr bool kRightForwardDir = false;

    // ---- 硬件对象（智能指针自动管理生命周期） ----
    std::unique_ptr<ls_atim_pwm> left_motor_pwm;
    std::unique_ptr<ls_atim_pwm> right_motor_pwm;
    std::unique_ptr<ls_gpio> left_motor_dir;
    std::unique_ptr<ls_gpio> right_motor_dir;
    std::unique_ptr<ls_encoder_pwm> left_motor_encoder;
    std::unique_ptr<ls_encoder_pwm> right_motor_encoder;
};

#endif // MOTOR_H
