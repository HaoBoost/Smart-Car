// motor.cpp —— 速度环（Motor）类实现
// 位置式PID，1ms周期，负责编码器读取、速度PID、差速分配、PWM输出

#include "motor.h"
#include <cmath>

// ========================== 构造 & 析构 ==========================

Motor::Motor(PID *Lmotor, PID *Rmotor, int duty)
    : Lmotor_PID(*Lmotor), Rmotor_PID(*Rmotor)
{
    // 限幅初始化占空比
    uint32_t init_duty = static_cast<uint32_t>(duty);
    if (init_duty > ATIM_PWM_DUTY_MAX)
        init_duty = ATIM_PWM_DUTY_MAX;

    // 初始化左/右电机PWM对象
    left_motor_pwm = std::make_unique<ls_atim_pwm>(
        kLeftMotorPwmPin, kMotorPwmFreqHz, init_duty, ATIM_PWM_POL_INV);
    right_motor_pwm = std::make_unique<ls_atim_pwm>(
        kRightMotorPwmPin, kMotorPwmFreqHz, init_duty, ATIM_PWM_POL_INV);

    // 初始化左/右电机方向GPIO
    left_motor_dir = std::make_unique<ls_gpio>(kLeftMotorDirPin, GPIO_MODE_OUT);
    right_motor_dir = std::make_unique<ls_gpio>(kRightMotorDirPin, GPIO_MODE_OUT);

    // 初始化左/右编码器
    left_motor_encoder = std::make_unique<ls_encoder_pwm>(
        kLeftEncoderPin, kLeftEncoderDirPin);
    right_motor_encoder = std::make_unique<ls_encoder_pwm>(
        kRightEncoderPin, kRightEncoderDirPin);

    // 设置默认前进方向
    left_motor_dir->gpio_level_set(kLeftForwardDir ? GPIO_HIGH : GPIO_LOW);
    right_motor_dir->gpio_level_set(kRightForwardDir ? GPIO_HIGH : GPIO_LOW);

    // PWM占空比初始置零
    left_motor_pwm->atim_pwm_set_duty(0);
    right_motor_pwm->atim_pwm_set_duty(0);

    // 速度清零
    L_speed = 0.0f;
    R_speed = 0.0f;

    // 滤波器状态清零
    L_filter_speed = 0.0f;
    R_filter_speed = 0.0f;

    kMotorInitialized = true;
}

Motor::~Motor()
{
    if (!kMotorInitialized)
        return;

    // 停止PWM输出
    left_motor_pwm->atim_pwm_set_duty(0);
    right_motor_pwm->atim_pwm_set_duty(0);

    PID_Reset(&Lmotor_PID);
    PID_Reset(&Rmotor_PID);

    left_motor_pwm->atim_pwm_disable();
    right_motor_pwm->atim_pwm_disable();

    kMotorInitialized = false;
    // 智能指针自动释放硬件资源
}

// ========================== 编码器读取 ==========================

Motor::EncoderData Motor::read_encoders()
{
    EncoderData enc;
    enc.left_count = -static_cast<float>(
        left_motor_encoder->encoder_get_count());
    enc.right_count = static_cast<float>(
        right_motor_encoder->encoder_get_count());
    return enc;
}

// ========================== 左/右电机PID ==========================

void Motor::PID_Lmotor(float target)
{
    // 读编码器
    EncoderData enc = read_encoders();
    float now_speed = enc.left_count;

    // 一阶低通滤波
    const float filt = 0.90f;
    float filter_speed = filt * now_speed + (1.0f - filt) * L_filter_speed;
    L_filter_speed = filter_speed;
    L_speed = filter_speed;

    // 位置式PID解算
    Positional_PID_Cal(&Lmotor_PID, target, L_speed);

    // PWM输出（输出已含符号，正=前进，负=后退）
    float out = Lmotor_PID.output;
    if (out < 0)
        left_pwm_out(static_cast<int>(-out), !kLeftForwardDir); // 后退
    else
        left_pwm_out(static_cast<int>(out), kLeftForwardDir); // 前进
}

void Motor::PID_Rmotor(float target)
{
    EncoderData enc = read_encoders();
    float now_speed = enc.right_count;

    const float filt = 0.90f;
    float filter_speed = filt * now_speed + (1.0f - filt) * R_filter_speed;
    R_filter_speed = filter_speed;
    R_speed = filter_speed;

    Positional_PID_Cal(&Rmotor_PID, target, R_speed);

    float out = Rmotor_PID.output;
    if (out < 0)
        right_pwm_out(static_cast<int>(-out), !kRightForwardDir);
    else
        right_pwm_out(static_cast<int>(out), kRightForwardDir);
}

// ========================== PWM输出 ==========================

void Motor::left_pwm_out(int duty, bool dir)
{
    // 限幅
    if (duty < 0)
        duty = 0;
    if (duty > ATIM_PWM_DUTY_MAX)
        duty = ATIM_PWM_DUTY_MAX;

    left_motor_dir->gpio_level_set(dir ? GPIO_HIGH : GPIO_LOW);
    left_motor_pwm->atim_pwm_set_duty(static_cast<uint32_t>(duty));
}

void Motor::right_pwm_out(int duty, bool dir)
{
    if (duty < 0)
        duty = 0;
    if (duty > ATIM_PWM_DUTY_MAX)
        duty = ATIM_PWM_DUTY_MAX;

    right_motor_dir->gpio_level_set(dir ? GPIO_HIGH : GPIO_LOW);
    right_motor_pwm->atim_pwm_set_duty(static_cast<uint32_t>(duty));
}

// ========================== 速度环主处理（1ms周期） ==========================

// 防堵转：单轮最低目标速度（编码器单位），低于此值电机可能堵转
static constexpr float kMinWheelSpeed = 50.0f;

void Motor::Motor_proc(float target_speed, float diff_speed)
{
    if (!kMotorInitialized)
        return;

    // 差速分配: 左轮加速、右轮减速 或 反之
    float L_target = target_speed + diff_speed;
    float R_target = target_speed - diff_speed;

    // 防堵转：前进时左右轮不低于下限，后退时不高于上限（即绝对值不低于下限）
    if (target_speed > 0.0f)
    {
        if (L_target < kMinWheelSpeed)
            L_target = kMinWheelSpeed;
        if (R_target < kMinWheelSpeed)
            R_target = kMinWheelSpeed;
    }
    else if (target_speed < 0.0f)
    {
        if (L_target > -kMinWheelSpeed)
            L_target = -kMinWheelSpeed;
        if (R_target > -kMinWheelSpeed)
            R_target = -kMinWheelSpeed;
    }

    // 分别执行左右电机PID
    PID_Lmotor(L_target);
    PID_Rmotor(R_target);
}

// ========================== 平滑起步 ==========================

void Motor::PID_CarStart(float target, float now_value, int step,
                         PID *left_speed, PID *right_speed)
{
    if (step <= 0)
        return;

    // 读取编码器
    EncoderData enc = read_encoders();
    L_speed = enc.left_count;
    R_speed = enc.right_count;

    // 首次调用时：保存原始限幅，清零限幅以平滑起步
    if (left_speed->maxOutput > limit_p)
    {
        limit_p = left_speed->maxOutput;
        left_speed->maxOutput = 0.0f;
        right_speed->maxOutput = 0.0f;
    }

    // 执行PID
    PID_Lmotor(target);
    PID_Rmotor(target);

    // 限幅逐步恢复
    if (target - now_value > target / step)
    {
        left_speed->maxOutput += limit_p / step;
        right_speed->maxOutput += limit_p / step;

        if (left_speed->maxOutput > limit_p)
            left_speed->maxOutput = limit_p;
        if (right_speed->maxOutput > limit_p)
            right_speed->maxOutput = limit_p;
    }
}

// ========================== 电机失能 ==========================

void Motor::disable()
{
    if (!kMotorInitialized)
        return;

    left_motor_pwm->atim_pwm_set_duty(0);
    right_motor_pwm->atim_pwm_set_duty(0);

    left_motor_pwm->atim_pwm_disable();
    right_motor_pwm->atim_pwm_disable();

    kMotorInitialized = false;
}