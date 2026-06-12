// motor.h 重构

#include <memory>
#include "pid.h"
#include "main.hpp"
#include "image.hpp"

extern float turn_error; // = ImageStatus.Det_True - (float)ImageStatus.MiddleLine;

class Motor{ //电机和速度环相关（app），位置式PID


public:
    Motor(PID* Lmotor_PID, PID* Rmotor_PID, int duty = 1000);
    ~Motor();

    PID Lmotor_PID, Rmotor_PID;

    typedef struct {
        float left_count = 0;
        float right_count = 0;
    }encoder;
    // void encoder_data(encoder& enc);
    encoder encoder_data();

    float L_speed;
    float R_speed;

    //PID左电机设置速度
    inline static float L_filter_speed = 0.0f;  //上一次滤波后的速度
    void PID_Lmotor();
    //PID右电机设置速度
    inline static float R_filter_speed = 0.0f;  //上一次滤波后的速度
    void PID_Rmotor();

    //PID智能车平滑起步，防止电机猛转
    float limit_p = 0.0f;
    void PID_CarStart(float target, float now_value, int step, PID *left_speed, PID *right_speed);
                      

    void Motor_proc();

private:
    int duty;

    // 电机初始化状态标志：false=未初始化，true=初始化完成
    bool kMotorInitialized = false;

    void left_pwm_out(int duty ,bool dir);
    void right_pwm_out(int duty,bool dir);

private:
    // 左电机PWM输出引脚定义（高级定时器通道）
    static constexpr atim_pwm_pin_t kLeftMotorPwmPin  = ATIM_PWM0_PIN81;
// 右电机PWM输出引脚定义（高级定时器通道）
    static constexpr atim_pwm_pin_t kRightMotorPwmPin = ATIM_PWM1_PIN82;
// 左电机方向控制GPIO引脚定义
    static constexpr gpio_pin_t kLeftMotorDirPin  = PIN_21;
// 右电机方向控制GPIO引脚定义
    static constexpr gpio_pin_t kRightMotorDirPin = PIN_22;
// 左编码器脉冲采集引脚定义
    static constexpr ls_enc_pwm_pin_t kLeftEncoderPin = ENC_PWM0_PIN64;
// 右编码器脉冲采集引脚定义
    static constexpr ls_enc_pwm_pin_t kRightEncoderPin = ENC_PWM1_PIN65;
// 左编码器方向判断GPIO引脚定义
    static constexpr gpio_pin_t kLeftEncoderDirPin  = PIN_72;
// 右编码器方向判断GPIO引脚定义
    static constexpr gpio_pin_t kRightEncoderDirPin = PIN_73;
// 电机PWM驱动频率：10kHz（避免电机啸叫，驱动效率最优）
    static constexpr uint32_t kMotorPwmFreqHz = 10000;
// 电机初始化默认PWM占空比
    static constexpr int kDefaultInitDuty = 1000;
// 左电机前进时，方向GPIO输出电平
    static constexpr bool kLeftForwardDir  = true;
// 右电机前进时，方向GPIO输出电平
    static constexpr bool kRightForwardDir = false;
// 左电机PWM控制对象（智能指针）
    std::unique_ptr<ls_atim_pwm> left_motor_pwm;
// 右电机PWM控制对象（智能指针）
    std::unique_ptr<ls_atim_pwm> right_motor_pwm;
// 左电机方向GPIO控制对象
    std::unique_ptr<ls_gpio> left_motor_dir;
// 右电机方向GPIO控制对象
    std::unique_ptr<ls_gpio> right_motor_dir;
// 左编码器采集对象
    std::unique_ptr<ls_encoder_pwm> left_motor_encoder;
// 右编码器采集对象
    std::unique_ptr<ls_encoder_pwm> right_motor_encoder;

};

void motor_isr();


