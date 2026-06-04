//motor.cpp重构
#include "motor.h"

Motor::Motor(PID* Lmotor, PID* Rmotor, int duty = 1000) : Lmotor_PID(*Lmotor), Rmotor_PID(*Rmotor) {
    // 初始化占空比限幅处理
    const uint32_t init_duty = static_cast<uint32_t>((duty));

    // 初始化左电机PWM对象
    left_motor_pwm  = std::make_unique<ls_atim_pwm>(kLeftMotorPwmPin,  kMotorPwmFreqHz, init_duty, ATIM_PWM_POL_INV);
    // 初始化右电机PWM对象
    right_motor_pwm = std::make_unique<ls_atim_pwm>(kRightMotorPwmPin, kMotorPwmFreqHz, init_duty, ATIM_PWM_POL_INV);
    // 初始化左电机方向GPIO
    left_motor_dir  = std::make_unique<ls_gpio>(kLeftMotorDirPin,  GPIO_MODE_OUT);
    // 初始化右电机方向GPIO
    right_motor_dir = std::make_unique<ls_gpio>(kRightMotorDirPin, GPIO_MODE_OUT);
    // 初始化左编码器对象
    left_motor_encoder  = std::make_unique<ls_encoder_pwm>(kLeftEncoderPin,  kLeftEncoderDirPin);
    // 初始化右编码器对象
    right_motor_encoder = std::make_unique<ls_encoder_pwm>(kRightEncoderPin, kRightEncoderDirPin);

    // 设置左电机默认前进方向
    left_motor_dir->gpio_level_set(kLeftForwardDir  ? GPIO_HIGH : GPIO_LOW);
    // 设置右电机默认前进方向
    right_motor_dir->gpio_level_set(kRightForwardDir ? GPIO_HIGH : GPIO_LOW);
    // 左电机PWM占空比置0
    left_motor_pwm->atim_pwm_set_duty(0);
    // 右电机PWM占空比置0
    right_motor_pwm->atim_pwm_set_duty(0);

    L_speed = 0.0f;
    R_speed = 0.0f;

    kMotorInitialized = true; // 设置电机初始化完成标志
}

Motor::~Motor() {
    if (!kMotorInitialized)
    {
        return;
    }
    // 左电机PWM置0
    left_motor_pwm->atim_pwm_set_duty(0);
    // 右电机PWM置0
    right_motor_pwm->atim_pwm_set_duty(0);

    PID_Reset(&Lmotor_PID);
    PID_Reset(&Rmotor_PID);
    // 关闭左电机PWM硬件
    left_motor_pwm->atim_pwm_disable();
    // 关闭右电机PWM硬件
    right_motor_pwm->atim_pwm_disable();
    // 标记电机未初始化
    kMotorInitialized = false;
} // 析构函数中不需要手动释放资源，智能指针会自动管理

// void Motor::encoder_data(Motor::encoder& enc) {
//     enc.left_count = -static_cast<int16_t>(left_motor_encoder->encoder_get_count());
//     enc.right_count = static_cast<int16_t>(right_motor_encoder->encoder_get_count());
// }
Motor::encoder Motor::encoder_data() {
    encoder enc;
    enc.left_count = -static_cast<int16_t>(left_motor_encoder->encoder_get_count());
    enc.right_count = static_cast<int16_t>(right_motor_encoder->encoder_get_count());
    return enc;
}

void Motor::PID_Lmotor(){
    //最新获取的编码器的值
    float now_speed = encoder_data().left_count * 1.0f;

    //一阶低通滤波
    float filt = 0.90f;
    float filter_speed = filt * now_speed + (1 - filt) * L_filter_speed;
    L_filter_speed = filter_speed;  //更新保存

    //将滤波后的值用于PID
    L_speed = filter_speed;

    //解算PID获得电机输出
    Positional_PID_Cal(&Lmotor_PID,target,L_speed);

    if(Lmotor_PID.output < 0) left_pwm_out(Lmotor_PID.output, true);
    else left_pwm_out(Lmotor_PID.output, false);
}

void Motor::PID_Rmotor(){
    //最新获取的编码器的值
    float now_speed = encoder_data().right_count * 1.0f;

    //一阶低通滤波
    float filt = 0.90f;
    float filter_speed = filt * now_speed + (1 - filt) * R_filter_speed;
    R_filter_speed = filter_speed;  //更新保存

    //将滤波后的值用于PID
    R_speed = filter_speed;

    //解算PID获得电机输出
    Positional_PID_Cal(&Rmotor_PID,target,R_speed);

    if(Rmotor_PID.output < 0) right_pwm_out(Rmotor_PID.output, true);
    else right_pwm_out(Rmotor_PID.output, false);
}

void Motor::left_pwm_out(int duty ,bool dir)
{
    // 设置左电机方向电平
    left_motor_dir->gpio_level_set(dir ? GPIO_HIGH : GPIO_LOW);
    // 设置左电机PWM占空比
    left_motor_pwm->atim_pwm_set_duty(static_cast<uint32_t>(duty)); // 占空比参数已经限幅了
}

void Motor::right_pwm_out(int duty, bool dir) {
    // 设置右电机方向电平
    right_motor_dir->gpio_level_set(dir ? GPIO_HIGH : GPIO_LOW);
    // 设置右电机PWM占空比
    right_motor_pwm->atim_pwm_set_duty(static_cast<uint32_t>(duty));
}

void Motor::PID_CarStart(float target, float now_value, int step, PID *left_speed, PID *right_speed){
    //参数保护
    if(step <= 0) return;

    //读取编码器数据
    // motor.update_encoders(); 

    //根据实际情况决定速度值
    L_speed = encoder_data().left_count * 1.0f;
    R_speed = encoder_data().right_count * 1.0f;

    //通过单边电机判断，先对预设值赋值，使两电机初始限幅为0，以便于平滑启动
    if(left_speed->maxOutput > limit_p)
    {
        limit_p = left_speed->maxOutput;
        left_speed->maxOutput = 0;
        right_speed->maxOutput = 0;
    }

    //PID的使用
    PID_Rmotor();
    //R_pwm = Rmotor_PID.output;
    left_pwm_out(Lmotor_PID.output, Lmotor_PID.output < 0);
    PID_Lmotor();
    // L_pwm = Lmotor_PID.output;
    right_pwm_out(Rmotor_PID.output, Rmotor_PID.output < 0);

    //左右电机限幅值根据步长缓慢上升，做到智能车平滑起步
    if(target - now_value > target / step)
    {
        //左右电机限幅值缓慢上升
        left_speed->maxOutput += limit_p/step;
        right_speed->maxOutput += limit_p/step;
        //防止电机实际限幅值超出预设值
        left_speed->maxOutput = left_speed->maxOutput > limit_p ? limit_p : left_speed->maxOutput; 
        right_speed->maxOutput = right_speed->maxOutput > limit_p ? limit_p : right_speed->maxOutput;
    }
}

void Motor::Motor_proc() {
    PERIODIC(1000) // 内环1ms周期
    
}

// // 接口示例
// // 左电机PWM+方向控制函数
// // @param duty  PWM占空比 (0~ATIM_PWM_DUTY_MAX)，自动限幅
// // @param dir   方向: true=前进(kLeftForwardDir), false=后退
// void Left_Motor_Pwm1(int duty, bool dir)
// {
//     // 检查电机是否初始化完成
//     ensure_motor_ready();
//     // 设置左电机方向电平
//     left_motor_dir->gpio_level_set(dir ? GPIO_HIGH : GPIO_LOW);
//     // 设置左电机PWM占空比
//     left_motor_pwm->atim_pwm_set_duty(static_cast<uint32_t>(duty));
// }