#include "pid.h"

// void PID_Init(PID_Data *pid,float Kp,float Ki,float Kd) {
//     pid->Kp = Kp;
//     pid->Ki = Ki;
//     pid->Kd = Kd;
//     // pid->COmax = COmax;
//     // pid->COmin = COmin;
//     // pid->COimax = COimax;
//     // pid->COimin = COimin;

//     pid->t_last = 0;
//     pid->err_last = 0;
//     pid->err_interval_last = 0;
// }

// float PID_out(PID_Data *pid,float FB) {
//     gettimeofday(&end_time, nullptr);
//     uint64_t t_now = end_time.tv_sec * 1000000 + end_time.tv_usec; //是微秒哦
//     float err = pid->SP - FB;
//     float err_dev = (err - pid->err_last) / (t_now - pid->t_last);
//     float err_interval = (err - pid->err_last) / (t_now - pid->t_last);
//     float CO = pid->Kp * err + pid->Ki * err_dev + pid->Kd * err_interval;

//     // if (CO > pid->COmax) CO = pid->COmax;
//     // else if (CO < pid->COmin) CO = pid->COmin;

//     // if (pid->Ki != 0) {
//     //     if (CO > pid->COimax) CO = pid->COimax;
//     //     else if (CO < pid->COimin) CO = pid->COimin;
//     // }

//     pid->t_last = t_now;
//     pid->err_last = err;
//     pid->err_interval_last = err_interval;

//     return CO;
// }


//定义类
 extern Motor motor;

/**
 * ************************************************************************
 * @brief 增量式PID参数的初始化
 * 
 * @param[in] pid  pid指针
 * @param[in] p  初始化设定的p
 * @param[in] i  初始化设定的i
 * @param[in] d  初始化设定的d
 * @param[in] maxOutput  输出限幅值
 * 
 * ************************************************************************
 */
//增量式PID参数的初始化
void Incremental_PID_Init(PID *pid, float p, float i, float d, float minOutput, float maxOutput)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxOutput = maxOutput;
    pid->minOutput = minOutput;
}

/**
 * ************************************************************************
 * @brief 增量式PID控制器
 *          
 * @param[in] pid  pid指针
 * @param[in] set_value  目标值
 * @param[in] get_value  反馈值
 * 
 * ************************************************************************
 */
//增量式PID控制器
void Incremental_PID_Cal(PID *pid, float set_value,float get_value)
{
	pid->error = set_value - get_value;      									        //计算偏差
	pid->output += pid->kp*(pid->error - pid->lastError) + pid->ki*pid->error + \
				pid->kd*(pid->error - 2*pid->lastError + pid->lastlastError);			//增量式PI控制器
	pid->lastlastError = pid->lastError;    											//保存上上次误差
	pid->lastError = pid->error;	           											//保存上一次偏差

	if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;                                                   //输出限幅
    else if (pid->output < pid->minOutput)
        pid->output = pid->minOutput;													//输出限幅
}


/**
 * ************************************************************************
 * @brief 位置式PID参数的初始化
 * 
 * @param[in] pid  pid指针
 * @param[in] p  初始化设定的p
 * @param[in] i  初始化设定的i
 * @param[in] d  初始化设定的d
 * @param[in] maxI  积分限幅值
 * @param[in] maxOutput  输出限幅值
 * 
 * ************************************************************************
 */
//位置式PID参数的初始化
void Positional_PID_Init (PID *pid, float p, float i, float d, float maxI,float minOutput, float maxOutput)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOutput;
	pid->minOutput = minOutput;
}

/**
 * ************************************************************************
 * @brief 位置式PID控制器
 * 
 * @param[in] pid  pid结构体
 * @param[in] set_value  目标值
 * @param[in] get_value  反馈值
 * 
 * ************************************************************************
 */
//位置式PID控制器
void Positional_PID_Cal(PID *pid,float set_value, float get_value)
{
	float dout,pout;
    //更新数据
    pid->lastError = pid->error; 							//将旧error存起来
    pid->error = set_value - get_value; 					//计算新error

    //计算微分
    dout = (pid->error - pid->lastError) * pid->kd;
    //计算比例
    pout = pid->error * pid->kp;
    //计算积分
    pid->integral += pid->error * pid->ki;

    //积分限幅
    if (pid->integral > pid->maxIntegral)
        pid->integral = pid->maxIntegral;
    else if (pid->integral < -pid->maxIntegral)
        pid->integral = -pid->maxIntegral;
    //计算输出
    pid->output = pout + dout + pid->integral;
    //输出限幅
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;
}

//清除PID环的任何时刻误差和积分
void PID_Reset(PID *pid)
{
    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->lastlastError = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

//PID左电机设置速度
static float L_filter_speed = 0.0f;  //上一次滤波后的速度
void PID_Lmotor(int target)
{    
    //最新获取的编码器的值
    float now_speed = -motor.encoder1_counts * 1.0f;

    //一阶低通滤波
    float filt = 0.90f;
    float filter_speed = filt * now_speed + (1 - filt) * L_filter_speed;
    L_filter_speed = filter_speed;  //更新保存

    //将滤波后的值用于PID
    L_speed = filter_speed;

    //解算PID获得电机输出
    Positional_PID_Cal(&Lmotor_PID,target,L_speed);

    if(Lmotor_PID.output < 0) motor.set_motor1(1,-Lmotor_PID.output);
    else motor.set_motor1(0,Lmotor_PID.output);
}

//PID右电机设置速度
static float R_filter_speed = 0.0f;  //上一次滤波后的速度
void PID_Rmotor(int target)
{
    //最新获取的编码器的值
    float now_speed = -motor.encoder2_counts * 1.0f;

    //一阶低通滤波
    float filt = 0.90f;
    float filter_speed = filt * now_speed + (1 - filt) * R_filter_speed;
    R_filter_speed = filter_speed;  //更新保存

    //将滤波后的值用于PID
    R_speed = filter_speed;

    //解算PID获得电机输出
    Positional_PID_Cal(&Rmotor_PID,target,R_speed);

    if(Rmotor_PID.output < 0) motor.set_motor2(1,-Rmotor_PID.output);
    else motor.set_motor2(0,Rmotor_PID.output);
}

//PID智能车平滑起步，防止电机猛转
float limit_p = 0.0f;
void PID_CarStart(float target, float now_value, int step, PID *left_speed, PID *right_speed)
{
    //参数保护
    if(step <= 0) return;

    //读取编码器数据
    motor.update_encoders(); 

    //根据实际情况决定速度值
    L_speed = -motor.encoder1_counts * 1.0f;
    R_speed = -motor.encoder2_counts * 1.0f;

    //通过单边电机判断，先对预设值赋值，使两电机初始限幅为0，以便于平滑启动
    if(left_speed->maxOutput > limit_p)
    {
        limit_p = left_speed->maxOutput;
        left_speed->maxOutput = 0;
        right_speed->maxOutput = 0;
    }

    //PID的使用
    PID_Rmotor(target);
    R_pwm = Rmotor_PID.output;
    PID_Lmotor(target);
    L_pwm = Lmotor_PID.output;

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


//PD+前馈控制器初始化
void PD_FF_Init(PD_FF* pd, float kp, float kd, float kff, float kff_acc, float max, float ms)
{
    pd->Kp = kp;
    pd->Kd = kd;
    pd->Kff = kff;
    pd->Kff_acc = kff_acc;

    pd->dt = ms/1000;  //输入毫秒

    pd->maxOutput = max;

    PD_FF_Reset(pd);
}

//PD+前馈控制状态清零
void PD_FF_Reset(PD_FF* pd)
{
    pd->error = 0.0f;
    pd->last_error = 0.0f;
    pd->last_target = 0.0f;
    pd->last_derivative = 0.0f;
    pd->last_target_acc = 0.0f;
    pd->output = 0.0f;
}

//PD+前馈 控制计算
void PD_FF_Cal(PD_FF* pd, float target, float actual)
{
    //防止采样周期过小
    if (pd->dt <= 0.0f) pd->dt = 0.001f;

    //误差
    pd->error = target - actual;

    //微分
    float derivative = (pd->error - pd->last_error) / pd->dt;

    //低通滤波
    float alpha = 0.8f;
    derivative = alpha * derivative + (1.0f-alpha) * pd->last_derivative;
    pd->last_derivative = derivative;

    //目标变化率
    float raw_target_acc = (target - pd->last_target) / pd->dt;

    //低通滤波
    float beta = 0.8f; 
    float filtered_target_acc = beta * raw_target_acc + (1.0f - beta) * pd->last_target_acc;
    pd->last_target_acc = filtered_target_acc;

    //前馈+反馈控制
    float feedforward_acc = pd->Kff_acc * filtered_target_acc;
    float feedforward = pd->Kff * target;
    float feedback = pd->Kp * pd->error + pd->Kd * derivative;

    pd->output = feedforward + feedback + feedforward_acc;    //输出

    //输出限幅
    if(pd->output > pd->maxOutput) pd->output = pd->maxOutput;
    else if(pd->output < -pd->maxOutput) pd->output = -pd->maxOutput;

    //更新状态
    pd->last_error = pd->error;
    pd->last_target = target;
}