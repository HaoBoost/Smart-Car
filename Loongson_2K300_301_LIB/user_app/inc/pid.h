#ifndef __PID_H
#define __PID_H

// #include <cstdint>
// #include "main.hpp"

// // extern timeval start_time, end_time;

// typedef struct PID_Data{
// 	float Kp,Ki,Kd;
// 	float SP;
// 	uint64_t t_last;
// 	float err_last;
// 	float err_interval_last;

// 	int64_t COmax,COmin;     //PID输出限幅
// 	int64_t COimax,COimin;   //积分限幅
// };
// PID_Data pid_motor_l,pid_motor_r; //左右电机

// void PID_Init(PID_Data *pid,float Kp,float Ki,float Kd,int64_t COmax,int64_t COmin,int64_t COimax,int64_t COimin);

// float PID_out(PID_Data *pid,float FB);

// typedef struct PID_Data_IMU{
// 	float Kp,Ki,Kd;
// 	float SP;
// 	uint64_t t_last;
// 	float err_last;
// 	float err_interval_last;

// 	int64_t COmax,COmin;     //PID输出限幅
// 	int64_t COimax,COimin;   //积分限幅

// };
// PID_Data_IMU pid_imu; //IMU

// __CONTROL_PID_H
// 普通PID结构体
typedef struct
{
    float kp, ki, kd;                      // 三个系数
    float error, lastError, lastlastError; // 误差、上次误差、上上次误差
    float integral, maxIntegral;           // 积分、积分限幅
    float output, maxOutput, minOutput;    // 输出、输出限幅
    // float desire = 0.0f; //SP
} PID;

// PD+前馈控制结构体
typedef struct
{
    float Kp;      // 比例系数
    float Kd;      // 微分系数
    float Kff;     // 前馈系数
    float Kff_acc; // 前馈变化率系数

    float output; // 输出

    float maxOutput; // 输出上限

    float dt; // 采样周期

    // 状态变量
    float error;           // 本次误差
    float last_error;      // 上一次的误差
    float last_target;     // 上一次的目标角速度
    float last_derivative; // 上一次的微分项（用于低通滤波)
    float last_target_acc; // 上一次的目标角速度变化律（用于低通滤波）

    // float desire = 0.0f; //SP
} PD_FF;

// 保证编译pid.h的时，优先编译PID结构体定义，避免编译器未识别到PID结构体后去编译其他用了PID结构体的地方，导致报错
#include "main.hpp"

// 普通PID控制，增量式和位置式
void Incremental_PID_Init(PID *pid, float p, float i, float d, float minOutput, float maxOutput);
void Incremental_PID_Cal(PID *pid, float set_value, float get_value);

void Positional_PID_Init(PID *pid, float p, float i, float d, float maxI, float minOutput, float maxOutput);
void Positional_PID_Cal(PID *pid, float set_value, float get_value);

void PID_Reset(PID *pid); // 清除PID环的任何时刻误差、积分、输出

void PID_Lmotor(int target);
void PID_Rmotor(int target);
void PID_CarStart(float target, float now_value, int step, PID *left_speed, PID *right_speed);

// PD+前馈 控制
void PD_FF_Init(PD_FF *pd, float kp, float kd, float kff, float kff_acc, float max, float ms);
void PD_FF_Reset(PD_FF *pd);
void PD_FF_Cal(PD_FF *pd, float target, float actual);

void PID_init(PID *lmotor, PID *rmotor, PID *angle, PD_FF *photo_ff, PID *photo);

// 声明全局PID/PD_FF结构体
extern PID Lmotor_PID;    // 左电机速度环PID
extern PID Rmotor_PID;    // 右电机速度环PID
extern PID Angle_PID;     // 角速度环PID（纯位置式，无前馈）
extern PID Photo_PID;     // 图像环PID（位置式）
extern PD_FF Photo_PID_F; // 图像环PD+前馈（转向用）

// PID servo_pid, makeup_pid;

#endif