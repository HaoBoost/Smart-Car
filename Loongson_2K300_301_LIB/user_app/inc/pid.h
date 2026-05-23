#ifndef __PID_H
#define __PID_H

#include <cstdint>
#include "main.hpp"

extern timeval start_time, end_time;

typedef struct PID_Data{
	float Kp,Ki,Kd;
	float SP;
	uint64_t t_last;
	float err_last;
	float err_interval_last;
	
	int64_t COmax,COmin;     //PID输出限幅
	int64_t COimax,COimin;   //积分限幅
};
PID_Data pid_motor_l,pid_motor_r; //左右电机


void PID_Init(PID_Data *pid,float Kp,float Ki,float Kd,int64_t COmax,int64_t COmin,int64_t COimax,int64_t COimin);

float PID_out(PID_Data *pid,float FB);

typedef struct PID_Data_IMU{
	float Kp,Ki,Kd;
	float SP;
	uint64_t t_last;
	float err_last;
	float err_interval_last;
	
	int64_t COmax,COmin;     //PID输出限幅
	int64_t COimax,COimin;   //积分限幅


};
PID_Data_IMU pid_imu; //IMU



#endif