#ifndef __PID_H
#define __PID_H

#include <stdint.h>
typedef struct{
	float Kp,Ki,Kd;
	float SP;
	uint64_t t_last;
	float err_last;
	float err_interval_last;
	
	int64_t COmax,COmin;//PID输出限幅
	int64_t COimax,COimin; //积分限幅
}PID_Data;



#endif