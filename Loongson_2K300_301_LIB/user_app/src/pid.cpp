#include "pid.h"

void PID_Init(PID_Data *pid,float Kp,float Ki,float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    // pid->COmax = COmax;
    // pid->COmin = COmin;
    // pid->COimax = COimax;
    // pid->COimin = COimin;

    pid->t_last = 0;
    pid->err_last = 0;
    pid->err_interval_last = 0;
}

float PID_out(PID_Data *pid,float FB) {
    gettimeofday(&end_time, nullptr);
    uint64_t t_now = end_time.tv_sec * 1000000 + end_time.tv_usec; //是微秒哦
    float err = pid->SP - FB;
    float err_dev = (err - pid->err_last) / (t_now - pid->t_last);
    float err_interval = (err - pid->err_last) / (t_now - pid->t_last);
    float CO = pid->Kp * err + pid->Ki * err_dev + pid->Kd * err_interval;

    // if (CO > pid->COmax) CO = pid->COmax;
    // else if (CO < pid->COmin) CO = pid->COmin;

    // if (pid->Ki != 0) {
    //     if (CO > pid->COimax) CO = pid->COimax;
    //     else if (CO < pid->COimin) CO = pid->COimin;
    // }

    pid->t_last = t_now;
    pid->err_last = err;
    pid->err_interval_last = err_interval;

    return CO;
}