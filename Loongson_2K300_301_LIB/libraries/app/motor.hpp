#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "lq_drv_inc.hpp"
#include "image.hpp"

typedef struct {
  float P;
  float I;
  float D;
  int LastError;
  int PrevError;
  int EC;
  float Kdin;
  float Kdout;
} PID_Datatypedef;

void Encoder_Test1(void);
float Encoder_Left1(void);
float Encoder_Right1(void);

void Motor_Argument(void);
void Motor_Init1(int duty = 1000);
void Left_Motor_Pwm1(int duty, bool dir);
void Right_Motor_Pwm1(int duty, bool dir);
void Motor_Disable1(void);
void Motor_Control(void);
void Motor_Diff_Pid1(void);
void Motor_PID_Left(void);
void Motor_PID_Right(void);

#endif
