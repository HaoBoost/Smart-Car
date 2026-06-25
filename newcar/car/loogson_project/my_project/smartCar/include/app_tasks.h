#pragma once

// 各周期任务拆分到独立源文件后，通过本头文件统一声明。
void motor_task(void* arg);
void photo_task();
void key_task(void* arg);
void imu_task(void* arg);
void encoder_get_task(void* arg);
void recv_task(void* arg);
void menu_task(void* arg);
