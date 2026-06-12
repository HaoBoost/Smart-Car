// IMU.h —— 角速度环（IMU）类，位置式PID / PD+前馈，2ms周期
// 负责：MPU6050陀螺仪读取、角速度PID控制、输出差速修正量给速度环

#ifndef IMU_H
#define IMU_H

#include <memory>
#include "pid.h"
#include "main.hpp"

// 前向声明（库头文件由 lq_app_inc.hpp 引入）
class lq_i2c_mpu6050;

class IMU
{
public:
    // 构造函数：传入角速度PID和PD+前馈指针（拷贝参数）
    IMU(PID *angle_pid, PD_FF *angle_ff);
    ~IMU();

    // ---- PID / PD_FF 对象（从全局拷贝，独立运行） ----
    PID Angle_PID;
    PD_FF Angle_PID_F;

    // ---- 核心接口 ----

    // 读取陀螺仪Z轴角速度（单位：°/s 或原始值，取决于标定）
    float read_gyro_z();

    // 角速度环主处理函数（2ms周期调用）
    // target_angle_rate: 目标角速度（来自图像环）
    // 返回：差速修正量 diff_speed，供速度环使用
    float IMU_proc(float target_angle_rate);

    // 获取当前角速度
    float get_angle_rate() const { return current_angle_rate_; }

    // 查询初始化状态
    bool is_initialized() const { return initialized_; }

private:
    std::unique_ptr<lq_i2c_mpu6050> mpu6050_; // MPU6050驱动对象

    float current_angle_rate_ = 0.0f; // 当前Z轴角速度
    float gyro_offset_ = 0.0f;        // 陀螺仪零偏（初始化时标定）
    bool initialized_ = false;
};

#endif // IMU_H