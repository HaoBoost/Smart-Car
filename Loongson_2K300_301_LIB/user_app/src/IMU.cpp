// IMU.cpp —— 角速度环（IMU）类实现
// 位置式PID / PD+前馈，2ms周期

#include "IMU.h"
#include "lq_i2c_mpu6050.hpp"

// ========================== 构造 & 析构 ==========================

IMU::IMU(PID *angle_pid, PD_FF *angle_ff)
    : Angle_PID(*angle_pid), Angle_PID_F(*angle_ff)
{
    // 初始化MPU6050驱动（使用默认设备路径）
    mpu6050_ = std::make_unique<lq_i2c_mpu6050>();

    // 陀螺仪零偏标定：采样若干次取平均
    const int calib_samples = 200;
    float sum_gz = 0.0f;
    for (int i = 0; i < calib_samples; i++)
    {
        int16_t gx = 0, gy = 0, gz = 0;
        if (mpu6050_->get_mpu6050_ang(&gx, &gy, &gz))
        {
            sum_gz += static_cast<float>(gz);
        }
        usleep(2000); // 2ms间隔
    }
    gyro_offset_ = sum_gz / static_cast<float>(calib_samples);

    initialized_ = true;
}

IMU::~IMU()
{
    initialized_ = false;
    // 智能指针自动释放MPU6050资源
}

// ========================== 陀螺仪读取 ==========================

float IMU::read_gyro_z()
{
    if (!initialized_)
        return 0.0f;

    int16_t gx = 0, gy = 0, gz = 0;
    if (mpu6050_->get_mpu6050_ang(&gx, &gy, &gz))
    {
        // 减去零偏，得到实际角速度（原始ADC值，需根据量程换算为°/s）
        current_angle_rate_ = static_cast<float>(gz) - gyro_offset_;
    }
    return current_angle_rate_;
}

// ========================== 角速度环主处理（2ms周期） ==========================

float IMU::IMU_proc(float target_angle_rate)
{
    if (!initialized_)
        return 0.0f;

    // 读取当前角速度
    read_gyro_z();

    // 使用位置式PID计算（角速度环也可以用PD_FF）
    Positional_PID_Cal(&Angle_PID, target_angle_rate, current_angle_rate_);

    // 若配置了PD+前馈，也可以同时使用（可选叠加）
    // PD_FF_Cal(&Angle_PID_F, target_angle_rate, current_angle_rate_);

    // 返回差速修正量（PID输出即为 diff_speed）
    return Angle_PID.output;
}
