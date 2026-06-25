#include "app_context.h"
#include "app_tasks.h"

// IMU 任务只负责更新角速度，并在进环阶段累计圆环相关角度。

void imu_task(void* arg)
{
    (void)arg;

    imu.upDataGyro();
    g_actual_gyro_dbg = imu.gyro_z;
    // 这里只在进环阶段积累角度，其它阶段立即清零，避免旧积分污染状态机。
    if (g_elem_type == ELEM_CIRCLE && g_circle_state == CIRCLE_APPROACH) {
        circle_yaw_angle += imu.gyro_z;
    } else {
        circle_yaw_angle = 0;
    }

    // printf("imu %.2f\r\n",imu.gyro_z);
}
