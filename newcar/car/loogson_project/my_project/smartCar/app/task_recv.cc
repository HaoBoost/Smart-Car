#include "app_context.h"
#include "app_tasks.h"

// 上位机参数接收任务。
// 当前只开放了少量通道，后续要扩展在线调参，优先在这里加映射关系。

void recv_task(void* arg)
{
    (void)arg;

    // 通道 1~3：直接在线改主角度环 PID。
    if (seekfree_client.is_parameter_updated(1)) {
        pid_angle_v.kp = seekfree_client.get_parameter(1);
        seekfree_client.clear_parameter_flag(1);
        printf("[Param] Updated  pid_angle_v.kp = %.2f\n",  pid_angle_v.kp);
    }
    if (seekfree_client.is_parameter_updated(2)) {
        pid_angle_v.kd = seekfree_client.get_parameter(2);
        seekfree_client.clear_parameter_flag(2);
        printf("[Param] Updated pid_angle_v.kd = %.2f\n", pid_angle_v.kd);
    }
    if (seekfree_client.is_parameter_updated(3)) {
        target_v = seekfree_client.get_parameter(3);
        seekfree_client.clear_parameter_flag(3);
        printf("[Param] Updated target_v = %.2f\n", target_v);
    }
    // 通道 4：启动 / 停车切换。
    if (seekfree_client.is_parameter_updated(4)) {
        // start_flag ^= 1;
        pid_speed_r.ki = seekfree_client.get_parameter(4);
        // printf("start_flag = %d\n", start_flag);
        seekfree_client.clear_parameter_flag(4);
    }
    if (seekfree_client.is_parameter_updated(5)) {
        base_speed_target_l = seekfree_client.get_parameter(5);

        seekfree_client.clear_parameter_flag(5);
    }
    if (seekfree_client.is_parameter_updated(6)) {
        base_speed_target_r = seekfree_client.get_parameter(6);
        seekfree_client.clear_parameter_flag(6);
    }
    if (seekfree_client.is_parameter_updated(7)) {
        seekfree_client.clear_parameter_flag(7);
    }
    if (seekfree_client.is_parameter_updated(8)) {
        seekfree_client.clear_parameter_flag(8);
    }
}
