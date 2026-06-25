#include "app_context.h"
#include "app_tasks.h"

// 编码器任务只负责累计“元素状态机需要的里程信息”。
// 正常闭环速度控制中的实时编码器读取，仍在 motor_task 中完成。

void encoder_get_task(void* arg)
{
    (void)arg;

    static int last_l_cnt = 0;
    static int last_r_cnt = 0;

    motor.thread_syn();
    int encoder1_counts = motor.syn_encoder1_counts;
    int encoder2_counts = motor.syn_encoder2_counts;

    last_l_cnt = encoder1_counts;
    last_r_cnt = encoder2_counts;

    // 十字和圆环只在特定阶段需要累计路程，其余阶段全部清零。
    if (g_elem_type == ELEM_CROSS && (g_cross_state == CROSS_BEGIN || g_cross_state == CROSS_END)) {
        cross_encoder_L += (-encoder1_counts);
        cross_encoder_R += (-encoder2_counts);
    } else if (g_elem_type == ELEM_CIRCLE && g_circle_state == CIRCLE_BEGIN) {
        circle_encoder_L += (-encoder1_counts);
        circle_encoder_R += (-encoder2_counts);
    } else {
        cross_encoder_L = 0;
        cross_encoder_R = 0;
        circle_encoder_L = 0;
        circle_encoder_R = 0;
    }

    if (g_ncnn_action_state == NCNN_ACT_BYPASS || g_ncnn_action_state == NCNN_ACT_RECOVER) {
        g_ncnn_action_encoder_l += (-encoder1_counts);
        g_ncnn_action_encoder_r += (-encoder2_counts);
    } else if (g_ncnn_action_state == NCNN_ACT_IDLE
               || g_ncnn_action_state == NCNN_ACT_WAIT_TRIGGER
               || g_ncnn_action_state == NCNN_ACT_SLOW_CLASSIFY) {
        g_ncnn_action_encoder_l = 0;
        g_ncnn_action_encoder_r = 0;
    }
    // printf("%d-%d\n",g_ncnn_action_encoder_l,g_ncnn_action_encoder_r);
}
