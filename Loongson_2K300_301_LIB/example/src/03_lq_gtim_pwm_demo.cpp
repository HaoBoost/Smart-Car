#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_gtim_pwm_demo.cpp
 * @brief   GTIM PWM 输出模式测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 按旧库 GTIM PWM 测试思路，调整 2 路舵机 PWM 的占空比.
 ********************************************************************************/

/********************************************************************************
 * @brief   GTIM PWM 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    对应旧库 GtimPwmTest，使用 88/89 引脚进行 GTIM PWM 输出.
 ********************************************************************************/
void lq_gtim_pwm_demo(void)
{
    ls_gtim_pwm pwm1(GTIM_PWM1_PIN88, 50, 1500);
    ls_gtim_pwm pwm2(GTIM_PWM2_PIN89, 50, 1500, GTIM_PWM_POL_NORMAL);

    while (1)
    {
        pwm1.gtim_pwm_set_duty(1000);
        pwm2.gtim_pwm_set_duty(1000);
        printf("Gtim PWM Set %d\n", 1000);
        sleep(1);

        pwm1.gtim_pwm_set_duty(5000);
        pwm2.gtim_pwm_set_duty(5000);
        printf("Gtim PWM Set %d\n", 5000);
        sleep(1);

        pwm1.gtim_pwm_set_duty(9000);
        pwm2.gtim_pwm_set_duty(9000);
        printf("Gtim PWM Set %d\n", 9000);
        sleep(1);
    }
}
