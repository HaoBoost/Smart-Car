#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_pwm_demo.cpp
 * @brief   PWM 输出模式测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 按旧库 PWM 硬件测试思路，依次调整 4 路 PWM 占空比.
 ********************************************************************************/

/********************************************************************************
 * @brief   PWM 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    对应旧库 HWPwm 测试，使用 64/65/66/67 引脚输出 PWM.
 ********************************************************************************/
void lq_pwm_demo(void)
{
    ls_pwm pwm0(PWM0_PIN64, 50, 1000);
    ls_pwm pwm1(PWM1_PIN65, 100, 1000);
    ls_pwm pwm2(PWM2_PIN66, 1000, 1000);
    ls_pwm pwm3(PWM3_PIN67, 10000, 1000);

    while (1)
    {
        pwm0.pwm_set_duty(500);
        pwm1.pwm_set_duty(1000);
        pwm2.pwm_set_duty(1000);
        pwm3.pwm_set_duty(1000);
        printf("Set HW PWM 1000\n");
        sleep(1);

        pwm0.pwm_set_duty(2500);
        pwm1.pwm_set_duty(5000);
        pwm2.pwm_set_duty(5000);
        pwm3.pwm_set_duty(5000);
        printf("Set HW PWM 5000\n");
        sleep(1);

        pwm0.pwm_set_duty(4500);
        pwm1.pwm_set_duty(9000);
        pwm2.pwm_set_duty(9000);
        pwm3.pwm_set_duty(9000);
        printf("Set HW PWM 9000\n");
        sleep(1);
    }
}
