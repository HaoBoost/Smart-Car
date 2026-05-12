#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_motor_demo.cpp
 * @brief   电机正反转测试.
 * @author  Claude
 * @date    2026-04-05
 * @version V2.2.0
 * @note    使用 ATIM_PWM0_PIN81 / ATIM_PWM1_PIN82 作为左右电机 PWM 输出,
 *          使用 PIN_21 / PIN_22 作为左右电机方向控制.
 *!         为避免烧坏电机, 默认仅使用较低占空比启动测试.
 *!         当前默认占空比为 1500(15%), 建议空载确认方向后, 再逐步上调.
 ********************************************************************************/

void lq_motor_demo(void)
{
    const uint32_t motor_freq_hz = 10000;
    const uint32_t safe_duty = 1500;

    ls_atim_pwm left_motor_pwm(ATIM_PWM0_PIN81, motor_freq_hz, safe_duty, ATIM_PWM_POL_INV);
    ls_atim_pwm right_motor_pwm(ATIM_PWM1_PIN82, motor_freq_hz, safe_duty, ATIM_PWM_POL_INV);

    ls_gpio left_motor_dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio right_motor_dir(PIN_22, GPIO_MODE_OUT);

    left_motor_pwm.atim_pwm_set_duty(safe_duty);
    right_motor_pwm.atim_pwm_set_duty(safe_duty);

    while (1)
    {
        left_motor_dir.gpio_level_set(GPIO_HIGH);
        right_motor_dir.gpio_level_set(GPIO_LOW);
        printf("motor forward 10s, duty=%u/10000, left_dir=%d, right_dir=%d\n", safe_duty, 1, 0);
        sleep(5);

        left_motor_dir.gpio_level_set(GPIO_LOW);
        right_motor_dir.gpio_level_set(GPIO_HIGH);
        printf("motor reverse 10s, duty=%u/10000, left_dir=%d, right_dir=%d\n", safe_duty, 0, 1);
        sleep(5);
    }
}
