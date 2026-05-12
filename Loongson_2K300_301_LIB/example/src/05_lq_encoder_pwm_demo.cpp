#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_encoder_pwm_demo.cpp
 * @brief   编码器 PWM 输出模式测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 延续旧库 EncoderTest 的验证目标，用编码器接口持续读取计数值.
 ********************************************************************************/

/********************************************************************************
 * @brief   编码器 PWM 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    方向引脚沿用旧测试里常见的 72/73 分组，便于对照迁移.
 ********************************************************************************/
void lq_encoder_pwm_demo(void)
{
    ls_encoder_pwm left_enc(ENC_PWM0_PIN64, PIN_72);
    ls_encoder_pwm right_enc(ENC_PWM1_PIN65, PIN_73);

    while (1)
    {
        printf("%8.2f %8.2f\r", left_enc.encoder_get_count(), right_enc.encoder_get_count());
        fflush(stdout);
        usleep(50000);
    }
}
