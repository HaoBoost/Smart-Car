#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_gpio_basic_demo.cpp
 * @brief   GPIO 输入输出基本功能测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *!         本 demo 按旧库 GPIO 输出测试的思路，批量翻转常用 GPIO 引脚.
 ********************************************************************************/

/********************************************************************************
 * @brief   GPIO 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    按旧库 LQ_Gpio_Out_Test 的寄存器版测试逻辑，批量翻转 22 个 GPIO.
 ********************************************************************************/
void lq_gpio_output_demo(void)
{
    #define LQ_GPIO_TEST_COUNT 22
    gpio_pin_t gpio_pins[LQ_GPIO_TEST_COUNT] = {
        PIN_88, PIN_89, PIN_73, PIN_72, PIN_75, PIN_74, PIN_50, PIN_51,
        PIN_64, PIN_65, PIN_66, PIN_67, PIN_60, PIN_62, PIN_63, PIN_42,
        PIN_43, PIN_44, PIN_45, PIN_48, PIN_49, PIN_61
    };
    ls_gpio gpios[LQ_GPIO_TEST_COUNT];

    for (int i = 0; i < LQ_GPIO_TEST_COUNT; ++i)
    {
        gpios[i] = ls_gpio(gpio_pins[i], GPIO_MODE_OUT);
    }

    while (1)
    {
        for (int i = 0; i < LQ_GPIO_TEST_COUNT; ++i)
        {
            gpios[i].gpio_level_set(GPIO_HIGH);
        }
        printf("Set gpio H\n");
        sleep(1);

        for (int i = 0; i < LQ_GPIO_TEST_COUNT; ++i)
        {
            gpios[i].gpio_level_set(GPIO_LOW);
        }
        printf("Set gpio L\n");
        sleep(1);
    }
}

/********************************************************************************
 * @brief   GPIO 输入模式测试.
 * @param   none.
 * @return  none.
 * @note    GPIO 输入测试, 使用引脚 74 作为输入引脚, 读取当前电平.
            该引脚位于母板编码器 3 所用引脚上.
 ********************************************************************************/
void lq_gpio_input_demo(void)
{
    ls_gpio gpio(PIN_74, GPIO_MODE_IN);

    while (1)
    {
        printf("gpio 74 value = %d\n", gpio.gpio_level_get());
        usleep(5000);
    }
}
