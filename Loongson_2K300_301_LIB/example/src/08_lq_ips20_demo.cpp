#include "lq_all_demo.hpp"
#include "lq_display_ips20.hpp"

/********************************************************************************
 * @file    lq_ips20_demo.cpp
 * @brief   IPS20 测试.
 * @author  龙邱科技-006
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 IPS20 功能，用于测试 IPS20 控制器的基本功能.
 ********************************************************************************/

/********************************************************************************
 * @brief   IPS20 测试.
 * @param   none.
 * @return  none.
 * @note    该程序会初始化 IPS 屏幕，然后填充黄色区域，绘制红色线，绘制黑色圆.
 ********************************************************************************/
void lq_ips20_demo()
{
    lq_ips20_drv_init(1);

    // 显示中文
    lq_display_font_t ssss = {
        .font_idx = tfont_Idx,              // 中文索引
        .font_data = tfont_32x32,           // 中文字库
        .font_size = sizeof(tfont_32x32)    // 字库长度
    };

    lq_ips20_drv_fill_area(10, 20, 30, 40, U16YELLOW);
    lq_ips20_drv_draw_line(10, 20, 30, 40, U16RED);
    lq_ips20_drv_draw_circle(50, 50, 30, U16BLACK);
    sleep(1);

    while (ls_system_running.load())
    {
        // 写中文字符
        lq_ips20_drv_cstr(0, 120, "龙邱科技", ssss, U16RED, U16PURPLE);
            
        // 显示图片
        lq_ips20_drv_image(10, 10, 100, 80, gImage_2);
        sleep(1);
    }
}