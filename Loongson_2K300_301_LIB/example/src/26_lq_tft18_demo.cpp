#include "lq_all_demo.hpp"
#include "lq_display_tft18.hpp"

/********************************************************************************
 * @file    lq_tft18_demo.cpp
 * @brief   TFT18 测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 TFT18 功能，用于测试 TFT18 控制器的基本功能.
 ********************************************************************************/

/********************************************************************************
 * @brief   TFT18 测试.
 * @param   none.
 * @return  none.
 * @note    none.
 ********************************************************************************/
void lq_tft18_demo()
{
    lq_tft18_drv_init(1);

    lq_tft18_drv_cls(U16WHITE);

    // 显示中文
    lq_display_font_t ssss = {
        .font_idx = tfont_Idx,              // 中文索引
        .font_data = tfont_16x16,           // 中文字库
        .font_size = sizeof(tfont_16x16)    // 字库长度
    };
    lq_tft18_drv_cstr(0, 120, "龙邱科技", ssss, U16RED, U16PURPLE);

    while (ls_system_running.load()) {
        // 全屏蓝色
        lq_tft18_drv_cls(U16BLUE);
        sleep(1);

        // 画矩形区域
        lq_tft18_drv_fill_area(10, 20, 30, 40, U16YELLOW);
        sleep(1);

        // 画线
        lq_tft18_drv_draw_line(10, 20, 30, 40, U16RED);
        sleep(1);

        // 画圆
        lq_tft18_drv_draw_circle(50, 50, 30, U16BLACK);
        sleep(1);

        // 写中文字符
        lq_tft18_drv_cstr(0, 120, "龙邱科技", ssss, U16RED, U16PURPLE);
        
        // 显示图片
        lq_tft18_drv_image(10, 10, 100, 80, gImage_2);        
        sleep(1);
    }
}