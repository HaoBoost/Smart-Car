#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_image_demo.cpp
 * @brief   图像闭环基础测试.
 * @author  Claude
 * @date    2026-04-05
 * @version V2.1.0
 * @note    使用 runtime 层驱动图像主链，demo 仅作验证入口.
 ********************************************************************************/

void lq_image_demo(void)
{
    (void)CarRuntime_RunCameraLoop(false, 160, 120, 120, 1000, 20 * 1000, 50 * 1000);
}
