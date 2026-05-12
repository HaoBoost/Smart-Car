#include "lq_all_demo.hpp"

void lq_image_motor_demo(void)
{
    (void)CarRuntime_RunCameraLoop(true, 160, 120, 120, 1000, 20 * 1000, 20 * 1000);
}
