#ifndef CAR_RUNTIME_HPP
#define CAR_RUNTIME_HPP

#include "lq_drv_inc.hpp"
#include "image.hpp"
#include "motor.hpp"

#ifdef LQ_HAVE_OPENCV
#include <opencv2/core/mat.hpp>
#endif

void CarRuntime_Init(bool enable_motor = true, int motor_init_duty = 1000);
void CarRuntime_Shutdown(bool disable_motor = true);

#ifdef LQ_HAVE_OPENCV
bool CarRuntime_ProcessFrame(const cv::Mat& frame, bool enable_motor = true);
#endif

bool CarRuntime_RunCameraLoop(bool enable_motor = true,
                              uint16_t width = 160,
                              uint16_t height = 120,
                              uint16_t fps = 120,
                              int motor_init_duty = 1000,
                              uint32_t empty_frame_delay_us = 20 * 1000,
                              uint32_t loop_delay_us = 20 * 1000);

#endif
