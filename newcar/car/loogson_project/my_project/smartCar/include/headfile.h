#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 公共头文件汇总
 * 版权所有 (c) 2025 wuwu
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 本文件是 Wuwu 开源库 的一部分。
 * 本文件按照 GNU 通用公共许可证 第3版（GPLv3）或您选择的任何后续版本的条款授权。
 * 您可以在遵守 GPL-3.0 许可条款的前提下，自由地使用、复制、修改和分发本文件及其衍生作品。
 * 在分发本文件或其衍生作品时，必须以相同的许可证（GPL-3.0）对源代码进行授权并随附许可证副本。
 *
 * 本软件按“原样”提供，不对适销性、特定用途适用性或不侵权做任何明示或暗示的保证。
 * 有关更多细节，请参阅 GNU 官方许可证文本： https://www.gnu.org/licenses/gpl-3.0.html
 *
 * 注：本注释为 GPL-3.0 许可证的中文说明与摘要，不构成法律意见。正式许可以 GPL 原文为准。
 * LICENSE 副本通常位于项目根目录的 LICENSE 文件或 libraries 文件夹下；若未找到，请访问上方链接获取。
 *
 * 文件名称：headfile.h
 * 所属模块：smartCar/include
 * 功能描述：项目公共包含头文件和系统依赖的汇总
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-6-2   wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#ifndef __HEADFILE_H_
#define __HEADFILE_H_

/* 第三方库 */
#include <opencv2/opencv.hpp>
#include <json/json.h>
#include <net.h>
#include <lq_camera_ex.h>
/* 系统库 */
#include <sys/socket.h>
#include <arpa/inet.h>  
#include <unistd.h>     
#include <iostream>
#include <chrono>
#include <pthread.h>
#include <time.h>
#include <fstream>
#include <cmath>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <cstdint>
#include <thread>
#include <mutex>
#include <linux/input.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sstream>
#include <vector>
#include <ctime>
#include <iomanip>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <cstring>
#include <linux/videodev2.h>


/* wuwu库 */
#include "ww_ioctl.h"
#include "ww_key.h"
#include "ww_brushless.h"
#include "ww_icm42688.h"
#include "ww_motor.h"
#include "ww_buzzer.h"
#include "ww_lcd.h"
#include "ww_vl53l0x.h"
#include "ww_timerThread.h"
#include "ww_transmission.h"
#include "ww_tcp_client.h"
#include "ww_camera.h"
#include "font.h"
#include "ww_camera_server.h"
#endif
