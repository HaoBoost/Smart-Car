/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 摄像头模块
 * 版权所有 (c) 2025 wuwu
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 本文件是 Wuwu 开源库 的一部分。
 *
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
 * 文件名称：ww_camera.h
 * 所属模块：wuwu_library
 * 功能描述：摄像头设备封装头文件
 *
 * 修改记录：
 * 日期        作者    说明
 * 2026-01-11  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#ifndef _WUWU_CAMERA_H_
#define _WUWU_CAMERA_H_

#include "headfile.h"

class Camera
{
public:
    Camera();
    ~Camera();

    /****************************
     * @brief   摄像头JPEG原始数据
     ****************************/
    std::vector<uchar> jpeg_nowdata;

/*******************************************************************
 * @brief       摄像头初始化(默认使用MJPG)
 * 
 * @return      返回初始化状态
 * @retval      0               初始化成功
 * @retval      -1              初始化失败
 * 
 * @param       width           摄像头输出宽度
 * @param       height          摄像头输出高度
 * @param       fps             摄像头输出帧率
 * @param       device          摄像头设备路经
 * @param       pixelformat     摄像头读取格式
 * 
 * @example     //摄像头格式设置
 *              init(160, 120, 120);
 * 
 * @note        摄像头初始化
 *              默认 /dev/video0
 *              默认 V4L2_PIX_FMT_MJPEG 格式
 ******************************************************************/
    int init(int width, int height, int fps, 
                const char *device = "/dev/video0", 
                uint32_t pixelformat = V4L2_PIX_FMT_MJPEG);

/*******************************************************************
 * @brief       捕获一帧(默认使用MJPG)
 * 
 * @return      返回初始化状态
 * @retval      false           捕获失败
 * @retval      true            捕获成功
 * 
 * @param       frame           输出Mat图像
 * @param       decode          是否需要解码(默认为true)
 * 
 * @example     //摄像头格式设置
 *              if(!capture_frame(img)){
 *                  break;
 *              }
 * 
 * @note        默认使用MJPG解码
 ******************************************************************/
    bool capture_frame(cv::Mat& frame, bool decode = true);

/*******************************************************************
 * @brief       初始化回放
 * 
 * @param       frame_max       最大保存帧数
 * 
 * @example     //摄像头格式设置
 *              playback_init(1200);
 * 
 * @note        默认保存JPEG格式
 ******************************************************************/
    void playback_init(int frame_max);

/*******************************************************************
 * @brief       保存缓冲图像
 * 
 * @example     playback_save();
 * 
 * @note        默认保存到 playback目录下（需要自己创建）
 ******************************************************************/
    void playback_save(void);

/*******************************************************************
 * @brief       开启图像缓存
 * 
 * @example     playback_start();
 * 
 * @note        缓存过程在capture_frame中执行, 确保该函数已调用
 ******************************************************************/
    void playback_start(void) {playback.start = true;}

/*******************************************************************
 * @brief       停止图像缓存
 * 
 * @example     playback_stop();
 * 
 * @note        缓存过程在capture_frame中执行, 确保该函数已调用
 ******************************************************************/
    void playback_stop(void) {playback.start = false;}

/*******************************************************************
 * @brief       重启图像缓存
 * 
 * @example     playback_restart();
 * 
 * @note        缓存过程在capture_frame中执行, 确保该函数已调用
 ******************************************************************/
    void playback_restart(void) {playback.index = 0;}

/*******************************************************************
 * @brief       设置摄像头曝光
 *
 * @return      返回设置状态
 * @retval      0               设置成功
 * @retval      -1              设置失败
 *
 * @param       exposure_value  曝光值(范围取决于摄像头硬件)
 *
 * @example     set_exposure(100);
 *
 * @note        会先切换到手动曝光模式，再设置曝光值
 *              需要在 init() 之后调用
 ******************************************************************/
    int set_exposure(int exposure_value);

    int get_fd(void) const {return fd;}

private:
    int fd;
    bool bufExist;

    struct Buffer {
        void* data;
        size_t size;
    };
    std::vector<Buffer> buffers;

    struct Playback {
        int frame_max;
        int index;
        bool start;
    };
    struct Playback playback;
    std::vector<std::vector<uchar>> jpeg_buffers;

/*******************************************************************
 * @brief       注册内存缓冲队列并映射地址
 * 
 * @return      返回初始化状态
 * @retval      0               初始化成功
 * @retval      -1              初始化失败
 * 
 * @param       count           内存缓冲队列大小
 * 
 * @example     //注册
 *              request_buffers(3);
 * 
 * @note        向V4L2注册缓冲队列，用户空间映射内存地址实现0拷贝
 *              count = 3 减少图像延迟(内部调用)
 ******************************************************************/
    int request_buffers(int count);

/*******************************************************************
 * @brief       取消内存映射并释放缓冲队列
 * 
 * @example     //注销
 *              destroy_buffers();
 ******************************************************************/
    void destroy_buffers(void);

/*******************************************************************
 * @brief       开启摄像头采集
 * 
 * @example     //开启采集
 *              start_capturing();
 ******************************************************************/
    int start_capturing(void);

/*******************************************************************
 * @brief       停止摄像头采集
 * 
 * @example     //停止采集
 *              stop_capturing();
 ******************************************************************/
    void stop_capturing(void);

};

#endif
