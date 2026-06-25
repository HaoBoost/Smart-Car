#/*********************************************************************************************************************
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
 * 额外说明：
 * - 本项目可能包含第三方组件，各组件的版权与许可以其各自随附的 LICENSE 为准；
 * - 分发、修改本文件时请保留本版权与许可声明以尊重原作者权利；
 * - 本文档为中文译述/摘要，英文许可证文本为法律权威版。
 *
 * 文件名称：ww_camera.cc
 * 所属模块：wuwu_library
 * 功能描述：摄像头设备封装（基于 V4L2 与 OpenCV），用于设备初始化、帧捕获、回放与参数查询等。
 * 版本信息：详见 libraries/doc/version
 * 开发环境：Linux，GCC / Clang，OpenCV，V4L2
 * 联系/主页：请参阅项目 README
 *
 * 修改记录：
 * 日期        作者    说明
 * 2026-01-11  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "ww_camera.h"

Camera::Camera() :
            fd(-1), bufExist(false)
{
    memset(&playback, 0, sizeof(playback));
}

Camera::~Camera()
{
    stop_capturing();
    destroy_buffers();
    if(fd != -1) {
        close(fd);
    }
}

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
int Camera::init(int width, int height, int fps, const char *device, uint32_t pixelformat)
{
    fd = open(device, O_RDWR);
    if(fd < 0) {
        std::cerr << "无法打开" << device << std::endl;
        return -1;
    }

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixelformat;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if(ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        std::cerr << "摄像头格式设置失败" << std::endl;
        return -1;
    }

    struct v4l2_streamparm setparm = {};
    setparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setparm.parm.capture.timeperframe.numerator = 1;  // 分子
    setparm.parm.capture.timeperframe.denominator = fps;  // 分母，fps = 分母/分子

    if (ioctl(fd, VIDIOC_S_PARM, &setparm) == -1) {
        std::cerr << "警告: 设置帧率失败，使用默认帧率" << std::endl;
        return -1;
    }

    v4l2_format get_fmt = {};
    get_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if (ioctl(fd, VIDIOC_G_FMT, &get_fmt) == 0) {
        std::cout << "摄像头输出尺寸: " << 
                get_fmt.fmt.pix.width << 'x' << get_fmt.fmt.pix.height 
                << std::endl;
    }

    struct v4l2_streamparm getparm = {};
    getparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if (ioctl(fd, VIDIOC_G_PARM, &getparm) == 0) {
        double actual_fps = (double)getparm.parm.capture.timeperframe.denominator /
                            getparm.parm.capture.timeperframe.numerator;
        std::cout << "帧率: " << actual_fps << " fps" << std::endl;
    }

    if(request_buffers(3) < 0) {
        return -1;
    }

    if(start_capturing() < 0) {
        return -1;
    }

    return 0;
}

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
int Camera::set_exposure(int exposure_value)
{
    struct v4l2_control ctrl = {};
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl.value = V4L2_EXPOSURE_MANUAL;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1) {
        std::cerr << "设置手动曝光模式失败" << std::endl;
        return -1;
    }

    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = exposure_value;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1) {
        std::cerr << "设置曝光值失败" << std::endl;
        return -1;
    }

    std::cout << "曝光设置为: " << exposure_value << std::endl;
    return 0;
}

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
bool Camera::capture_frame(cv::Mat &frame, bool decode)
{
    if(bufExist == false) {
        std::cerr << "摄像头未初始化" << std::endl;
        return false;
    }

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    // 出队一帧
    if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
        std::cerr << "出队缓冲区失败" << std::endl;
        return false;
    }

    // 是否保存回放
    if(playback.start && playback.index < playback.frame_max) 
    {
        uchar* data_star = static_cast<uchar*>(buffers[buf.index].data);
        jpeg_buffers[playback.index].assign(data_star, data_star + buf.bytesused);
        playback.index++;

        if(playback.index == playback.frame_max) {
            playback_stop();
        }
    }
    
    //获取JPEG原始数据
    jpeg_nowdata.assign(static_cast<uchar*>(buffers[buf.index].data), static_cast<uchar*>(buffers[buf.index].data) + buf.bytesused);
    // MJPG解码
    if(decode) {
        frame = cv::imdecode(jpeg_nowdata, cv::IMREAD_COLOR);
        if(frame.empty()) {
            return false;
        }
    }
    
    // 重新入队
    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
        std::cerr << "入队缓冲区失败" << std::endl;
        return false;
    }

    return true;
}

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
void Camera::playback_init(int frame_max)
{
    playback.frame_max = frame_max;
    playback.index = 0;

    jpeg_buffers.clear();
    jpeg_buffers.resize(playback.frame_max);

    for(auto& buffer : jpeg_buffers) {
        buffer.reserve(20 * 1024);
    }
}

/*******************************************************************
 * @brief       保存缓冲图像
 * 
 * @example     playback_save();
 * 
 * @note        默认保存到 playback目录下（需要自己创建）
 ******************************************************************/
void Camera::playback_save(void)
{
    if(playback.index == 0) {
        std::cout << "没有可保存的帧" << std::endl;
        return;
    }

    playback_stop();

    std::cout << "开始保存 " << playback.index << " 帧到 playback/ 目录..." << std::endl;

    // 保存每帧 JPEG 数据
    for (int i = 0; i < playback.index; i++) {
        if (jpeg_buffers[i].empty()) {
            continue;  // 跳过空帧
        }

        char filename[256];
        snprintf(filename, sizeof(filename), "playback/fps%d.jpg", i);
        
        // 打开文件
        FILE* file = fopen(filename, "wb");
        if (!file) {
            std::cerr << "无法创建文件: " << filename << std::endl;
            continue;
        }
        
        // 写入 JPEG 数据
        size_t written = fwrite(jpeg_buffers[i].data(), 1, jpeg_buffers[i].size(), file);
        fclose(file);
        
        if (written != jpeg_buffers[i].size()) {
            std::cerr << "写入文件不完整: " << filename << std::endl;
        }
        
        //反馈进度
        if(i == playback.index / 4) {
            std::cout << "已保存25%" << std::endl;
        }

        if(i == (playback.index / 4) * 2) {
            std::cout << "已保存50%" << std::endl;
        }

        if(i == (playback.index / 4) * 3) {
            std::cout << "已保存75%" << std::endl;
        }
    }
    
    std::cout << "完成! 共保存 " << playback.index << " 帧到 playback/ 目录" << std::endl;

}

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
 * @note        向V4L2注册缓冲队列，用户空间映射内存地址实现零拷贝
 *              count = 3 减少图像延迟(内部调用)
 ******************************************************************/
int Camera::request_buffers(int count)
{
    struct v4l2_requestbuffers req = {};
    req.count = count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        std::cerr << "请求缓冲区失败" << std::endl;
        return -1;
    }

    if (req.count < 2) {
        std::cerr << "缓冲区不足" << std::endl;
        return -1;
    }

    bufExist = true;
    buffers.resize(req.count);

    for(int i = 0; i < buffers.size(); i++)
    {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            std::cerr << "查询缓冲区失败" << std::endl;
            return -1;
        }

        buffers[i].size = buf.length;
        buffers[i].data = mmap(NULL, buf.length, 
                                PROT_READ | PROT_WRITE, 
                                MAP_SHARED, 
                                fd, buf.m.offset);

        if (buffers[i].data == MAP_FAILED) {
            std::cerr << "内存映射失败" << std::endl;
            return -1;
        }

        // 入队缓冲区
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            std::cerr << "初始入队缓冲区失败" << std::endl;
            return -1;
        }
    }

    return 0;
}

/*******************************************************************
 * @brief       取消内存映射并释放缓冲队列
 * 
 * @example     //注销
 *              destroy_buffers();
 ******************************************************************/
void Camera::destroy_buffers(void)
{
    if(bufExist == false) return;

    for(int i = 0; i < buffers.size(); i++) {
        if(buffers[i].data != MAP_FAILED) {
            munmap(buffers[i].data, buffers[i].size);
        }
    }

    struct v4l2_requestbuffers req = {};
    req.count = 0;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    
    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        std::cerr << "警告: 释放缓冲区失败: " << strerror(errno) << std::endl;
    } else {
        std::cout << "缓冲区已释放" << std::endl;
    }
}

/*******************************************************************
 * @brief       开启摄像头采集
 * 
 * @example     //开启采集
 *              start_capturing();
 ******************************************************************/
int Camera::start_capturing(void)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        std::cout << "采集开启失败" << std::endl;
        return -1;
    }

    return 0;
}

/*******************************************************************
 * @brief       停止摄像头采集
 * 
 * @example     //停止采集
 *              stop_capturing();
 ******************************************************************/
void Camera::stop_capturing(void) 
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);
}
