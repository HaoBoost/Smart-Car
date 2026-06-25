#ifndef __LQ_CAMERA_EX_HPP
#define __LQ_CAMERA_EX_HPP

#include <iostream>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <mutex>
#include <stdint.h>
#include <sys/ioctl.h>
#include <memory>

#include <opencv2/opencv.hpp>

/****************************************************************************************************
 * @brief   宏定义
 ****************************************************************************************************/

#define LQ_CAMERA_PATH          ( "/dev/video0" )

/****************************************************************************************************
 * @brief   枚举定义
 ****************************************************************************************************/

// 摄像头获取图像方式枚举
typedef enum
{
    LQ_CAMERA_HIGH_MJPG = 0x00, // 高帧率
    LQ_CAMERA_0CPU_MJPG,        // 低 CPU 占用
} lq_camera_format_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class lq_camera_ex
{
public:
    // 有参构造函数构造函数
    explicit lq_camera_ex(uint16_t _width, uint16_t _height, uint16_t _fps, /* 摄像头宽高和帧率设置 */
                          lq_camera_format_t _fmt = LQ_CAMERA_HIGH_MJPG,    /* 获取图像方式 */
                          const std::string _path = LQ_CAMERA_PATH);        /* 摄像头设备路径 */
    
    lq_camera_ex(const lq_camera_ex&) = delete;             // 禁用拷贝
    lq_camera_ex& operator=(const lq_camera_ex&) = delete;  // 禁用赋值
    lq_camera_ex(lq_camera_ex&&) = delete;                  // 禁用移动构造函数
    lq_camera_ex& operator=(lq_camera_ex&&) = delete;       // 禁用移动赋值运算符

    // 析构函数
    ~lq_camera_ex();

public:
    /********************************************************************************
     * @brief   初始化摄像头.
     * @param   _width  : 图像宽度.
     * @param   _height : 图像高度.
     * @param   _fps    : 帧率.
     * @param   _format : 图像格式.
     * @param   _path   : 设备路径.
     * @return  成功返回 0, 失败返回负数.
     * @note    如果已经初始化, 会先释放资源再重新初始化.
     ********************************************************************************/
    int init(uint16_t _width, uint16_t _height, uint16_t _fps, 
             lq_camera_format_t _format = LQ_CAMERA_HIGH_MJPG, 
             const std::string _path = LQ_CAMERA_PATH);

    /********************************************************************************
     * @brief   开始采集图像.
     * @param   none.
     * @return  成功返回 0, 失败返回负数.
     ********************************************************************************/
    int start_collect();

    /********************************************************************************
     * @brief   停止采集图像.
     * @param   none.
     * @return  成功返回 0, 失败返回负数.
     ********************************************************************************/
    int stop_collect();

    /********************************************************************************
     * @brief   获取一帧图像.
     * @param   frame  : 输出图像.
     * @param   decode : 是否解码, 选择解码 frame 最终会返回数据, 不解码时 frame 不返回数据.
     * @return  成功返回 true, 失败返回 false.
     ********************************************************************************/
    bool get_frame(cv::Mat& frame, bool decode = true);

public:
    // 获取摄像头信息
    uint16_t get_camera_width()  const; // 摄像头宽度
    uint16_t get_camera_height() const; // 摄像头高度
    uint16_t get_camera_fps()    const; // 摄像头帧率
    
    // 检查摄像头是否已打开
    bool is_cam_opened() const;

private:
    struct lq_camera_ex_Impl;                   // 类内结构体
    std::unique_ptr<lq_camera_ex_Impl> pImpl;   // 使用智能指针管理
};

#endif
