#include "lq_camera_ex.hpp"
#include "lq_assert.hpp"
#include "lq_utils.hpp"
#include <sys/mman.h>
#include <sys/select.h>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <errno.h>
#include <random>
#include <thread>
#include <linux/videodev2.h>

/******************************************************* 摄像头处理核心程序 *********************************************************/

/********************************************************************************
 * @brief   类内部结构体定义
 * @details 定义了摄像头数据缓存结构体和私有成员变量，以及清理缓冲区、初始化缓冲区和释放缓冲区的私有成员函数
 * @param   none
 * @return  void
 * @note    内部调用，无需在意，主要负责摄像头数据的管理和资源清理，确保程序稳定运行
 ********************************************************************************/
struct lq_camera_ex::lq_camera_ex_Impl {
    // 摄像头数据缓存结构体
    typedef struct 
    {
        void  *data = MAP_FAILED; // 初始化映射地址为空
        size_t size = 0;          // 缓冲区大小
    } camera_ex_buf_t;

    // 私有成员变量
    int         dev_id    = -1;         // 设备 ID
    uint16_t    width     = 0;          // 图像宽度
    uint16_t    height    = 0;          // 图像高度
    uint16_t    fps       = 0;          // 帧率
    bool        is_string = false;      // 是否正在采集
    bool        is_open   = false;      // 摄像头是否已打开

    lq_camera_format_t img_fmt = LQ_CAMERA_HIGH_MJPG;    // 获取图像数据格式

    std::vector<camera_ex_buf_t> bufs;      // 缓冲区列表
    std::vector<uchar>           jepg_bufs; // JPEG 缓冲区列表

    // 私有成员函数

    /********************************************************************************
     * @brief   释放缓冲区.
     * @param   none.
     * @return  none.
     ********************************************************************************/
    int release_buffer(void)
    {
        struct v4l2_requestbuffers req = {0};
        req.count  = 0;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        return ioctl(this->dev_id, VIDIOC_REQBUFS, &req);
    }

    /********************************************************************************
     * @brief   清理所有映射的缓冲区.
     * @param   none.
     * @return  none.
     ********************************************************************************/
    void cleanup_buffers()
    {
        for (size_t i = 0; i < this->bufs.size(); i++) {
            if (this->bufs[i].data != MAP_FAILED) {
                if (munmap(this->bufs[i].data, this->bufs[i].size) == -1) {
                    printf("lq_camera_ex: 资源释放失败, 索引: %d\n", i);
                }
                this->bufs[i].data = MAP_FAILED;
                this->bufs[i].size = 0;
            }
        }
        this->bufs.clear();
        this->jepg_bufs.clear();
    }

    /********************************************************************************
     * @brief   注册内存缓冲队列并映射地址.
     * @param   _num : 申请的缓冲区数量.
     * @return  成功返回 0, 失败返回负数.
     * @example init_buffers(5);
     * @note    向 V4L2 设备申请缓冲区并映射地址.
     ********************************************************************************/
    int init_buffers(int _num) {
        if (_num <= 0 || _num > 10) {   // 限制缓冲区数量上限，避免过度申请
            printf("lq_camera_ex: 无效的资源设置：%d\n", _num);
            return -1;
        }
        if (this->dev_id < 0) {
            printf("lq_camera_ex: 设备未初始化\n");
            return -2;
        }
        // 初始化请求缓冲区
        struct v4l2_requestbuffers req = {0};
        req.count  = _num;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(this->dev_id, VIDIOC_REQBUFS, &req) == -1) {
            printf("lq_camera_ex: 初始化内部调用函数第一部分失败\n");
            return -3;
        }
        if (req.count < 2) {
            printf("lq_camera_ex: 初始化内部调用函数第二部分失败\n");
            return -4;
        }

        this->bufs.resize(req.count);   // 初始化缓冲区列表

        for (int i = 0; i < static_cast<int>(this->bufs.size()); i++)
        {
            // 查询缓冲区信息
            struct v4l2_buffer buf = {0};
            buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index  = static_cast<uint32_t>(i);
            if (ioctl(this->dev_id, VIDIOC_QUERYBUF, &buf) == -1) {
                printf("lq_camera_ex: 初始化内部调用函数第三部分失败, 索引: %d\n", i);
                this->cleanup_buffers();
                release_buffer();
                return -5;
            }
            // 映射缓冲区
            this->bufs[i].size = buf.length;
            this->bufs[i].data = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, this->dev_id, buf.m.offset);
            if (this->bufs[i].data == MAP_FAILED) {
                printf("lq_camera_ex: 初始化内部调用函数第四部分失败, 索引: %d\n", i);
                this->cleanup_buffers();
                release_buffer();
                return -6;
            }
            // 入队缓冲区
            if (ioctl(this->dev_id, VIDIOC_QBUF, &buf) == -1) {
                printf("lq_camera_ex: 初始化内部调用函数第五部分失败, 索引: %d\n", i);
                this->cleanup_buffers();
                release_buffer();
                return -7;
            }
        }
        this->is_open = true;
        return 0;
    }

    /********************************************************************************
     * @brief   取消注册内存并释放资源.
     * @param   none.
     * @return  none.
     ********************************************************************************/
    void uninit_buffers()
    {
        if (!this->is_open) {
            return;
        }
        // 先停止采集
        if (this->is_string && this->dev_id >= 0) {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(this->dev_id, VIDIOC_STREAMOFF, &type) == -1) {
                // ENODEV: 设备已拔出，内核已自动清理，无需报错
                if (errno != ENODEV) {
                    printf("lq_camera_ex: 停止采集失败: errno : %d\n", errno);
                }
            }
            this->is_string = false;
        }
        // 先清理映射（munmap），再释放 V4L2 内核缓冲区
        this->cleanup_buffers();
        if (this->dev_id >= 0) {
            if (release_buffer() == -1) {
                // ENODEV: 设备已拔出，内核已自动清理，无需报错
                if (errno != ENODEV) {
                    printf("lq_camera_ex: 释放资源失败\n");
                }
            }
        }
        this->is_open = false;
    }

    // 检查设备是否有效
    bool is_cam_valid() const {
        return (this->dev_id >= 0) && (this->is_open);
    }
};

/********************************************************************************
 * @brief   构造函数
 * @param   _width  : 图像宽度.
 * @param   _height : 图像高度.
 * @param   _fps    : 帧率.
 * @param   _format : 图像格式.
 * @param   _path   : 设备路径.
 * @return  void
 * @note    构造函数中调用init()初始化摄像头，并设置默认参数，如果系统不是LoongOS，则调用punishUnauthorizedUser()执行惩罚逻辑
 ********************************************************************************/
lq_camera_ex::lq_camera_ex(uint16_t _width, uint16_t _height, uint16_t _fps, lq_camera_format_t _format, const std::string _path) :
    pImpl(std::make_unique<lq_camera_ex_Impl>())
{
    this->init(_width, _height, _fps, _format, _path);
}

/********************************************************************************
 * @brief   析构函数
 * @param   none.
 * @return  void
 * @note    释放摄像头资源，关闭设备文件描述符并重置为-1，确保资源正确清理，避免内存泄漏和设备占用
 ********************************************************************************/
lq_camera_ex::~lq_camera_ex()
{
    if (this->pImpl) {
        this->pImpl->uninit_buffers();
        if (this->pImpl->dev_id >= 0) {
            if (close(this->pImpl->dev_id) == -1) {
                lq_log_error("关闭摄像头失败");
            }
            this->pImpl->dev_id = -1;
        }
    }
}

/********************************************************************************
 * @brief   初始化摄像头.
 * @param   _width  : 图像宽度.
 * @param   _height : 图像高度.
 * @param   _fps    : 帧率.
 * @param   _format : 图像格式.
 * @param   _path   : 设备路径.
 * @return  成功返回 0, 失败返回负数.
 * @note    摄像头初始化成功返回 0, 否则返回错误码.
 ********************************************************************************/
int lq_camera_ex::init(uint16_t _width, uint16_t _height, uint16_t _fps, lq_camera_format_t _format, const std::string _path)
{
    // 参数校验
    if (_width == 0 || _height == 0 || _fps == 0) {
        lq_log_error("无效的参数: width: %d, height: %d, fps: %d", _width, _height, _fps);
        return -1;
    }
    // 如果已经初始化，先清理资源
    if (this->pImpl->is_open) {
        this->pImpl->uninit_buffers();
    }
    if (this->pImpl->dev_id >= 0) {
        close(this->pImpl->dev_id);
        this->pImpl->dev_id = -1;
    }
    // 打开摄像头设备
    this->pImpl->dev_id = open(_path.c_str(), O_RDWR);
    if (this->pImpl->dev_id < 0) {
        lq_log_error("打开摄像头失败: %s", _path.c_str());
        return -2;
    }
    // 配置摄像头采集格式
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // 视频采集类型
    fmt.fmt.pix.width = _width;             // 设置图像宽度
    fmt.fmt.pix.height = _height;           // 设置图像高度
    fmt.fmt.pix.pixelformat = (_format == LQ_CAMERA_0CPU_MJPG) ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_MJPEG;   // 设置图像格式
    fmt.fmt.pix.field = V4L2_FIELD_ANY;     // 设置图像扫描方式为任意(自动设置)
    if (ioctl(this->pImpl->dev_id, VIDIOC_S_FMT, &fmt) < 0) {
        lq_log_error("初始化第一部分失败");
        close(this->pImpl->dev_id);
        this->pImpl->dev_id = -1;
        return -3;
    }
    // 配置摄像头帧率
    struct v4l2_streamparm setparm = {0};
    setparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;         // 视频采集类型
    setparm.parm.capture.timeperframe.numerator = 1;    // 分子
    setparm.parm.capture.timeperframe.denominator = _fps;// 分母，fps = 分母/分子
    if (ioctl(this->pImpl->dev_id, VIDIOC_S_PARM, &setparm) == -1) {
        lq_log_error("初始化第二部分失败");
    }
    // 读取摄像头数据
    v4l2_format get_fmt = {};
    get_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(this->pImpl->dev_id, VIDIOC_G_FMT, &get_fmt) == 0) {
        this->pImpl->width  = static_cast<uint16_t>(get_fmt.fmt.pix.width);
        this->pImpl->height = static_cast<uint16_t>(get_fmt.fmt.pix.height);
    } else {
        lq_log_error("获取摄像头参数失败");
        this->pImpl->width  = _width;
        this->pImpl->height = _height;
    }
    // 读取摄像头帧率
    struct v4l2_streamparm getparm = {};
    getparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(this->pImpl->dev_id, VIDIOC_G_PARM, &getparm) == 0) {
        double actual_fps = static_cast<double>(getparm.parm.capture.timeperframe.denominator / getparm.parm.capture.timeperframe.numerator);
        this->pImpl->fps  = static_cast<uint16_t>(actual_fps);
    } else {
        lq_log_error("获取摄像头帧率参数失败");
        this->pImpl->fps = _fps;
    }
    // 申请缓冲区
    if (this->pImpl->init_buffers(3) < 0) {
        lq_log_error("初始化第三部分失败");
        close(this->pImpl->dev_id);
        this->pImpl->dev_id = -1;
        return -4;
    }
    this->pImpl->img_fmt = _format;
    // 启动采集
    if (this->start_collect() < 0) {
        lq_log_error("启动采集失败");
        this->pImpl->uninit_buffers();
        close(this->pImpl->dev_id);
        this->pImpl->dev_id = -1;
        return -5;
    }
    lq_log_info("龙邱科技图像采集已初始化完毕，愿你代码无错，开发无忧！");
    return 0;
}

/********************************************************************************
 * @brief   启动摄像头采集.
 * @param   none.
 * @return  成功返回 0, 失败返回负数.
 * @note    启动摄像头采集，调用VIDIOC_STREAMON命令，如果采集已启动则直接返回，确保采集状态正确管理
 ********************************************************************************/
int lq_camera_ex::start_collect()
{
    if (!this->pImpl->is_cam_valid()) {
        lq_log_error("摄像头未打开");
        return -1;
    }
    if (this->pImpl->is_string) {
        lq_log_info("采集已启动");
        return 0;
    }
    // 启动采集
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(this->pImpl->dev_id, VIDIOC_STREAMON, &type) == -1) {
        lq_log_error("采集开启失败");
        return -2;
    }
    this->pImpl->is_string = true;
    return 0;
}

/********************************************************************************
 * @brief   停止摄像头采集.
 * @param   none.
 * @return  成功返回 0, 失败返回负数.
 * @note    停止摄像头采集，调用VIDIOC_STREAMOFF命令，如果采集未启动则直接返回，确保采集状态正确管理
 ********************************************************************************/
int lq_camera_ex::stop_collect()
{
    if (!this->pImpl->is_cam_valid()) {
        lq_log_error("摄像头未打开");
        return -1;
    }
    if (!this->pImpl->is_string) {
        lq_log_info("采集已停止");
        return 0;
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(this->pImpl->dev_id, VIDIOC_STREAMOFF, &type) == -1) {
        lq_log_error("采集关闭失败");
        return -1;
    }
    this->pImpl->is_string = false;
    return 0;
}

/********************************************************************************
 * @brief   获取一帧原始图像.
 * @return  成功返回原始图像, 失败返回空cv::Mat.
 * @note    调用该函数会从摄像头获取一帧原始图像，并返回.
 * @note    无论解码是否成功，都要确保缓冲区正确入队，避免资源泄漏和缓冲区丢失，确保程序稳定运行.
 ********************************************************************************/
cv::Mat lq_camera_ex::get_frame_raw()
{
    static uint32_t tick = 0;

    // 初始化为空 Mat 对象
    cv::Mat result;
    if (!this->pImpl->is_cam_valid()) {
        if (lq_get_tick_ms() - tick > 1000) {
            tick = lq_get_tick_ms();
            lq_log_error("摄像头未初始化");
        }
        return result;
    }
    if (!this->pImpl->is_string) {
        if (lq_get_tick_ms() - tick > 1000) {
            tick = lq_get_tick_ms();
            lq_log_error("请先启动采集");
        }
        return result;
    }
    // select 超时等待，防止摄像头拔出后 DQBUF 永久阻塞
    static fd_set fds;
    FD_ZERO(&fds);
    FD_SET(this->pImpl->dev_id, &fds);
    static struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    int sel_ret = select(this->pImpl->dev_id + 1, &fds, NULL, NULL, &tv);
    if (sel_ret <= 0) {
        if (sel_ret == 0) {
            if (lq_get_tick_ms() - tick > 1000) {
                tick = lq_get_tick_ms();
                lq_log_error("数据读取超时");
            }
        } else {
            if (lq_get_tick_ms() - tick > 1000) {
                tick = lq_get_tick_ms();
                lq_log_error("数据读取异常, errno: %d", errno);
            }
        }
        return result;
    }

    static struct v4l2_buffer buf = {0};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    // 出队一帧数据
    if (ioctl(this->pImpl->dev_id, VIDIOC_DQBUF, &buf) == -1) {
        if (lq_get_tick_ms() - tick > 1000) {
            tick = lq_get_tick_ms();
            lq_log_error("数据读取失败");
        }
        return result;
    }
    // 边界检查：确保 index 在有效范围内
    if (buf.index >= this->pImpl->bufs.size()) {
        if (lq_get_tick_ms() - tick > 1000) {
            tick = lq_get_tick_ms();
            lq_log_error("无效的缓冲区索引: %d\n", buf.index);
        }
        // 尝试重新入队，避免缓冲区丢失
        ioctl(this->pImpl->dev_id, VIDIOC_QBUF, &buf);
        return result;
    }
    // 获取原始数据
    if (this->pImpl->img_fmt == LQ_CAMERA_0CPU_MJPG) {
        uchar* data = static_cast<uchar*>(this->pImpl->bufs[buf.index].data);
        cv::Mat yuv_frame(this->pImpl->height, this->pImpl->width, CV_8UC2, data);
        try {
            cv::cvtColor(yuv_frame, result, cv::COLOR_YUV2BGR_YUYV);
        } catch (const cv::Exception& e) {
            if (lq_get_tick_ms() - tick > 1000) {
                tick = lq_get_tick_ms();
                lq_log_error("数据转换失败");
            }
        }
    } else {
        this->pImpl->jepg_bufs.assign(static_cast<uchar*>(this->pImpl->bufs[buf.index].data), static_cast<uchar*>(this->pImpl->bufs[buf.index].data) + buf.bytesused);
        try {
            result = cv::imdecode(this->pImpl->jepg_bufs, cv::IMREAD_COLOR);
        } catch (const cv::Exception& e) {
            if (lq_get_tick_ms() - tick > 1000) {
                tick = lq_get_tick_ms();
                lq_log_error("数据转换失败");
            }
        }
    }
    // 重新入队（无论解码是否成功，都要归还缓冲区）
    if (ioctl(this->pImpl->dev_id, VIDIOC_QBUF, &buf) == -1) {
        lq_log_error("资源回收失败");
        return result;
    }
    return result;
}

/********************************************************************************
 * @brief   获取一帧灰度图像.
 * @return  成功返回灰度图像, 失败返回空cv::Mat.
 * @note    调用该函数会从摄像头获取一帧原始图像，并转为灰度后返回.
 ********************************************************************************/
cv::Mat lq_camera_ex::get_frame_gray()
{
    static uint32_t tick = 0;
    // 获取原始彩色帧数据
    cv::Mat color_frame = this->get_frame_raw();
    if (color_frame.empty()) {
        return cv::Mat();
    }
    // 转换为灰度帧数据
    cv::Mat gray_frame;
    try {
        cv::cvtColor(color_frame, gray_frame, cv::COLOR_BGR2GRAY);
    } catch (const cv::Exception& e) {
        if (lq_get_tick_ms() - tick > 1000) {
            tick = lq_get_tick_ms();
            lq_log_error("灰度转换失败");
        }
        return cv::Mat();
    }
    return gray_frame;
}

/********************************************************************************
 * @brief   获取一帧原始图像和灰度图像.
 * @param   [out] raw    原始图像.
 * @param   [out] gray   灰度图像.
 * @return  成功返回true, 否则返回false.
 * @note    调用该函数会从摄像头获取一帧原始图像，并转为灰度后返回原始图像和灰度图像.
 ********************************************************************************/
bool lq_camera_ex::get_frame_raw_gray(cv::Mat &raw, cv::Mat &gray)
{
    static uint32_t tick = 0;
    cv::Mat frame = this->get_frame_raw();
    if (frame.empty()) {
        return false;
    }
    try {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } catch (const cv::Exception& e) {
        if (lq_get_tick_ms() - tick > 1000) {
            tick = lq_get_tick_ms();
            lq_log_error("灰度转换失败");
        }
        gray = cv::Mat();
        return false;
    }
    raw = frame.clone();
    return true; 
}

/********************************************************************************
 * @brief   设置摄像头的手动曝光.
 * @param   expo : 曝光值.
 * @return  成功返回true, 否则返回false.
 * @note    建议曝光值在[20, 200]之间，当然可以超出该范围.
 ********************************************************************************/
bool lq_camera_ex::set_exposure_manual(int16_t expo)
{
    if(!this->is_cam_opened()){
        lq_log_error("摄像头未打开");
        return false;
    }

    struct v4l2_control auto_ctrl = {0};
    auto_ctrl.id    = V4L2_CID_EXPOSURE_AUTO;   // 自动曝光
    auto_ctrl.value = V4L2_EXPOSURE_MANUAL;     // 设置为手动曝光
    if (ioctl(this->pImpl->dev_id, VIDIOC_S_CTRL, &auto_ctrl) == -1) {
        lq_log_error("自动曝光关闭失败");
        return false;
    }

    struct v4l2_control exp_ctrl = {0};
    exp_ctrl.id    = V4L2_CID_EXPOSURE_ABSOLUTE;    // 手动曝光绝对值
    exp_ctrl.value = expo;                          // 设置曝光值
    if (ioctl(this->pImpl->dev_id, VIDIOC_S_CTRL, &exp_ctrl) == -1) {
        lq_log_error("手动曝光设置失败");
        return false;
    }
    lq_log_info("手动曝光设置成功，%d", expo);
    return true;
}

/********************************************************************************
 * @brief   保存一帧图片为jpg文件.
 * @param   frame : 图片数据.
 * @param   filename : 文件名.
 * @return  成功返回true, 否则返回false.
 ********************************************************************************/
bool lq_camera_ex::save_image_picture(const cv::Mat &frame, const std::string &filename)
{
    if (frame.empty()) {
        lq_log_error("图像数据为空，无法保存");
        return false;
    }
    if (filename.empty()) {
        lq_log_error("文件名不能为空，无法保存");
        return false;
    }
    return cv::imwrite(filename, frame);
}

/********************************************************************************
 * @brief   获取摄像头宽度信息.
 * @param   none.
 * @return  返回摄像头宽度信息.
 * @note    none.
 ********************************************************************************/
uint16_t lq_camera_ex::get_camera_width() const
{
    return this->pImpl->width;
}

/********************************************************************************
 * @brief   获取摄像头高度信息.
 * @param   none.
 * @return  返回摄像头高度信息.
 * @note    none.
 ********************************************************************************/
uint16_t lq_camera_ex::get_camera_height() const
{
    return this->pImpl->height;
}

/********************************************************************************
 * @brief   获取摄像头帧率信息.
 * @param   none.
 * @return  返回摄像头帧率信息.
 * @note    none.
 ********************************************************************************/
uint16_t lq_camera_ex::get_camera_fps() const
{
    return this->pImpl->fps;
}

/********************************************************************************
 * @brief   判断是否打开摄像头.
 * @param   none.
 * @return  true: 打开, false: 未打开.
 * @note    none.
 ********************************************************************************/
bool lq_camera_ex::is_cam_opened() const
{
    return this->pImpl->is_open;
}
