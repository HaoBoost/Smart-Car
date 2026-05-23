// 包含小车运行时核心头文件，声明本模块对外接口函数
#include "car_runtime.hpp"

// 包含通用工具库头文件，提供日志打印、基础工具函数等功能
#include "lq_common.hpp"

// 条件编译：若项目启用OpenCV，才包含OpenCV核心头文件
#ifdef LQ_HAVE_OPENCV
// 包含OpenCV图像矩阵(Mat)核心头文件，用于图像处理
#include <opencv2/core/mat.hpp>
#endif

// 匿名命名空间：封装小车运行时内部全局变量，外部无法直接访问
namespace
{
// 小车运行时初始化状态标志：false=未初始化，true=初始化完成
bool car_runtime_initialized = false;
// 电机使能状态标志：false=电机禁用，true=电机已启用
bool car_runtime_motor_enabled = false;
// 电机初始化默认PWM占空比：存储电机启动时的初始占空比值
int car_runtime_motor_init_duty = 1000;
}

/**
 * @brief 小车运行时初始化函数
 * @param enable_motor 是否启用电机模块
 * @param motor_init_duty 电机初始化PWM占空比
 */
void CarRuntime_Init(bool enable_motor, int motor_init_duty)
{
    // 加载小车全局参数配置（传感器、PID、图像参数等）
    Data_Settings();
    // 保存电机使能状态到内部变量
    car_runtime_motor_enabled = enable_motor;
    // 保存电机初始化占空比到内部变量
    car_runtime_motor_init_duty = motor_init_duty;

    // 如果配置为启用电机
    if (enable_motor)
    {
        // 初始化电机硬件（PWM、GPIO、编码器）
        Motor_Init1(motor_init_duty);
        // 配置电机PID控制参数
        Motor_Argument();
    }

    // 标记小车运行时初始化完成
    car_runtime_initialized = true;
}

/**
 * @brief 小车运行时关机/资源清理函数
 * @param disable_motor 是否关闭电机模块
 */
void CarRuntime_Shutdown(bool disable_motor)
{
    // 如果需要关闭电机 且 电机当前处于启用状态
    if (disable_motor && car_runtime_motor_enabled)
    {
        // 失能电机，停止PWM输出，关闭硬件
        Motor_Disable1();
    }

    // 调用全局清理函数，释放所有动态资源
    cleanup();
    // 标记小车运行时未初始化
    car_runtime_initialized = false;
}

// 条件编译：仅启用OpenCV时，编译图像处理相关代码
#ifdef LQ_HAVE_OPENCV
/**
 * @brief 处理单帧图像并执行控制逻辑
 * @param frame 输入的OpenCV图像帧
 * @param enable_motor 是否启用电机控制
 * @return 处理成功返回true，空帧/失败返回false
 */
bool CarRuntime_ProcessFrame(const cv::Mat& frame, bool enable_motor)
{
    // 若运行时未初始化 或 电机状态不匹配，则重新初始化
    if (!car_runtime_initialized || car_runtime_motor_enabled != enable_motor)
    {
        CarRuntime_Init(enable_motor, car_runtime_motor_init_duty);
    }

    // 将相机采集的原始帧赋值给全局图像变量
    First_image = frame;
    // 判断图像是否为空（无效帧）
    if (First_image.empty())
    {
        return false;
    }

    // 执行图像处理核心逻辑（巡线、目标识别、路径计算）
    ImageProcess();
    // 如果启用电机，执行电机运动控制
    if (enable_motor)
    {
        // 电机PID+差速控制，输出PWM驱动电机
        Motor_Control();
    }

    // 帧处理完成，返回成功
    return true;
}
#endif

/**
 * @brief 相机主循环函数（小车核心运行循环）
 * @param enable_motor 是否启用电机
 * @param width 相机分辨率宽度
 * @param height 相机分辨率高度
 * @param fps 相机帧率
 * @param motor_init_duty 电机初始占空比
 * @param empty_frame_delay_us 空帧时的延时(微秒)
 * @param loop_delay_us 主循环延时(微秒)
 * @return 运行成功返回true，初始化失败返回false
 */
bool CarRuntime_RunCameraLoop(bool enable_motor,
                              uint16_t width,
                              uint16_t height,
                              uint16_t fps,
                              int motor_init_duty,
                              uint32_t empty_frame_delay_us,
                              uint32_t loop_delay_us)
{
// 若未启用OpenCV，直接报错并退出
#ifndef LQ_HAVE_OPENCV
    // 忽略未使用的参数，避免编译警告
    (void)enable_motor;
    (void)width;
    (void)height;
    (void)fps;
    (void)motor_init_duty;
    (void)empty_frame_delay_us;
    (void)loop_delay_us;
    // 打印错误日志：未启用OpenCV，无法运行小车逻辑
    lq_log_error("OpenCV not enabled, car runtime is unavailable");
    return false;
#else
    // 初始化相机对象，设置分辨率、帧率
    lq_camera_ex cam(width, height, fps);
    // 判断相机是否成功打开
    if (!cam.is_cam_opened())
    {
        // 打印相机打开失败日志
        lq_log_error("Failed to open camera for car runtime");
        return false;
    }

    // 初始化小车运行时（参数+电机+硬件）
    CarRuntime_Init(enable_motor, motor_init_duty);

    // 小车核心无限循环
    while (1)
    {
        // 获取相机原始帧并处理
        if (!CarRuntime_ProcessFrame(cam.get_frame_raw(), enable_motor))
        {
            // 帧处理失败（空帧），延时后继续循环
            usleep(empty_frame_delay_us);
            continue;
        }

        // 单帧处理完成，延时控制循环频率
        usleep(loop_delay_us);
    }

    // 理论上无限循环，不会执行到此处
    return true;
#endif
}