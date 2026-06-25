#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 摄像头服务器模块
 * 版权所有 (c) 2025 Blockingsys
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
 * 文件名称：ww_camera_server.h
 * 所属模块：wuwu_library
 * 功能描述：摄像头服务器（HTTP/MJPEG）头文件
 *
 * 修改记录：
 * 日期         作者            说明
 * 2025-12-16  Blockingsys    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#ifndef __CAMERA_SERVER_H__
#define __CAMERA_SERVER_H__

#include "headfile.h"

// 默认端口号
#define CAMERA_STREAM_DEFAULT_PORT 8080

enum CameraTransportMode
{
    CAMERA_TRANSPORT_ALL = 0,        // 图像 + 参数
    CAMERA_TRANSPORT_IMAGE_ONLY = 1, // 只传图像
    CAMERA_TRANSPORT_TELEMETRY_ONLY = 2 // 只传参数
};

struct CameraTelemetry
{
    double target_speed_l = 0.0;
    double target_speed_r = 0.0;
    double actual_speed_l = 0.0;
    double actual_speed_r = 0.0;
    double pwm_l = 0.0;
    double pwm_r = 0.0;
    double target_gyro = 0.0;
    double actual_gyro = 0.0;
    double pure_angle = 0.0;

    double pid_speed_l_kp = 0.0;
    double pid_speed_l_ki = 0.0;
    double pid_speed_l_kd = 0.0;
    double pid_speed_r_kp = 0.0;
    double pid_speed_r_ki = 0.0;
    double pid_speed_r_kd = 0.0;
    double pid_gyro_kp = 0.0;
    double pid_gyro_ki = 0.0;
    double pid_gyro_kd = 0.0;
    double pid_angle_kp = 0.0;
    double pid_angle_ki = 0.0;
    double pid_angle_kd = 0.0;
    double pid_circle_kp = 0.0;
    double pid_circle_ki = 0.0;
    double pid_circle_kd = 0.0;

    int start_flag = 0;
    int look_ahead_point = 0;
    int track_side = 0;
    int elem_type = 0;
    int cross_state = 0;
    int circle_state = 0;
    int circle_type = 0;

    int ipts0_num = 0;
    int ipts1_num = 0;
    int rptsn_num = 0;
    int rpts0s_num = 0;
    int rpts1s_num = 0;
    int far_rpts0s_num = 0;
    int far_rpts1s_num = 0;

    int lpt0_found = 0;
    int lpt1_found = 0;
    int lpt0_id = 0;
    int lpt1_id = 0;
    double lpt0_x = 0.0;
    double lpt0_y = 0.0;
    double lpt1_x = 0.0;
    double lpt1_y = 0.0;
    int far_lpt0_found = 0;
    int far_lpt1_found = 0;
    int far_lpt0_id = 0;
    int far_lpt1_id = 0;
    double far_lpt0_x = 0.0;
    double far_lpt0_y = 0.0;
    double far_lpt1_x = 0.0;
    double far_lpt1_y = 0.0;

    std::string elem_name;
    std::string cross_name;
    std::string circle_name;
    std::string track_name;
    std::string circle_type_name;
    std::string classify_label;
    double classify_score = 0.0;
    std::string classify_probs_text;
    std::string ncnn_action_state_name;
    std::string ncnn_action_cmd_name;
    int ncnn_classify_stable_count = 0;
    int ncnn_red_center_y = -1;
    int ncnn_action_encoder_l = 0;
    int ncnn_action_encoder_r = 0;
    int ncnn_high_level_frozen = 0;
    int red_detected_this_frame = 0;
    int red_waiting_distance = 0;
    int red_cycle_done = 0;
    int red_classify_enabled = 0;
    int red_model_input_ready = 0;
    double red_estimated_distance_cm = 0.0;
    double red_trigger_distance_cm = 0.0;
    double red_pixels_per_cm = 0.0;
    int red_box_x = 0;
    int red_box_y = 0;
    int red_box_w = 0;
    int red_box_h = 0;
    int red_model_box_x = 0;
    int red_model_box_y = 0;
    int red_model_box_w = 0;
    int red_model_box_h = 0;
};

class CameraStreamServer
{
public:
    CameraStreamServer(void);
    ~CameraStreamServer(void);

/*******************************************************************
 * @brief       启动摄像头图传服务器
 * 
 * @param       port            服务器监听端口(默认8080)
 * 
 * @return      返回启动状态
 * @retval      0               启动成功
 * @retval      -1              启动失败
 * 
 * @example     //启动摄像头图传服务器
 *              if(camera_server.start_server(8080) < 0) {
 *                  return -1;
 *              }
 * 
 * @note        在后台线程中启动HTTP服务器，支持浏览器访问
 *              访问 http://<开发板IP>:<port> 即可查看实时画面
 ******************************************************************/
    int start_server(int port = CAMERA_STREAM_DEFAULT_PORT);

/*******************************************************************
 * @brief       更新摄像头帧数据
 * 
 * @param       frame           OpenCV Mat格式的图像帧
 * 
 * @example     camera_server.update_frame(frame);
 * 
 * @note        将最新的摄像头帧推送到服务器，供客户端获取
 *              自动编码为JPEG格式并计算帧率
 ******************************************************************/
    void update_frame(const cv::Mat& frame);
    void update_frame(const cv::Mat& frame, const cv::Mat& perspective_frame, const CameraTelemetry& telemetry);
    void set_transport_mode(CameraTransportMode mode);
    CameraTransportMode get_transport_mode(void);

/*******************************************************************
 * @brief       停止摄像头图传服务器
 * 
 * @example     camera_server.stop_server();
 * 
 * @note        停止服务器并释放所有资源
 ******************************************************************/
    void stop_server(void);

/*******************************************************************
 * @brief       检查服务器是否正在运行
 * 
 * @return      返回服务器运行状态
 * @retval      true            服务器正在运行
 * @retval      false           服务器已停止
 * 
 * @example     if(camera_server.is_running()) {
 *                  //服务器正在运行
 *              }
 ******************************************************************/
    bool is_running(void);

private:
    // 服务器socket文件描述符
    int server_sock_fd;
    // 服务器端口
    int server_port;
    // 服务器运行状态
    bool running;
    // 服务器线程ID
    pthread_t server_thread_id;
    
    // 帧数据互斥锁
    pthread_mutex_t frame_mutex;
    // 帧数据条件变量
    pthread_cond_t frame_cond;
    // socket互斥锁
    pthread_mutex_t sock_mutex;
    pthread_mutex_t telemetry_mutex;
    pthread_mutex_t mode_mutex;
    
    // 当前JPEG数据
    std::vector<unsigned char> current_jpeg;
    std::vector<unsigned char> current_perspective_jpeg;
    std::vector<unsigned char> current_crop_jpeg;
    // 最新帧ID
    uint64_t latest_frame_id;
    // 最新捕获时间戳(毫秒)
    uint64_t latest_capture_ts_ms;
    // EMA帧率
    double ema_fps;
    
    // 原始帧数据（用于保存高质量图片）
    cv::Mat original_frame;
    cv::Mat crop_frame;
    // 原始帧互斥锁
    pthread_mutex_t original_frame_mutex;
    // 调试数据
    CameraTelemetry telemetry;
    CameraTransportMode transport_mode;
    
    // 内部方法
    std::string get_local_ip(void);
    void close_server_socket(void);
    uint64_t now_ms(void);
    std::string format_timestamp(uint64_t ts_ms);
    void send_response(int sock, const char* content_type, const char* body, size_t body_len);
    void send_stats_response(int sock);
    void send_mjpeg_stream(int sock, bool perspective, bool crop);
    void handle_control_request(int sock, const std::string& action);
    void handle_pid_request(int sock, const std::string& path);
    void handle_transport_mode_request(int sock, const std::string& path);
    void handle_client_request(int sock);
    void handle_snapshot_request(int sock, const std::string& prefix, bool crop);
    
    // 静态线程函数
    static void* server_thread_func(void* arg);
    static void* client_thread_func(void* arg);
    static void signal_handler(int sig);
    
    // 全局实例指针(用于信号处理)
    static CameraStreamServer* instance;
};

#endif
