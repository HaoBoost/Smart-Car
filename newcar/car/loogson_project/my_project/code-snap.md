# code-snap

## 顶层入口
- `int main()`
  初始化硬件、启动周期线程、主循环取帧与图传。

## 主要类型
- `pid_positional_t`
- `pid_incremental_t`
- `elem_type_t`
- `cross_state_t`
- `circle_state_t`
- `ncnn_action_state_t`
- `ncnn_action_cmd_t`
- `protocol_oscilloscope_t`
- `protocol_parameter_t`
- `CameraTelemetry`
- `CameraTransportMode`
- `MenuNode`
- `MenuValueType`
- `MenuNodeType`

## 全局宏/固定基线
- 图像：`IMG_W=160` `IMG_H=120` `POINTS_MAX=512`
- 采样：`SAMPLE_DIST=3.0f` `ANGLE_DIST_N=8`
- 巡线：`LOOK_AHEAD_POINT_EN=10` `ROAD_WIDTH_PX=21.6f` `WHEELBASE_PX=16.0f`
- 图传端口：`CAMERA_STREAM_DEFAULT_PORT=8080`
- Seekfree 协议：`PROTOCOL_SEND_HEAD=0xAA` `PROTOCOL_RECV_HEAD=0x55`

## 全局对象
- `Menu* g_menu`
- `bool g_menu_mode`
- `lq_camera_ex camera`
- `CameraStreamServer camera_server`
- `ICM42688 imu`
- `VL53L0X tof`
- `Key key`
- `LCD lcd`
- `Motor motor`
- `Buzzer buzzer_1`
- `Brushless brush`
- `VofaClient vofa_tcp_client`
- `SeekfreeTcpClient seekfree_client`

## 关键全局状态
- 图像：`frame` `frame_full` `perspective_frame` `gray` `g_red_model_input_40`
- 巡线：`look_ahead_point` `g_ipts0/1` `g_rpts0s/1s` `g_rptsc` `g_rptsn`
- 角点/中线：`g_Lpt0_found` `g_Lpt1_found` `g_Lpt0_id` `g_Lpt1_id`
- 运行：`start_flag` `__1000ms` `start__1000ms_flag` `g_launch_ramp_elapsed_ms` `g_run_elapsed_ms`
- 控制调试：`g_target_gyro_dbg` `g_actual_gyro_dbg` `g_speed_l_dbg` `g_speed_r_dbg` `g_pwm_l_dbg` `g_pwm_r_dbg`
- 元素：`g_elem_type` `g_cross_state` `g_circle_state` `g_circle_type` `g_zebra_found`
- NCNN：`g_ncnn_action_state` `g_ncnn_locked_cmd` `g_ncnn_locked_label` `g_ncnn_locked_score`

## 关键配置变量
- 速度：`base_speed_target_l` `base_speed_target_r` `target_v`
- 发车：`g_launch_delay_ms` `g_launch_ramp_ms` `g_launch_start_ratio` `g_launch_turn_ratio`
- 前馈：`g_curve_ff_enable` `g_curve_ff_far_offset` `g_curve_ff_k_delta` `g_curve_ff_limit`
- 前瞻：`g_line_lookahead_straight` `g_line_lookahead_mid` `g_line_lookahead_turn`
- 限幅：`g_diff_output_limit` `g_target_speed_limit` `g_pwm_output_min` `g_pwm_output_max`
- 元素：`g_Lpt0_offset_x` `g_Lpt0_offset_y` `g_Lpt1_offset_x` `g_Lpt1_offset_y` `far_start_thres`
- NCNN：`g_ncnn_trigger_center_y` `g_ncnn_trigger_confirm_frames` `g_ncnn_classify_stable_frames`
- PID：`pid_speed_1` `pid_speed_r` `pid_angle_v` `pid_angle_v_circle` `pid_angle` `pid_angle_circle`

## app 层函数
- `void motor_task(void* arg)`
- `void photo_task()`
- `void key_task(void* arg)`
- `void imu_task(void* arg)`
- `void encoder_get_task(void* arg)`
- `void recv_task(void* arg)`
- `void menu_task(void* arg)`
- `void reset_pid_state(pid_positional_t& pid)`
- `void reset_pid_state(pid_incremental_t& pid)`
- `bool update_pid_params(const std::string& group, float kp, float ki, float kd, std::string& message)`
- `void reset_element_state(void)`
- `int calc_speed_related_look_ahead_point(float vehicle_speed)`
- `void stop_quick()`
- `bool run_red_model_classification_if_needed(void)`
- `const char* ncnn_action_state_name(ncnn_action_state_t state)`
- `const char* ncnn_action_cmd_name(ncnn_action_cmd_t cmd)`

## 视觉/算法骨架
- `camera_param_init() -> void`
- `persp_transform(float px, float py, float& ox, float& oy) -> void`
- `normalize_centerline() -> void`
- `process_image(const cv::Mat& frame) -> void`
- `calc_pure_pursuit() -> void`
- `look_ahead_point_decision() -> void`
- `is_long_straight(...) -> bool`
- `is_curve(...) -> bool`
- `detect_zebra(const cv::Mat& gray) -> bool`
- `inv_persp_transform(float bx, float by, float& ox, float& oy) -> void`

## 分类骨架
- `load_labels(std::vector<std::string>& labels) -> bool`
- `resolve_classifier_paths() -> bool`
- `init_classifier_once() -> bool`
- `argmax_score(const ncnn::Mat& out, float& best_score) -> int`
- `softmax_scores(const ncnn::Mat& out) -> std::vector<float>`
- `format_probabilities(const std::vector<float>& probs) -> std::string`
- `run_inference_with_pixel_type(int pixel_type, ncnn::Mat& out) -> bool`
- `run_red_model_classification_if_needed() -> bool`

## 设备/通信接口骨架
- `Motor::motor_init() -> int`
- `Motor::set_motor1(int level, int duty) -> void`
- `Motor::set_motor2(int level, int duty) -> void`
- `Motor::update_encoders() -> void`
- `Motor::thread_syn() -> void`
- `ICM42688::icm42688_init() -> int`
- `ICM42688::upDataGyro() -> void`
- `ICM42688::upDataAcc() -> void`
- `Key::key_init() -> int`
- `Key::key_listeners() -> void`
- `LCD::lcd_init() -> int`
- `LCD::clearScreen() -> void`
- `Brushless::brushless_init() -> int`
- `Brushless::set_duty(int duty) -> void`
- `Buzzer::buzzer_init() -> int`
- `Buzzer::set_duty_freq(int duty_value, int freq_value) -> void`
- `CameraStreamServer::start_server(int port) -> int`
- `CameraStreamServer::update_frame(const cv::Mat& frame) -> void`
- `CameraStreamServer::update_frame(const cv::Mat& frame, const cv::Mat& perspective_frame, const CameraTelemetry& telemetry) -> void`
- `CameraStreamServer::set_transport_mode(CameraTransportMode mode) -> void`
- `CameraStreamServer::is_running() -> bool`
- `TcpClient::connect_server(const char* ip, int port) -> int`
- `TcpClient::send_bytes(const void* data, size_t len) -> bool`
- `SeekfreeTcpClient::init_protocol() -> void`
- `SeekfreeTcpClient::send_oscilloscope(...) -> void`
- `SeekfreeTcpClient::process_parameters() -> void`
- `SeekfreeTcpClient::get_parameter(uint8_t channel) -> float`
- `SeekfreeTcpClient::is_parameter_updated(uint8_t channel) -> bool`
- `Menu::refresh() -> void`
- `Menu::tick() -> void`
- `TimerThread::start() -> bool`
