#include "app_context.h"
#include "app_tasks.h"

// main 只负责三件事：
// 1. 初始化硬件和通信；
// 2. 启动周期任务线程；
// 3. 主循环取图并调用视觉任务。
// 具体控制逻辑已经拆到各自任务文件，后续不要再把业务逻辑堆回 main。

int main()
{
    Menu menu(&lcd);
    g_menu = &menu;

    // 硬件初始化：保持原有顺序，避免拆分后引入启动时序问题。
    if (!camera.is_cam_opened()) { fprintf(stderr, "摄像头初始化失败\n"); return -1; }
    if (lcd.lcd_init() < 0) { fprintf(stderr, "初始化失败\n"); return -1; }
    if (key.key_init() < 0) { fprintf(stderr, "KEY初始化失败\n"); return -1; }
    if (motor.motor_init() < 0) { fprintf(stderr, "电机初始化失败\n"); return -1; }
    if (brush.brushless_init() < 0) { fprintf(stderr, "无刷电机初始化失败\n"); return -1; }
    // if (buzzer_1.buzzer_init() < 0) { fprintf(stderr, "蜂鸣器初始化失败\n"); return -1; }
    if (imu.icm42688_init() < 0) { fprintf(stderr, "IMU初始化失败\n"); return -1; }
    if (seekfree_client.connect_server("192.168.179.64", 8086) < 0) {
    // if (seekfree_client.connect_server("192.168.195.64", 8086) < 0) {
        printf("连接失败\n");
        return -1;
    }

    // buzzer_1.set_duty_freq(100, 50);
    lcd.clearScreen();
    seekfree_client.init_protocol();

    // 周期任务拆分后，主函数只负责创建和启动线程。
    TimerThread imu_tof_thread(imu_task, nullptr, 4);
    TimerThread motor_thread(motor_task, nullptr, 1);
    TimerThread key_thread(key_task, nullptr, 10);
    TimerThread menu_thread(menu_task, nullptr, 100);
    TimerThread encoder_thread(encoder_get_task, nullptr, 5);
    TimerThread recv_thread(recv_task, nullptr, 10);
    motor_thread.start();
    key_thread.start();
    // menu_thread.start();
    imu_tof_thread.start();
    encoder_thread.start();
    recv_thread.start();

    // 巡线透视参数初始化。
    camera_param_init();
    menu.refresh();
    // brush.set_duty(0);

    if (camera_server.start_server(8080) < 0) {
        printf("图传服务器启动失败\n");
        return -1;
    }
    printf("图传服务器已启动，浏览器访问 http://192.168.250.227:8080 查看画面\n");
    while (camera_server.is_running()) {
        if (!camera.get_frame(frame)) {
            usleep(10000);
            continue;
        }

        // 视觉任务单独成文件后，主循环只保留取帧和调用。
        photo_task();
        if(param_photo_flag)
        camera_server.update_frame(frame, perspective_frame, camera_telemetry);
    }
    // sleep(0.001);
    printf("\n已退出\n");
    return 0;
}
