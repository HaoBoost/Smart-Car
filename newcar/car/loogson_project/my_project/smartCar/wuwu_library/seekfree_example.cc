/*********************************************************************************************************************
 * Seekfree 协议使用示例
 * 演示如何使用 SeekfreeTcpClient 发送图像、示波器数据和接收参数
 ********************************************************************************************************************/

#include "seekfree_tcp_adapter.h"
#include <stdio.h>
#include <unistd.h>

// 静态 Seekfree TCP 客户端实例（仅在本文件中可见，避免重复定义）
static SeekfreeTcpClient seekfree_client;

/*******************************************************************
 * @brief       初始化 Seekfree TCP 连接
 * @param       ip    服务器IP地址
 * @param       port  服务器端口号
 * @return      0=成功, -1=失败
 ******************************************************************/
int seekfree_init(const char* ip, int port)
{
    // 连接到 Seekfree 上位机
    if (seekfree_client.connect_server(ip, port) < 0)
    {
        printf("[Seekfree] Failed to connect to %s:%d\\n", ip, port);
        return -1;
    }

    // 初始化协议
    seekfree_client.init_protocol();

    printf("[Seekfree] Connected to %s:%d\\n", ip, port);
    return 0;
}

/*******************************************************************
 * @brief       发送示波器数据示例
 ******************************************************************/
void seekfree_send_oscilloscope_example(void)
{
    // 示例：发送 3 个通道的数据
    float speed = 1.5f;
    float angle = 30.0f;
    float error = 0.2f;

    seekfree_client.send_oscilloscope(speed, angle, error);
}

/*******************************************************************
 * @brief       发送图像和边线示例
 * @param       image       图像数据（灰度图）
 * @param       width       图像宽度
 * @param       height      图像高度
 * @param       left_line   左边线数组
 * @param       center_line 中线数组
 * @param       right_line  右边线数组
 ******************************************************************/
void seekfree_send_camera_example(uint8_t *image, uint16_t width, uint16_t height,
                                  uint8_t *left_line, uint8_t *center_line, uint8_t *right_line)
{
    seekfree_client.send_camera_gray(image, width, height, left_line, center_line, right_line);
}

/*******************************************************************
 * @brief       接收参数示例（在主循环中调用）
 ******************************************************************/
void seekfree_process_parameters_example(void)
{
    // 周期调用解析函数
    seekfree_client.process_parameters();

    // 检查参数是否更新
    for (int i = 1; i <= 8; i++)
    {
        if (seekfree_client.is_parameter_updated(i))
        {
            float value = seekfree_client.get_parameter(i);
            printf("[Seekfree] Parameter %d updated: %.2f\\n", i, value);

            // 清除更新标志
            seekfree_client.clear_parameter_flag(i);

            // TODO: 使用参数值，例如更新 PID 参数
            // if (i == 1) pid_kp = value;
            // if (i == 2) pid_ki = value;
            // if (i == 3) pid_kd = value;
        }
    }
}

/*******************************************************************
 * @brief       完整使用示例
 ******************************************************************/
void seekfree_usage_example(void)
{
    // 1. 初始化连接
    if (seekfree_init("192.168.2.10", 2233) < 0)
    {
        return;
    }

    // 2. 模拟图像数据
    uint8_t image[120][188];
    uint8_t left_line[120];
    uint8_t center_line[120];
    uint8_t right_line[120];

    // 填充测试数据
    for (int i = 0; i < 120; i++)
    {
        left_line[i] = 30;
        center_line[i] = 94;
        right_line[i] = 158;
    }

    // 3. 主循环
    while (seekfree_client.is_connected())
    {
        // 发送图像和边线
        seekfree_send_camera_example((uint8_t*)image, 188, 120,
                                    left_line, center_line, right_line);

        // 发送示波器数据
        seekfree_send_oscilloscope_example();

        // 接收参数
        seekfree_process_parameters_example();

        // 延时
        usleep(50000);  // 50ms
    }

    // 4. 断开连接
    seekfree_client.disconnect_server();
}
