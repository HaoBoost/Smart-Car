/*********************************************************************************************************************
 * Seekfree 协议 TCP 适配层（简化版）
 * 直接在类内部实现协议，不使用静态回调
 ********************************************************************************************************************/

#ifndef __SEEKFREE_TCP_ADAPTER_H__
#define __SEEKFREE_TCP_ADAPTER_H__

#include "ww_tcp_client.h"
#include <stdint.h>

// 协议常量
#define PROTOCOL_SEND_HEAD              0xAA
#define PROTOCOL_RECV_HEAD              0x55
#define PROTOCOL_FUNC_OSCILLOSCOPE      0x10
#define PROTOCOL_FUNC_SET_PARAMETER     0x20
#define PROTOCOL_OSCILLOSCOPE_CH_MAX    8
#define PROTOCOL_PARAMETER_CH_MAX       8
#define PROTOCOL_RX_BUFFER_SIZE         128

// 示波器数据包结构
#pragma pack(push, 1)
typedef struct
{
    uint8_t  head;
    uint8_t  channel_num;
    uint8_t  check_sum;
    uint8_t  length;
    float    data[PROTOCOL_OSCILLOSCOPE_CH_MAX];
} protocol_oscilloscope_t;

// 参数设置包（接收，固定8字节）
typedef struct
{
    uint8_t  head;                                    // 0x55
    uint8_t  function;                                // 0x20
    uint8_t  channel;                                 // 通道号 1~8
    uint8_t  check_sum;                               // 字节累加和
    float    data;                                    // 参数值
} protocol_parameter_t;
#pragma pack(pop)

/*******************************************************************
 * SeekfreeTcpClient: Seekfree 协议 TCP 客户端（简化版）
 ******************************************************************/
class SeekfreeTcpClient : public TcpClient
{
public:
    SeekfreeTcpClient(void);
    ~SeekfreeTcpClient(void);

    // 初始化协议
    void init_protocol(void);

    // 发送示波器数据
    void send_oscilloscope(float ch1, float ch2 = 0, float ch3 = 0, float ch4 = 0,
                          float ch5 = 0, float ch6 = 0, float ch7 = 0, float ch8 = 0);

    // 发送图像（暂未实现）
    void send_camera_gray(uint8_t *image, uint16_t width, uint16_t height,
                         uint8_t *left_line = nullptr,
                         uint8_t *center_line = nullptr,
                         uint8_t *right_line = nullptr);

    // 参数接收相关
    void process_parameters(void);                    // 周期调用，接收并解析参数
    float get_parameter(uint8_t channel);             // 获取参数值（通道 1~8）
    bool is_parameter_updated(uint8_t channel);       // 检查参数是否更新
    void clear_parameter_flag(uint8_t channel);       // 清除参数更新标志

private:
    // 参数存储
    float parameter_values[PROTOCOL_PARAMETER_CH_MAX];
    volatile uint8_t parameter_update_flags[PROTOCOL_PARAMETER_CH_MAX];

    // 接收缓冲区（环形缓冲区）
    uint8_t rx_buffer[PROTOCOL_RX_BUFFER_SIZE];
    uint32_t rx_head;
    uint32_t rx_tail;
    uint32_t rx_count;

    // 内部辅助函数
    void fifo_push(const uint8_t *data, uint32_t len);
    uint32_t fifo_peek(uint8_t *out, uint32_t len);
    void fifo_discard(uint32_t len);
    uint8_t checksum(uint8_t *buf, uint32_t len);
};

#endif
