/*********************************************************************************************************************
 * Seekfree 协议 TCP 适配层（简化版）
 * 直接在类内部实现协议，不使用静态回调
 ********************************************************************************************************************/

#include "seekfree_tcp_adapter.h"
#include <stdio.h>
#include <string.h>

SeekfreeTcpClient::SeekfreeTcpClient(void)
    : TcpClient()
    , rx_head(0)
    , rx_tail(0)
    , rx_count(0)
{
    memset(parameter_values, 0, sizeof(parameter_values));
    memset((void*)parameter_update_flags, 0, sizeof(parameter_update_flags));
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

SeekfreeTcpClient::~SeekfreeTcpClient(void)
{
}

void SeekfreeTcpClient::init_protocol(void)
{
    printf("[Seekfree] Protocol initialized (direct mode)\n");
}

/*******************************************************************
 * @brief       发送示波器数据（直接实现，不使用回调）
 ******************************************************************/
void SeekfreeTcpClient::send_oscilloscope(float ch1, float ch2, float ch3, float ch4,
                                         float ch5, float ch6, float ch7, float ch8)
{
    if (!is_connected()) {
        static int warn_count = 0;
        if (warn_count++ % 100 == 0) {
            printf("[Seekfree] Warning: Not connected! (count=%d)\n", warn_count);
        }
        return;
    }

    // 构建示波器数据包
    protocol_oscilloscope_t osc;
    memset(&osc, 0, sizeof(osc));  // 先清零

    uint8_t channel_count = 8;  // 发送3个通道

    osc.head = 0xAA;  // PROTOCOL_SEND_HEAD
    osc.channel_num = (0x10 | channel_count);  // 功能字0x10 + 通道数
    osc.length = 4 + channel_count * 4;
    osc.data[0] = ch1;
    osc.data[1] = ch2;
    osc.data[2] = ch3;
    osc.data[3] = ch4;
    osc.data[4] = ch5;
    osc.data[5] = ch6;
    osc.data[6] = ch7;
    osc.data[7] = ch8;

    // 计算校验和（注意：check_sum 字段本身不参与计算）
    osc.check_sum = 0;
    uint8_t *p = (uint8_t*)&osc;
    uint8_t sum = 0;
    sum += p[0];  // head
    sum += p[1];  // channel_num
    // skip p[2] (check_sum itself)
    sum += p[3];  // length
    // 加上数据部分
    for (int i = 4; i < osc.length; i++) {
        sum += p[i];
    }
    osc.check_sum = sum;

    // 直接发送
    bool success = send_bytes(&osc, osc.length);

    // 调试输出（包含十六进制数据）
    static int send_count = 0;
    if (send_count++ % 100 == 0) {
        // printf("[Seekfree] Sent packet #%d: ch1=%.2f, ch2=%.2f, ch3=%.2f, success=%d, bytes=%d\n",
        //        send_count, ch1, ch2, ch3, success, osc.length);

        // 打印前16字节的十六进制数据
        // printf("[Seekfree] HEX: ");
        for (int i = 0; i < 16 && i < osc.length; i++) {
            // printf("%02X ", p[i]);
        }
        // printf("\n");
    }
}

/*******************************************************************
 * @brief       发送灰度图像和边线
 ******************************************************************/
void SeekfreeTcpClient::send_camera_gray(uint8_t *image, uint16_t width, uint16_t height,
                                        uint8_t *left_line, uint8_t *center_line, uint8_t *right_line)
{
    if (!is_connected() || image == nullptr) {
        return;
    }

    // TODO: 实现图像发送
    // 这里暂时不实现，因为主要问题是示波器数据
}

void SeekfreeTcpClient::process_parameters(void)
{
    if (!is_connected()) {
        return;
    }

    // 从 TCP 接收数据到缓冲区
    uint8_t temp[PROTOCOL_RX_BUFFER_SIZE];
    int received = recv_bytes(temp, sizeof(temp), 0);  // 非阻塞接收
    if (received > 0) {
        fifo_push(temp, received);
    }

    // 逐包解析
    while (rx_count >= sizeof(protocol_parameter_t))
    {
        uint8_t pkt[sizeof(protocol_parameter_t)];
        fifo_peek(pkt, sizeof(protocol_parameter_t));

        if (pkt[0] != PROTOCOL_RECV_HEAD)
        {
            // 不是帧头，丢弃 1 字节
            fifo_discard(1);
            continue;
        }

        // 校验和验证
        protocol_parameter_t *p = (protocol_parameter_t *)pkt;
        uint8_t saved_sum = p->check_sum;
        p->check_sum = 0;
        uint8_t calc_sum = checksum(pkt, sizeof(protocol_parameter_t));

        if (saved_sum == calc_sum)
        {
            // 校验通过
            if (p->channel >= 1 && p->channel <= PROTOCOL_PARAMETER_CH_MAX)
            {
                parameter_values[p->channel - 1] = p->data;
                parameter_update_flags[p->channel - 1] = 1;
                printf("[Seekfree] Received parameter: channel=%d, value=%.2f\n",
                       p->channel, p->data);
            }
            fifo_discard(sizeof(protocol_parameter_t));
        }
        else
        {
            // 校验失败，丢弃 1 字节
            fifo_discard(1);
        }
    }
}

float SeekfreeTcpClient::get_parameter(uint8_t channel)
{
    if (channel >= 1 && channel <= PROTOCOL_PARAMETER_CH_MAX) {
        return parameter_values[channel - 1];
    }
    return 0.0f;
}

bool SeekfreeTcpClient::is_parameter_updated(uint8_t channel)
{
    if (channel >= 1 && channel <= PROTOCOL_PARAMETER_CH_MAX) {
        return parameter_update_flags[channel - 1] != 0;
    }
    return false;
}

void SeekfreeTcpClient::clear_parameter_flag(uint8_t channel)
{
    if (channel >= 1 && channel <= PROTOCOL_PARAMETER_CH_MAX) {
        parameter_update_flags[channel - 1] = 0;
    }
}

/*******************************************************************
 * 内部辅助函数：环形缓冲区操作
 ******************************************************************/
void SeekfreeTcpClient::fifo_push(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len && rx_count < PROTOCOL_RX_BUFFER_SIZE; i++)
    {
        rx_buffer[rx_tail] = data[i];
        rx_tail = (rx_tail + 1) % PROTOCOL_RX_BUFFER_SIZE;
        rx_count++;
    }
}

uint32_t SeekfreeTcpClient::fifo_peek(uint8_t *out, uint32_t len)
{
    if (len > rx_count) len = rx_count;
    uint32_t pos = rx_head;
    for (uint32_t i = 0; i < len; i++)
    {
        out[i] = rx_buffer[pos];
        pos = (pos + 1) % PROTOCOL_RX_BUFFER_SIZE;
    }
    return len;
}

void SeekfreeTcpClient::fifo_discard(uint32_t len)
{
    if (len > rx_count) len = rx_count;
    rx_head = (rx_head + len) % PROTOCOL_RX_BUFFER_SIZE;
    rx_count -= len;
}

uint8_t SeekfreeTcpClient::checksum(uint8_t *buf, uint32_t len)
{
    uint8_t sum = 0;
    for (uint32_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return sum;
}
