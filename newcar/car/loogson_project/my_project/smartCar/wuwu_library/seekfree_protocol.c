#include <string.h>
#include "seekfree_protocol.h"

/*===========================================================================
 * 内部变量
 *===========================================================================*/
static protocol_send_func_t  g_send = NULL;
static protocol_recv_func_t  g_recv = NULL;

static protocol_camera_header_t      g_camera_header;
static protocol_camera_dot_header_t  g_dot_header;
static protocol_camera_buffer_t      g_camera_buf;

// 接收环形缓冲区（简易 FIFO）
static uint8  g_rx_fifo[PROTOCOL_RX_BUFFER_SIZE];
static uint32 g_rx_head = 0;
static uint32 g_rx_tail = 0;
static uint32 g_rx_count = 0;

/*===========================================================================
 * 全局变量
 *===========================================================================*/
protocol_oscilloscope_t  protocol_oscilloscope_data;
float                    protocol_parameter[PROTOCOL_PARAMETER_CH_MAX]        = {0};
vuint8                   protocol_parameter_update_flag[PROTOCOL_PARAMETER_CH_MAX] = {0};

/*===========================================================================
 * 内部工具函数
 *===========================================================================*/

// 字节累加和
static uint8 checksum(uint8 *buf, uint32 len)
{
    uint8 sum = 0;
    while (len--)
        sum += *buf++;
    return sum;
}

// FIFO 写入
static void fifo_push(const uint8 *data, uint32 len)
{
    for (uint32 i = 0; i < len && g_rx_count < PROTOCOL_RX_BUFFER_SIZE; i++)
    {
        g_rx_fifo[g_rx_tail] = data[i];
        g_rx_tail = (g_rx_tail + 1) % PROTOCOL_RX_BUFFER_SIZE;
        g_rx_count++;
    }
}

// FIFO 窥视（不移除）
static uint32 fifo_peek(uint8 *out, uint32 len)
{
    if (len > g_rx_count) len = g_rx_count;
    uint32 pos = g_rx_head;
    for (uint32 i = 0; i < len; i++)
    {
        out[i] = g_rx_fifo[pos];
        pos = (pos + 1) % PROTOCOL_RX_BUFFER_SIZE;
    }
    return len;
}

// FIFO 丢弃
static void fifo_discard(uint32 len)
{
    if (len > g_rx_count) len = g_rx_count;
    g_rx_head = (g_rx_head + len) % PROTOCOL_RX_BUFFER_SIZE;
    g_rx_count -= len;
}

/*===========================================================================
 * API 实现
 *===========================================================================*/

void protocol_init(protocol_send_func_t send_func, protocol_recv_func_t recv_func)
{
    g_send = send_func;
    g_recv = recv_func;
}

// ----------------------------- 示波器发送 ---------------------------------
void protocol_oscilloscope_send(protocol_oscilloscope_t *osc)
{
    uint8 packet_size;

    osc->channel_num &= 0x0F;
    osc->head = PROTOCOL_SEND_HEAD;

    packet_size = 4 + osc->channel_num * 4;
    osc->length = packet_size;

    osc->channel_num |= PROTOCOL_FUNC_OSCILLOSCOPE;

    osc->check_sum = 0;
    osc->check_sum = checksum((uint8 *)osc, packet_size);

    if (g_send)
        g_send((const uint8 *)osc, packet_size);
}

// ----------------------------- 图像配置 -----------------------------------
void protocol_camera_config(protocol_image_type_e type, void *image_addr,
                            uint16 width, uint16 height)
{
    g_dot_header.head     = PROTOCOL_SEND_HEAD;
    g_dot_header.function = PROTOCOL_FUNC_CAMERA_DOT;
    g_dot_header.length   = sizeof(protocol_camera_dot_header_t);

    g_camera_buf.camera_type = type;
    g_camera_buf.image_addr  = image_addr;
    g_camera_buf.width       = width;
    g_camera_buf.height      = height;
}

// ----------------------------- 边线配置 -----------------------------------
void protocol_camera_boundary_config(protocol_boundary_type_e type, uint16 dot_num,
                                     void *x1, void *x2, void *x3,
                                     void *y1, void *y2, void *y3)
{
    uint8 i = 0;
    uint8 boundary_num = 0;
    uint8 boundary_data_type = 0;

    g_dot_header.dot_num    = dot_num;
    g_dot_header.valid_flag = 0;
    memset(g_camera_buf.boundary_x, 0, sizeof(g_camera_buf.boundary_x));
    memset(g_camera_buf.boundary_y, 0, sizeof(g_camera_buf.boundary_y));

    switch (type)
    {
    case BOUNDARY_X_ONLY:
        if (x1) { boundary_num++; g_dot_header.valid_flag |= (1<<0); g_camera_buf.boundary_x[i++] = x1; }
        if (x2) { boundary_num++; g_dot_header.valid_flag |= (1<<1); g_camera_buf.boundary_x[i++] = x2; }
        if (x3) { boundary_num++; g_dot_header.valid_flag |= (1<<2); g_camera_buf.boundary_x[i++] = x3; }
        if (g_camera_buf.height > 255) boundary_data_type = 1;
        break;

    case BOUNDARY_Y_ONLY:
        if (y1) { boundary_num++; g_dot_header.valid_flag |= (1<<0); g_camera_buf.boundary_y[i++] = y1; }
        if (y2) { boundary_num++; g_dot_header.valid_flag |= (1<<1); g_camera_buf.boundary_y[i++] = y2; }
        if (y3) { boundary_num++; g_dot_header.valid_flag |= (1<<2); g_camera_buf.boundary_y[i++] = y3; }
        if (g_camera_buf.width > 255) boundary_data_type = 1;
        break;

    case BOUNDARY_XY:
        if (x1 && y1) { boundary_num++; g_dot_header.valid_flag |= (1<<0); g_camera_buf.boundary_x[i] = x1; g_camera_buf.boundary_y[i++] = y1; }
        if (x2 && y2) { boundary_num++; g_dot_header.valid_flag |= (1<<1); g_camera_buf.boundary_x[i] = x2; g_camera_buf.boundary_y[i++] = y2; }
        if (x3 && y3) { boundary_num++; g_dot_header.valid_flag |= (1<<2); g_camera_buf.boundary_x[i] = x3; g_camera_buf.boundary_y[i++] = y3; }
        if (g_camera_buf.width > 255 || g_camera_buf.height > 255) boundary_data_type = 1;
        break;

    case BOUNDARY_NONE:
        break;
    }

    g_dot_header.dot_type = (type << 6) | (boundary_data_type << 5) | boundary_num;
}

// ----------------------------- 图像发送 -----------------------------------
static void camera_image_send(void)
{
    uint32 image_size = 0;

    g_camera_header.head         = PROTOCOL_SEND_HEAD;
    g_camera_header.function     = PROTOCOL_FUNC_CAMERA;
    g_camera_header.camera_type  = (g_camera_buf.camera_type << 5)
                                 | ((g_camera_buf.image_addr ? 0 : 1) << 4)
                                 | (g_dot_header.dot_type & 0x0F);
    g_camera_header.length       = sizeof(protocol_camera_header_t);
    g_camera_header.image_width  = g_camera_buf.width;
    g_camera_header.image_height = g_camera_buf.height;

    if (g_send)
        g_send((const uint8 *)&g_camera_header, sizeof(protocol_camera_header_t));

    switch (g_camera_buf.camera_type)
    {
        case IMAGE_TYPE_BINARY: image_size = g_camera_buf.width * g_camera_buf.height / 8; break;
        case IMAGE_TYPE_GRAY:   image_size = g_camera_buf.width * g_camera_buf.height;     break;
        case IMAGE_TYPE_RGB565: image_size = g_camera_buf.width * g_camera_buf.height * 2; break;
    }

    if (g_camera_buf.image_addr && g_send)
        g_send((const uint8 *)g_camera_buf.image_addr, image_size);
}

static void camera_dot_send(void)
{
    uint8  i;
    uint16 dot_bytes = g_dot_header.dot_num;

    if (g_dot_header.dot_type & (1 << 5))
        dot_bytes *= 2;

    if (g_send)
        g_send((const uint8 *)&g_dot_header, sizeof(protocol_camera_dot_header_t));

    for (i = 0; i < PROTOCOL_BOUNDARY_MAX; i++)
    {
        if (g_camera_buf.boundary_x[i] && g_send)
            g_send((const uint8 *)g_camera_buf.boundary_x[i], dot_bytes);
        if (g_camera_buf.boundary_y[i] && g_send)
            g_send((const uint8 *)g_camera_buf.boundary_y[i], dot_bytes);
    }
}

void protocol_camera_send(void)
{
    camera_image_send();
    if (g_dot_header.dot_type & 0x0F)
        camera_dot_send();
}

// ----------------------------- 接收解析 -----------------------------------
void protocol_data_analysis(void)
{
    uint8 temp[PROTOCOL_RX_BUFFER_SIZE];
    uint32 read_len;

    // 从接收回调读取新数据
    if (g_recv)
    {
        read_len = g_recv(temp, PROTOCOL_RX_BUFFER_SIZE);
        if (read_len)
            fifo_push(temp, read_len);
    }

    // 逐包解析
    while (g_rx_count >= sizeof(protocol_parameter_t))
    {
        uint8 pkt[sizeof(protocol_parameter_t)];
        fifo_peek(pkt, sizeof(protocol_parameter_t));

        if (pkt[0] != PROTOCOL_RECV_HEAD)
        {
            // 不是帧头，丢弃 1 字节
            fifo_discard(1);
            continue;
        }

        // 校验和验证
        protocol_parameter_t *p = (protocol_parameter_t *)pkt;
        uint8 saved_sum = p->check_sum;
        p->check_sum = 0;
        uint8 calc_sum = checksum(pkt, sizeof(protocol_parameter_t));

        if (saved_sum == calc_sum)
        {
            // 校验通过
            if (p->channel >= 1 && p->channel <= PROTOCOL_PARAMETER_CH_MAX)
            {
                protocol_parameter[p->channel - 1] = p->data;
                protocol_parameter_update_flag[p->channel - 1] = 1;
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
