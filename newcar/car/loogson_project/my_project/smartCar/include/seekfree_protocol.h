#ifndef _SEEKFREE_PROTOCOL_H_
#define _SEEKFREE_PROTOCOL_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 类型别名（适配原工程的 uint8/uint16/uint32/vuint8）
 *===========================================================================*/
typedef uint8_t   uint8;
typedef uint16_t  uint16;
typedef uint32_t  uint32;
typedef volatile uint8_t vuint8;

/*===========================================================================
 * 协议常量
 *===========================================================================*/
#define PROTOCOL_SEND_HEAD              0xAA        // 单片机→上位机 帧头
#define PROTOCOL_RECV_HEAD              0x55        // 上位机→单片机 帧头

#define PROTOCOL_FUNC_CAMERA            0x02        // 图像功能字
#define PROTOCOL_FUNC_CAMERA_DOT        0x03        // 图像边线功能字
#define PROTOCOL_FUNC_OSCILLOSCOPE      0x10        // 示波器功能字
#define PROTOCOL_FUNC_SET_PARAMETER     0x20        // 参数设置功能字

#define PROTOCOL_OSCILLOSCOPE_CH_MAX    8           // 示波器最大通道数
#define PROTOCOL_PARAMETER_CH_MAX       8           // 参数调节最大通道数
#define PROTOCOL_BOUNDARY_MAX           8           // 图像边线最大数量
#define PROTOCOL_RX_BUFFER_SIZE         128         // 接收 FIFO 大小

/*===========================================================================
 * 图像类型枚举
 *===========================================================================*/
typedef enum
{
    IMAGE_TYPE_BINARY = 1,      // 二值化   数据大小: W*H/8
    IMAGE_TYPE_GRAY   = 2,      // 灰度     数据大小: W*H
    IMAGE_TYPE_RGB565 = 3,      // RGB565   数据大小: W*H*2
} protocol_image_type_e;

/*===========================================================================
 * 边界类型枚举
 *===========================================================================*/
typedef enum
{
    BOUNDARY_X_ONLY  = 0,       // 只有横坐标
    BOUNDARY_Y_ONLY  = 1,       // 只有纵坐标
    BOUNDARY_XY      = 2,       // 横纵坐标都有
    BOUNDARY_NONE    = 3,       // 无边线
} protocol_boundary_type_e;

/*===========================================================================
 * 数据包结构体（均为 packed，保证内存布局与协议一致）
 *===========================================================================*/

// 示波器数据包
#pragma pack(push, 1)
typedef struct
{
    uint8  head;                                    // 0xAA
    uint8  channel_num;                             // 高4位=功能字0x10, 低4位=通道数
    uint8  check_sum;                               // 字节累加和
    uint8  length;                                  // 整包长度
    float  data[PROTOCOL_OSCILLOSCOPE_CH_MAX];      // 通道数据
} protocol_oscilloscope_t;

// 图像信息包头（8字节）
typedef struct
{
    uint8  head;                                    // 0xAA
    uint8  function;                                // 0x02
    uint8  camera_type;                             // BIT7-5:图像类型 BIT4:有无图像 BIT3-0:边界数
    uint8  length;                                  // 协议头长度=8
    uint16 image_width;
    uint16 image_height;
} protocol_camera_header_t;

// 图像边线包头（8字节）
typedef struct
{
    uint8  head;                                    // 0xAA
    uint8  function;                                // 0x03
    uint8  dot_type;                                // BIT7-6:坐标类型 BIT5:16/8位 BIT3-0:边界数
    uint8  length;                                  // 协议头长度=8
    uint16 dot_num;                                 // 每条边线点数
    uint8  valid_flag;                              // BIT0~2 对应边线1~3
    uint8  reserve;
} protocol_camera_dot_header_t;

// 参数设置包（接收，固定8字节）
typedef struct
{
    uint8  head;                                    // 0x55
    uint8  function;                                // 0x20
    uint8  channel;                                 // 通道号 1~8
    uint8  check_sum;                               // 字节累加和
    float  data;                                    // 参数值
} protocol_parameter_t;
#pragma pack(pop)

/*===========================================================================
 * 图像+边线缓冲区（用于 camera_send 的配置）
 *===========================================================================*/
typedef struct
{
    void                  *image_addr;
    uint16                 width;
    uint16                 height;
    protocol_image_type_e  camera_type;
    void                  *boundary_x[PROTOCOL_BOUNDARY_MAX];
    void                  *boundary_y[PROTOCOL_BOUNDARY_MAX];
} protocol_camera_buffer_t;

/*===========================================================================
 * 收发函数指针类型
 *===========================================================================*/
typedef uint32 (*protocol_send_func_t)(const uint8 *buff, uint32 length);
typedef uint32 (*protocol_recv_func_t)(uint8 *buff, uint32 length);

/*===========================================================================
 * API
 *===========================================================================*/

// 初始化：注册你的 TCP/串口 收发函数
void protocol_init(protocol_send_func_t send_func, protocol_recv_func_t recv_func);

// 发送示波器数据
void protocol_oscilloscope_send(protocol_oscilloscope_t *osc);

// 配置图像信息（调用一次即可，后续重复调用 protocol_camera_send）
void protocol_camera_config(protocol_image_type_e type, void *image_addr,
                            uint16 width, uint16 height);

// 配置边线信息
void protocol_camera_boundary_config(protocol_boundary_type_e type, uint16 dot_num,
                                     void *x1, void *x2, void *x3,
                                     void *y1, void *y2, void *y3);

// 发送图像（含边线）
void protocol_camera_send(void);

// 接收解析（放在主循环或定时中断中周期调用）
void protocol_data_analysis(void);

/*===========================================================================
 * 全局变量（外部可访问）
 *===========================================================================*/
extern protocol_oscilloscope_t  protocol_oscilloscope_data;
extern float                    protocol_parameter[PROTOCOL_PARAMETER_CH_MAX];
extern vuint8                   protocol_parameter_update_flag[PROTOCOL_PARAMETER_CH_MAX];

#ifdef __cplusplus
}
#endif

#endif
