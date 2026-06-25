# Seekfree 协议 TCP 传输使用指南

## 文件说明

### 核心文件
- `seekfree_protocol.h` - Seekfree 协议头文件
- `seekfree_protocol.c` - Seekfree 协议实现
- `seekfree_tcp_adapter.h` - TCP 适配层头文件
- `seekfree_tcp_adapter.cc` - TCP 适配层实现
- `seekfree_example.cc` - 使用示例

### 依赖文件
- `ww_tcp_client.h` - 已有的 TCP 客户端（项目中已存在）
- `ww_tcp_client.cc` - TCP 客户端实现（项目中已存在）

## 快速开始

### 1. 包含头文件

```cpp
#include "seekfree_tcp_adapter.h"
```

### 2. 创建客户端实例

```cpp
SeekfreeTcpClient seekfree_client;
```

### 3. 连接服务器并初始化协议

```cpp
// 连接到 Seekfree 上位机（修改为你的上位机 IP 和端口）
if (seekfree_client.connect_server("192.168.2.10", 2233) < 0) {
    printf("连接失败\\n");
    return -1;
}

// 初始化协议
seekfree_client.init_protocol();
```

### 4. 发送示波器数据

```cpp
// 发送 3 个通道的数据
float speed = 1.5f;
float angle = 30.0f;
float error = 0.2f;

seekfree_client.send_oscilloscope(speed, angle, error);
```

### 5. 发送图像和边线

```cpp
// 假设你有以下数据
uint8_t image[120][188];      // 灰度图像
uint8_t left_line[120];       // 左边线
uint8_t center_line[120];     // 中线
uint8_t right_line[120];      // 右边线

// 发送图像和边线
seekfree_client.send_camera_gray((uint8_t*)image, 188, 120,
                                left_line, center_line, right_line);
```

### 6. 接收参数（在主循环中）

```cpp
while (seekfree_client.is_connected()) {
    // 周期调用解析函数
    seekfree_client.process_parameters();

    // 检查参数是否更新
    for (int i = 1; i <= 8; i++) {
        if (seekfree_client.is_parameter_updated(i)) {
            float value = seekfree_client.get_parameter(i);
            printf("参数 %d 更新为: %.2f\\n", i, value);

            // 清除更新标志
            seekfree_client.clear_parameter_flag(i);

            // 使用参数值
            // 例如：if (i == 1) pid_kp = value;
        }
    }

    usleep(50000);  // 50ms
}
```

## API 说明

### SeekfreeTcpClient 类

#### 连接管理
- `int connect_server(const char* ip, int port)` - 连接服务器
- `void disconnect_server()` - 断开连接
- `bool is_connected()` - 检查连接状态
- `void init_protocol()` - 初始化 Seekfree 协议

#### 数据发送
- `void send_oscilloscope(float ch1, ..., float ch8)` - 发送示波器数据（最多8通道）
- `void send_camera_gray(uint8_t *image, uint16_t width, uint16_t height, ...)` - 发送灰度图像和边线

#### 参数接收
- `void process_parameters()` - 接收并解析参数（周期调用）
- `float get_parameter(uint8_t channel)` - 获取参数值（通道 1~8）
- `bool is_parameter_updated(uint8_t channel)` - 检查参数是否更新
- `void clear_parameter_flag(uint8_t channel)` - 清除参数更新标志

## 编译说明

在你的 Makefile 或 CMakeLists.txt 中添加以下文件：

```makefile
SOURCES += seekfree_protocol.c
SOURCES += seekfree_tcp_adapter.cc
SOURCES += seekfree_example.cc  # 如果需要示例
```

## 协议格式

### 发送（单片机→上位机）
- 帧头：`0xAA`
- 示波器：功能字 `0x10`
- 图像：功能字 `0x02`
- 边线：功能字 `0x03`

### 接收（上位机→单片机）
- 帧头：`0x55`
- 参数设置：功能字 `0x20`
- 固定 8 字节：`[55] [20] [通道] [校验和] [float值]`

## 注意事项

1. 确保上位机 IP 和端口正确
2. 图像数据必须是连续的内存块
3. 边线数组长度必须等于图像高度
4. 参数通道号范围：1~8
5. 周期调用 `process_parameters()` 以接收参数更新

## 示例代码

完整示例请参考 `seekfree_example.cc` 文件。
