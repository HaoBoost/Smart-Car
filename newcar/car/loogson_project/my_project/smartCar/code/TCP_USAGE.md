# TCP 通信使用指南

## 概述

项目提供了两个 TCP 客户端类：
1. **TcpClient**：通用 TCP 客户端，可连接任何 TCP 服务器
2. **VofaClient**：继承自 TcpClient，专门用于 VOFA+ 上位机调试

## 一、TcpClient 基础用法

### 1.1 连接服务器

```cpp
#include "ww_tcp_client.h"

TcpClient tcp_client;

// 方法1：使用默认 IP 和端口（192.168.2.10:2233）
if (tcp_client.connect_server() < 0) {
    printf("连接失败\n");
    return -1;
}

// 方法2：指定 IP 和端口
if (tcp_client.connect_server("192.168.1.100", 8080) < 0) {
    printf("连接失败\n");
    return -1;
}
```

### 1.2 发送数据

#### a) 发送字符串
```cpp
tcp_client.send_string("Hello, Server!");
```

#### b) 发送原始字节
```cpp
// 发送结构体
struct SensorData {
    float temperature;
    float humidity;
    int timestamp;
} data = {25.5f, 60.0f, 12345};

tcp_client.send_bytes(&data, sizeof(data));

// 发送数组
float values[] = {1.0f, 2.0f, 3.0f};
tcp_client.send_bytes(values, sizeof(values));
```

### 1.3 检查连接状态

```cpp
if (tcp_client.is_connected()) {
    printf("服务器已连接\n");
} else {
    printf("服务器未连接\n");
}
```

### 1.4 断开连接

```cpp
tcp_client.disconnect_server();
```

---

## 二、VofaClient 用法（VOFA+ 上位机）

### 2.1 什么是 VOFA+？

VOFA+ 是一款强大的串口/网络调试工具，支持实时波形显示。
- 官网：https://www.vofa.plus/
- 协议：FireWater 格式（`name:value1,value2,...\n`）

### 2.2 连接 VOFA+

```cpp
#include "ww_tcp_client.h"

VofaClient vofa;

// 连接到 PC 上的 VOFA+ 服务器
if (vofa.connect_server("192.168.1.100", 2233) < 0) {
    printf("连接 VOFA+ 失败\n");
    return -1;
}
```

### 2.3 发送数据到 VOFA+

#### a) 发送单个数据
```cpp
float speed = 1000.0f;
vofa.send_firewater("speed: %.2f\n", speed);
```

#### b) 发送多个数据（波形显示）
```cpp
float left_speed = 1000.0f;
float right_speed = 1050.0f;
float angle = 15.5f;

vofa.send_firewater("motor: %.2f, %.2f, %.2f\n",
    left_speed, right_speed, angle);
```

#### c) 发送 PID 调试数据
```cpp
vofa.send_firewater("pid: %.2f, %.2f, %.2f, %.2f\n",
    target_speed, actual_speed, pid_output, error);
```

---

## 三、实战示例

### 示例1：发送电机速度到 VOFA+

```cpp
#include "headfile.h"
#include "ww_tcp_client.h"

VofaClient vofa;

void motor_debug_task(void* arg) {
    while (1) {
        // 读取电机速度
        float spd_l = -motor.encoder1_counts * 12.2f;
        float spd_r = -motor.encoder2_counts * 12.2f;

        // 发送到 VOFA+ 显示波形
        vofa.send_firewater("speed: %.2f, %.2f\n", spd_l, spd_r);

        usleep(20000);  // 50Hz 发送频率
    }
}

int main() {
    // 初始化硬件
    motor.motor_init();

    // 连接 VOFA+
    if (vofa.connect_server("192.168.1.100", 2233) < 0) {
        printf("VOFA+ 连接失败\n");
        return -1;
    }

    // 启动调试线程
    TimerThread debug_thread(motor_debug_task, NULL, 20);
    debug_thread.start();

    // 主循环
    while (1) {
        // 你的控制代码
        usleep(10000);
    }

    return 0;
}
```

### 示例2：发送巡线数据

```cpp
void vision_debug_task(void* arg) {
    while (1) {
        // 发送巡线数据到 VOFA+
        vofa.send_firewater("vision: %d, %d, %d, %.2f\n",
            g_ipts0_num,      // 左边线点数
            g_ipts1_num,      // 右边线点数
            g_rptsn_num,      // 中线点数
            g_pure_angle);    // 航向角误差

        usleep(50000);  // 20Hz
    }
}
```

### 示例3：发送 PID 调试数据

```cpp
void pid_debug_task(void* arg) {
    while (1) {
        // 读取速度环数据
        float target = speed_target_l;
        float actual = -motor.encoder1_counts * 12.2f;
        float error = target - actual;
        float output = g_pwm_l;  // 假设这是全局变量

        // 发送到 VOFA+ 观察 PID 响应曲线
        vofa.send_firewater("pid: %.2f, %.2f, %.2f, %.2f\n",
            target, actual, error, output);

        usleep(10000);  // 100Hz
    }
}
```

---

## 四、集成到现有项目

### 4.1 在 main.cc 中添加 TCP 调试

```cpp
#include "headfile.h"
#include "line_track.h"
#include "ww_tcp_client.h"

// 全局 VOFA 客户端
VofaClient vofa;

// 调试任务
void debug_task(void* arg) {
    while (vofa.is_connected()) {
        // 发送电机速度
        float spd_l = -motor.encoder1_counts * 12.2f;
        float spd_r = -motor.encoder2_counts * 12.2f;

        // 发送巡线数据
        vofa.send_firewater("data: %.2f, %.2f, %.2f, %d, %d\n",
            spd_l, spd_r, g_pure_angle,
            g_ipts0_num, g_ipts1_num);

        usleep(20000);  // 50Hz
    }
}

int main() {
    // ... 初始化硬件 ...

    // 连接 VOFA+（可选，调试时启用）
    if (vofa.connect_server("192.168.1.100", 2233) == 0) {
        printf("VOFA+ 已连接\n");

        // 启动调试线程
        TimerThread debug_thread(debug_task, NULL, 20);
        debug_thread.start();
    }

    // ... 主循环 ...
}
```

---

## 五、VOFA+ 配置

### 5.1 启动 VOFA+ TCP 服务器

1. 打开 VOFA+ 软件
2. 选择 **TCP Server** 模式
3. 设置端口：`2233`（默认）
4. 点击 **启动服务器**
5. 等待智能车连接

### 5.2 配置波形显示

1. 选择 **FireWater** 协议
2. 在 **通道配置** 中添加通道名称（与代码中发送的名称对应）
3. 点击 **开始显示**

### 5.3 示例配置

如果代码发送：
```cpp
vofa.send_firewater("motor: %.2f, %.2f, %.2f\n", spd_l, spd_r, angle);
```

VOFA+ 配置：
- 通道1：`motor[0]` → 左轮速度
- 通道2：`motor[1]` → 右轮速度
- 通道3：`motor[2]` → 航向角

---

## 六、常见问题

### Q1: 连接失败怎么办？
**A:** 检查以下几点：
1. PC 和智能车在同一局域网
2. PC 防火墙允许端口 2233
3. IP 地址正确（`ifconfig` 查看 PC IP）
4. VOFA+ 服务器已启动

### Q2: 数据发送频率多少合适？
**A:** 建议：
- 波形显示：20-100Hz（20-100ms 间隔）
- 日志记录：1-10Hz（100-1000ms 间隔）
- 过高频率会占用带宽，影响控制性能

### Q3: 如何发送多组数据？
**A:** 使用不同的名称前缀：
```cpp
vofa.send_firewater("motor: %.2f, %.2f\n", spd_l, spd_r);
vofa.send_firewater("pid: %.2f, %.2f\n", error, output);
vofa.send_firewater("vision: %d, %d\n", left_pts, right_pts);
```

### Q4: 可以同时连接多个客户端吗？
**A:** 可以，创建多个 TcpClient 实例：
```cpp
TcpClient client1;
TcpClient client2;
client1.connect_server("192.168.1.100", 8080);
client2.connect_server("192.168.1.101", 8081);
```

---

## 七、性能优化建议

1. **控制发送频率**：避免过高频率占用 CPU
2. **使用独立线程**：不要在主控制循环中发送
3. **检查连接状态**：发送前检查 `is_connected()`
4. **异常处理**：连接断开时自动重连

```cpp
void safe_send_data() {
    if (!vofa.is_connected()) {
        // 尝试重连
        vofa.connect_server("192.168.1.100", 2233);
        return;
    }

    // 发送数据
    vofa.send_firewater("data: %.2f\n", value);
}
```

---

## 八、API 参考

### TcpClient 类

| 方法 | 说明 | 返回值 |
|------|------|--------|
| `connect_server(ip, port)` | 连接服务器 | 0=成功, -1=失败 |
| `send_string(str)` | 发送字符串 | true=成功, false=失败 |
| `send_bytes(data, len)` | 发送原始字节 | true=成功, false=失败 |
| `is_connected()` | 检查连接状态 | true=已连接, false=未连接 |
| `disconnect_server()` | 断开连接 | 无 |

### VofaClient 类

| 方法 | 说明 |
|------|------|
| `send_firewater(format, ...)` | 发送格式化数据（FireWater 协议） |

继承 TcpClient 的所有方法。

---

## 九、完整示例代码

参考 [smartCar/code/tcp_example.cpp](tcp_example.cpp)（待创建）

---

**作者**: Wuwu 开源库
**许可**: GPL-3.0
**更新日期**: 2025-01-08
