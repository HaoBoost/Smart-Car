# AGENTS.md — 龙邱智能车项目 AI 编码助手指南

## 项目概述

全国大学生智能汽车竞赛（第21届"走马观碑"组）参赛项目。基于**龙芯 2K301 核心板**（LoongArch64 架构），通过摄像头视觉 + IMU 陀螺仪 + 编码器实现**三环级联 PID** 自动巡航。

详见 [README.md](./README.md)。

## 构建命令

```bash
cd Loongson_2K300_301_LIB

# 仅编译
./main/build.sh

# 编译 + SCP 传输到开发板
./main/build.sh <开发板IP>

# 编译 + 传输 + 远程运行
./main/build.sh <开发板IP> -r

# 清理
./main/clean_project.sh
```

- **工具链**: `loongarch64-linux-gnu-g++`（GCC 8.3），C++17，存放在 `tools/`
- **依赖**: OpenCV 4、NCNN（预编译在 `tools/LQ_Dep_libs/`）
- 编译产物在 `Loongson_2K300_301_LIB/build/`

## 架构分层

```
user_app/          ← ✅ 用户代码（可修改）：PID、Motor、IMU、图像、速度决策
libraries/         ← ❌ 库代码（禁止修改）：drv(驱动)/common(基础)/app(传感器)
driver/            ← ❌ Linux 内核驱动模块（禁止修改）
main/              ← 主入口 main.cpp + CMakeLists.txt + build.sh
example/           ← 24 个外设示例程序
```

### 核心规则：**绝不修改 `libraries/` 和 `driver/` 目录下的任何文件。**

## 用户代码约定 (`user_app/`)

### 头文件包含链

```cpp
// main.hpp 已统一包含，user_app 文件只需包含自身头文件即可：
#include "lq_drv_inc.hpp"   // → lq_common.hpp → lq_app_inc.hpp → lq_all_demo.hpp
```

### 三环 PID 控制架构

```
图像环(5ms) → target_angle_rate → 角速度环(2ms) → diff_speed → 速度环(1ms) → PWM
```

- **全局变量 `target_speed`** 是速度决策连接到各环的桥梁
- PID 参数**集中初始化**：`PID_init(PID* lmotor, PID* rmotor, PID* angle, PD_FF* angle_ff, PID* photo)`
- 三个环均使用**位置式 PID**（`Positional_PID_Cal`）
- 参考：[程序控制框架.md](./Loongson_2K300_301_LIB/程序控制框架.md)

### 定时器使用

```cpp
lq_timer timer;
timer.set_seconds_ms(1, []{ /* 1ms 回调 */ });   // 毫秒级
timer.set_seconds_s(1, []{ /* 1s 回调 */ });      // 秒级
timer.stop();
```

也可用 `main.hpp` 中的 `PERIODIC(x)` 宏。

### 常用库类

| 类 | 用途 | 路径 |
|---|------|------|
| `lq_i2c_mpu6050` | IMU 陀螺仪 | `libraries/app/gyro/` |
| `ls_atim_pwm` | 电机 PWM（DUTY_MAX=10000） | `libraries/drv/inc/` |
| `ls_encoder_pwm` | 编码器（512线） | `libraries/drv/inc/` |
| `lq_camera_ex` | 摄像头（V4L2） | `libraries/drv/inc/` |
| `lq_timer` | 定时器 | `libraries/drv/inc/` |
| `ls_gpio` | GPIO | `libraries/drv/inc/` |
| `LQ_NCNN` | 神经网络推理 | `libraries/drv/inc/` |

完整列表：[libraries_classes.md](./Loongson_2K300_301_LIB/libraries_classes.md)

### 代码风格

- C++17，使用 `std::mutex`、智能指针
- 类禁用拷贝/移动（`= delete`）
- PWM/PID 输出必须限幅，防止堵转
- 头文件使用 `#pragma once` 或 include guard

### 开发工作流

1. 在 `user_app/inc/` 写头文件，`user_app/src/` 写实现
2. 运行 `./main/build.sh` 编译检查错误（warning 可忽略）
3. 使用 Git 提交：
   ```bash
   ./../git_add.sh
   git commit -m "<描述>"
   ```

## 关键文档

- [程序控制框架](./Loongson_2K300_301_LIB/程序控制框架.md) — PID 架构设计思路
- [need.md](./Loongson_2K300_301_LIB/need.md) — 用户需求说明
- [收获.md](./Loongson_2K300_301_LIB/收获.md) — 调试经验笔记
- [libraries_classes.md](./Loongson_2K300_301_LIB/libraries_classes.md) — 库类索引
- [更新日志](./Loongson_2K300_301_LIB/main/更新日志.md) — 版本变更
