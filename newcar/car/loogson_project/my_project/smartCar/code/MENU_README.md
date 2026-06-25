# LCD 菜单系统使用说明

## 功能特性
- 支持整型和浮点型参数调节
- 最多支持 10 个菜单项
- 自动滚动显示（屏幕显示 4 行）
- 参数范围限制和步长控制
- 实时刷新显示

## 按键定义
| 按键 | 功能 |
|------|------|
| KEY1 | 上移光标（选择上一个参数） |
| KEY2 | 下移光标（选择下一个参数） |
| KEY3 | 减小当前参数值 |
| KEY4 | 增大当前参数值 |
| KEY5 | 保留（可用于保存/退出） |

## 使用步骤

### 1. 在 main.cc 中包含头文件
```cpp
#include "ww_menu.h"

// 声明全局菜单指针（供按键回调使用）
Menu* g_menu = nullptr;
```

### 2. 定义需要调节的参数
```cpp
// 整型参数
int speed_target_l = 1000;
int speed_target_r = 1000;

// 浮点型参数
float pid_kp = 1.0f;
float pid_ki = 0.0f;
float pid_kd = 0.0f;
```

### 3. 在 main() 中创建菜单
```cpp
int main() {
    // 初始化硬件
    LCD lcd;
    Key key;
    if (lcd.lcd_init() < 0) return -1;
    if (key.key_init() < 0) return -1;

    // 创建菜单
    Menu menu(&lcd);
    g_menu = &menu;  // 设置全局指针

    // 添加菜单项
    //                 名称      指针              最小值  最大值  步长
    menu.add_item("SpdL",   &speed_target_l,    0,      3000,   100);
    menu.add_item("SpdR",   &speed_target_r,    0,      3000,   100);
    menu.add_item("Kp",     &pid_kp,            0.0f,   10.0f,  0.1f);
    menu.add_item("Ki",     &pid_ki,            0.0f,   5.0f,   0.05f);
    menu.add_item("Kd",     &pid_kd,            0.0f,   5.0f,   0.05f);

    // 首次显示菜单
    menu.refresh();

    // 启动按键监听线程
    TimerThread key_thread(key_task, nullptr, 0);
    key_thread.start();

    // 主循环
    while (1) {
        // 使用调节后的参数
        motor.set_motor1(1, speed_target_l);
        motor.set_motor2(1, speed_target_r);

        usleep(10000);
    }

    return 0;
}
```

### 4. 按键任务函数
```cpp
void key_task(void* arg) {
    key.key_listeners();
}
```

## 显示效果示例
```
=== MENU ===
> SpdL:  1000
  SpdR:  1000
  Kp:    1.00
  Ki:    0.00
```

光标 `>` 指示当前选中的参数。

## API 说明

### Menu 类

#### 构造函数
```cpp
Menu(LCD* lcd_ptr);
```
- 参数：LCD 对象指针

#### 添加菜单项
```cpp
// 添加整型参数
void add_item(const char* name, int* value_ptr, int min_val, int max_val, int step);

// 添加浮点型参数
void add_item(const char* name, float* value_ptr, float min_val, float max_val, float step);
```
- `name`: 参数名称（最多 10 个字符）
- `value_ptr`: 参数变量指针
- `min_val`: 最小值
- `max_val`: 最大值
- `step`: 调节步长

#### 按键处理
```cpp
void key_up();          // 上移光标
void key_down();        // 下移光标
void key_decrease();    // 减小参数
void key_increase();    // 增大参数
```

#### 刷新显示
```cpp
void refresh();
```

## 注意事项
1. 参数名称不要超过 10 个字符，否则显示会重叠
2. 最多支持 10 个菜单项
3. 屏幕一次显示 4 行，超过会自动滚动
4. 参数调节会自动限制在 [min_val, max_val] 范围内
5. 必须在 main.cc 中声明 `Menu* g_menu = nullptr;`

## 扩展功能
如果需要添加保存功能，可以在 KEY5 回调中实现：
```cpp
static void KEY5_CallBack(void) {
    // 保存参数到文件
    save_params_to_file();
    // 或者显示保存成功提示
    lcd->showString(0, 120, "Saved!");
}
```
