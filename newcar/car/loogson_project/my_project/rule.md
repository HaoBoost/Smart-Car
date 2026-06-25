# rule

## 文档规则
- 固定只维护：`README.md` `progress.md` `rule.md` `code-snap.md`
- 日常只优先更新 `progress.md`
- 不新增工作流类文档，不复制大段源码到文档
- 任何代码修改后，必须同步检查四份固定文档是否需要更新
- 若改动影响使用方式、调试流程、参数意义、接口、架构或现存行为，禁止只改代码不改文档
- 文档更新分工固定：
  - `progress.md`：记录本次完成内容、现状变化、问题变化
  - `README.md`：记录上手方式、说明书、调参流程、对外使用变化
  - `rule.md`：记录硬规约、禁止项、固定边界变化
  - `code-snap.md`：记录函数/结构体/全局变量/接口骨架变化

## 架构硬规约
- `main.cc` 只允许保留：初始化、线程启动、取帧主循环
- 运行时共享状态只放 `app_context.*`
- 默认调参只放 `app_config.*`
- 周期任务只放 `smartCar/app/task_*.cc`
- 算法主链优先留在 `smartCar/code/`

## 初始化与线程硬规约
- 硬件初始化顺序保持现状：相机 -> LCD -> Key -> Motor -> Brushless -> IMU -> 上位机连接
- 周期任务周期基线：
  - `motor_task` 1ms
  - `imu_task` 4ms
  - `encoder_get_task` 5ms
  - `key_task` 10ms
  - `recv_task` 10ms
  - `menu_task` 100ms
- 非必要不把业务逻辑塞回 `main`

## 图像/控制基线
- 视觉主处理尺寸固定按 `160x120`
- 相机当前实例基线：`320x240 @120fps MJPG`
- `SAMPLE_DIST=3.0f` `ROAD_WIDTH_PX=21.6f` `WHEELBASE_PX=16.0f`
- 无实车验证，不随意改透视点、采样间距、PID 限幅、状态机阈值

## 硬件/设备约束
- 当前源码可确认的固定设备节点：
  - 电机：`/dev/wuwu_motor`
  - IMU：`/dev/wuwu_icm42688`
  - TOF：`/dev/wuwu_tof`
  - Key：`/dev/input/event0`
  - LCD：`/dev/fb0`
- 当前源码未显式给出真实 GPIO 引脚号；涉及引脚修改必须以设备树/驱动真值为准，禁止脑补
- 按键定义以 `ww_menu.h` 注释为准：`KEY1~KEY5 code=2~6`

## 通信规约
- Seekfree TCP：
  - 帧头发送 `0xAA`
  - 帧头接收 `0x55`
  - 参数通道范围 `1~8`
- 当前主程序默认连接：`192.168.179.64:8086`
- 图传默认端口：`8080`
- 修改 IP/端口前先记录到 `progress.md`

## 编码规约
- 先查四文档，再动代码
- 只做局部修改，不做整文件重写
- 新增共享变量前先判断是否应进 `app_context` 或 `app_config`
- 文档内不粘贴函数实现，只记接口、变量、框架
- 无依据不改命名、不改单位、不改符号方向

## 禁止项
- 禁止凭猜测补充硬件引脚、协议、阈值、单位
- 禁止把运行时状态塞进配置文件
- 禁止在未确认影响前修改 `cross_lib/`
- 禁止随意更改初始化顺序、线程周期、图像尺寸基线
