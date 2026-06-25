# README

## 项目定位
- Linux 嵌入式智能车项目，主程序位于 `smartCar/`。
- 主目标：摄像头巡线 + 元素状态机 + 双电机闭环控制 + 图传/上位机调参。

## 硬件平台
- 主控平台：`loongarch64 Linux`
- 交叉工具链：`/opt/loongson-gnu-toolchain-13.2`
- 视觉：龙邱相机库 `lq_camera_ex`，当前实例参数 `320x240 @120fps MJPG`
- 执行/外设：双电机、无刷、ICM42688、VL53L0X、LCD、按键
- 通信：Seekfree TCP 上位机、HTTP 图传

## 开发环境
- 构建：`CMake + make`
- 三方库：`OpenCV / jsoncpp / ncnn / lq_camera_ex`
- 构建脚本：`build.sh`

## 分层架构
- `smartCar/main.cc`
  只做初始化、启动周期线程、主循环取帧。
- `smartCar/app/`
  运行态组织层。含全局上下文、可调参数、周期任务。
- `smartCar/code/`
  算法层。含巡线、元素识别、纯跟踪、PID、视觉辅助工具。
- `smartCar/wuwu_library/`
  设备与通信封装层。相机、电机、IMU、LCD、按键、图传、TCP。
- `cross_lib/`
  已编译三方库，默认只链接不改源码。

## 主运行链路
1. `main()` 按固定顺序初始化相机/LCD/按键/电机/无刷/IMU/上位机连接。
2. 启动周期任务：`imu` `motor` `key` `encoder` `recv`。
3. 主循环取帧后调用 `photo_task()`。
4. `photo_task()` 完成缩放、灰度、巡线、元素识别、纯跟踪、telemetry 打包。
5. `motor_task()` 依据视觉结果、IMU、编码器做速度环/角度环控制。

## 当前核心模块
- 视觉主链：`line_track.h`
- 元素/红色目标/NCNN 状态机：`cross.h` `task_classifier.cc`
- 控制主环：`task_motor.cc`
- 共享状态：`app_context.cc/.h`
- 调参默认值：`app_config.cc/.h`

## 不可改动核心区域
- `main.cc` 初始化顺序与线程分工
- `app_context.*` 中共享对象/共享状态的集中定义方式
- `app_config.*` 仅放默认调参，不混入运行时状态
- `line_track.h` 的图像尺寸、采样间距、透视链路基线
- `cross_lib/` 预编译库目录结构

## 文档协作约定
- 固定只维护：`README.md` `progress.md` `rule.md` `code-snap.md`
- 日常优先更新 `progress.md`
- 无大架构变更，不改本文件

## 快速上手

### 1. 到手先确认什么
- 目标板系统与工具链匹配：`loongarch64 Linux`
- 设备节点存在：`/dev/wuwu_motor` `/dev/wuwu_icm42688` `/dev/input/event0` `/dev/fb0`
- 目标板与电脑网络互通
- 若使用 NCNN，目标板上必须同时有：
  - `main`
  - `tiny_classifier_fp32.ncnn.param`
  - `tiny_classifier_fp32.ncnn.bin`
  - `labels.txt`

### 2. 如何编译与下发
- 本地构建：
  - `cmake -B build -S . -DENABLE_NCNN=ON`
  - `make -C build -j$(nproc)`
- 一键脚本：
  - `build.sh`
- `build.sh` 默认下发到：
  - 目标板：`root@192.168.179.33`
  - 目标路径：`/root`
- 若目标板 IP 变化，先改 `build.sh`

### 3. 程序启动后做什么
- 程序会按顺序初始化：相机 -> LCD -> Key -> Motor -> Brushless -> IMU -> 上位机 TCP
- 图传服务默认监听 `8080`
- 代码打印提示中的访问地址以板载实际 IP 为准
- 浏览器打开：
  - `http://<目标板IP>:8080`

## 图传与调试

### 图传页面有什么
- 主图：`/stream`
- 透视/边线图：`/perspective`
- 模型裁剪图：`/crop`
- 状态 JSON：`/stats`
- PID 更新接口：`/pid?group=<group>&kp=<v>&ki=<v>&kd=<v>`
- 控制接口：
  - `/control?action=start`
  - `/control?action=delay_start`
  - `/control?action=stop`
  - `/control?action=reset_element`
- 图传模式：
  - `/transport_mode?mode=all`
  - `/transport_mode?mode=image_only`
  - `/transport_mode?mode=telemetry_only`
- 拍照：
  - `/snapshot?prefix=<name>&target=main`
  - `/snapshot?prefix=<name>&target=crop`

### 图传页面怎么用
- 先看主图是否持续刷新，确认相机与服务正常
- 再看透视图，确认边线、角点、中线是否稳定
- 裁剪图主要给红色目标和 NCNN 识别链路排查
- 页面会实时显示：
  - 目标/实际速度
  - PWM
  - 目标/实际角速度
  - 纯跟踪角度
  - 左右角点和远角点
  - 当前元素状态与 NCNN 状态
  - 各组 PID 当前值

### 图传常见坑
- `main.cc` 里只有 `param_photo_flag` 为真才会调用 `camera_server.update_frame(...)`
- `app_config.cc` 当前默认：
  - `param_photo_flag = 0`
  - `pesrsp_photo_flag = 0`
- 这意味着可能出现：
  - 网页能打开，但流不更新
  - 视觉调试叠加图不开
- 若要启用图传更新，先确认这两个开关的使用意图再改

## 透视变换参数获取

### 当前工程的真实基线
- 巡线算法固定工作在 `160x120`
- `line_track.h` 中 `camera_param_init()` 维护一组 160x120 透视四点
- `USE_PERSP_LUT=1` 时，实际优先使用 `smartCar/code/persp_lut.h`
- 当前仓库里的 `persp_lut.h` 是 `160x120` LUT
- `tools/calib_persp.py` 是交互式取点脚本，用来获取 `SRC_PTS / DST_PTS`
- `tools/generate_persp_lut.py` 是离线生成脚本，用来把标定点和相机内参生成最终 `persp_lut.h`
- `generate_persp_lut.py` 里的示例点不一定等于当前车上实测真值

### 推荐获取流程
1. 先确定以后长期使用哪套链路。
2. 若继续用 LUT，最终真值应落在 `persp_lut.h`。
3. 若临时调试，可先改 `camera_param_init()` 的 `src/dst` 四点观察趋势。
4. 四点稳定后，再生成新的 LUT 覆盖 `persp_lut.h`。

### 两个脚本各自干什么
- `tools/calib_persp.py`
  - 用来交互式点四个源点
  - 实时看鸟瞰图效果
  - 输出并保存 `SRC_PTS / DST_PTS`
- `tools/generate_persp_lut.py`
  - 用来读取你确认好的 `SRC_PTS / DST_PTS`
  - 结合相机内参 `K / D`
  - 生成最终的 `smartCar/code/persp_lut.h`

### 四点怎么取
- 取赛道在原图中的梯形四个角
- 顺序固定：
  - 左下
  - 右下
  - 右上
  - 左上
- 目标是把梯形映射成鸟瞰矩形，矩形宽度与赛道宽度一致
- 原则：
  - 下边两点落在近处赛道左右边缘
  - 上边两点落在远处赛道左右边缘
  - 点必须跟当前巡线分辨率一致，不能拿 320x240 的点直接填 160x120

### `calib_persp.py` 使用步骤
1. 准备一张当前相机视角下的赛道图片
2. 在 PC 上运行：
   - `python tools/calib_persp.py <图片路径>`
3. 在源图窗口按顺序点击 4 个点：
   - 左下
   - 右下
   - 右上
   - 左上
4. 在右侧鸟瞰窗口观察效果
5. 用滑动条微调：
   - `Translate X`
   - `Translate Y`
   - `Scale`
6. 满意后按 `s`
7. 脚本会输出并保存到：
   - `calib_result.txt`

### `calib_persp.py` 的结果怎么用
- 若只是临时验证透视点：
  - 把输出的 `SRC_PTS` 先填进 `line_track.h` 的 `camera_param_init()`
- 若要生成正式 LUT：
  - 把输出的 `SRC_PTS` 和 `DST_PTS` 填进 `tools/generate_persp_lut.py`
  - 再生成新的 `persp_lut.h`

### LUT 生成步骤
1. 先用 `tools/calib_persp.py` 拿到确认后的 `SRC_PTS / DST_PTS`
2. 用 OpenCV/Matlab 做相机内参标定，拿到 `K` 和 `D`
3. 在 `tools/generate_persp_lut.py` 填：
   - `K`
   - `D`
   - `IMG_W` `IMG_H`
   - `SRC_PTS`
   - `DST_PTS`
4. 运行：
   - `python tools/generate_persp_lut.py`
5. 生成 `persp_lut.h`
6. 覆盖到：
   - `smartCar/code/persp_lut.h`
7. 重新编译下载
8. 打开图传透视图，确认：
   - 直道边线接近平行
   - 左右边线宽度基本恒定
   - 中线不随位置大幅漂移

### 透视好坏怎么看
- 好的表现：
  - 直道时左右边线近似垂直、平滑
  - 中线稳定，纯跟踪角度小
  - 左右角点位置合理，不乱跳
- 不好的表现：
  - 直道中线左右摆
  - 近处宽远处窄得离谱或反过来
  - 进入十字/圆环时角点识别随机

## PID 调法

### 控制链路理解
- `pid_angle`
  - 角度环，输入是纯跟踪误差，输出目标角速度
- `pid_angle_v`
  - 角速度环，输入是目标角速度与 IMU 角速度误差，输出差速
- `pid_speed_1` `pid_speed_r`
  - 左右速度环，输入目标轮速与实际轮速误差，输出 PWM 增量
- `pid_angle_circle`
  - 圆环工况角度环

### 可调入口
- 图传页面可改 5 组：
  - `speed_l`
  - `speed_r`
  - `gyro`
  - `angle`
  - `circle`
- 菜单页可改：
  - `PID/SpeedL`
  - `PID/SpeedR`
  - `PID/Gyro`
  - `PID/Angle`
  - `PID/Circle`
- Seekfree TCP 当前只映射了少量通道：
  - 1 -> `pid_angle_v.kp`
  - 2 -> `pid_angle_v.kd`
  - 3 -> `target_v`
  - 4 -> `pid_speed_r.ki`
  - 5 -> `base_speed_target_l`
  - 6 -> `base_speed_target_r`

### 调 PID 的推荐顺序
1. 先关复杂功能：
   - 直道加速先保持小
   - `g_curve_ff_enable = 0`
   - NCNN 绕行先不作为主调试项
2. 先调速度环，再调角速度环，再调角度环
3. 最后再调圆环专用角度环

### 速度环怎么调
- 看图传里的：
  - `targetSpeedL/R`
  - `actualSpeedL/R`
  - `pwmL/R`
- 现象与处理：
  - 速度明显跟不上：
    - 先加 `pid_speed_1.kp` / `pid_speed_r.kp`
  - 能跟上但长期有静差：
    - 小幅加 `ki`
  - PWM 抖、速度振：
    - 降 `kp`，必要时降 `ki`
  - 一直顶到限幅：
    - 检查 `base_speed_target_*` 是否过高
    - 检查 `out_max`、PWM 限幅是否太小

### 角速度环怎么调
- 看图传里的：
  - `targetGyro`
  - `actualGyro`
- 现象与处理：
  - 转向响应慢、明显跟不上：
    - 加 `pid_angle_v.kp`
  - 抖动、左右来回摆：
    - 降 `kp`
    - 适度加 `kd`
  - 瞬态尖峰大：
    - 检查 `d_max`

### 角度环怎么调
- 看图传里的：
  - `pureAngle`
  - 赛道上实车是否入弯及时
- 现象与处理：
  - 入弯慢、转不过：
    - 加 `pid_angle.kp`
    - 适当减小前瞻
  - 弯中抽动、左右扫：
    - 降 `pid_angle.kp`
    - 若用了 `kp_2`，先减小或归零
  - 直道不稳：
    - 先查透视和边线质量，再动 PID

### 圆环 PID 怎么调
- 只调 `PID/Circle` 这一组
- 调法和普通角度环类似，但必须在圆环状态下看
- 推荐：
  - 先只动 `Kp`
  - `Ki` 通常先保持 0
  - `Kd` 小步加

## 菜单与按键

### 当前状态说明
- 菜单系统代码和按键映射已具备
- 但 `main.cc` 当前未启动 `menu_thread`
- 因此接手时若要依赖 LCD 菜单做现场调参，先确认是否恢复 `menu_thread.start()`

### 按键定义
- `KEY1 code=2` 上移
- `KEY2 code=3` 下移
- `KEY3 code=4` 返回上级 / 参数减小
- `KEY4 code=5` 进入 / 执行动作 / 参数增大
- `KEY5 code=6` 菜单页与运行状态页切换

### 菜单主要分组
- `Launch`
  - 发车、停车、起步参数
- `Speed`
  - 基础速度、直道加速、前瞻速度映射、总限幅
- `PID`
  - 速度环、角速度环、普通角度环、圆环角度环
- `Brushless`
  - 无刷占空比
- `CurveFF`
  - 双前瞻前馈
- `Status`
  - 实时只读状态

## 十字调法

### 十字识别逻辑要点
- 先在近处检测到 L 角点
- 再从该角点反投影到原图，按偏移量重新爬远线
- 若远处也找到对应角点，连续确认后进入十字
- 关键参数：
  - `g_Lpt0_offset_x`
  - `g_Lpt0_offset_y`
  - `g_Lpt1_offset_x`
  - `g_Lpt1_offset_y`
  - `far_start_thres`
  - `CROSS_CONFIRM_FRAMES`

### 十字调试步骤
1. 先确保透视图上近角点稳定
2. 看远线是否能被重新爬到
3. 看十字入口处是否连续进入 `CROSS_BEGIN`
4. 看十字中央是否能进入 `CROSS_RUNNING`
5. 看出十字后能否恢复 `ELEM_NONE`

### 常见问题与处理
- 进十字太晚或不触发：
  - 检查近处角点是否太靠后
  - 调 `g_Lpt*_offset_x/y`
  - 检查 `far_start_thres`
- 误把普通弯当十字：
  - 提高确认条件，优先核查角点质量与透视
- 十字中间丢线：
  - 先看远线爬线起点是否合理
  - 再看 `far_start_thres` 是否过高/过低

## 圆环调法

### 圆环状态机
- `CIRCLE_BEGIN`
  - 识别到单侧角点 + 对侧长直道，准备进环
- `CIRCLE_APPROACH`
  - 引导进环
- `CIRCLE_RUNNING`
  - 沿外环跑
- `CIRCLE_OUT`
  - 检出出环角点，准备出环
- `CIRCLE_END`
  - 恢复正常循线

### 关键判据
- 入口确认：
  - 单侧近角点
  - 另一侧长直道
  - 连续确认计数满足
- 进环阶段：
  - 编码器累计值满足
  - `circle_yaw_angle` 积分满足
- 出环阶段：
  - 检到对应侧近角点
  - 后续角点消失且边线恢复足够长

### 主要相关参数
- `pid_angle_circle`
- `g_circle_type`
- `g_track_side`
- `g_centerline_offset`
- `g_Lpt0_offset_*` `g_Lpt1_offset_*`
- `far_start_thres`
- `CIRCLE_CONFIRM_FRAMES`

### 圆环调试顺序
1. 先让普通循线足够稳
2. 确认入口识别稳定，不误触发
3. 先调 `CIRCLE_BEGIN -> CIRCLE_APPROACH`
4. 再调 `CIRCLE_APPROACH -> CIRCLE_RUNNING`
5. 最后调 `CIRCLE_OUT -> CIRCLE_END`

### 现象与处理
- 识别不到圆环：
  - 先看近角点和对侧长直道是否稳定
- 进环过早/过晚：
  - 看编码器阈值与角点位置是否匹配赛道
- 环内压线：
  - 优先调 `pid_angle_circle`
  - 再看 `g_track_side` 切换是否符合左右环
- 出环拖泥带水：
  - 看出环角点触发是否太晚
  - 看边线恢复点数条件是否过严

## 直道和前瞻相关参数

### 直道加速
- 相关参数：
  - `g_straight_add_speed`
  - `g_straight_angle_limit`
  - `g_straight_confirm_frames`
  - `g_straight_min_centerline_pts`
  - `g_straight_min_side_pts`
  - `g_straight_corner_min_id`
- 调法：
  - 先保证不误判直道
  - 再逐步增加 `g_straight_add_speed`

### 前瞻与速度映射
- 相关参数：
  - `g_line_lookahead_speed_gain`
  - `g_line_lookahead_dist_min`
  - `g_line_lookahead_dist_max`
- 调法：
  - 转向太钝：
    - 降 `DistMin/DistMax`
    - 或减小速度映射增益后的实际前瞻
  - 车头太冲、来回扫：
    - 增大 `DistMin/DistMax`

### 弯道前馈
- 相关参数：
  - `g_curve_ff_enable`
  - `g_curve_ff_far_offset`
  - `g_curve_ff_k_delta`
  - `g_curve_ff_limit`
  - `g_curve_ff_low_pass`
  - `g_curve_ff_active_angle_min`
- 推荐顺序：
  1. `Enable=1`
  2. `FarOff` 先保持 3
  3. 小步加 `Kdelta`
  4. 发飘则减 `Kdelta` 或增 `LowPass`
  5. 过冲则减 `Limit` 或增 `ActAng`

## NCNN/红色目标链路

### 触发逻辑
- 先识别红色区域
- 红框中心 `center_y` 达阈值后进入等待触发
- 连续若干帧满足后进入慢速分类
- 连续稳定分类后锁定动作

### 关键参数
- `g_ncnn_trigger_center_y`
- `g_ncnn_trigger_confirm_frames`
- `g_ncnn_classify_stable_frames`
- `g_ncnn_classify_min_score`
- `g_ncnn_slow_speed_ratio`
- `g_ncnn_bypass_offset_scale`
- `g_ncnn_recover_offset_scale`
- `g_ncnn_recover_bias_deg`

### 调法
- 先只看红框是否稳定
- 再看裁剪图 `crop` 是否真正截到目标
- 再看分类标签是否连续稳定
- 最后再调绕行动作尺度

## 排查顺序

### 小车跑不稳时
1. 先看图传主图与透视图
2. 再看 `pureAngle`
3. 再看 `targetGyro/actualGyro`
4. 最后看速度环和 PWM

### 识别类问题
1. 先确认透视是否正常
2. 再确认角点是否稳定
3. 再确认元素状态机切换条件
4. 最后才改阈值

### 不要上来就改
- 初始化顺序
- 线程周期
- 图像尺寸
- 大量 PID 同时改
- 十字、圆环、前馈、NCNN 一起开着同时调
