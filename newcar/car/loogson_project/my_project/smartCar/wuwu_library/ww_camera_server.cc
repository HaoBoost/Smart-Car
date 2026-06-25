#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 摄像头模块
 * 版权所有 (c) 2025 Blockingsys
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 本文件是 Wuwu 开源库 的一部分。
 *
 * 本文件按照 GNU 通用公共许可证 第3版（GPLv3）或您选择的任何后续版本的条款授权。
 * 您可以在遵守 GPL-3.0 许可条款的前提下，自由地使用、复制、修改和分发本文件及其衍生作品。
 * 在分发本文件或其衍生作品时，必须以相同的许可证（GPL-3.0）对源代码进行授权并随附许可证副本。
 *
 * 本软件按“原样”提供，不对适销性、特定用途适用性或不侵权做任何明示或暗示的保证。
 * 有关更多细节，请参阅 GNU 官方许可证文本： https://www.gnu.org/licenses/gpl-3.0.html
 *
 * 注：本注释为 GPL-3.0 许可证的中文说明与摘要，不构成法律意见。正式许可以 GPL 原文为准。
 * LICENSE 副本通常位于项目根目录的 LICENSE 文件或 libraries 文件夹下；若未找到，请访问上方链接获取。
 *
 * 额外说明：
 * - 本项目可能包含第三方组件，各组件的版权与许可以其各自随附的 LICENSE 为准；
 * - 分发、修改本文件时请保留本版权与许可声明以尊重原作者权利；
 * - 本文档为中文译述/摘要，英文许可证文本为法律权威版。
 *
 * 文件名称：ww_camera_server.cc
 * 所属模块：wuwu_library
 * 功能描述：摄像头服务器，用于 MJPEG 流与 HTTP 接口
 * 版本信息：详见 libraries/doc/version
 * 开发环境：Linux，GCC / Clang，OpenCV，V4L2
 * 联系/主页：请参阅项目 README
 *
 * 修改记录：
 * 日期        作者              说明
 * 2025-12-16  Blockingsys    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "ww_camera_server.h"

extern int start_flag;
extern int start__1000ms_flag;
extern int __1000ms;
extern float g_pwm_l_dbg;
extern float g_pwm_r_dbg;
extern Motor motor;
extern Brushless brush;
extern void reset_element_state(void);
extern bool update_pid_params(const std::string& group, float kp, float ki, float kd, std::string& message);
extern cv::Mat g_red_model_input_40;
extern bool g_red_model_input_ready;

// 静态成员初始化
CameraStreamServer* CameraStreamServer::instance = nullptr;

static std::string url_decode(const std::string& value)
{
    std::string decoded;
    decoded.reserve(value.size());
    for (size_t i = 0; i < value.size(); ++i) {
        if (value[i] == '%' && i + 2 < value.size()) {
            char hex[3] = { value[i + 1], value[i + 2], '\0' };
            decoded.push_back(static_cast<char>(strtol(hex, nullptr, 16)));
            i += 2;
        } else if (value[i] == '+') {
            decoded.push_back(' ');
        } else {
            decoded.push_back(value[i]);
        }
    }
    return decoded;
}

static std::string get_query_param(const std::string& path, const std::string& key)
{
    const std::string pattern = key + "=";
    size_t query_pos = path.find('?');
    if (query_pos == std::string::npos) {
        return "";
    }

    size_t key_pos = path.find(pattern, query_pos + 1);
    while (key_pos != std::string::npos) {
        if (key_pos == query_pos + 1 || path[key_pos - 1] == '&') {
            size_t value_start = key_pos + pattern.size();
            size_t value_end = path.find('&', value_start);
            if (value_end == std::string::npos) {
                value_end = path.size();
            }
            return url_decode(path.substr(value_start, value_end - value_start));
        }
        key_pos = path.find(pattern, key_pos + 1);
    }

    return "";
}

static const char* transport_mode_name(CameraTransportMode mode)
{
    switch (mode) {
        case CAMERA_TRANSPORT_IMAGE_ONLY: return "image_only";
        case CAMERA_TRANSPORT_TELEMETRY_ONLY: return "telemetry_only";
        case CAMERA_TRANSPORT_ALL:
        default:
            return "all";
    }
}

static bool is_image_transport_enabled(CameraTransportMode mode)
{
    return mode == CAMERA_TRANSPORT_ALL || mode == CAMERA_TRANSPORT_IMAGE_ONLY;
}

static bool is_telemetry_transport_enabled(CameraTransportMode mode)
{
    return mode == CAMERA_TRANSPORT_ALL || mode == CAMERA_TRANSPORT_TELEMETRY_ONLY;
}

static bool parse_transport_mode(const std::string& text, CameraTransportMode& mode_out)
{
    if (text == "all") {
        mode_out = CAMERA_TRANSPORT_ALL;
        return true;
    }
    if (text == "image_only") {
        mode_out = CAMERA_TRANSPORT_IMAGE_ONLY;
        return true;
    }
    if (text == "telemetry_only") {
        mode_out = CAMERA_TRANSPORT_TELEMETRY_ONLY;
        return true;
    }
    return false;
}

// HTML查看器内容
static const char* viewer_html = R"HTML(
<!DOCTYPE html> 
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>巡线图传调试界面</title>
    <style>
        :root {
            --bg: #07090c;
            --bg-soft: #10161d;
            --panel: rgba(10, 13, 17, 0.88);
            --panel-strong: rgba(7, 10, 14, 0.96);
            --line: rgba(173, 191, 211, 0.18);
            --line-strong: rgba(173, 191, 211, 0.32);
            --text: #edf2f7;
            --dim: #96a3b3;
            --blue: #7ed0ff;
            --cyan: #63e0d8;
            --yellow: #f3d36c;
            --red: #ff8c8c;
            --orange: #ffb977;
            --pink: #f3a6d8;
            --green: #74e4a5;
            --white: #f7fafc;
            --shadow: 0 18px 50px rgba(0, 0, 0, 0.34);
            --radius: 16px;
        }
        * { box-sizing: border-box; }
        body {
            margin: 0;
            background:
                radial-gradient(circle at top left, rgba(76, 124, 255, 0.10), transparent 28%),
                radial-gradient(circle at top right, rgba(38, 190, 187, 0.10), transparent 24%),
                linear-gradient(180deg, #081018 0%, #06080c 100%);
            color: var(--text);
            font-family: ui-monospace, "SFMono-Regular", "Cascadia Mono", "Consolas", "Liberation Mono", monospace;
            overflow: hidden;
        }
        .shell {
            height: 100vh;
            max-width: 1800px;
            margin: 0 auto;
            padding: 12px 12px 76px;
            display: grid;
            grid-template-columns: minmax(520px, 1.45fr) minmax(280px, 0.72fr) minmax(280px, 0.72fr);
            grid-template-areas:
                "main main metrics1 metrics2"
                "perspective crop metrics1 metrics2";
            grid-template-rows: 1fr 1fr;
            gap: 10px;
        }
        .panel {
            background:
                linear-gradient(180deg, rgba(255,255,255,0.025), rgba(255,255,255,0.01)),
                var(--panel);
            border: 1px solid var(--line);
            overflow: hidden;
            position: relative;
            min-height: 0;
            border-radius: var(--radius);
            box-shadow: var(--shadow);
            backdrop-filter: blur(12px);
        }
        .panel::before {
            content: "";
            position: absolute;
            inset: 0;
            pointer-events: none;
            border-radius: inherit;
            box-shadow: inset 0 1px 0 rgba(255,255,255,0.04);
        }
        #panel-main { grid-area: main; }
        #panel-perspective { grid-area: perspective; }
        #panel-crop { grid-area: crop; }
        #panel-metrics-main { grid-area: metrics1; }
        #panel-metrics-extra { grid-area: metrics2; }
        .panel-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 10px 12px;
            border-bottom: 1px solid var(--line);
            background: linear-gradient(180deg, rgba(255,255,255,0.035), rgba(255,255,255,0.012));
            color: var(--dim);
            font-size: 11px;
            letter-spacing: 0.12em;
            text-transform: uppercase;
        }
        .panel-title {
            font-weight: 700;
            color: var(--white);
            letter-spacing: 0.14em;
        }
        .panel-actions {
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .panel-action-btn {
            border: 1px solid var(--line-strong);
            background: rgba(255,255,255,0.03);
            color: var(--text);
            padding: 4px 8px;
            font: inherit;
            cursor: pointer;
            border-radius: 999px;
            transition: 140ms ease;
        }
        .panel-action-btn:hover {
            background: rgba(126, 208, 255, 0.12);
            border-color: rgba(126, 208, 255, 0.34);
            color: var(--white);
        }
        .panel-body {
            height: calc(100% - 44px);
        }
        .stream {
            width: 100%;
            height: 100%;
            object-fit: contain;
            display: block;
            background:
                radial-gradient(circle at center, rgba(255,255,255,0.03), transparent 40%),
                #020304;
            image-rendering: auto;
        }
        .metrics {
            padding: 12px 14px 14px;
            margin: 0;
            height: calc(100% - 44px);
            overflow: auto;
            white-space: pre-wrap;
            font-size: 12.5px;
            line-height: 1.5;
            color: var(--white);
            background:
                linear-gradient(180deg, rgba(255,255,255,0.012), rgba(255,255,255,0.0)),
                var(--panel-strong);
            scrollbar-width: thin;
            scrollbar-color: rgba(126, 208, 255, 0.35) transparent;
        }
        .metrics::-webkit-scrollbar { width: 8px; }
        .metrics::-webkit-scrollbar-thumb {
            background: rgba(126, 208, 255, 0.28);
            border-radius: 999px;
        }
        .metrics::-webkit-scrollbar-track { background: transparent; }
        .metrics-section {
            margin-bottom: 14px;
            padding-bottom: 10px;
            border-bottom: 1px dashed rgba(173, 191, 211, 0.14);
        }
        .metrics-title {
            color: var(--yellow);
            margin-bottom: 5px;
            font-weight: 700;
            letter-spacing: 0.08em;
        }
        .metrics-line {
            display: block;
            color: var(--white);
            margin-bottom: 3px;
        }
        .metrics-label {
            color: var(--dim);
            display: inline;
        }
        .metrics-value {
            color: var(--blue);
            display: inline;
        }
        .metrics-value.warn {
            color: var(--yellow);
        }
        .metrics-value.danger {
            color: var(--red);
        }
        .metrics-value.cyan {
            color: var(--cyan);
        }
        .metrics-value.orange {
            color: var(--orange);
        }
        .metrics-value.pink {
            color: var(--pink);
        }
        .info-bar {
            position: absolute;
            right: 10px;
            bottom: 8px;
            color: var(--dim);
            font-size: 11px;
            background: rgba(4, 6, 8, 0.76);
            padding: 5px 9px;
            border: 1px solid rgba(255,255,255,0.10);
            border-radius: 999px;
            backdrop-filter: blur(8px);
        }
        .status-dot {
            width: 8px;
            height: 8px;
            display: inline-block;
            border-radius: 999px;
            background: var(--green);
            box-shadow: 0 0 12px rgba(116,228,165,0.75);
            animation: pulse 1.6s infinite;
        }
        .toolbar {
            position: fixed;
            left: 12px;
            right: 12px;
            bottom: 12px;
            display: flex;
            gap: 8px;
            justify-content: flex-end;
            pointer-events: none;
        }
        .toolbar button, .toolbar input {
            pointer-events: auto;
            border: 1px solid var(--line-strong);
            background: rgba(8, 11, 15, 0.88);
            color: var(--text);
            padding: 9px 13px;
            font: inherit;
            border-radius: 12px;
            box-shadow: 0 10px 26px rgba(0, 0, 0, 0.24);
            backdrop-filter: blur(10px);
            transition: 140ms ease;
        }
        .toolbar button:hover, .toolbar input:focus {
            border-color: rgba(126, 208, 255, 0.34);
            background: rgba(16, 22, 29, 0.96);
            outline: none;
        }
        .toolbar button.active-mode {
            border-color: rgba(126, 208, 255, 0.46);
            background: rgba(126, 208, 255, 0.16);
            color: var(--white);
        }
        .toolbar input {
            width: 150px;
        }
        .metrics-panel-body {
            height: calc(100% - 44px);
            display: flex;
            flex-direction: column;
            min-height: 0;
        }
        #metrics-main.metrics {
            flex: 1;
            min-height: 0;
            height: auto;
        }
        .pid-tuner {
            border-top: 1px solid var(--line);
            background: rgba(8, 11, 15, 0.94);
            backdrop-filter: blur(10px);
            overflow: hidden;
            flex-shrink: 0;
        }
        .pid-tuner summary {
            list-style: none;
            cursor: pointer;
            padding: 10px 14px;
            color: var(--white);
            font-size: 12px;
            letter-spacing: 0.08em;
            text-transform: uppercase;
        }
        .pid-tuner summary::-webkit-details-marker {
            display: none;
        }
        .pid-tuner-body {
            padding: 0 12px 12px;
            display: grid;
            gap: 8px;
        }
        .pid-tuner-top {
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 10px;
            padding-top: 4px;
        }
        .pid-step {
            display: flex;
            align-items: center;
            gap: 8px;
            color: var(--dim);
            font-size: 12px;
        }
        .pid-row {
            display: grid;
            grid-template-columns: 82px repeat(3, minmax(0, 1fr)) 68px;
            gap: 8px;
            align-items: center;
        }
        .pid-row-label {
            color: var(--dim);
            font-size: 12px;
        }
        .pid-row input, .pid-row button, .pid-step input {
            border: 1px solid var(--line-strong);
            background: rgba(16, 22, 29, 0.96);
            color: var(--text);
            padding: 8px 10px;
            font: inherit;
            border-radius: 10px;
            min-width: 0;
        }
        .pid-row input:focus, .pid-step input:focus {
            outline: none;
            border-color: rgba(126, 208, 255, 0.34);
        }
        .pid-row button:hover {
            border-color: rgba(126, 208, 255, 0.34);
        }
        .pid-step input {
            width: 88px;
        }
        .warn { color: var(--yellow); }
        .accent { color: var(--cyan); }
        .danger { color: var(--red); }
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.35; } }
        @media (max-width: 1180px), (max-height: 780px) {
            body { overflow: auto; }
            .shell {
                height: auto;
                padding-bottom: 12px;
                grid-template-columns: 1fr;
                grid-template-areas:
                    "main"
                    "perspective"
                    "crop"
                    "metrics1"
                    "metrics2";
                grid-template-rows: auto;
            }
            .panel {
                min-height: 240px;
            }
            .toolbar {
                position: static;
                padding: 0 12px 12px;
                justify-content: stretch;
                flex-wrap: wrap;
            }
            .pid-row {
                grid-template-columns: 1fr 1fr;
            }
            .pid-tuner-top {
                flex-direction: column;
                align-items: flex-start;
            }
        }
    </style>
</head>
<body>
    <div class="shell">
        <section class="panel" id="panel-main">
            <div class="panel-header">
                <span class="panel-title">Line Camera</span>
                <div class="panel-actions">
                    <span><span class="status-dot"></span> <span id="fps-top">-- FPS</span></span>
                    <button class="panel-action-btn" onclick="togglePanelFullscreen('panel-main')">全屏</button>
                </div>
            </div>
            <div class="panel-body">
                <img id="stream-main" class="stream" src="/stream" alt="巡线原图">
            </div>
            <div class="info-bar">左上: 巡线原图 / 浏览器访问: <span class="accent" id="main-url"></span></div>
        </section>

        <section class="panel" id="panel-metrics-main">
            <div class="panel-header">
                <span class="panel-title">Target / PID</span>
                <div class="panel-actions">
                    <span id="latency">-- ms</span>
                    <button class="panel-action-btn" onclick="togglePanelFullscreen('panel-metrics-main')">全屏</button>
                </div>
            </div>
            <div class="metrics-panel-body">
                <div id="metrics-main" class="metrics">加载中...</div>
                <details class="pid-tuner" open>
                    <summary>PID Tuner</summary>
                    <div class="pid-tuner-body">
                        <div class="pid-tuner-top">
                            <span class="pid-row-label">在线 PID 调参</span>
                            <label class="pid-step">
                                <span>步进值</span>
                                <input id="pid-step-input" type="number" step="0.001" min="0.0001" value="0.001">
                            </label>
                        </div>
                        <form class="pid-row" onsubmit="submitPid(event, 'speed_l')">
                            <span class="pid-row-label">SPD_L</span>
                            <input class="pid-input" id="pid-speed_l-kp" type="number" step="0.001" placeholder="KP">
                            <input class="pid-input" id="pid-speed_l-ki" type="number" step="0.001" placeholder="KI">
                            <input class="pid-input" id="pid-speed_l-kd" type="number" step="0.001" placeholder="KD">
                            <button type="submit">应用</button>
                        </form>
                        <form class="pid-row" onsubmit="submitPid(event, 'speed_r')">
                            <span class="pid-row-label">SPD_R</span>
                            <input class="pid-input" id="pid-speed_r-kp" type="number" step="0.001" placeholder="KP">
                            <input class="pid-input" id="pid-speed_r-ki" type="number" step="0.001" placeholder="KI">
                            <input class="pid-input" id="pid-speed_r-kd" type="number" step="0.001" placeholder="KD">
                            <button type="submit">应用</button>
                        </form>
                        <form class="pid-row" onsubmit="submitPid(event, 'gyro')">
                            <span class="pid-row-label">GYRO</span>
                            <input class="pid-input" id="pid-gyro-kp" type="number" step="0.001" placeholder="KP">
                            <input class="pid-input" id="pid-gyro-ki" type="number" step="0.001" placeholder="KI">
                            <input class="pid-input" id="pid-gyro-kd" type="number" step="0.001" placeholder="KD">
                            <button type="submit">应用</button>
                        </form>
                        <form class="pid-row" onsubmit="submitPid(event, 'angle')">
                            <span class="pid-row-label">ANGLE</span>
                            <input class="pid-input" id="pid-angle-kp" type="number" step="0.001" placeholder="KP">
                            <input class="pid-input" id="pid-angle-ki" type="number" step="0.001" placeholder="KI">
                            <input class="pid-input" id="pid-angle-kd" type="number" step="0.001" placeholder="KD">
                            <button type="submit">应用</button>
                        </form>
                        <form class="pid-row" onsubmit="submitPid(event, 'circle')">
                            <span class="pid-row-label">CIRCLE</span>
                            <input class="pid-input" id="pid-circle-kp" type="number" step="0.001" placeholder="KP">
                            <input class="pid-input" id="pid-circle-ki" type="number" step="0.001" placeholder="KI">
                            <input class="pid-input" id="pid-circle-kd" type="number" step="0.001" placeholder="KD">
                            <button type="submit">应用</button>
                        </form>
                    </div>
                </details>
            </div>
        </section>

        <section class="panel" id="panel-perspective">
            <div class="panel-header">
                <span class="panel-title">Perspective Lines</span>
                <div class="panel-actions">
                    <span>Black Background</span>
                    <button class="panel-action-btn" onclick="togglePanelFullscreen('panel-perspective')">全屏</button>
                </div>
            </div>
            <div class="panel-body">
                <img id="stream-perspective" class="stream" src="/perspective" alt="透视边线图">
            </div>
            <div class="info-bar">左下: 透视边线图，仅绘制边线与关键点</div>
        </section>

        <section class="panel" id="panel-crop">
            <div class="panel-header">
                <span class="panel-title">Model Crop</span>
                <div class="panel-actions">
                    <span>40 x 40</span>
                    <button class="panel-action-btn" onclick="togglePanelFullscreen('panel-crop')">全屏</button>
                </div>
            </div>
            <div class="panel-body">
                <img id="stream-crop" class="stream" src="/crop" alt="模型裁剪图">
            </div>
            <div class="info-bar">右下: 红块映射后裁出的模型输入图</div>
        </section>

        <section class="panel" id="panel-metrics-extra">
            <div class="panel-header">
                <span class="panel-title">Element / Speed / Debug</span>
                <div class="panel-actions">
                    <span class="warn" id="clock-hint">--</span>
                    <button class="panel-action-btn" onclick="togglePanelFullscreen('panel-metrics-extra')">全屏</button>
                </div>
            </div>
            <div id="metrics-extra" class="metrics">加载中...</div>
        </section>
    </div>

    <div class="toolbar">
        <button id="transport-all-btn" onclick="setTransportMode('all')">图像+参数</button>
        <button id="transport-image-btn" onclick="setTransportMode('image_only')">只传图像</button>
        <button id="transport-telemetry-btn" onclick="setTransportMode('telemetry_only')">只传参数</button>
        <input type="text" id="filenamePrefix" value="snapshot" placeholder="snapshot" />
        <button onclick="takeSnapshot('main')">保存原图</button>
        <button onclick="takeSnapshot('crop')">保存裁剪图</button>
        <button onclick="sendControl('start')">发车</button>
        <button onclick="sendControl('delay_start')">延时发车</button>
        <button onclick="sendControl('stop')">停车</button>
        <button onclick="sendControl('reset_element')">元素重置状态</button>
        <button onclick="reconnectStreams()">重连图传</button>
    </div>

    <script>
        const mainImg = document.getElementById('stream-main');
        const perspectiveImg = document.getElementById('stream-perspective');
        const cropImg = document.getElementById('stream-crop');
        document.getElementById('main-url').textContent = '/stream';
        let transportMode = 'all';
        let lastAppliedTransportMode = '';

        const savedPrefix = localStorage.getItem('filenamePrefix') || 'snapshot';
        document.getElementById('filenamePrefix').value = savedPrefix;
        const pidStepInput = document.getElementById('pid-step-input');
        pidStepInput.value = localStorage.getItem('pidStepValue') || '0.001';

        document.getElementById('filenamePrefix').addEventListener('change', function() {
            const prefix = this.value.trim() || 'snapshot';
            localStorage.setItem('filenamePrefix', prefix);
        });

        function applyPidStepValue() {
            const raw = Number(pidStepInput.value);
            const stepValue = Number.isFinite(raw) && raw > 0 ? raw : 0.001;
            pidStepInput.value = stepValue.toFixed(4).replace(/0+$/, '').replace(/\.$/, '');
            localStorage.setItem('pidStepValue', pidStepInput.value);
            document.querySelectorAll('.pid-input').forEach((input) => {
                input.step = pidStepInput.value;
            });
        }

        pidStepInput.addEventListener('change', applyPidStepValue);
        applyPidStepValue();

        function takeSnapshot(target) {
            const prefix = document.getElementById('filenamePrefix').value.trim() || 'snapshot';
            const now = new Date();
            const year = now.getFullYear();
            const month = String(now.getMonth() + 1).padStart(2, '0');
            const day = String(now.getDate()).padStart(2, '0');
            const hour = String(now.getHours()).padStart(2, '0');
            const minute = String(now.getMinutes()).padStart(2, '0');
            const second = String(now.getSeconds()).padStart(2, '0');
            const suffix = target === 'crop' ? '_crop' : '_main';
            const filename = `${prefix}${suffix}_${year}${month}${day}_${hour}${minute}${second}.png`;

            const a = document.createElement('a');
            a.href = `/snapshot?prefix=${encodeURIComponent(prefix)}&target=${encodeURIComponent(target || 'main')}`;
            a.download = filename;
            a.style.display = 'none';
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
        }

        function reconnectStreams() {
            if (transportMode === 'telemetry_only') {
                mainImg.removeAttribute('src');
                perspectiveImg.removeAttribute('src');
                cropImg.removeAttribute('src');
                return;
            }
            const t = new Date().getTime();
            mainImg.src = `/stream?t=${t}`;
            perspectiveImg.src = `/perspective?t=${t}`;
            cropImg.src = `/crop?t=${t}`;
        }

        function svgPlaceholder(text) {
            const svg = `
                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1280 720">
                    <rect width="1280" height="720" fill="#04070a"/>
                    <text x="50%" y="50%" dominant-baseline="middle" text-anchor="middle"
                          fill="#7ed0ff" font-size="40" font-family="monospace">${text}</text>
                </svg>`;
            return `data:image/svg+xml;charset=utf-8,${encodeURIComponent(svg)}`;
        }

        function applyTransportModeUi() {
            const modeChanged = transportMode !== lastAppliedTransportMode;
            lastAppliedTransportMode = transportMode;

            document.getElementById('transport-all-btn').classList.toggle('active-mode', transportMode === 'all');
            document.getElementById('transport-image-btn').classList.toggle('active-mode', transportMode === 'image_only');
            document.getElementById('transport-telemetry-btn').classList.toggle('active-mode', transportMode === 'telemetry_only');

            if (transportMode === 'telemetry_only') {
                mainImg.src = svgPlaceholder('图像传输已关闭');
                perspectiveImg.src = svgPlaceholder('图像传输已关闭');
                cropImg.src = svgPlaceholder('图像传输已关闭');
            } else if (modeChanged) {
                reconnectStreams();
            }

            if (transportMode === 'image_only') {
                document.getElementById('metrics-main').textContent = '当前模式为只传图像，参数轮询已关闭。';
                document.getElementById('metrics-extra').textContent = '当前模式为只传图像，参数轮询已关闭。';
                document.getElementById('latency').textContent = '--';
            }
        }

        async function syncTransportMode() {
            try {
                const response = await fetch('/transport_mode');
                if (!response.ok) throw new Error('mode fetch failed');
                const data = await response.json();
                transportMode = data.transportMode || 'all';
                applyTransportModeUi();
            } catch (err) {
                document.getElementById('clock-hint').textContent = '图传模式读取失败';
            }
        }

        async function setTransportMode(mode) {
            try {
                const response = await fetch(`/transport_mode?mode=${encodeURIComponent(mode)}`);
                if (!response.ok) throw new Error('mode update failed');
                const data = await response.json();
                transportMode = data.transportMode || mode;
                applyTransportModeUi();
                document.getElementById('clock-hint').textContent = data.message || '图传模式已更新';
                if (transportMode !== 'image_only') {
                    updateStats();
                }
            } catch (err) {
                document.getElementById('clock-hint').textContent = '图传模式更新失败';
            }
        }

        async function sendControl(action) {
            try {
                const response = await fetch(`/control?action=${encodeURIComponent(action)}`);
                if (!response.ok) throw new Error('control failed');
                const data = await response.json();
                const hintEl = document.getElementById('clock-hint');
                hintEl.textContent = data.message || '控制已发送';
                updateStats();
            } catch (err) {
                document.getElementById('clock-hint').textContent = '控制失败';
            }
        }

        function togglePanelFullscreen(id) {
            const target = document.getElementById(id);
            if (!document.fullscreenElement) {
                target.requestFullscreen();
            } else {
                document.exitFullscreen();
            }
        }

        document.addEventListener('keydown', function(event) {
            const activeElement = document.activeElement;
            if (activeElement && (activeElement.tagName === 'INPUT' || activeElement.tagName === 'TEXTAREA')) {
                return;
            }
            if (event.key === 'k' || event.key === 'K') {
                event.preventDefault();
                takeSnapshot('main');
            }
            else if (event.key === 'j' || event.key === 'J') {
                event.preventDefault();
                takeSnapshot('crop');
            }
            else if (event.key === 'f' || event.key === 'F') {
                event.preventDefault();
                togglePanelFullscreen('panel-main');
            }
            else if (event.key === 'p' || event.key === 'P') {
                event.preventDefault();
                togglePanelFullscreen('panel-perspective');
            }
            else if (event.key === 'c' || event.key === 'C') {
                event.preventDefault();
                togglePanelFullscreen('panel-crop');
            }
            else if (event.key === 'r' || event.key === 'R') {
                event.preventDefault();
                reconnectStreams();
            }
        });

        function formatNumber(value, digits = 1) {
            const num = Number(value);
            return Number.isFinite(num) ? num.toFixed(digits) : '--';
        }

        function formatInt(value) {
            const num = Number(value);
            return Number.isFinite(num) ? String(Math.round(num)) : '--';
        }

        function formatPoint(found, x, y) {
            return Number(found) ? `(${formatNumber(x, 1)}, ${formatNumber(y, 1)})` : '--';
        }

        function syncPidInput(id, value) {
            const input = document.getElementById(id);
            if (!input || document.activeElement === input) {
                return;
            }
            input.value = Number(value).toFixed(3);
        }

        function syncPidInputs(data) {
            syncPidInput('pid-speed_l-kp', data.pidSpeedLKp);
            syncPidInput('pid-speed_l-ki', data.pidSpeedLKi);
            syncPidInput('pid-speed_l-kd', data.pidSpeedLKd);
            syncPidInput('pid-speed_r-kp', data.pidSpeedRKp);
            syncPidInput('pid-speed_r-ki', data.pidSpeedRKi);
            syncPidInput('pid-speed_r-kd', data.pidSpeedRKd);
            syncPidInput('pid-gyro-kp', data.pidGyroKp);
            syncPidInput('pid-gyro-ki', data.pidGyroKi);
            syncPidInput('pid-gyro-kd', data.pidGyroKd);
            syncPidInput('pid-angle-kp', data.pidAngleKp);
            syncPidInput('pid-angle-ki', data.pidAngleKi);
            syncPidInput('pid-angle-kd', data.pidAngleKd);
            syncPidInput('pid-circle-kp', data.pidCircleKp);
            syncPidInput('pid-circle-ki', data.pidCircleKi);
            syncPidInput('pid-circle-kd', data.pidCircleKd);
        }

        async function submitPid(event, group) {
            event.preventDefault();
            const kp = document.getElementById(`pid-${group}-kp`).value;
            const ki = document.getElementById(`pid-${group}-ki`).value;
            const kd = document.getElementById(`pid-${group}-kd`).value;

            try {
                const response = await fetch(`/pid?group=${encodeURIComponent(group)}&kp=${encodeURIComponent(kp)}&ki=${encodeURIComponent(ki)}&kd=${encodeURIComponent(kd)}`);
                if (!response.ok) throw new Error('pid update failed');
                const data = await response.json();
                document.getElementById('clock-hint').textContent = data.message || 'PID 已更新';
                updateStats();
            } catch (err) {
                document.getElementById('clock-hint').textContent = 'PID 更新失败';
            }
        }

        function metricLine(label, value, valueClass = '') {
            return `<div class="metrics-line"><span class="metrics-label">${label}</span><span class="metrics-value ${valueClass}">${value}</span></div>`;
        }

        function metricSection(title, rows) {
            return `<div class="metrics-section"><div class="metrics-title">${title}</div>${rows.join('')}</div>`;
        }

        async function updateStats() {
            if (transportMode === 'image_only') {
                return;
            }
            try {
                const response = await fetch('/stats');
                if (!response.ok) throw new Error('stats fetch failed');
                const data = await response.json();
                transportMode = data.transportMode || transportMode;
                applyTransportModeUi();
                const latencyEl = document.getElementById('latency');
                const fpsEl = document.getElementById('fps-top');
                const hintEl = document.getElementById('clock-hint');
                const mainPanelEl = document.getElementById('metrics-main');
                const extraPanelEl = document.getElementById('metrics-extra');
                hintEl.textContent = '--';
                syncPidInputs(data);

                const captureTs = Number(data.latestCaptureTsMs) || 0;
                const serverTs = Number(data.serverTsMs) || 0;
                const browserNow = Date.now();

                if (captureTs && serverTs) {
                    const internalLatency = Math.max(0, serverTs - captureTs);
                    const clockOffset = browserNow - serverTs;
                    const networkLatency = Math.max(0, clockOffset);
                    if (Math.abs(clockOffset) > 2000) {
                        latencyEl.textContent = internalLatency + ' ms (板载)';
                        hintEl.textContent = '板载与浏览器时钟未同步';
                    } else {
                        const endToEnd = internalLatency + networkLatency;
                        latencyEl.textContent = endToEnd + ' ms';
                    }
                } else {
                    latencyEl.textContent = '--';
                }

                if (data.estimatedFps && data.estimatedFps > 0) {
                    fpsEl.textContent = Number(data.estimatedFps).toFixed(1) + ' FPS';
                } else {
                    fpsEl.textContent = '-- FPS';
                }

                const startText = Number(data.startFlag) ? 'RUN' : 'STOP';
                const trackText = data.trackName || '--';
                const elemText = data.elemName || '--';
                const crossText = data.crossName || '--';
                const circleText = data.circleName || '--';
                const circleTypeText = data.circleTypeName || '--';
                const classifyLabelText = data.classifyLabel || '--';
                const classifyProbsText = data.classifyProbsText || '--';
                const wheelSpeedGap = (Number(data.actualSpeedR) || 0) - (Number(data.actualSpeedL) || 0);

                mainPanelEl.innerHTML = [
                    metricSection('=== TARGET DEBUG ===', [
                        metricLine('LAB   L_VAL:', `${formatNumber(data.targetSpeedL)}   R_VAL: ${formatNumber(data.targetSpeedR)}`, 'cyan'),
                        metricLine('SPD   L_ACT:', `${formatNumber(data.actualSpeedL)}   R_ACT: ${formatNumber(data.actualSpeedR)}`, 'orange'),
                        metricLine('GYRO  TARGET:', `${formatNumber(data.targetGyro, 2)}   ACTUAL: ${formatNumber(data.actualGyro, 2)}`, 'pink'),
                        metricLine('PWM   L/R:', `${formatNumber(data.pwmL, 0)} / ${formatNumber(data.pwmR, 0)}`, 'yellow'),
                        metricLine('STAT  RUN:', `${startText}   TRACK: ${trackText}`, startText === 'RUN' ? 'cyan' : 'danger'),
                        metricLine('LOOK  AHEAD:', `${formatInt(data.lookAheadPoint)}`, 'orange'),
                        metricLine('PURE  ANGLE:', `${formatNumber(data.pureAngle, 2)}`, 'yellow')
                    ]),
                    metricSection('=== PID DEBUG ===', [
                        metricLine('SPD_L KP/KI/KD:', `${formatNumber(data.pidSpeedLKp, 2)} / ${formatNumber(data.pidSpeedLKi, 2)} / ${formatNumber(data.pidSpeedLKd, 3)}`, 'cyan'),
                        metricLine('SPD_R KP/KI/KD:', `${formatNumber(data.pidSpeedRKp, 2)} / ${formatNumber(data.pidSpeedRKi, 2)} / ${formatNumber(data.pidSpeedRKd, 3)}`, 'cyan'),
                        metricLine('GYRO  KP/KI/KD:', `${formatNumber(data.pidGyroKp, 2)} / ${formatNumber(data.pidGyroKi, 2)} / ${formatNumber(data.pidGyroKd, 3)}`, 'orange'),
                        metricLine('ANGLE KP/KI/KD:', `${formatNumber(data.pidAngleKp, 2)} / ${formatNumber(data.pidAngleKi, 2)} / ${formatNumber(data.pidAngleKd, 3)}`, 'pink'),
                        metricLine('CIRCL KP/KI/KD:', `${formatNumber(data.pidCircleKp, 2)} / ${formatNumber(data.pidCircleKi, 2)} / ${formatNumber(data.pidCircleKd, 3)}`, 'yellow')
                    ])
                ].join('');

                extraPanelEl.innerHTML = [
                    metricSection('=== VEHICLE DEBUG ===', [
                        metricLine('L_WHEEL SPEED:', `${formatNumber(data.actualSpeedL)}`, 'cyan'),
                        metricLine('R_WHEEL SPEED:', `${formatNumber(data.actualSpeedR)}`, 'cyan'),
                        metricLine('WHEEL GAP    :', `${formatNumber(wheelSpeedGap, 2)}`, 'orange'),
                        metricLine('ANGLE SPEED  :', `${formatNumber(data.actualGyro, 2)}`, 'pink'),
                        metricLine('FRAME ID     :', `${formatInt(data.latestFrameId)}`, 'yellow'),
                        metricLine('LATENCY      :', `${latencyEl.textContent}`, 'yellow')
                    ]),
                    metricSection('=== ELEMENT DEBUG ===', [
                        metricLine('TYPE         :', elemText, 'cyan'),
                        metricLine('CROSS        :', crossText, 'orange'),
                        metricLine('CIRCLE       :', circleText, 'orange'),
                        metricLine('CIRCLE DIR   :', circleTypeText, 'pink')
                    ]),
                    metricSection('=== CLASSIFIER ===', [
                        metricLine('TOP1 LABEL   :', classifyLabelText, 'cyan'),
                        metricLine('TOP1 SCORE   :', `${formatNumber(data.classifyScore, 6)}`, 'orange'),
                        metricLine('PROBS        :', classifyProbsText, 'pink')
                    ]),
                    metricSection('=== RED MODEL DEBUG ===', [
                        metricLine('RED DETECT   :', `${formatInt(data.redDetectedThisFrame)}`, 'cyan'),
                        metricLine('WAIT DIST    :', `${formatInt(data.redWaitingDistance)}`, 'orange'),
                        metricLine('CYCLE DONE   :', `${formatInt(data.redCycleDone)}`, 'pink'),
                        metricLine('MODEL ENABLE :', `${formatInt(data.redClassifyEnabled)}`, 'yellow'),
                        metricLine('INPUT READY  :', `${formatInt(data.redModelInputReady)}`, 'yellow'),
                        metricLine('EST DIST CM  :', `${formatNumber(data.redEstimatedDistanceCm, 2)}`, 'cyan'),
                        metricLine('TRIG DIST CM :', `${formatNumber(data.redTriggerDistanceCm, 2)}`, 'orange'),
                        metricLine('PX PER CM    :', `${formatNumber(data.redPixelsPerCm, 2)}`, 'pink'),
                        metricLine('RED BOX      :', `(${formatInt(data.redBoxX)}, ${formatInt(data.redBoxY)}, ${formatInt(data.redBoxW)}, ${formatInt(data.redBoxH)})`, 'cyan'),
                        metricLine('MODEL BOX    :', `(${formatInt(data.redModelBoxX)}, ${formatInt(data.redModelBoxY)}, ${formatInt(data.redModelBoxW)}, ${formatInt(data.redModelBoxH)})`, 'orange')
                    ]),
                    metricSection('=== L POINT DEBUG ===', [
                        metricLine('NEAR LEFT    :', `${formatInt(data.lpt0Found)} #${formatInt(data.lpt0Id)}`, 'cyan'),
                        metricLine('NEAR LEFT XY :', formatPoint(data.lpt0Found, data.lpt0X, data.lpt0Y), 'cyan'),
                        metricLine('NEAR RIGHT   :', `${formatInt(data.lpt1Found)} #${formatInt(data.lpt1Id)}`, 'cyan'),
                        metricLine('NEAR RIGHT XY:', formatPoint(data.lpt1Found, data.lpt1X, data.lpt1Y), 'cyan'),
                        metricLine('FAR LEFT     :', `${formatInt(data.farLpt0Found)} #${formatInt(data.farLpt0Id)}`, 'orange'),
                        metricLine('FAR LEFT XY  :', formatPoint(data.farLpt0Found, data.farLpt0X, data.farLpt0Y), 'orange'),
                        metricLine('FAR RIGHT    :', `${formatInt(data.farLpt1Found)} #${formatInt(data.farLpt1Id)}`, 'orange'),
                        metricLine('FAR RIGHT XY :', formatPoint(data.farLpt1Found, data.farLpt1X, data.farLpt1Y), 'orange')
                    ]),
                    metricSection('=== LINE COUNT ===', [
                        metricLine('IPTS L / R   :', `${formatInt(data.ipts0Num)} / ${formatInt(data.ipts1Num)}`, 'cyan'),
                        metricLine('RPTS L / R   :', `${formatInt(data.rpts0sNum)} / ${formatInt(data.rpts1sNum)}`, 'pink'),
                        metricLine('CENTER       :', `${formatInt(data.rptsnNum)}`, 'yellow'),
                        metricLine('FAR L / R    :', `${formatInt(data.farRpts0sNum)} / ${formatInt(data.farRpts1sNum)}`, 'orange')
                    ])
                ].join('');
            } catch (err) {
                document.getElementById('latency').textContent = 'N/A';
                document.getElementById('fps-top').textContent = 'N/A';
                document.getElementById('clock-hint').textContent = '数据拉取失败';
            }
        }

        setInterval(updateStats, 250);
        syncTransportMode().then(updateStats);
    </script>
</body>
</html>
)HTML";

CameraStreamServer::CameraStreamServer(void)
    : server_sock_fd(-1)
    , server_port(CAMERA_STREAM_DEFAULT_PORT)
    , running(false)
    , server_thread_id(0)
    , latest_frame_id(0)
    , latest_capture_ts_ms(0)
    , ema_fps(0.0)
    , transport_mode(CAMERA_TRANSPORT_ALL)
{
    pthread_mutex_init(&frame_mutex, NULL);
    pthread_cond_init(&frame_cond, NULL);
    pthread_mutex_init(&sock_mutex, NULL);
    pthread_mutex_init(&original_frame_mutex, NULL);
    pthread_mutex_init(&telemetry_mutex, NULL);
    pthread_mutex_init(&mode_mutex, NULL);
}

CameraStreamServer::~CameraStreamServer(void)
{
    stop_server();
    pthread_mutex_destroy(&frame_mutex);
    pthread_cond_destroy(&frame_cond);
    pthread_mutex_destroy(&sock_mutex);
    pthread_mutex_destroy(&original_frame_mutex);
    pthread_mutex_destroy(&telemetry_mutex);
    pthread_mutex_destroy(&mode_mutex);
}


/*******************************************************************
 * @brief       获取本机IP地址
 * 
 * @return      返回本机IP地址字符串
 * 
 * @note        自动选择优先级最高的网络接口IP
 ******************************************************************/
std::string CameraStreamServer::get_local_ip(void)
{
    struct ifaddrs *ifaddr, *ifa;
    char host[NI_MAXHOST];
    
    if (getifaddrs(&ifaddr) == -1) {
        return "127.0.0.1";
    }
    
    std::string result = "127.0.0.1";
    std::string fallback_ip = "";
    int best_priority = -1;
    
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL) continue;
        
        int family = ifa->ifa_addr->sa_family;
        
        // 只处理IPv4地址
        if (family == AF_INET) {
            // 跳过回环地址
            if (strcmp(ifa->ifa_name, "lo") == 0) continue;
            
            int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                              host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (s != 0) continue;
            
            // 计算接口优先级
            int priority = 0;
            bool is_up = (ifa->ifa_flags & IFF_UP) != 0;
            bool is_running = (ifa->ifa_flags & IFF_RUNNING) != 0;
            
            // UP 且 RUNNING 的接口优先级最高
            if (is_up && is_running) {
                priority = 100;
                // wlan/eth 接口额外加分
                if (strncmp(ifa->ifa_name, "wlan", 4) == 0) priority += 20;
                else if (strncmp(ifa->ifa_name, "eth", 3) == 0) priority += 15;
                else if (strncmp(ifa->ifa_name, "en", 2) == 0) priority += 15; // macOS/BSD
                else priority += 5; // 其他接口
            } 
            // 只有 UP 没有 RUNNING 的接口作为备选
            else if (is_up) {
                priority = 50;
                if (strncmp(ifa->ifa_name, "wlan", 4) == 0) priority += 10;
                else if (strncmp(ifa->ifa_name, "eth", 3) == 0) priority += 8;
                else if (strncmp(ifa->ifa_name, "en", 2) == 0) priority += 8;
            }
            // 其他情况优先级很低
            else {
                priority = 10;
            }
            
            // 选择优先级最高的接口
            if (priority > best_priority) {
                best_priority = priority;
                result = host;
            }
            
            // 保存第一个有效IP作为最终备选
            if (fallback_ip.empty() && strcmp(host, "127.0.0.1") != 0) {
                fallback_ip = host;
            }
        }
    }
    
    freeifaddrs(ifaddr);
    
    // 如果没找到合适的，使用备选IP
    if (result == "127.0.0.1" && !fallback_ip.empty()) {
        result = fallback_ip;
    }
    
    return result;
}

/*******************************************************************
 * @brief       关闭服务器socket
 * 
 * @note        线程安全的关闭操作
 ******************************************************************/
void CameraStreamServer::close_server_socket(void)
{
    pthread_mutex_lock(&sock_mutex);
    if (server_sock_fd >= 0) {
        shutdown(server_sock_fd, SHUT_RDWR);
        close(server_sock_fd);
        server_sock_fd = -1;
    }
    pthread_mutex_unlock(&sock_mutex);
}

/*******************************************************************
 * @brief       获取当前时间戳(毫秒)
 * 
 * @return      返回当前时间戳(毫秒)
 ******************************************************************/
uint64_t CameraStreamServer::now_ms(void)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

/*******************************************************************
 * @brief       格式化时间戳
 * 
 * @param       ts_ms           时间戳(毫秒)
 * 
 * @return      返回格式化后的时间字符串
 ******************************************************************/
std::string CameraStreamServer::format_timestamp(uint64_t ts_ms)
{
    if (ts_ms == 0) return "--";
    time_t seconds = static_cast<time_t>(ts_ms / 1000);
    int ms = static_cast<int>(ts_ms % 1000);
    struct tm tm_time;
    localtime_r(&seconds, &tm_time);
    char date_buf[64];
    strftime(date_buf, sizeof(date_buf), "%Y-%m-%d %H:%M:%S", &tm_time);
    char buf[80];
    snprintf(buf, sizeof(buf), "%s.%03d", date_buf, ms);
    return std::string(buf);
}

/*******************************************************************
 * @brief       发送HTTP响应
 * 
 * @param       sock            客户端socket
 * @param       content_type    内容类型
 * @param       body            响应体
 * @param       body_len        响应体长度
 ******************************************************************/
void CameraStreamServer::send_response(int sock, const char* content_type, const char* body, size_t body_len)
{
    std::ostringstream header;
    header << "HTTP/1.1 200 OK\r\n";
    header << "Content-Type: " << content_type << "\r\n";
    header << "Content-Length: " << body_len << "\r\n";
    header << "Connection: close\r\n\r\n";
    std::string h = header.str();
    send(sock, h.c_str(), h.length(), 0);
    send(sock, body, body_len, 0);
}

void CameraStreamServer::set_transport_mode(CameraTransportMode mode)
{
    pthread_mutex_lock(&mode_mutex);
    transport_mode = mode;
    pthread_mutex_unlock(&mode_mutex);
    pthread_cond_broadcast(&frame_cond);
}

CameraTransportMode CameraStreamServer::get_transport_mode(void)
{
    pthread_mutex_lock(&mode_mutex);
    const CameraTransportMode mode = transport_mode;
    pthread_mutex_unlock(&mode_mutex);
    return mode;
}

/*******************************************************************
 * @brief       发送统计信息响应
 * 
 * @param       sock            客户端socket
 ******************************************************************/
void CameraStreamServer::send_stats_response(int sock)
{
    const CameraTransportMode mode = get_transport_mode();
    uint64_t capture_ts = latest_capture_ts_ms;
    uint64_t frame_id = 0;
    CameraTelemetry telemetry_copy;
    pthread_mutex_lock(&frame_mutex);
    frame_id = latest_frame_id;
    pthread_mutex_unlock(&frame_mutex);
    pthread_mutex_lock(&telemetry_mutex);
    telemetry_copy = telemetry;
    pthread_mutex_unlock(&telemetry_mutex);

    uint64_t server_ts = now_ms();
    double fps = ema_fps;
    Json::Value root;
    root["latestFrameId"] = Json::UInt64(frame_id);
    root["latestCaptureTsMs"] = Json::UInt64(capture_ts);
    root["serverTsMs"] = Json::UInt64(server_ts);
    root["estimatedFps"] = fps;
    root["transportMode"] = transport_mode_name(mode);
    root["imageEnabled"] = is_image_transport_enabled(mode);
    root["telemetryEnabled"] = is_telemetry_transport_enabled(mode);

    if (!is_telemetry_transport_enabled(mode)) {
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";
        std::string json = Json::writeString(builder, root);
        send_response(sock, "application/json; charset=utf-8", json.c_str(), json.size());
        return;
    }

    root["targetSpeedL"] = telemetry_copy.target_speed_l;
    root["targetSpeedR"] = telemetry_copy.target_speed_r;
    root["actualSpeedL"] = telemetry_copy.actual_speed_l;
    root["actualSpeedR"] = telemetry_copy.actual_speed_r;
    root["pwmL"] = telemetry_copy.pwm_l;
    root["pwmR"] = telemetry_copy.pwm_r;
    root["targetGyro"] = telemetry_copy.target_gyro;
    root["actualGyro"] = telemetry_copy.actual_gyro;
    root["pureAngle"] = telemetry_copy.pure_angle;

    root["pidSpeedLKp"] = telemetry_copy.pid_speed_l_kp;
    root["pidSpeedLKi"] = telemetry_copy.pid_speed_l_ki;
    root["pidSpeedLKd"] = telemetry_copy.pid_speed_l_kd;
    root["pidSpeedRKp"] = telemetry_copy.pid_speed_r_kp;
    root["pidSpeedRKi"] = telemetry_copy.pid_speed_r_ki;
    root["pidSpeedRKd"] = telemetry_copy.pid_speed_r_kd;
    root["pidGyroKp"] = telemetry_copy.pid_gyro_kp;
    root["pidGyroKi"] = telemetry_copy.pid_gyro_ki;
    root["pidGyroKd"] = telemetry_copy.pid_gyro_kd;
    root["pidAngleKp"] = telemetry_copy.pid_angle_kp;
    root["pidAngleKi"] = telemetry_copy.pid_angle_ki;
    root["pidAngleKd"] = telemetry_copy.pid_angle_kd;
    root["pidCircleKp"] = telemetry_copy.pid_circle_kp;
    root["pidCircleKi"] = telemetry_copy.pid_circle_ki;
    root["pidCircleKd"] = telemetry_copy.pid_circle_kd;

    root["startFlag"] = telemetry_copy.start_flag;
    root["lookAheadPoint"] = telemetry_copy.look_ahead_point;
    root["trackSide"] = telemetry_copy.track_side;
    root["elemType"] = telemetry_copy.elem_type;
    root["crossState"] = telemetry_copy.cross_state;
    root["circleState"] = telemetry_copy.circle_state;
    root["circleType"] = telemetry_copy.circle_type;

    root["ipts0Num"] = telemetry_copy.ipts0_num;
    root["ipts1Num"] = telemetry_copy.ipts1_num;
    root["rptsnNum"] = telemetry_copy.rptsn_num;
    root["rpts0sNum"] = telemetry_copy.rpts0s_num;
    root["rpts1sNum"] = telemetry_copy.rpts1s_num;
    root["farRpts0sNum"] = telemetry_copy.far_rpts0s_num;
    root["farRpts1sNum"] = telemetry_copy.far_rpts1s_num;

    root["lpt0Found"] = telemetry_copy.lpt0_found;
    root["lpt1Found"] = telemetry_copy.lpt1_found;
    root["lpt0Id"] = telemetry_copy.lpt0_id;
    root["lpt1Id"] = telemetry_copy.lpt1_id;
    root["lpt0X"] = telemetry_copy.lpt0_x;
    root["lpt0Y"] = telemetry_copy.lpt0_y;
    root["lpt1X"] = telemetry_copy.lpt1_x;
    root["lpt1Y"] = telemetry_copy.lpt1_y;
    root["farLpt0Found"] = telemetry_copy.far_lpt0_found;
    root["farLpt1Found"] = telemetry_copy.far_lpt1_found;
    root["farLpt0Id"] = telemetry_copy.far_lpt0_id;
    root["farLpt1Id"] = telemetry_copy.far_lpt1_id;
    root["farLpt0X"] = telemetry_copy.far_lpt0_x;
    root["farLpt0Y"] = telemetry_copy.far_lpt0_y;
    root["farLpt1X"] = telemetry_copy.far_lpt1_x;
    root["farLpt1Y"] = telemetry_copy.far_lpt1_y;

    root["elemName"] = telemetry_copy.elem_name;
    root["crossName"] = telemetry_copy.cross_name;
    root["circleName"] = telemetry_copy.circle_name;
    root["trackName"] = telemetry_copy.track_name;
    root["circleTypeName"] = telemetry_copy.circle_type_name;
    root["classifyLabel"] = telemetry_copy.classify_label;
    root["classifyScore"] = telemetry_copy.classify_score;
    root["classifyProbsText"] = telemetry_copy.classify_probs_text;
    root["redDetectedThisFrame"] = telemetry_copy.red_detected_this_frame;
    root["redWaitingDistance"] = telemetry_copy.red_waiting_distance;
    root["redCycleDone"] = telemetry_copy.red_cycle_done;
    root["redClassifyEnabled"] = telemetry_copy.red_classify_enabled;
    root["redModelInputReady"] = telemetry_copy.red_model_input_ready;
    root["redEstimatedDistanceCm"] = telemetry_copy.red_estimated_distance_cm;
    root["redTriggerDistanceCm"] = telemetry_copy.red_trigger_distance_cm;
    root["redPixelsPerCm"] = telemetry_copy.red_pixels_per_cm;
    root["redBoxX"] = telemetry_copy.red_box_x;
    root["redBoxY"] = telemetry_copy.red_box_y;
    root["redBoxW"] = telemetry_copy.red_box_w;
    root["redBoxH"] = telemetry_copy.red_box_h;
    root["redModelBoxX"] = telemetry_copy.red_model_box_x;
    root["redModelBoxY"] = telemetry_copy.red_model_box_y;
    root["redModelBoxW"] = telemetry_copy.red_model_box_w;
    root["redModelBoxH"] = telemetry_copy.red_model_box_h;

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    std::string json = Json::writeString(builder, root);
    send_response(sock, "application/json; charset=utf-8", json.c_str(), json.size());
}

/*******************************************************************
 * @brief       发送MJPEG流
 * 
 * @param       sock            客户端socket
 ******************************************************************/
void CameraStreamServer::send_mjpeg_stream(int sock, bool perspective, bool crop)
{
    if (!is_image_transport_enabled(get_transport_mode())) {
        const char* body = "image transport disabled";
        send_response(sock, "text/plain; charset=utf-8", body, strlen(body));
        return;
    }

    const char* header = 
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n";
    send(sock, header, strlen(header), 0);
    
    uint64_t last_frame_sent = 0;

    while (running) {
        std::vector<unsigned char> jpeg_copy;

        pthread_mutex_lock(&frame_mutex);
        while (running && latest_frame_id == last_frame_sent) {
            pthread_cond_wait(&frame_cond, &frame_mutex);
        }

        if (!running) {
            pthread_mutex_unlock(&frame_mutex);
            break;
        }

        const std::vector<unsigned char>& active_jpeg = crop
            ? current_crop_jpeg
            : (perspective ? current_perspective_jpeg : current_jpeg);
        if (active_jpeg.empty()) {
            pthread_mutex_unlock(&frame_mutex);
            continue;
        }

        jpeg_copy = active_jpeg;
        last_frame_sent = latest_frame_id;
        pthread_mutex_unlock(&frame_mutex);
        
        char boundary[256];
        snprintf(boundary, sizeof(boundary),
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                jpeg_copy.size());
        
        if (send(sock, boundary, strlen(boundary), 0) < 0) break;
        if (send(sock, jpeg_copy.data(), jpeg_copy.size(), 0) < 0) break;
        if (send(sock, "\r\n", 2, 0) < 0) break;
    }
}

void CameraStreamServer::handle_control_request(int sock, const std::string& action)
{
    bool ok = true;
    std::string message;

    if (action == "start") {
        reset_element_state();
        start__1000ms_flag = 0;
        __1000ms = 0;
        start_flag = 1;
        message = "已发车";
    } else if (action == "delay_start") {
        reset_element_state();
        start_flag = 0;
        __1000ms = 0;
        start__1000ms_flag = 1;
        g_pwm_l_dbg = 0.0f;
        g_pwm_r_dbg = 0.0f;
        motor.set_motor1(0, 0);
        motor.set_motor2(0, 0);
        brush.set_duty(0);
        message = "已进入延时发车";
    } else if (action == "stop") {
        start_flag = 0;
        start__1000ms_flag = 0;
        __1000ms = 0;
        g_pwm_l_dbg = 0.0f;
        g_pwm_r_dbg = 0.0f;
        motor.set_motor1(0, 0);
        motor.set_motor2(0, 0);
        brush.set_duty(0);
        message = "已停车";
    } else if (action == "reset_element") {
        reset_element_state();
        message = "元素状态已重置";
    } else {
        ok = false;
        message = "未知控制命令";
    }

    Json::Value root;
    root["ok"] = ok;
    root["message"] = message;
    root["startFlag"] = start_flag;
    root["delayStartFlag"] = start__1000ms_flag;
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    std::string json = Json::writeString(builder, root);
    send_response(sock, "application/json; charset=utf-8", json.c_str(), json.size());
}

void CameraStreamServer::handle_pid_request(int sock, const std::string& path)
{
    bool ok = false;
    std::string message;

    const std::string group = get_query_param(path, "group");
    const std::string kp_text = get_query_param(path, "kp");
    const std::string ki_text = get_query_param(path, "ki");
    const std::string kd_text = get_query_param(path, "kd");

    char* end = nullptr;
    const float kp = strtof(kp_text.c_str(), &end);
    const bool kp_ok = !kp_text.empty() && end != kp_text.c_str() && *end == '\0';
    end = nullptr;
    const float ki = strtof(ki_text.c_str(), &end);
    const bool ki_ok = !ki_text.empty() && end != ki_text.c_str() && *end == '\0';
    end = nullptr;
    const float kd = strtof(kd_text.c_str(), &end);
    const bool kd_ok = !kd_text.empty() && end != kd_text.c_str() && *end == '\0';

    if (!group.empty() && kp_ok && ki_ok && kd_ok &&
        std::isfinite(kp) && std::isfinite(ki) && std::isfinite(kd)) {
        ok = update_pid_params(group, kp, ki, kd, message);
    } else {
        message = "PID 参数非法";
    }

    Json::Value root;
    root["ok"] = ok;
    root["message"] = message;
    root["group"] = group;
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    std::string json = Json::writeString(builder, root);
    send_response(sock, "application/json; charset=utf-8", json.c_str(), json.size());
}

void CameraStreamServer::handle_transport_mode_request(int sock, const std::string& path)
{
    bool ok = true;
    std::string message;
    const std::string mode_text = get_query_param(path, "mode");
    CameraTransportMode mode = CAMERA_TRANSPORT_ALL;

    if (!mode_text.empty()) {
        if (parse_transport_mode(mode_text, mode)) {
            set_transport_mode(mode);
            message = "图传模式已更新";
        } else {
            ok = false;
            message = "图传模式非法";
            mode = get_transport_mode();
        }
    } else {
        mode = get_transport_mode();
        message = "当前图传模式";
    }

    Json::Value root;
    root["ok"] = ok;
    root["message"] = message;
    root["transportMode"] = transport_mode_name(mode);
    root["imageEnabled"] = is_image_transport_enabled(mode);
    root["telemetryEnabled"] = is_telemetry_transport_enabled(mode);
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    std::string json = Json::writeString(builder, root);
    send_response(sock, "application/json; charset=utf-8", json.c_str(), json.size());
}

/*******************************************************************
 * @brief       处理拍照请求(发送原始高质量图片到客户端)
 * 
 * @param       sock            客户端socket
 * @param       prefix          文件名前缀
 ******************************************************************/
void CameraStreamServer::handle_snapshot_request(int sock, const std::string& prefix, bool crop)
{
    pthread_mutex_lock(&original_frame_mutex);
    cv::Mat frame_copy = crop ? crop_frame.clone() : original_frame.clone();
    pthread_mutex_unlock(&original_frame_mutex);
    
    if (frame_copy.empty()) {
        const char* error_html = crop
            ? "<h1>Error</h1><p>没有可用的裁剪图像帧</p>"
            : "<h1>Error</h1><p>没有可用的图像帧</p>";
        send_response(sock, "text/html; charset=utf-8", error_html, strlen(error_html));
        return;
    }
    
    // 编码为PNG无损格式（用于数据集采集）
    std::vector<unsigned char> image_buffer;
    std::vector<int> params;
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(3);
    
    if (!cv::imencode(".png", frame_copy, image_buffer, params)) {
        const char* error_html = "<h1>Error</h1><p>图像编码失败</p>";
        send_response(sock, "text/html; charset=utf-8", error_html, strlen(error_html));
        std::cerr << "✗ 图像编码失败" << std::endl;
        return;
    }
    
    // 生成文件名（使用自定义前缀）
    time_t now = time(NULL);
    struct tm tm_time;
    localtime_r(&now, &tm_time);
    char filename[256];
    snprintf(filename, sizeof(filename), "%s_%s_%04d%02d%02d_%02d%02d%02d.png",
             prefix.c_str(),
             crop ? "crop" : "main",
             tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday,
             tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);
    
    // 发送HTTP响应头
    std::ostringstream header;
    header << "HTTP/1.1 200 OK\r\n";
    header << "Content-Type: image/png\r\n";
    header << "Content-Length: " << image_buffer.size() << "\r\n";
    header << "Content-Disposition: attachment; filename=\"" << filename << "\"\r\n";
    header << "Cache-Control: no-cache\r\n";
    header << "Connection: close\r\n\r\n";
    
    std::string h = header.str();
    send(sock, h.c_str(), h.length(), 0);
    send(sock, image_buffer.data(), image_buffer.size(), 0);
    
    std::cout << "✓ 已发送" << (crop ? "裁剪图" : "原图") << "到客户端: " << filename 
              << " (大小: " << image_buffer.size() / 1024 << " KB, PNG无损)" << std::endl;
}

/*******************************************************************
 * @brief       处理客户端HTTP请求
 * 
 * @param       sock            客户端socket
 ******************************************************************/
void CameraStreamServer::handle_client_request(int sock)
{
    char buffer[4096];
    ssize_t n = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (n <= 0) {
        close(sock);
        return;
    }
    buffer[n] = '\0';
    
    // 解析请求路径
    std::string request(buffer);
    size_t path_start = request.find(" ") + 1;
    size_t path_end = request.find(" ", path_start);
    std::string path = request.substr(path_start, path_end - path_start);
    
    if (path == "/" || path.find("/viewer") == 0 || path.find("/?") == 0) {
        // 返回HTML查看器
        send_response(sock, "text/html; charset=utf-8", viewer_html, strlen(viewer_html));
    } else if (path.find("/stream") == 0) {
        // 返回视频流
        send_mjpeg_stream(sock, false, false);
    } else if (path.find("/perspective") == 0) {
        // 返回透视边线图视频流
        send_mjpeg_stream(sock, true, false);
    } else if (path.find("/crop") == 0) {
        // 返回裁剪图视频流
        send_mjpeg_stream(sock, false, true);
    } else if (path.find("/stats") == 0) {
        send_stats_response(sock);
    } else if (path.find("/transport_mode") == 0) {
        handle_transport_mode_request(sock, path);
    } else if (path.find("/pid") == 0) {
        handle_pid_request(sock, path);
    } else if (path.find("/control") == 0) {
        std::string action;
        size_t query_pos = path.find("?action=");
        if (query_pos != std::string::npos) {
            size_t action_start = query_pos + 8;
            size_t action_end = path.find("&", action_start);
            if (action_end == std::string::npos) {
                action_end = path.length();
            }
            action = path.substr(action_start, action_end - action_start);
        }
        handle_control_request(sock, action);
    } else if (path.find("/snapshot") == 0) {
        // 解析文件名前缀参数
        std::string prefix = "snapshot";  // 默认前缀
        size_t query_pos = path.find("?prefix=");
        if (query_pos != std::string::npos) {
            size_t prefix_start = query_pos + 8;  // "?prefix=" 长度为8
            size_t prefix_end = path.find("&", prefix_start);
            if (prefix_end == std::string::npos) {
                prefix_end = path.length();
            }
            prefix = path.substr(prefix_start, prefix_end - prefix_start);
            
            // URL解码（简单处理，只处理常见字符）
            size_t pos = 0;
            while ((pos = prefix.find("%20", pos)) != std::string::npos) {
                prefix.replace(pos, 3, " ");
                pos += 1;
            }
            
            // 安全检查：只允许字母、数字、下划线、中划线
            bool valid = true;
            for (char c : prefix) {
                if (!isalnum(c) && c != '_' && c != '-') {
                    valid = false;
                    break;
                }
            }
            if (!valid || prefix.empty()) {
                prefix = "snapshot";
            }
        }
        
        const std::string target = get_query_param(path, "target");
        handle_snapshot_request(sock, prefix, target == "crop");
    } else {
        // 404
        const char* not_found = "<h1>404 Not Found</h1>";
        send_response(sock, "text/html", not_found, strlen(not_found));
    }
    
    close(sock);
}

/*******************************************************************
 * @brief       客户端处理线程函数
 * 
 * @param       arg             客户端socket指针
 * 
 * @return      返回NULL
 ******************************************************************/
void* CameraStreamServer::client_thread_func(void* arg)
{
    int sock = *(int*)arg;
    delete (int*)arg;
    
    if (instance) {
        instance->handle_client_request(sock);
    } else {
        close(sock);
    }
    
    return NULL;
}

/*******************************************************************
 * @brief       服务器线程函数
 * 
 * @param       arg             CameraStreamServer实例指针
 * 
 * @return      返回NULL
 ******************************************************************/
void* CameraStreamServer::server_thread_func(void* arg)
{
    CameraStreamServer* server = static_cast<CameraStreamServer*>(arg);
    if (!server) return NULL;
    
    int server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        std::cerr << "创建socket失败" << std::endl;
        return NULL;
    }

    pthread_mutex_lock(&server->sock_mutex);
    server->server_sock_fd = server_sock;
    pthread_mutex_unlock(&server->sock_mutex);
    
    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(server->server_port);
    
    if (bind(server_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "绑定端口失败" << std::endl;
        close(server_sock);
        return NULL;
    }
    
    if (listen(server_sock, 10) < 0) {
        std::cerr << "监听失败" << std::endl;
        close(server_sock);
        return NULL;
    }
    
    // 获取本机IP地址
    std::string local_ip = server->get_local_ip();
    
    std::cout << "\n======================================" << std::endl;
    std::cout << "📡 MJPEG摄像头图传服务器启动成功!" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "监听端口: " << server->server_port << std::endl;
    std::cout << "本机IP: " << local_ip << std::endl;
    std::cout << "请在浏览器访问: http://" << local_ip << ":" << server->server_port << std::endl;
    std::cout << "======================================\n" << std::endl;
    
    while (server->running) {
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);
        int client_sock = accept(server_sock, (struct sockaddr*)&client_addr, &len);
        
        if (client_sock < 0) {
            if (!server->running) break;
            continue;
        }
        
        // 优化socket选项
        int flag = 1;
        setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        
        // 设置发送缓冲区大小
        int sndbuf = 65536;
        setsockopt(client_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        pthread_t tid;
        int* sock_ptr = new int(client_sock);
        pthread_create(&tid, NULL, client_thread_func, sock_ptr);
        pthread_detach(tid);
    }
    
    server->close_server_socket();
    return NULL;
}

/*******************************************************************
 * @brief       信号处理函数
 * 
 * @param       sig             信号值
 ******************************************************************/
void CameraStreamServer::signal_handler(int sig)
{
    std::cout << "\n正在关闭服务器..." << std::endl;
    if (instance) {
        instance->stop_server();
    }
}

/*******************************************************************
 * @brief       启动摄像头图传服务器
 * 
 * @param       port            服务器监听端口(默认8080)
 * 
 * @return      返回启动状态
 * @retval      0               启动成功
 * @retval      -1              启动失败
 * 
 * @example     //启动摄像头图传服务器
 *              if(camera_server.start_server(8080) < 0) {
 *                  return -1;
 *              }
 * 
 * @note        在后台线程中启动HTTP服务器，支持浏览器访问
 *              访问 http://<开发板IP>:<port> 即可查看实时画面
 ******************************************************************/
int CameraStreamServer::start_server(int port)
{
    if (running) {
        std::cout << "服务器已经在运行中" << std::endl;
        return 0;
    }
    
    server_port = port;
    running = true;
    latest_frame_id = 0;
    current_jpeg.clear();
    current_perspective_jpeg.clear();
    current_crop_jpeg.clear();
    
    // 设置全局实例指针(用于信号处理)
    instance = this;
    
    // 注册信号处理函数
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (pthread_create(&server_thread_id, NULL, server_thread_func, this) != 0) {
        std::cerr << "创建服务器线程失败" << std::endl;
        running = false;
        return -1;
    }
    pthread_detach(server_thread_id);
    
    std::cout << "摄像头图传服务器启动中..." << std::endl;
    return 0;
}

/*******************************************************************
 * @brief       更新摄像头帧数据
 * 
 * @param       frame           OpenCV Mat格式的图像帧
 * 
 * @example     camera_server.update_frame(frame);
 * 
 * @note        将最新的摄像头帧推送到服务器，供客户端获取
 *              自动编码为JPEG格式并计算帧率
 ******************************************************************/
void CameraStreamServer::update_frame(const cv::Mat& frame)
{
    update_frame(frame, cv::Mat(), CameraTelemetry());
}

void CameraStreamServer::update_frame(const cv::Mat& frame, const cv::Mat& perspective_frame, const CameraTelemetry& telemetry_data)
{
    if (frame.empty()) return;

    const CameraTransportMode mode = get_transport_mode();
    const bool image_enabled = is_image_transport_enabled(mode);
    const bool telemetry_enabled = is_telemetry_transport_enabled(mode);

    uint64_t capture_ts_ms = now_ms();
    
    // 保存原始帧（用于高质量拍照）
    pthread_mutex_lock(&original_frame_mutex);
    original_frame = frame.clone();
    if (g_red_model_input_ready && !g_red_model_input_40.empty()) {
        crop_frame = g_red_model_input_40.clone();
    } else {
        crop_frame.release();
    }
    pthread_mutex_unlock(&original_frame_mutex);
    
    // 计算FPS
    static uint64_t last_capture_ts_local = 0;
    static double local_fps_estimate = 0.0;
    if (last_capture_ts_local != 0) {
        uint64_t delta = capture_ts_ms - last_capture_ts_local;
        if (delta > 0) {
            double instant_fps = 1000.0 / static_cast<double>(delta);
            if (local_fps_estimate <= 0.0) {
                local_fps_estimate = instant_fps;
            } else {
                local_fps_estimate = 0.85 * local_fps_estimate + 0.15 * instant_fps;
            }
            ema_fps = local_fps_estimate;
        }
    }
    last_capture_ts_local = capture_ts_ms;

    // 只在图像通道启用时做 JPEG 编码，避免“只传参数”时还在白白消耗 CPU。
    std::vector<unsigned char> jpeg_buffer;
    std::vector<unsigned char> perspective_jpeg_buffer;
    std::vector<unsigned char> crop_jpeg_buffer;
    bool main_ok = false;
    bool perspective_ok = false;
    bool crop_ok = false;
    if (image_enabled) {
        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(90);
        main_ok = cv::imencode(".jpg", frame, jpeg_buffer, params);
        perspective_ok = perspective_frame.empty() ? false
                                                   : cv::imencode(".jpg", perspective_frame, perspective_jpeg_buffer, params);
        crop_ok = g_red_model_input_ready && !g_red_model_input_40.empty()
            && cv::imencode(".jpg", g_red_model_input_40, crop_jpeg_buffer, params);
    }

    if (telemetry_enabled) {
        pthread_mutex_lock(&telemetry_mutex);
        telemetry = telemetry_data;
        pthread_mutex_unlock(&telemetry_mutex);
    }

    if (image_enabled && main_ok) {
        latest_capture_ts_ms = capture_ts_ms;
        pthread_mutex_lock(&frame_mutex);
        current_jpeg.swap(jpeg_buffer);
        if (perspective_ok) {
            current_perspective_jpeg.swap(perspective_jpeg_buffer);
        } else {
            current_perspective_jpeg.clear();
        }
        if (crop_ok) {
            current_crop_jpeg.swap(crop_jpeg_buffer);
        } else {
            current_crop_jpeg.clear();
        }
        ++latest_frame_id;
        pthread_cond_broadcast(&frame_cond);
        pthread_mutex_unlock(&frame_mutex);
    } else if (!image_enabled) {
        pthread_mutex_lock(&frame_mutex);
        current_jpeg.clear();
        current_perspective_jpeg.clear();
        current_crop_jpeg.clear();
        pthread_cond_broadcast(&frame_cond);
        pthread_mutex_unlock(&frame_mutex);
    }
}

/*******************************************************************
 * @brief       停止摄像头图传服务器
 * 
 * @example     camera_server.stop_server();
 * 
 * @note        停止服务器并释放所有资源
 ******************************************************************/
void CameraStreamServer::stop_server(void)
{
    if (!running) return;
    
    std::cout << "正在停止摄像头图传服务器..." << std::endl;
    running = false;
    close_server_socket();
    pthread_cond_broadcast(&frame_cond);
    
    // 清空实例指针
    if (instance == this) {
        instance = nullptr;
    }
    
    std::cout << "摄像头图传服务器已停止" << std::endl;
}

/*******************************************************************
 * @brief       检查服务器是否正在运行
 * 
 * @return      返回服务器运行状态
 * @retval      true            服务器正在运行
 * @retval      false           服务器已停止
 * 
 * @example     if(camera_server.is_running()) {
 *                  //服务器正在运行
 *              }
 ******************************************************************/
bool CameraStreamServer::is_running(void)
{
    return running;
}
