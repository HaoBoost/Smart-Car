#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — TCP 客户端模块
 * 版权所有 (c) 2025 chao_8xx
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
 * 文件名称：ww_tcp_client.cc
 * 所属模块：wuwu_library
 * 功能描述：TCP 客户端封装及 VOFA 协议适配
 *
 * 修改记录：
 * 日期        作者          说明
 * 2025-12-24  chao_8xx    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "headfile.h"

/*******************************************************************
 * [父类] Tcp Client: 底层通信驱动
 * 职责：只负责 TCP 连接、断开、原始字节的发送和检查连接状态
 * 特点：可用于连接任何 TCP 服务器
 ******************************************************************/

TcpClient::TcpClient(void)
    : sock_fd(-1)
    , connected(false)
{
    pthread_mutex_init(&sock_mutex, NULL);
}

TcpClient::~TcpClient(void)
{
    disconnect_server();
    pthread_mutex_destroy(&sock_mutex);
}

/*******************************************************************
 * @brief       连接TCP服务器 
 * 
 * @param       ip              服务器IP地址
 * @param       port            服务器端口号
 * 
 * @return      返回连接状态
 * @retval      0               连接成功
 * @retval      -1              连接失败
 * 
 * @example     //连接到主机的VOFA+服务器
 *              if(tcp_client.connect_server("192.168.1.101", 2233) < 0) {
 *                  return -1;
 *              }
 * 
 * @note        建立与TCP服务器的连接，供后续数据传输使用
 ******************************************************************/
int TcpClient::connect_server(const char* ip, int port)
{
    if (connected) {
        disconnect_server();
    }

    pthread_mutex_lock(&sock_mutex);

    // 创建 Socket
    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0) {
        perror("Socket 创建失败");
        pthread_mutex_unlock(&sock_mutex);
        return -1;  
    }

    // 设置服务器地址结构体
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;                   // IPv4(地址族协议)
    server_addr.sin_port = htons(port);                 // 端口号
    server_addr.sin_addr.s_addr = inet_addr(ip);        // IP地址

    // 连接到服务器
    if (connect(sock_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("[TCP] Connect failed 连接失败");
        close(sock_fd);
        sock_fd = -1;
        connected = false;
        pthread_mutex_unlock(&sock_mutex);
        return -1;
    }

    connected = true;
    printf("[TCP] Successfully connected to server %s:%d 连接成功\n", ip, port);

    pthread_mutex_unlock(&sock_mutex);
    return 0;

}

/*******************************************************************
 * @brief       发送原始字节数据
 * 
 * @param       data    待发送数据指针
 * @param       len     数据长度
 * 
 * @return      true:发送成功, false:发送失败
 * 
 * @note        将上层 TCP 设备协议打包来通信
 ******************************************************************/
bool TcpClient::send_bytes(const void* data, size_t len)
{
    pthread_mutex_lock(&sock_mutex);

    if (!connected || sock_fd < 0 || data == nullptr || len == 0) {
        pthread_mutex_unlock(&sock_mutex);
        return false;
    }

    const char* p = static_cast<const char*>(data);
    size_t offset = 0;

    // 循环发送直到所有数据发送完毕
    while (offset < len) {
        ssize_t sent = send(sock_fd, p + offset, len - offset, MSG_NOSIGNAL);
        if (sent > 0) {
            offset += (size_t)sent;
            continue;
        }
        // 发送失败时的处理
        perror("[TCP] Send failed 发送失败");
        close(sock_fd);
        sock_fd = -1;
        connected = false;
        pthread_mutex_unlock(&sock_mutex);
        return false;
    }

    pthread_mutex_unlock(&sock_mutex);
    return true;
}

/*******************************************************************
 * @brief       发送字符串
 * 
 * @param       s       待发送字符串
 * 
 * @return      true:发送成功, false:发送失败
 * 
 * @example     tcp_client.send_string("Hello, TCP Server!");
 * 
 * @note        将上层 TCP 设备协议打包来通信
 ******************************************************************/
bool TcpClient::send_string(const std::string& s)
{
    return send_bytes(s.data(), s.size());
}

/*******************************************************************
 * @brief       断开与TCP服务器的连接
 * 
 * @example     tcp_client.disconnect_server();
 * 
 * @note        断开连接并释放Socket资源
 ******************************************************************/
void TcpClient::disconnect_server(void)
{
    pthread_mutex_lock(&sock_mutex);

    if (connected && sock_fd >= 0) {
        close(sock_fd);
        sock_fd = -1;
        connected = false;
        printf("Disconnect 断开连接\n");
    }

    pthread_mutex_unlock(&sock_mutex);
}

/*******************************************************************
 * @brief       检查服务器是否已连接
 *
 * @return      返回服务器运行状态
 * @retval      true            服务器已连接
 * @retval      false           服务器未连接
 *
 * @example     if(tcp_client.is_connected()) {
 *                  //服务器已连接
 *              }
 ******************************************************************/
bool TcpClient::is_connected(void)
{
    pthread_mutex_lock(&sock_mutex);
    bool status = connected;

    pthread_mutex_unlock(&sock_mutex);
    return status;
}

/*******************************************************************
 * @brief       接收数据（非阻塞）
 *
 * @param       buffer  接收缓冲区
 * @param       len     缓冲区大小
 * @param       timeout_ms  超时时间（毫秒），0表示立即返回
 *
 * @return      实际接收的字节数，-1表示错误，0表示无数据
 ******************************************************************/
int TcpClient::recv_bytes(void* buffer, size_t len, int timeout_ms)
{
    pthread_mutex_lock(&sock_mutex);

    if (!connected || sock_fd < 0 || buffer == nullptr || len == 0) {
        pthread_mutex_unlock(&sock_mutex);
        return -1;
    }

    // 使用 select 实现超时
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock_fd, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(sock_fd + 1, &read_fds, NULL, NULL, &timeout);
    if (ret < 0) {
        // select 错误
        perror("[TCP] Select failed");
        pthread_mutex_unlock(&sock_mutex);
        return -1;
    } else if (ret == 0) {
        // 超时，无数据
        pthread_mutex_unlock(&sock_mutex);
        return 0;
    }

    // 有数据可读
    ssize_t received = recv(sock_fd, buffer, len, 0);
    if (received < 0) {
        perror("[TCP] Recv failed");
        pthread_mutex_unlock(&sock_mutex);
        return -1;
    } else if (received == 0) {
        // 连接关闭
        printf("[TCP] Connection closed by peer\n");
        close(sock_fd);
        sock_fd = -1;
        connected = false;
        pthread_mutex_unlock(&sock_mutex);
        return -1;
    }

    pthread_mutex_unlock(&sock_mutex);
    return (int)received;
}


/*******************************************************************
 * [子类] VofaClient: VOFA+ 协议层封装
 * 负责：将发送数据封装为 VOFA+ 的 FireWater 协议格式帧
 ******************************************************************/

VofaClient::VofaClient(void)
    : TcpClient()
{
    // 初始化缓冲区
    memset(firewater_buffer, 0, sizeof(firewater_buffer));
    memset(recv_buffer, 0, sizeof(recv_buffer));
    recv_buffer_pos = 0;
}

VofaClient::~VofaClient(void)
{
}

/*******************************************************************
 * @brief        发送格式化数据到VOFA+服务器 (FireWater协议)
 *
 * @param        format  格式化字符串 (如 "data0: %f, data1: %f\n")
 * @param        ...     可变参数列表
 *
 * @example      // 发送浮点数
 * client.send_firewater("sin: %.2f, %.2f\n", sin_val, cos_val);
 *
 * @example      // 发送混合数据
 * client.send_firewater("temp: %.1f, %d\n", 25.5, 1);
 *
 * @note         底层使用 vsnprintf 格式化，完全解耦，不限制数据类型
 *               发送数据格式说明: FireWater 协议格式:"name:csv_numbers\n"
 *******************************************************************/
void VofaClient::send_firewater(const char* format, ...)
{
    if (!this->is_connected())
        return;

    va_list args;

    // 格式化字符串 (类似 printf)
    va_start(args, format);
    vsnprintf(firewater_buffer, sizeof(firewater_buffer), format, args);
    va_end(args);

    // 发送字符串
    this->send_string(std::string(firewater_buffer));
}

/*******************************************************************
 * @brief        接收并解析 VOFA+ 数据（FireWater 协议）
 *
 * @param        name    输出：数据名称
 * @param        values  输出：数值数组
 * @param        max_values  values 数组最大长度
 *
 * @return       实际解析到的数值个数，0表示无数据，-1表示错误
 *
 * @note         格式："name: value1, value2, ...\n"
 *               或 "(name,Kp,value)\n" 格式（VOFA+ RawData 协议）
 *******************************************************************/
int VofaClient::recv_firewater(char* name, float* values, int max_values)
{
    if (!this->is_connected() || name == nullptr || values == nullptr || max_values <= 0) {
        return -1;
    }

    // 接收数据到缓冲区
    char temp_buf[256];
    int received = this->recv_bytes(temp_buf, sizeof(temp_buf) - 1, 10);  // 10ms 超时

    if (received <= 0) {
        return 0;  // 无数据或错误
    }

    // 添加调试输出：显示接收到的原始数据
    temp_buf[received] = '\0';
    printf("[VOFA-DEBUG] Received %d bytes: [%s]\n", received, temp_buf);

    // 将接收到的数据追加到缓冲区
    for (int i = 0; i < received && recv_buffer_pos < (int)sizeof(recv_buffer) - 1; i++) {
        recv_buffer[recv_buffer_pos++] = temp_buf[i];
    }
    recv_buffer[recv_buffer_pos] = '\0';
    printf("[VOFA-DEBUG] Buffer now: [%s]\n", recv_buffer);

    // 查找完整的一行（以 \n 结尾）或完整的括号对 ()
    char* line_end = strchr(recv_buffer, '\n');

    // 如果没有换行符，尝试查找完整的括号对
    if (line_end == nullptr && recv_buffer[0] == '(') {
        char* close_paren = strchr(recv_buffer, ')');
        if (close_paren != nullptr) {
            // 找到完整的括号对，将其作为一行处理
            line_end = close_paren;
            printf("[VOFA-DEBUG] Found complete parentheses pair\n");
        }
    }

    if (line_end == nullptr) {
        // 没有完整的一行，继续等待
        printf("[VOFA-DEBUG] No complete line yet, waiting...\n");
        return 0;
    }

    // 提取一行数据
    char line[256];
    int line_length = line_end - recv_buffer + 1;
    if (line_length > (int)sizeof(line) - 1) {
        line_length = sizeof(line) - 1;
    }
    strncpy(line, recv_buffer, line_length);
    line[line_length] = '\0';

    // 如果是括号对，去掉末尾的 )
    if (line[line_length - 1] == ')') {
        // 保持完整
    } else if (line[line_length - 1] == '\n') {
        line[line_length - 1] = '\0';  // 去掉换行符
    }

    printf("[VOFA-DEBUG] Extracted line: [%s]\n", line);

    // 移除已处理的数据
    int processed_len = line_end - recv_buffer + 1;
    memmove(recv_buffer, line_end + 1, recv_buffer_pos - processed_len);
    recv_buffer_pos -= processed_len;
    recv_buffer[recv_buffer_pos] = '\0';

    // 解析数据
    // 支持两种格式：
    // 1. FireWater: "name: value1, value2, ...\n"
    // 2. RawData: "(name,Kp,value)\n"

    // 尝试解析 RawData 格式：(name,Kp,value) 或 (command)
    if (line[0] == '(') {
        printf("[VOFA-DEBUG] Trying RawData format...\n");
        char param_name[32];
        char param_type[32];
        float value;

        // 先尝试解析带参数的格式：(name,Kp,value)
        if (sscanf(line, "(%[^,],%[^,],%f)", param_name, param_type, &value) == 3) {
            // 组合名称：name_type
            snprintf(name, 32, "%s_%s", param_name, param_type);
            values[0] = value;
            printf("[VOFA-DEBUG] RawData parsed: %s = %.2f\n", name, value);
            return 1;
        }
        // 尝试解析简单指令格式：(command)
        else if (sscanf(line, "(%[^)])", param_name) == 1) {
            strncpy(name, param_name, 32);
            name[31] = '\0';
            printf("[VOFA-DEBUG] Command parsed: %s\n", name);
            return 0;  // 返回0表示这是一个指令，没有数值
        } else {
            printf("[VOFA-DEBUG] RawData parse failed\n");
        }
    }

    // 尝试解析 FireWater 格式：name: value1, value2, ...
    char* colon = strchr(line, ':');
    if (colon != nullptr) {
        printf("[VOFA-DEBUG] Trying FireWater format...\n");
        // 提取名称
        *colon = '\0';
        strncpy(name, line, 32);
        name[31] = '\0';

        // 去除名称前后的空格
        char* name_start = name;
        while (*name_start == ' ') name_start++;
        if (name_start != name) {
            memmove(name, name_start, strlen(name_start) + 1);
        }

        // 解析数值
        char* value_str = colon + 1;
        int count = 0;

        char* token = strtok(value_str, ",");
        while (token != nullptr && count < max_values) {
            values[count++] = atof(token);
            token = strtok(nullptr, ",");
        }

        printf("[VOFA-DEBUG] FireWater parsed: %s with %d values\n", name, count);
        return count;
    }

    printf("[VOFA-DEBUG] No format matched, line: [%s]\n", line);
    return 0;  // 无法解析
}
