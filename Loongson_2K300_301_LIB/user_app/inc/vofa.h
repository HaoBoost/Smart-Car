#ifndef __VOFA_H
#define __VOFA_H

#include "main.hpp"
#include "lq_udp_client.hpp"
// JustFloat 
#define CH_COUNT 3 // 通道数量
struct Frame
{
    float fdata[CH_COUNT];
    unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};
};

#define VOFA_UDP_IP "172.21.196.61"
#define VOFA_UDP_PORT 1347

inline lq_udp_client vofa(VOFA_UDP_IP, VOFA_UDP_PORT);

void vofa_send(float target_speed, float L_speed, float R_speed);

#endif // __VOFA_H

// #ifndef __VOFA_H
// #define __VOFA_H
// #include "headfile.h"

// //关于使用vofa相关的类
// class Vofa{
// public:
//     int vofa_init(void);                //向vofa发送数据的初始化
//     void vofa_read(float parameter1, float parameter2, float parameter3, float parameter4, float parameter5, float parameter6);     //vofa读取数据

// private:

// };

// extern Vofa vofa;
// extern VofaClient client;

// #endif
