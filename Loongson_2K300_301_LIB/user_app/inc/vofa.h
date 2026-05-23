#ifndef __VOFA_H
#define __VOFA_H

#include "main.hpp"
#include "lq_udp_client.hpp"

#define VOFA_UDP_IP ""
#define VOFA_UDP_PORT 22

lq_udp_client vofa(VOFA_UDP_IP, VOFA_UDP_PORT);

#endif // __VOFA_H