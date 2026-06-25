#include "vofa.h"

void vofa_send(float target_speed, float L_speed, float R_speed)
{
    Frame frame;
    frame.fdata[0] = target_speed;
    frame.fdata[1] = L_speed;
    frame.fdata[2] = R_speed;
    vofa.udp_send(&frame, sizeof(Frame));
}
