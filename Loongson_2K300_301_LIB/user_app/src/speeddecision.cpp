// speeddecision.cpp —— 速度决策实现

#include "speeddecision.h"

SpeedDecision::SpeedDecision()
{
}

float SpeedDecision::decide_target_speed()
{
    // 根据ImageStatus中的道路类型决策目标速度
    switch (ImageStatus.Road_type)
    {
    case Straight:
        return straight_speed_;

    case Cross:
    case Cross_ture:
        return cross_speed_;

    case LeftCirque:
    case RightCirque:
        return curve_speed_;

    case Ramp:
        return ramp_speed_;

    // 环岛相关类型使用环岛速度
    case Forkin:
    case Forkout:
    case Barn_out:
    case Barn_in:
        return roundabout_speed_;

    case Normol:
    default:
        return straight_speed_;
    }
}