// speeddecision.h —— 速度决策
// 根据赛道元素（弯道/十字/环岛/坡道等）决定基础目标速度

#ifndef SPEEDDECISION_H
#define SPEEDDECISION_H

#include "pid.h"
#include "main.hpp"
#include "image.hpp"

// 赛道元素枚举
enum class TrackElement
{
    Straight,   // 直道
    Curve,      // 弯道
    Cross,      // 十字路口
    Roundabout, // 环岛
    Ramp,       // 坡道
    Obstacle,   // 障碍物
    Unknown     // 未知
};

// 速度决策类
class SpeedDecision
{
public:
    SpeedDecision();

    // 根据当前图像状态决策目标速度
    float decide_target_speed();

    // 获取各元素对应的预设速度
    float get_straight_speed() const { return straight_speed_; }
    float get_curve_speed() const { return curve_speed_; }
    float get_cross_speed() const { return cross_speed_; }
    float get_roundabout_speed() const { return roundabout_speed_; }

    // 设置各元素速度
    void set_straight_speed(float v) { straight_speed_ = v; }
    void set_curve_speed(float v) { curve_speed_ = v; }
    void set_cross_speed(float v) { cross_speed_ = v; }
    void set_roundabout_speed(float v) { roundabout_speed_ = v; }

private:
    float straight_speed_ = 300.0f;   // 直道速度
    float curve_speed_ = 200.0f;      // 弯道速度
    float cross_speed_ = 250.0f;      // 十字速度
    float roundabout_speed_ = 180.0f; // 环岛速度
    float ramp_speed_ = 220.0f;       // 坡道速度
};

#endif // SPEEDDECISION_H