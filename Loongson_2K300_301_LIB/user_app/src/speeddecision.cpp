// speeddecision.cpp —— 速度决策实现（前瞻白列长度法）
// 参考 Second_Version92_menu 项目：扫描图像中间列，统计赛道前方可见白列长度
// 白列长 → 直道 → 高速；白列短 → 弯道/元素 → 减速

#include "speeddecision.h"

SpeedDecision::SpeedDecision()
{
}

float SpeedDecision::decide_target_speed()
{
    // ---- 前瞻白列长度法：扫描图像中间几列，从底部向上统计连续白点数 ----
    const int col_start = LCDW / 2 - 2; // 38
    const int col_end = LCDW / 2 + 2;   // 42
    const int row_start = LCDH - 1;     // 59（底部）

    int see_sum = 0;
    int see_cnt = 0;

    for (int col = col_start; col <= col_end; col++)
    {
        int white_cnt = 0;
        for (int row = row_start; row > 0; row--)
        {
            if (Pixle[row][col] == 1) // 白=赛道
                white_cnt++;
            else
                break; // 遇到黑（背景/边界）停止
        }
        see_sum += white_cnt;
        see_cnt++;
    }

    int see = (see_cnt > 0) ? (see_sum / see_cnt) : 0;

    // ---- 速度映射：看得越远速度越快 ----
    const float speed_max = straight_speed_; // 300 直道极速
    const float speed_min = curve_speed_;    // 200 弯道最低速
    const float speed_fall = 2.5f;           // 每少1px白列降速2.5
    const int see_ref = 10;                  // 参考白列长度

    float spd = speed_max - speed_fall * (static_cast<float>(see) - see_ref);

    if (spd > speed_max)
        spd = speed_max;
    if (spd < speed_min)
        spd = speed_min;

    return spd;
}