/*******************************************************************************************************************************
 *  代码用途: pd型模糊控制器
 *  作者: WuwuSama
 *  邮箱: 1635202242@qq.com
 *  创建日期: 2025-3-20
 *  版本: 2.0.0
 *  
 *  调用函数    void get_updataPD_pid (float er, float *kp, float *kd);
 * 
 *  输入介绍:
 *  er                      图像输出误差值
 *  kp                      需要模糊的pid的kp(传入的是kp的地址)
 *  kd                      需要模糊的pid的kd(传入的是kd的地址)
 * 
 *  参数介绍:
 *  ERROR_MAX               误差最大值
 *  DEDT_MAX                误差变化率最大值
 *  KP_Fuzzy                KP的模糊值
 *  KD_Fuzzy                KD的模糊值
 *  KP_Base                 KP基础值
 *  KD_Base                 KD基础值
 * 
 *  ER_ZO0~ER_PBB           误差的论域, 输入误差映射论域的区间, 默认线性0~6 也可改为非线性 例如: 0 1 1.5 2.5 4.5 6
 *  DT_NB~DT_PB             误差变化的论域, 误差变化映射论域的区间, 默认线性0~6 也可改为非线性 例如: 0 1 1.5 2.5 4.5 6
 *  KP_ZO0~KP_PBB           KP大小"归一化"的范围, 默认线性0~6 也可改为非线性 例如: 0 1 1.5 2.5 4.5 6(后结合KP_BUFF得到最后输出的KP)
 *  KD_ZO0~KD_PBB           KD大小"归一化"的范围, 默认线性0~6 也可改为非线性 例如: 0 1 1.5 2.5 4.5 6(后结合KD_BUFF得到最后输出的KP)
 * 
 ******************************************************************************************************************************/
/*************************/
// .--,       .--,
//( (  \.---./  ) )
// '.__/o   o\__.'
//    {=  ^  =}
//     >  -  <
//    /       \
//   //       \\
//  //|   .   |\\
//  "'\       /'"_.-~^`'-.
//     \  _  /--'         `
//   ___)( )(___
//  (((__) (__)))    高山仰止,景行行止.虽不能至,心向往之。
//     WuwuSama
/*************************/ 
#ifndef _FUZZYPID_H_
#define _FUZZYPID_H_

/* WWWWWWWWWWWWWWWWWWWWWWWWWWW外部接口WWWWWWWWWWWWWWWWWWWWWWWWWWWWWW */
/* 函数用处：   模糊PD控制器
 * 函数名：     get_updataPD_pid (float er, float *kp, float *kd)
 * 
 * 传入参数：   float er        差值
 * 传入参数：   float *kp       传递pid->kp用于更新
 * 传入参数：   float *kd       传递pid->kd用于更新
 * 函数介绍：   
 * 实现模糊控制，同时更新最新的pid->kp 和 pid->kd
 */
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW */
void get_updataPD_pid (float er, float *kp, float *kd);


#endif
