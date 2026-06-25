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
// #include "fuzzyCtrl.h"

/* WWWWWWWWWWWWWWWWWWW以下内容根据实际情况修改WWWWWWWWWWWWWWWWWWWWWWW */
float ERROR_MAX = 40.0;                         //误差最大值
float DEDT_MAX = 1.0;                           //误差变化率最大值
float KP_Fuzzy = 2.0;                           //kp模糊值
float KD_Fuzzy = 10.0;                          //kd模糊值

float KP_Base = 0;                              //kp基础值
float KD_Base = 0;                              //kd基础值
/* WWWWWWWWWWWWWWWWWWW以上内容根据实际情况修改WWWWWWWWWWWWWWWWWWWWWWW */



/* WWWWWWWWWWWWWWWWWWWWWWWWWWWW Error论域 WWWWWWWWWWWWWWWWWWWWWWWWWWWWW */

#define ER_ZO0                              (0.0f)
#define ER_PSS                              (3.5f)
#define ER_PSB                              (4.0f)
#define ER_PMS                              (4.5f)
#define ER_PMB                              (5.0f)
#define ER_PBS                              (5.5f)
#define ER_PBB                              (6.0f)

const float Er_lunYu[7] = {ER_ZO0, ER_PSS, ER_PSB, ER_PMS, ER_PMB, ER_PBS, ER_PBB};
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW */

/* WWWWWWWWWWWWWWWWWWWWWWWWWWWW dedt论域 WWWWWWWWWWWWWWWWWWWWWWWWWWWWW */
#define DT_NB                               (-3.0f)
#define DT_NM                               (-2.0f)
#define DT_NS                               (-1.0f)
#define DT_Z0                               (0.0f)
#define DT_PS                               (1.0f)
#define DT_PM                               (2.0f)
#define DT_PB                               (3.0f)

const float Dedt_lunYu[7] = {DT_NB, DT_NM, DT_NS, DT_Z0, DT_PS, DT_PM, DT_PB};
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW */








static float er_liShu[2];                      //er隶属
static float dedt_liShu[2];                    //dedt隶属
static float er_liShuDu[2];                    //er隶属度
static float dedt_liShuDu[2];                  //dedt隶属度
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWW KP规则 WWWWWWWWWWWWWWWWWWWWWWWWWWWWW */
#define KP_ZO0                              (0.0f)
#define KP_PSS                              (1.0f)
#define KP_PSB                              (2.0f)
#define KP_PMS                              (3.0f)
#define KP_PMB                              (4.0f)
#define KP_PBS                              (5.0f)
#define KP_PBB                              (6.0f)
const float KP_Rule[7][7] = {
/*  er/dedt          DT_NB   DT_NM   DT_NS   DT_Z0   DT_PS   DT_PM   DT_PB */
    /*ER_ZO0*/      {KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0},
    /*ER_PSS*/      {KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS},
    /*ER_PSB*/      {KP_PSS, KP_PSS, KP_PSS, KP_PSB, KP_PMS, KP_PMS, KP_PMS},
    /*ER_PMS*/      {KP_PSB, KP_PSB, KP_PMS, KP_PMS, KP_PMS, KP_PMB, KP_PMB},
    /*ER_PMB*/      {KP_PMS, KP_PMS, KP_PMB, KP_PMB, KP_PMB, KP_PBS, KP_PBS},
    /*ER_PBS*/      {KP_PMB, KP_PBS, KP_PBS, KP_PBS, KP_PBS, KP_PBS, KP_PBB},
    /*ER_PBB*/      {KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB},
};
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW */

/* WWWWWWWWWWWWWWWWWWWWWWWWWWWW KD规则 WWWWWWWWWWWWWWWWWWWWWWWWWWWWW */
#define KD_ZO0                              (0.0f)
#define KD_PSS                              (1.0f)
#define KD_PSB                              (2.0f)
#define KD_PMS                              (3.0f)
#define KD_PMB                              (4.0f)
#define KD_PBS                              (5.0f)
#define KD_PBB                              (6.0f)
const float KD_Rule[7][7] = {
/*  er/dedt          DT_NB   DT_NM   DT_NS   DT_Z0   DT_PS   DT_PM   DT_PB */
    /*ER_ZO0*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMB, KD_PMB, KD_PMS, KD_PSB},
    /*ER_PSS*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMB, KD_PMB, KD_PMS, KD_PSB},
    /*ER_PSB*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMS, KD_PMS, KD_PSB, KD_PSB},
    /*ER_PMS*/      {KD_PBS, KD_PMB, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS},
    /*ER_PMB*/      {KD_PMB, KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSS, KD_ZO0},
    /*ER_PBS*/      {KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS, KD_ZO0},
    /*ER_PBB*/      {KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS, KD_ZO0},
};
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW */

/* 函数用处：   隶属度获取
 *
 * 传入参数：   float value         映射到论域上的值
 * 传入参数：   float* liShu        映射值的隶属
 * 传入参数：   float* liShuDu      映射值的隶属度
 * 传入参数：   const int* lunYu    论域
 * 函数介绍：   统一隶属度获取接口，由于er和dedt的隶属逻辑相同，故封装统一接口便于代码阅读
 */
void calculateMembership (float value, float* liShu, float* liShuDu, const float* lunYu)
{
    if (value > lunYu[0] && value < lunYu[6]) {
        for (int i = 0; i < 6; i++) {
            if (lunYu[i] < value && value <= lunYu[i + 1]) {             /* 判断在哪个区间内 */
                /* 三角隶属度 */
                liShu[0] = i;                                                                   /* 记录前隶属 */
                liShu[1] = i + 1;                                                               /* 记录后隶属 */
                liShuDu[0] = (value - lunYu[i]) / (lunYu[i + 1] - lunYu[i]);                    /* 记录前隶属度 */
                liShuDu[1] = (lunYu[i + 1] - value) / (lunYu[i + 1] - lunYu[i]);                /* 记录后隶属度 */
                break;
            }
        }
    } else if (value <= lunYu[0]) {
        /* 三角隶属度 */
        liShu[0] = 0;                   /* 记录前隶属 */
        liShu[1] = 1;                   /* 记录后隶属 */
        liShuDu[0] = 1.0;               /* 记录前隶属度(100%) */
        liShuDu[1] = 0;                 /* 记录后隶属度 */
    } else if (value >= lunYu[6]) {
        /* 三角隶属度 */
        liShu[0] = 5;                   /* 记录前隶属 */
        liShu[1] = 6;                   /* 记录后隶属 */
        liShuDu[0] = 0;                 /* 记录前隶属度 */
        liShuDu[1] = 1.0;               /* 记录后隶属度(100%) */
    }
}

/* 函数用处：   获取er与de/dt的隶属度
 * 
 * 传入参数：   float er        当前差值
 * 函数介绍：   传入输出与输入目标值的差值作为er, 内部自动更新上一次误差, 对误差和变化率做归一化映射
 */
void get_degree_of_membership (float er)
{
    static float laster = 0;
    float dedt = er - laster;

    float erRemap = er / ERROR_MAX * ER_PBB;        /* 对er映射 */
    float dedtRemap = dedt / DEDT_MAX * DT_PB;      /* 对dedt映射 */

    //误差为负时 取反
    if(erRemap < 0) {
        erRemap = -erRemap;
        dedtRemap = -dedtRemap;
    }
    //误差在正负震荡时让KD最大
    if(laster * er < 0) {
        dedtRemap = -DT_NB;
    }

    calculateMembership(erRemap, er_liShu, er_liShuDu, Er_lunYu);
    calculateMembership(dedtRemap, dedt_liShu, dedt_liShuDu, Dedt_lunYu);

    laster = er;
}

/* 函数用处：   更新pd参数
 *
 * 传入参数：   float *kp           传递pid->kp用于更新
 * 传入参数：   float *kd           传递pid->kd用于更新
 * 函数介绍：   通过隶属规则更新pid中pd的参数
 */
void upPIDdata_pd(float* kp, float* kd)
{
    float kptmp = er_liShuDu[0] * dedt_liShuDu[0] * KP_Rule[(int)er_liShu[0]][(int)dedt_liShu[0]]
                + er_liShuDu[0] * dedt_liShuDu[1] * KP_Rule[(int)er_liShu[0]][(int)dedt_liShu[1]]
                + er_liShuDu[1] * dedt_liShuDu[0] * KP_Rule[(int)er_liShu[1]][(int)dedt_liShu[0]]
                + er_liShuDu[1] * dedt_liShuDu[1] * KP_Rule[(int)er_liShu[1]][(int)dedt_liShu[1]];

    float kdtmp = er_liShuDu[0] * dedt_liShuDu[0] * KD_Rule[(int)er_liShu[0]][(int)dedt_liShu[0]]
                + er_liShuDu[0] * dedt_liShuDu[1] * KD_Rule[(int)er_liShu[0]][(int)dedt_liShu[1]]
                + er_liShuDu[1] * dedt_liShuDu[0] * KD_Rule[(int)er_liShu[1]][(int)dedt_liShu[0]]
                + er_liShuDu[1] * dedt_liShuDu[1] * KD_Rule[(int)er_liShu[1]][(int)dedt_liShu[1]];

    kptmp = (kptmp / KP_PBB) * KP_Fuzzy;
    kdtmp = (kdtmp / KD_PBB) * KD_Fuzzy;

    *kp = KP_Base + kptmp;
    *kd = KD_Base + kdtmp;
}

/* 函数用处：   模糊PD控制器
 * 
 * 传入参数：   float er        差值
 * 传入参数：   float *kp       传递pid->kp用于更新
 * 传入参数：   float *kd       传递pid->kd用于更新
 * 函数介绍：   
 * 实现模糊控制，同时更新最新的pid->kp 和 pid->kd
 */
void get_updataPD_pid (float er, float *kp, float *kd)
{
    get_degree_of_membership(er);
    upPIDdata_pd(kp, kd);
}