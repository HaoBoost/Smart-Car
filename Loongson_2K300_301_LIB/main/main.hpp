#ifndef __MAIN_HPP
#define __MAIN_HPP

#include <stdio.h>

// 包含所有底层驱动头文件
#include "lq_drv_inc.hpp"

// 包含所有工具头文件
#include "lq_common.hpp"

// 包含所有应用层头文件
#include "lq_app_inc.hpp"

// 包含所有测试程序头文件
#include "lq_all_demo.hpp"

  //实现根据x值进行的任务调度，比如：
  /*
   *  #define PERIODIC(x) \ 
   *    static uint64_t nxt = 0; \
   *    if (get_time_ms() < nxt) { \
   *     return; \
   *    } \
   *    nxt += (x); \
   * 但linux版，我们只创建一个线程，main中用裸机多任务模型，注意不要加分号！！！
   */
#define PERIODIC(x) \
    static uint64_t nxt = 0; \
    if (end_time.tv_sec * 1000000 + end_time.tv_usec < nxt) { \
     return 0; \
    } \
    nxt += (x); \

#endif


