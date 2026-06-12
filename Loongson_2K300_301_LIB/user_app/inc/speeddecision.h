#ifndef _SPEEDDECISION_H_
#define _SPEEDDECISION_H_

#include "pid.h"
#include "main.hpp"
#include <array>

using namespace std;
#define N 7 // 定义速度决策的元素数量

array<string , N> element = {
    "curse", "cross" , "round" , "ramp" , "object1" , "object2" , "object3"
  //弯道，   十字路口，   环岛，    坡道，   物体种类1，  物体种类2，  物体种类3
};

#endif // _SPEEDDECISION_H_