#include "app_context.h"
#include "app_tasks.h"

// 按键任务保持极简：所有按键行为都交给底层 key 库回调处理。

void key_task(void* arg)
{
    (void)arg;
    key.key_listeners();
}
