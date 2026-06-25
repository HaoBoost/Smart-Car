#include "app_context.h"
#include "app_tasks.h"

void menu_task(void* arg)
{
    (void)arg;
    if (g_menu) {
        g_menu->tick();
    }
}
