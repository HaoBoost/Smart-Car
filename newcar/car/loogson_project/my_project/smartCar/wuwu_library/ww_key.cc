#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 按键模块
 * 版权所有 (c) 2025 wuwu
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 本文件是 Wuwu 开源库 的一部分。
 *
 * 本文件按照 GNU 通用公共许可证 第3版（GPLv3）或您选择的任何后续版本的条款授权。
 * 您可以在遵守 GPL-3.0 许可条款的前提下，自由地使用、复制、修改和分发本文件及其衍生作品。
 * 在分发本文件或其衍生作品时，必须以相同的许可证（GPL-3.0）对源代码进行授权并随附许可证副本。
 *
 * 本软件按“原样”提供，不对适销性、特定用途适用性或不侵权做任何明示或暗示的保证。
 * 有关更多细节，请参阅 GNU 官方许可证文本： https://www.gnu.org/licenses/gpl-3.0.html
 *
 * 注：本注释为 GPL-3.0 许可证的中文说明与摘要，不构成法律意见。正式许可以 GPL 原文为准。
 * LICENSE 副本通常位于项目根目录的 LICENSE 文件或 libraries 文件夹下；若未找到，请访问上方链接获取。
 *
 * 文件名称：ww_key.cc
 * 所属模块：wuwu_library
 * 功能描述：按键扫描与事件处理封装
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-12-6  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "ww_key.h"
#include "ww_menu.h"
#include "buzzer_sound.h"
#define KEY_NUM         (5)

// 全局菜单指针（在 main.cc 中初始化）
extern Menu* g_menu;
extern bool g_menu_mode;  // 菜单模式标志

static void KEY1_CallBack(void)
{
    // printf("KEY1 pressed - UP\n");
    if (g_menu && g_menu_mode) g_menu->key_up();
}

static void KEY2_CallBack(void)
{
    // printf("KEY2 pressed - DOWN\n");
    if (g_menu && g_menu_mode) g_menu->key_down();
}

static void KEY3_CallBack(void)
{
    // printf("KEY3 pressed - DECREASE\n");
    if (g_menu && g_menu_mode) g_menu->key_decrease();
}

static void KEY4_CallBack(void)
{
    // printf("KEY4 pressed - INCREASE\n");
    if (g_menu && g_menu_mode) g_menu->key_increase();
}

static void KEY5_CallBack(void)  // 切换菜单/运行模式
{
    g_menu_mode = !g_menu_mode;
    if (g_menu_mode && g_menu) {
        g_menu->refresh();
    }
}

Key::Key(void) {}

Key::~Key(void)
{
    if(fd < 0) return;
    close(fd);
}

/*******************************************************************
 * @brief       按键初始化
 * 
 * @return      返回初始化状态
 * @retval      0               初始化成功
 * @retval      -1              初始化失败
 * 
 * @example     //按键初始化
 *              if(key.key_init() < 0) {
 *                  return -1;             
 *              }
 * 
 * @note        不使用此函数直接使用下面函数会报错
 ******************************************************************/
int Key::key_init(void)
{
    fd = open(KEY_FILE_ADD, O_RDONLY);
    if(fd < 0) {
        perror("key init error!\n");
        return -1;
    } 
    std::cout << "按键初始化成功" << std::endl;
    return 0;
}

/*******************************************************************
 * @brief       案件监听
 * 
 * @example     //按键阻塞监听
 *              key.key_listeners();
 ******************************************************************/
void Key::key_listeners(void)
{
    struct input_event event;
    if(read(fd, &event, sizeof(event)) != sizeof(event)) 
        return;

    if(event.type == EV_KEY && (event.value == 1 || event.value == 2)) {
        int keycode = event.code;

        switch (keycode)
        {
        case 2: KEY1_CallBack(); break;
        case 3: KEY2_CallBack(); break;
        case 4: KEY3_CallBack(); break;
        case 5: KEY4_CallBack(); break;
        case 6: KEY5_CallBack(); break;
        //...(可以根据设备树继续添加)
        default: break;
        }
    }
}
