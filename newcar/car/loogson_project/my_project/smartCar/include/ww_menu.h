/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — LCD 多级菜单系统
 * 版权所有 (c) 2025 wuwu
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 文件名称：ww_menu.h
 * 功能描述：LCD 多级菜单系统，配合按键实现调参、发车和状态显示
 *
 * 按键定义：
 *   KEY1 (code=2): 上移光标
 *   KEY2 (code=3): 下移光标
 *   KEY3 (code=4): 返回上级 / 减小参数
 *   KEY4 (code=5): 进入下级 / 执行动作 / 增大参数
 *   KEY5 (code=6): 菜单页与运行状态页切换
 ********************************************************************************************************************/

#ifndef __WUWU_MENU_H
#define __WUWU_MENU_H

#include "headfile.h"
#include "ww_lcd.h"

enum MenuValueType {
    MENU_VALUE_NONE = 0,
    MENU_VALUE_INT,
    MENU_VALUE_FLOAT,
};

enum MenuNodeType {
    MENU_NODE_SUBMENU = 0,
    MENU_NODE_ACTION,
    MENU_NODE_PARAM,
    MENU_NODE_READONLY,
};

typedef void (*MenuActionFunc)();
typedef void (*MenuFormatterFunc)(char* out, size_t out_size);

struct MenuNode {
    const char* name = nullptr;
    MenuNodeType node_type = MENU_NODE_SUBMENU;
    MenuValueType value_type = MENU_VALUE_NONE;
    void* value_ptr = nullptr;
    float min_val = 0.0f;
    float max_val = 0.0f;
    float step = 0.0f;
    unsigned char decimals = 0;
    int parent = -1;
    std::vector<int> children;
    MenuActionFunc action = nullptr;
    MenuFormatterFunc formatter = nullptr;
};

class Menu {
public:
    Menu(LCD* lcd_ptr);
    ~Menu();

    void key_up();
    void key_down();
    void key_decrease();
    void key_increase();

    void refresh();
    void tick();

private:
    int add_submenu(int parent, const char* name);
    int add_action(int parent, const char* name, MenuActionFunc action);
    int add_int_param(int parent, const char* name, int* value_ptr,
                      int min_val, int max_val, int step,
                      MenuFormatterFunc formatter = nullptr);
    int add_float_param(int parent, const char* name, float* value_ptr,
                        float min_val, float max_val, float step,
                        unsigned char decimals,
                        MenuFormatterFunc formatter = nullptr);
    int add_readonly_int(int parent, const char* name, int* value_ptr,
                         MenuFormatterFunc formatter = nullptr);
    int add_readonly_float(int parent, const char* name, float* value_ptr,
                           unsigned char decimals,
                           MenuFormatterFunc formatter = nullptr);
    int add_readonly_custom(int parent, const char* name, MenuFormatterFunc formatter);

    void build_default_menu();
    void render_locked();
    void render_menu_locked();
    void render_runtime_locked();
    void enter_child_locked();
    void back_or_decrease_locked();
    void increase_or_enter_locked();
    void adjust_value_locked(MenuNode& node, int direction);
    void mark_dirty_locked();
    const MenuNode* current_selected_node_locked() const;
    void format_node_value_locked(const MenuNode& node, char* out, size_t out_size) const;

    LCD* lcd;
    std::vector<MenuNode> nodes;
    int current_menu_index;
    int selected_index;
    int display_offset;
    bool dirty;
    std::chrono::steady_clock::time_point last_refresh_time;
    mutable std::mutex menu_mutex;

    static const int MAX_DISPLAY_ITEMS = 6;
    static const int TITLE_Y = 0;
    static const int HINT_Y = 10;
    static const int START_Y = 24;
    static const int LINE_HEIGHT = 14;
};

#endif
