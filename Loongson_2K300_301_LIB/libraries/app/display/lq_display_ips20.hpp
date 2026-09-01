#ifndef __LQ_DISPLAY_IPS20_HPP
#define __LQ_DISPLAY_IPS20_HPP 

#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include "lq_display_font.hpp"
#include "lq_display_types.hpp"

/****************************************************************************************************
 * @brief   宏定义
 ****************************************************************************************************/

/* 汉字编码格式，默认 1 是 UTF-8 格式, 给 0 则表示 GB2312 格式 */
#define IPS20_HANZI_UTF_8   1

/* 文件名称 */
#define IPS20_DEV_NAME      ( "/dev/LQ_IPS_ST7789" )

/* 2.0寸 IPS 屏幕尺寸定义 */
#define IPS20_WIDTH         ( 240 )
#define IPS20_HEIGHT        ( 320 )
#define IPS20_LAND_WIDTH    ( 320 )
#define IPS20_LAND_HEIGHT   ( 240 )

/* 帧缓冲大小 */
#define IPS20_PageSize      ( 16384 )
#define IPS20_FB_SIZE       ( IPS20_PageSize * 10 )

#define IPS20_SSIZE         ( IPS20_WIDTH * IPS20_HEIGHT )

/* 2.0寸 IPS 屏幕相关幻数号 */
#define IOCTL_IPS20_MAGIC   'p'                         // 自定义幻数号, 用于区分不同的设备驱动
#define IOCTL_IPS20_FLUSH   _IO(IOCTL_IPS20_MAGIC, 1)   // 刷新屏幕
#define IOCTL_IPS20_L_INIT  _IO(IOCTL_IPS20_MAGIC, 2)   // 横屏初始化
#define IOCTL_IPS20_V_INIT  _IO(IOCTL_IPS20_MAGIC, 3)   // 竖屏初始化

/****************************************************************************************************
 * @brief   函数定义
 ****************************************************************************************************/

void lq_ips20_drv_init(uint8_t type);       // IPS20 屏幕初始化

void lq_ips20_drv_cls      (lq_display_color_t color_dat);                                                      // 全屏显示单色画面
void lq_ips20_drv_fill_area(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, lq_display_color_t color_dat);  // 填充指定区域

void lq_ips20_drv_draw_dot      (uint16_t x,  uint16_t y,  lq_display_color_t color);                           // 画点
void lq_ips20_drv_draw_line     (uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, lq_display_color_t color); // 画线
void lq_ips20_drv_draw_rectangle(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, lq_display_color_t color); // 画矩形边框
void lq_ips20_drv_draw_circle   (uint16_t x,  uint16_t y,  uint16_t r, lq_display_color_t color_dat);           // 画圆

void lq_ips20_drv_p6x8_str (uint16_t x, uint16_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color);   // 液晶字符串输出(6*8字体)
void lq_ips20_drv_p8x8_str (uint16_t x, uint16_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color);   // 液晶字符串输出(8*8字体)
void lq_ips20_drv_p8x16_str(uint16_t x, uint16_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color);   // 液晶字符串输出(8*16字体)

void lq_ips20_drv_cstr(uint8_t x, uint8_t y, const char *s_dat, const lq_display_font_t &font, lq_display_color_t word_color, lq_display_color_t back_color);    // 根据传入的字库，显示任意大小的汉字

void lq_ips20_drv_image(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const unsigned char* Pixle);    // 显示图像取模软件获取的图像数据

void lq_ips20_drv_road   (uint16_t wide_start, uint16_t high_start, uint16_t high, uint16_t wide, uint8_t *Pixle);  // 绘制灰度图像
void lq_ips20_drv_binRoad(uint16_t wide_start, uint16_t high_start, uint16_t high, uint16_t wide, uint8_t *Pixle);  // 绘制二值图像

#ifdef LQ_HAVE_OPENCV
#include <opencv2/opencv.hpp>
void lq_ips20_drv_road_color(uint16_t wide_start, uint16_t high_start, const cv::Mat &Pixle);  // 绘制彩色图像
#endif

void lq_ips20_drv_flush();  // 刷新屏幕

#endif
