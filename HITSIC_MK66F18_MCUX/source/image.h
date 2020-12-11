/*
 * image.h
 *
 *  Created on: 2020年11月10日
 *      Author: liuhe
 */

#ifndef _IMAGE_H
#define _IMAGE_H
#include <stdio.h>
#include <stdlib.h>
//#include "cv.h"
//#include "highgui.h"
#include <math.h>
#include "image.h"

//#define MISS 94
#define CAMERA_H  120                            //图片高度
#define CAMERA_W  188                            //图片宽度
#define FAR_LINE 1//图像处理上边界
#define NEAR_LINE 113//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define MISS 94
#define white_num_MAX 10//每行最多允许白条数

/////////////////////////////
#define black 0x00
#define white 0x01
#define blue  0x02
#define green 0x03
#define red   0x04
#define gray  0x05
#define purple 0x06
///////////////////////////

extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//指向灰度图的首地址
extern int threshold;
extern int foresight;
extern uint8_t banmaxian_flag;
extern uint8_t out_flag;

void head_clear(void);
void THRE(void);
int find_f(int a);
void search_white_range();
void find_all_connect();
void find_road();
uint8_t find_continue(uint8_t i_start, uint8_t j_start);
void ordinary_two_line(void);
void image_main();
void get_mid_line(void);
float get_error(void);

void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);

void search_rightup_point();
void  search_leftup_point();
void search_rightdown_point();
void search_leftdown_point();
void connect_line_plan();
void  connect_line(int x1, int y1, int x2, int y2,int flag);
float check_k(int line, uint8_t* array, int length, int flag);
void find_cross();
void banmaxian();
void ckeck_out_road();

#endif //







