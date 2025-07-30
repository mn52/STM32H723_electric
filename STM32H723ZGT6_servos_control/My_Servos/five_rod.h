#ifndef __FIVE_ROD_H
#define __FIVE_ROD_H

// 五连杆参数
#define L1 6.0f    // alpha舵机小臂长度(cm)
#define L2 10.0f    // alpha舵机大腿长度(cm)
#define L3 10.0f    // beta舵机大腿长度(cm)
#define L4 6.0f    // beta舵机小臂长度(cm)
#define L5 3.80f    // 舵机中心间距(cm)
#define LAMDA 57.297f // 弧度转角度系数(180/π)

// 运动范围限制
#define MIN_H 6.5f  // 最小高度
#define MAX_H 15.7f // 最大高度
#define MIN_X 0  // 最小水平位移
#define MAX_X 0 //最大水平位移
#include "stm32h7xx.h"
#include "PCA9685.h"
#include "five_rod.h"


float Count_angle_alpha(float alpha_count);
float Count_angle_beta(float beta_count);
float Count_pack_alpha(float horizon,float height);
float Count_pack_beta(float horizon,float height);
void Servos_left(float horizon,float height);
void Servos_right(float horizon,float height);


#endif