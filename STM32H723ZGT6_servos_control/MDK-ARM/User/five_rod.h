#ifndef __FIVE_ROD_H
#define __FIVE_ROD_H

// 五连杆参数
#define L1 6.0f    // alpha舵机小臂长度(cm)
#define L2 9.0f    // alpha舵机大腿长度(cm)
#define L3 9.0f    // beta舵机大腿长度(cm)
#define L4 6.0f    // beta舵机小臂长度(cm)
#define L5 3.8f    // 舵机中心间距(cm)
#define LAMDA 57.297f // 弧度转角度系数(180/π)

// 运动范围限制
#define MIN_H 3.2f  // 最小高度
#define MAX_H 13.2f // 最大高度
#define MIN_X 0.0f  // 最小水平位移
#define MAX_X 7.6f  //最大水平位移


#endif