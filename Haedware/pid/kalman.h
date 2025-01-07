#ifndef __KALMAN_H
#define __KALMAN_H

#include "sys.h"




// 定义卡尔曼滤波器结构体
struct _1_ekf_filter {
    float LastP;  // 上一时刻的估计误差协方差
    float Now_P;  // 当前时刻的估计误差协方差
    float Kg;     // 卡尔曼增益
    float Q;      // 过程噪声协方差
    float R;      // 测量噪声协方差
    float out;    // 滤波后的输出
};


extern void Com_Kalman_1(struct _1_ekf_filter *ekf, float input);
extern struct _1_ekf_filter ekf[3];
#endif

