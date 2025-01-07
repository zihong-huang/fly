#ifndef __KALMAN_H
#define __KALMAN_H

#include "sys.h"




// ���忨�����˲����ṹ��
struct _1_ekf_filter {
    float LastP;  // ��һʱ�̵Ĺ������Э����
    float Now_P;  // ��ǰʱ�̵Ĺ������Э����
    float Kg;     // ����������
    float Q;      // ��������Э����
    float R;      // ��������Э����
    float out;    // �˲�������
};


extern void Com_Kalman_1(struct _1_ekf_filter *ekf, float input);
extern struct _1_ekf_filter ekf[3];
#endif

