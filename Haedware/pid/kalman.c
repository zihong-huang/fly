#include "kalman.h"

struct _1_ekf_filter ekf[3] = {
		{0.02, 0, 0, 0, 0.001, 0.543},
		{0.02, 0, 0, 0, 0.001, 0.543},
		{0.02, 0, 0, 0, 0.001, 0.543}
};


// 卡尔曼滤波更新函数
void Com_Kalman_1(struct _1_ekf_filter *ekf, float input) 
{
    // 1. 预测步骤：更新估计误差协方差
    ekf->Now_P = ekf->LastP + ekf->Q;  // 预测误差协方差

    // 2. 计算卡尔曼增益
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);  // 卡尔曼增益

    // 3. 更新估计值（滤波后的输出）
    ekf->out = ekf->out + ekf->Kg * (input - ekf->out);  // 使用卡尔曼增益更新估计值

    // 4. 更新估计误差协方差
    ekf->LastP = (1 - ekf->Kg) * ekf->Now_P;  // 更新误差协方差
}

