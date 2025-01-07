#include "kalman.h"

struct _1_ekf_filter ekf[3] = {
		{0.02, 0, 0, 0, 0.001, 0.543},
		{0.02, 0, 0, 0, 0.001, 0.543},
		{0.02, 0, 0, 0, 0.001, 0.543}
};


// �������˲����º���
void Com_Kalman_1(struct _1_ekf_filter *ekf, float input) 
{
    // 1. Ԥ�ⲽ�裺���¹������Э����
    ekf->Now_P = ekf->LastP + ekf->Q;  // Ԥ�����Э����

    // 2. ���㿨��������
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);  // ����������

    // 3. ���¹���ֵ���˲���������
    ekf->out = ekf->out + ekf->Kg * (input - ekf->out);  // ʹ�ÿ�����������¹���ֵ

    // 4. ���¹������Э����
    ekf->LastP = (1 - ekf->Kg) * ekf->Now_P;  // �������Э����
}

