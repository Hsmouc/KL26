
#ifndef __APP_H__
#define __APP_H__

#include "stdint.h"

//����ƽ�⻷ռ�ձ�
int32_t balanceCtrl(void);

//�����ٶȻ�ռ�ձ�
int32_t speedCtrl(void);

//���㷽��ռ�ձ�
int32_t directionCtrl(void);

//ʹ��ռ�ձȿ��Ƶ��
void motorControl(int32_t,int32_t,int32_t);

#endif
