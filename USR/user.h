
#ifndef __USR_H__
#define __USR_H__

#include "stdint.h"


//ADC��ʼ������
void ADC_userInit(void);
//======================================================================
//��ȡADC���źź���
//��ڣ�ͨ��(channel):
//	0: ���ٶȼ�
//	1,2: ������
//	3,4,5,6,7: ��Ŵ�����
//���أ��ź�ֵ
//======================================================================
uint32_t ADC_GetValue(uint8_t chl);
//GPIO��ʼ������
void GPIO_userInit(void);
//PIT��ʼ������
void PIT_userInit(void);
#endif
