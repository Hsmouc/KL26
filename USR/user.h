
#ifndef __USR_H__
#define __USR_H__

#include "stdint.h"

//======================================================================
//��ȡADC���źź���
//��ڣ�ͨ��(channel):
//	0: ���ٶȼ�
//	1,2: ������
//	3,4,5,6,7: ��Ŵ�����
//���أ��ź�ֵ
//======================================================================
uint32_t ADC_GetValue(uint32_t channel);
//ADC��ʼ������
void ADC_userInit(void);
//GPIO��ʼ������
void GPIO_userInit(void);
//UART��ʼ������
void UART_userInit(void);
#endif
