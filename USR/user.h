
#ifndef __USR_H__
#define __USR_H__

#include "stdint.h"
#include "sys.h"

typedef struct{
  GPIO_Type* GPIO;
  uint16_t Pin;
} gpioPinTypeDef;

extern uint8_t pwmLeft;
extern uint8_t pwmRight;

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
