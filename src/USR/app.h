
#ifndef __APP_H__
#define __APP_H__

#include "stdint.h"
extern const float balancedAngle;
typedef struct {
	float m_angle;
	float m_rate;
} angleTypeDef;

typedef struct {
	int32_t leftDuty;
	int32_t rightDuty;
} dutyTypeDef;

typedef struct {
	float m_accz;
	float m_gyro;
} balanceDataTypeDef;

typedef struct {
	float m_leftSpeed;
	float m_rightSpeed;
	float m_avgeSpeed;
} speedDataTypeDef;

//����ƽ�⻷ռ�ձ�
int32_t balanceCtrl(void);

//�����ٶȻ�ռ�ձ�
int32_t speedCtrl(void);

//���㷽��ռ�ձ�
int32_t directionCtrl(void);

//ʹ��ռ�ձȿ��Ƶ��
void motorControl(int32_t, int32_t, int32_t);

#endif
