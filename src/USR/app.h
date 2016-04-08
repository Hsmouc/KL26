
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
void balanceCtrl(dutyTypeDef* output);

//�����ٶȻ�ռ�ձ�
void speedCtrl(dutyTypeDef* output);

//���㷽��ռ�ձ�
void directionCtrl(dutyTypeDef* output);

// �������˲�����
void kalmanFilter(const balanceDataTypeDef* data, angleTypeDef* outAngle);

//ʹ��ռ�ձȿ��Ƶ��
void motorControl(const dutyTypeDef* output);

#endif
