#include "app.h"
#include "user.h"
#include "TPM.h"


//倾角值表，注意：以加速度计竖直状态时为0度
const float  Asin_to_Angle[] = {
-90.000000,-81.890386,-78.521659,-75.930132,-73.739795,-71.805128,-70.051556,-68.434815,-66.926082,-65.505352,
-64.158067,-62.873247,-61.642363,-60.458639,-59.316583,-58.211669,-57.140120,-56.098738,-55.084794,-54.095931,
-53.130102,-52.185511,-51.260575,-50.353889,-49.464198,-48.590378,-47.731416,-46.886394,-46.054480,-45.234915,
-44.427004,-43.630109,-42.843643,-42.067065,-41.299873,-40.541602,-39.791819,-39.050123,-38.316134,-37.589503,
-36.869898,-36.157008,-35.450543,-34.750226,-34.055798,-33.367013,-32.683639,-32.005455,-31.332251,-30.663830,
-30.000000,-29.340582,-28.685402,-28.034297,-27.387108,-26.743684,-26.103881,-25.467560,-24.834587,-24.204835,
-23.578178,-22.954499,-22.333683,-21.715617,-21.100196,-20.487315,-19.876874,-19.268775,-18.662925,-18.059230,
-17.457603,-16.857956,-16.260205,-15.664267,-15.070062,-14.477512,-13.886540,-13.297072,-12.709033,-12.122352,
-11.536959,-10.952784,-10.369760,-9.787819, -9.206896, -8.626927, -8.047846, -7.469592, -6.892103, -6.315316,
-5.739170, -5.163607, -4.588566, -4.013987, -3.439813, -2.865984, -2.292443, -1.719131, -1.145992, -0.572967,
0.000000,0.572967, 1.145992, 1.719131, 2.292443, 2.865984, 3.439813, 4.013987, 4.588566, 5.163607,  5.739170,
6.315316, 6.892103, 7.469592, 8.047846, 8.626927, 9.206896, 9.787819, 10.369760,10.952784,11.536959,
12.122352,12.709033,13.297072,13.886540,14.477512,15.070062,15.664267,16.260205,16.857956,17.457603,
18.059230,18.662925,19.268775,19.876874,20.487315,21.100196,21.715617,22.333683,22.954499,23.578178,
24.204835,24.834587,25.467560,26.103881,26.743684,27.387108,28.034297,28.685402,29.340582,30.000000,
30.663830,31.332251,32.005455,32.683639,33.367013,34.055798,34.750226,35.450543,36.157008,36.869898,
37.589503,38.316134,39.050123,39.791819,40.541602,41.299873,42.067065,42.843643,43.630109,44.427004,
45.234915,46.054480,46.886394,47.731416,48.590378,49.464198,50.353889,51.260575,52.185511,53.130102,
54.095931,55.084794,56.098738,57.140120,58.211669,59.316583,60.458639,61.642363,62.873247,64.158067,
65.505352,66.926082,68.434815,70.051556,71.805128,73.739795,75.930132,78.521659,81.890386,90.000000,
};
#define GYRO_ZERO  0x980 //平衡陀螺仪静止时的输出值
#define ACCZ_ZERO  0x4F0 //加速度计竖直时的输出值
	uint16_t tmp_accz, tmp_gyro;

//采集平衡环所需数据
void getBalanceData(balanceDataTypeDef* data){
	float tmp;

	//采集并初步处理加速度计的值
	tmp_accz = ADC_GetValue(0)>>4;
	tmp = (tmp_accz-ACCZ_ZERO)*0.100708;
	if(tmp>100) { tmp = 100; }
	if(tmp<-100) { tmp = -100; }
	data->m_accz = Asin_to_Angle[(uint8_t)(tmp+100)];
	//采集并初步处理陀螺仪的值
	tmp_gyro = ADC_GetValue(1)>>4;
	data->m_gyro = (GYRO_ZERO-tmp_gyro)*0.120248;
}

// 计算平衡环占空比
int32_t balanceControl(const balanceDataTypeDef* data, angleTypeDef* angle) {
	static const float balanceKp = 1600;
	static const float balanceKd = 0;
	static const float balancedAngle = 31.2;
	kalman(data, angle);
	return (int32_t)(balanceKp*(angle->angle - balancedAngle)+balanceKd*angle->angleDot);
}

// 卡尔曼滤波函数
void kalman(const balanceDataTypeDef* data, angleTypeDef* angle){
	static const float qAngle=0.001, qGyro=0.003, rAngle=0.67, dt=0.005;
	static float e;
	static float qBias;
	static float k0, k1;
	static float t0, t1;
	static float angleErr;
	static float pCt0, pCt1;
	static float pDot[4] ={0,0,0,0};
	static float p[2][2] = {{ 1, 0 },{ 0, 1 }};
	// 先验估计
	angle->angle += (data->m_gyro-qBias)*dt;
	// Pk-' 先验估计误差协方差的微分
	pDot[0] = qAngle - p[0][1] - p[1][0]; 
	pDot[1] = -p[1][1];
	pDot[2] = -p[1][1];
	pDot[3] = qGyro;
	// Pk- 先验估计误差协方差微分的积分=先验估计误差协方差
	p[0][0] += pDot[0] * dt;              
	p[0][1] += pDot[1] * dt;
	p[1][0] += pDot[2] * dt;
	p[1][1] += pDot[3] * dt;
	// zk-先验估计
	angleErr = data->m_accz - angle->angle;
	
	pCt0 = p[0][0];
	pCt1 = p[1][0];
	
	e = rAngle + pCt0;
	// Kk
	k0 = pCt0 / e;
	k1 = pCt1 / e;
	
	t0 = pCt0;
	t1 = p[0][1];
	// 后验估计误差协方差
	p[0][0] -= k0 * t0;
	p[0][1] -= k0 * t1;
	p[1][0] -= k1 * t0;
	p[1][1] -= k1 * t1;
	
	// 后验估计	
	qBias+= k1 * angleErr;
	// 输出值(后验估计)的微分=角速度
	angle->angle	+= k0 * angleErr;
	angle->angleDot = data->m_gyro-qBias;
}

//const uint32_t deathVotageLeft = 800;
//const uint32_t deathVotageRight = 600;

int32_t left, right;

//使用占空比控制电机
void motorControl(const spdTypeDef* spd){
	left = (uint32_t)(32768-spd->m_spd_balance);
	right = (uint32_t)(32768-spd->m_spd_balance);
	
//	right += deathVotageRight * (right>0x8000?1:-1);
//	left += deathVotageLeft * (left>0x8000?1:-1);
	
	if(left>MAX_SPD) left = MAX_SPD;
	else if(left<MIN_SPD)left = MIN_SPD;
	if(right>MAX_SPD) right = MAX_SPD;
	else if(right<MIN_SPD) right = MIN_SPD;

	PWMOutput(PTA5,right);
	PWMOutput(PTA12,left);
}
