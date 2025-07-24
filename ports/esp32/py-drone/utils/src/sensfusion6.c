#include <math.h>
#include "sensfusion6.h"
#include "config.h"
#include "ledseq.h"
#include "maths.h"


#define ACCZ_SAMPLE		350

float Kp = 0.4f;		/*比例增益*/
float Ki = 0.001f;		/*积分增益*/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*积分误差累计*/

static float q0 = 1.0f;	/*四元数*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;	
static float rMat[3][3];/*旋转矩阵*/

static float maxError = 0.f;		/*最大误差*/
bool isGravityCalibrated = false;	/*是否校校准完成*/
static float baseAcc[3] = {0.f,0.f,1.0f};	/*静态加速度*/

// 重力校准状态变量（移到文件级便于重置）
static uint16_t gravityCalCnt = 0;
static float gravityAccZMin = 1.5f;
static float gravityAccZMax = 0.5f;
static float gravitySumAcc[3] = {0.f};


static float invSqrt(float x);	/*快速开平方求倒*/
// resetGravityCalibration函数声明已移到头文件

static void calBaseAcc(float* acc)	/*计算静态加速度*/
{
	// 使用文件级变量（便于重置）
	for(uint8_t i=0; i<3; i++)
		gravitySumAcc[i] += acc[i];
		
	if(acc[2] < gravityAccZMin)	gravityAccZMin = acc[2];
	if(acc[2] > gravityAccZMax)	gravityAccZMax = acc[2];
	
	if(++gravityCalCnt >= ACCZ_SAMPLE) /*缓冲区满*/
	{
		gravityCalCnt = 0;
		maxError = gravityAccZMax - gravityAccZMin;
		gravityAccZMin = 1.5f;
		gravityAccZMax = 0.5f;
		
		if(maxError < 0.015f)
		{
			for(uint8_t i=0; i<3; i++)
				baseAcc[i] = gravitySumAcc[i] / ACCZ_SAMPLE;
			
			isGravityCalibrated = true;
			
			ledseqRun(SYS_LED, seq_calibrated);	/*校准通过指示灯*/
		}
		
		for(uint8_t i=0; i<3; i++)		
			gravitySumAcc[i] = 0.f;		
	}	
}

// 重置重力校准状态和IMU融合状态
void resetGravityCalibration(void)
{
	// 1. 重置重力校准状态
	isGravityCalibrated = false;
	gravityCalCnt = 0;
	gravityAccZMin = 1.5f;
	gravityAccZMax = 0.5f;
	maxError = 0.f;
	
	// 2. 重置重力校准累积变量
	for(uint8_t i=0; i<3; i++) {
		gravitySumAcc[i] = 0.f;
		baseAcc[i] = (i == 2) ? 1.0f : 0.f; // 重置为默认值 [0,0,1]
	}
	
	// 3. *** 关键修复：重置四元数到初始状态 ***
	q0 = 1.0f;  // 单位四元数
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	
	// 4. 重置旋转矩阵到单位矩阵
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			rMat[i][j] = (i == j) ? 1.0f : 0.0f;
		}
	}
}

/*计算旋转矩阵*/
void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt)	/*数据融合 互补滤波*/
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;
	float accBuf[3] = {0.f};
	Axis3f tempacc = acc;
	
	gyro.x = gyro.x * DEG2RAD;	/* 度转弧度 */
	gyro.y = gyro.y * DEG2RAD;
	gyro.z = gyro.z * DEG2RAD;

	/* 加速度计输出有效时,利用加速度计补偿陀螺仪*/
	if((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
	{
		/*单位化加速计测量值*/
		normalise = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
		acc.x *= normalise;
		acc.y *= normalise;
		acc.z *= normalise;

		/*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
		ex = (acc.y * rMat[2][2] - acc.z * rMat[2][1]);
		ey = (acc.z * rMat[2][0] - acc.x * rMat[2][2]);
		ez = (acc.x * rMat[2][1] - acc.y * rMat[2][0]);
		
		/*误差累计，与积分常数相乘*/
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		/*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
		gyro.x += Kp * ex + exInt;
		gyro.y += Kp * ey + eyInt;
		gyro.z += Kp * ez + ezInt;
	}
	/* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
	q0 += (-q1Last * gyro.x - q2Last * gyro.y - q3Last * gyro.z) * halfT;
	q1 += ( q0Last * gyro.x + q2Last * gyro.z - q3Last * gyro.y) * halfT;
	q2 += ( q0Last * gyro.y - q1Last * gyro.z + q3Last * gyro.x) * halfT;
	q3 += ( q0Last * gyro.z + q1Last * gyro.y - q2Last * gyro.x) * halfT;
	
	/*单位化四元数*/
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
	
	imuComputeRotationMatrix();	/*计算旋转矩阵*/
	
	/*计算roll pitch yaw 欧拉角*/
	state->attitude.pitch = -asinf(rMat[2][0]) * RAD2DEG; 
	state->attitude.roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	state->attitude.yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
	
	if (!isGravityCalibrated)	/*未校准*/
	{		
//		accBuf[0] = tempacc.x* rMat[0][0] + tempacc.y * rMat[0][1] + tempacc.z * rMat[0][2];	/*accx*/
//		accBuf[1] = tempacc.x* rMat[1][0] + tempacc.y * rMat[1][1] + tempacc.z * rMat[1][2];	/*accy*/
		accBuf[2] = tempacc.x* rMat[2][0] + tempacc.y * rMat[2][1] + tempacc.z * rMat[2][2];	/*accz*/
		calBaseAcc(accBuf);		/*计算静态加速度*/				
	}
}
//获取四元数
void getQuarternion(float *pQ0,float *pQ1,float *pQ2,float *pQ3)
{
	*pQ0 =  q0;
	*pQ1 =  q1;
	*pQ2 =  q2;
	*pQ3 =  q3;
}
/*机体到地球*/
void imuTransformVectorBodyToEarth(Axis3f * v)
{
    /* From body frame to earth frame */
    const float x = rMat[0][0] * v->x + rMat[0][1] * v->y + rMat[0][2] * v->z;
    const float y = rMat[1][0] * v->x + rMat[1][1] * v->y + rMat[1][2] * v->z;
    const float z = rMat[2][0] * v->x + rMat[2][1] * v->y + rMat[2][2] * v->z;

	float yawRad = atan2f(rMat[1][0], rMat[0][0]);
	float cosy = cosf(yawRad);
	float siny = sinf(yawRad);
	float vx = x * cosy + y * siny;
	float vy = y * cosy - x * siny;	
	
    v->x = vx;
    v->y = -vy;
    v->z = z - baseAcc[2] *  980.f;	/*去除重力加速度*/
}

/*地球到机体*/
void imuTransformVectorEarthToBody(Axis3f * v)
{
    v->y = -v->y;

    /* From earth frame to body frame */
    const float x = rMat[0][0] * v->x + rMat[1][0] * v->y + rMat[2][0] * v->z;
    const float y = rMat[0][1] * v->x + rMat[1][1] * v->y + rMat[2][1] * v->z;
    const float z = rMat[0][2] * v->x + rMat[1][2] * v->y + rMat[2][2] * v->z;

    v->x= x;
    v->y = y;
    v->z = z;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)	/*快速开平方求倒*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

bool getIsCalibrated(void)
{
	return isGravityCalibrated;
}
void setCalibrated(bool set)
{
	isGravityCalibrated = set;
}


