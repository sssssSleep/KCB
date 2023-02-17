#include "mpu6050.h"
#include "delay.h"
#include "usart.h"
#include "i2c.h"
#include "Print.h"
#include "KalMan.h"
static MPU_data _mpu_data;
static MPU_data _mpu_data_kalman;
static KalmanFilterData KFD_pitch;
static KalmanFilterData KFD_roll;
static KalmanFilterData KFD_yaw;
/**
 * @brief 初始化MPU6050
 * @param 无
 * @return 状态 0成功 其他失败
 * @author HZ12138
 * @date 2022-08-08 14:51:59
 */
uint8_t MPU_Init(void)
{
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); //复位MPU6050
	HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); //唤醒MPU6050
	MPU_Set_Gyro_Fsr(3);					 //陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					 //加速度传感器,±2g
	MPU_Set_Rate(50);						 //设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	 //关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	 //关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
	MPU_Read_Byte(MPU_DEVICE_ID_REG);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); //设置CLKSEL,PLL X轴为参考
	MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); //加速度与陀螺仪都工作
	MPU_Set_Rate(50);						 //设置采样率为50Hz
	MPU_Kalmam_Init();
	_mpu_data.linear_cor_val_k = 0.000004625;
	_mpu_data.first = 1;
	_mpu_data.pitch_b = 0;
			_mpu_data.roll_b = 0;
			_mpu_data.yaw_b = 0;
	return 0;
}
/**
 * @brief 初始化MPU6050的欧拉角卡尔曼滤波
 * @param 无
 * @return 状态 0成功 其他失败
 * @author HZ12138
 * @date 2022-08-08 14:51:59
 */
uint8_t MPU_Kalmam_Init(void)
{
	KFD_pitch.Q = P_Q;
	KFD_pitch.R = P_R;
	KFD_roll.Q  = R_Q;
	KFD_roll.R  = R_R;
	KFD_yaw.Q   = Y_Q;
	KFD_yaw.R   = Y_R;
	KFD_pitch.p_last = 0.0f;
	KFD_roll.p_last  = 0.0f;
	KFD_yaw.p_last   = 0.0f;
  KFD_pitch.x_last = 0.0f;
	KFD_roll.x_last  = 0.0f;
	KFD_yaw.x_last   = 0.0f;
	return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); //设置陀螺仪满量程范围
}
//设置MPU6050加速度传感器满量程范围
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); //设置加速度传感器满量程范围
}
//设置MPU6050的数字低通滤波器
// lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data = 0;
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;
	return MPU_Write_Byte(MPU_CFG_REG, data); //设置数字低通滤波器
}
//设置MPU6050的采样率(假定Fs=1KHz)
// rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); //设置数字低通滤波器
	return MPU_Set_LPF(rate / 2);					  //自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
	uint8_t buf[2];
	short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
	raw = ((uint16_t)buf[0] << 8) | buf[1];
	temp = 36.53 + ((double)raw) / 340;
	return temp * 100;
	;
}
//得到陀螺仪值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*gx = ((uint16_t)buf[0] << 8) | buf[1];
		*gy = ((uint16_t)buf[2] << 8) | buf[3];
		*gz = ((uint16_t)buf[4] << 8) | buf[5];
	}
	return res;
}
//得到加速度值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*ax = ((uint16_t)buf[0] << 8) | buf[1];
		*ay = ((uint16_t)buf[2] << 8) | buf[3];
		*az = ((uint16_t)buf[4] << 8) | buf[5];
	}
	return res;
	;
}


void Update_MPU_Data(void)
{
		if(_mpu_data.first == 2)
		{
		}
		else if(_mpu_data.first == 0)
		{
			_mpu_data.base_time = HAL_GetTick();
			_mpu_data.first = 1;
		}
		else if(_mpu_data.first == 1&& HAL_GetTick()-_mpu_data.base_time >= 40000 )
		{
			mpu_dmp_get_data(&_mpu_data.pitch,&_mpu_data.roll,&_mpu_data.yaw);
			_mpu_data.pitch_b = _mpu_data.pitch;
			_mpu_data.roll_b = _mpu_data.roll;
			_mpu_data.yaw_b = _mpu_data.yaw;
			_mpu_data.first = 2;
		}
		mpu_dmp_get_data(&_mpu_data.pitch,&_mpu_data.roll,&_mpu_data.yaw);
		
			if(_mpu_data.pitch_b > __DBL_EPSILON__)
					_mpu_data.pitch -= _mpu_data.pitch_b;
			else
				_mpu_data.pitch += _mpu_data.pitch_b;		
				if(_mpu_data.roll_b > __DBL_EPSILON__)
					_mpu_data.roll += _mpu_data.roll_b;
			else
				_mpu_data.roll -= _mpu_data.roll_b;
		
				if(_mpu_data.yaw_b > __DBL_EPSILON__)
					_mpu_data.yaw += _mpu_data.yaw_b;
			else
				_mpu_data.yaw -= _mpu_data.yaw_b;
			
		MPU_Get_Accelerometer(&_mpu_data.ax,&_mpu_data.ay,&_mpu_data.az);
		_mpu_data.temp = MPU_Get_Temperature();
}
MPU_data Get_MPU_Data(void)
{
	return _mpu_data;

}
MPU_data Get_MPU_Data_Kalman(void)
{
	_mpu_data_kalman.pitch = KalmanFilter(_mpu_data.pitch,&KFD_pitch);
	_mpu_data_kalman.roll = KalmanFilter(_mpu_data.roll,&KFD_roll);
	_mpu_data_kalman.yaw = KalmanFilter(_mpu_data.yaw,&KFD_yaw);

	return _mpu_data_kalman;

}