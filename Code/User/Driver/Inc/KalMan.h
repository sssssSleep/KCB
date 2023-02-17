#ifndef __KAERMAN_H
#define __KAERMAN_H

/*
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
 
R参数调整滤波后的曲线与实测曲线的相近程度，r越小越接近。
 
Q参数调滤波后的曲线平滑程度，q越小越平滑。
 
*/

typedef struct
{
	float Q;//过程噪音
	float R;//测量噪声
	float x_last;
	float p_last;
}KalmanFilterData;
	
	

float KalmanFilter(float ResrcData,KalmanFilterData* KFD);


#endif  
