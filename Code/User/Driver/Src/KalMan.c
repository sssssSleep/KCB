/*
描述本文件

作者+日期
*/

#include "KalMan.h"


//注释格式
/***************************************************
*@brief:  描述																				 
*@param:  参数                                      
*@retval: 返回值                                        
*@author: 作者+日期                                         
****************************************************/
/*
 ============================================================================
 Name        : KaerMan.c
 Author      : 
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */
#include <stdio.h>
#include <stdlib.h>
 
 
 
static float p_last = 0;
static float x_last = 0;
 
/*
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
 
r参数调整滤波后的曲线与实测曲线的相近程度，r越小越接近。
 
q参数调滤波后的曲线平滑程度，q越小越平滑。
 
*/
float KalmanFilter(float ResrcData,KalmanFilterData* KFD)
{
	
    float R = KFD->R;
    float Q = KFD->Q;
 
    float x_mid = KFD->x_last;
    float x_now;
 
    float p_mid ;
    float p_now;
 
    float kg;
 
    //这里p_last 等于 kalmanFilter_A 的p直接取0
    x_mid=KFD->x_last;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=KFD->p_last+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
 
    /*
      *  卡尔曼滤波的五个重要公式
      */
    kg=p_mid/(p_mid+R);                 //kg为kalman filter，R 为噪声
    x_now=x_mid+kg*(ResrcData-x_mid);   //估计出的最优值
    p_now=(1-kg)*p_mid;                 //最优值对应的covariance
		//printf("KDF: R = %f , Q = %f, pmid = %f,kg = %f ,xmid = %f, x_now = %f p_now = %f,indata = %f \r\n",R,Q,p_mid,kg,x_mid,x_now,p_now,ResrcData);
    KFD->p_last = p_now;                     //更新covariance 值
    KFD->x_last = x_now;                     //更新系统状态值
		//printf("kdf = %f\r\n",x_now);
    return x_now;
}
// 
//float KalmanFilter_A(float inData)
//{
//  static float prevData=0;
//  //其中p的初值可以随便取，但是不能为0（为0的话卡尔曼滤波器就认为已经是最优滤波器了）
//  static float p=0.01, q=P_Q, r=M_R, kGain=0;
//    p = p+q;
//    kGain = p/(p+r);
// 
//    inData = prevData+(kGain*(inData-prevData));
//    p = (1-kGain)*p;
// 
//    prevData = inData;
// 
//    return inData;
//}