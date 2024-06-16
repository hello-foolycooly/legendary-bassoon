#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
#include "timer.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/4
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//********************************************************************************
float PID_realize(float v);

void TIM2_PWM_Init(u16 arr, u16 psc);
void TIM3_PWM_Init(u16 arr, u16 psc);

void TIM6_Int_Init(u16 arr,u16 psc);
void Encoder_Init_TIM3(u16 arr,u16 psc);
int16_t Encoder_Get(void);
void TIM4_PWM_Init(u16 arr,u16 psc);
//void Car_Go1(int Su_Du);
//void Car_R(int Su_Du);
//void Car_L(int Su_Du);
//void Car_S(int Su_Du);
//#define XR1 PGin(13)
//#define XR PGin(12)
//#define XL PGin(15)
//#define XL1 PGin(14)

extern int O_pwm;//实时ccr
extern float v_;//目标速度
#endif
