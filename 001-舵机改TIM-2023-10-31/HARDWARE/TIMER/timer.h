#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
#include "timer.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/4
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
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

extern int O_pwm;//ʵʱccr
extern float v_;//Ŀ���ٶ�
#endif
