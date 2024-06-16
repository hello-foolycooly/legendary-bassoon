#include "timer.h"
#include "usart.h"
#include "stdio.h"
#include "led.h"

//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��

void TIM2_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ���Ľṹ��
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //����һ����ʱ�жϵĽṹ��	
	TIM_OCInitTypeDef TIM_OCInitTypeStrue; //����һ��PWM����Ľṹ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��GPIOAʱ�ӣ���STM32��ʹ��IO��ǰ��Ҫʹ�ܶ�Ӧʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʹ��ͨ�ö�ʱ��2ʱ�ӣ�A0���Ŷ�ӦTIM2CHN1
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;//����0
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //�����������ģʽ����ʱ������ΪA0���Ÿ��ù���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //�������������ٶ�Ϊ50MHZ
  GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ������GPIOA0
	 
	TIM_TimeBaseInitStrue.TIM_Period=arr; //����ģʽΪ���ϼ���ʱ����ʱ����0��ʼ����������������arrʱ������ʱ�жϷ�����
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //Ԥ��Ƶϵ��������ÿһ��������ʱ��
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //����ģʽ�����ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=0; //һ�㲻ʹ�ã�Ĭ��TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStrue); //����TIM_TimeBaseInitStrue�Ĳ�����ʼ����ʱ��TIM2
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1; //PWMģʽ1������ʱ������С��TIM_Pulseʱ����ʱ����ӦIO�����Ч��ƽ
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCNPolarity_High; //�����Ч��ƽΪ�ߵ�ƽ
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable; //ʹ��PWM���
	TIM_OCInitTypeStrue.TIM_Pulse = 0; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OC1Init(TIM2, &TIM_OCInitTypeStrue); //��TIM_OCInitTypeStrue������ʼ����ʱ��2ͨ��1
	TIM_OC2Init(TIM2, &TIM_OCInitTypeStrue);


	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable); //CH1Ԥװ��ʹ��
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

	
	TIM_ARRPreloadConfig(TIM2, ENABLE); //CH1Ԥװ��ʹ��
	
	TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��TIM2
}

void TIM6_Int_Init(u16 arr,u16 psc)
{
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 NVIC_InitTypeDef NVIC_InitStructure;
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //��ʱ�� TIM3 ʹ��
 
 //��ʱ�� TIM3 ��ʼ��
 TIM_TimeBaseStructure.TIM_Period = arr; //�����Զ���װ�ؼĴ������ڵ�ֵ
 TIM_TimeBaseStructure.TIM_Prescaler =psc; //����ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM ���ϼ���

 TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //�ڳ�ʼ�� TIM3
 TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //����������ж�
 //�ж����ȼ� NVIC ����
 NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn; //TIM3 �ж�
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ� 0 ��
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //�����ȼ� 3 ��
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ����ʹ��
 NVIC_Init(&NVIC_InitStructure); //�ܳ�ʼ�� NVIC �Ĵ���
 TIM_Cmd(TIM6, ENABLE); //��ʹ�� TIM3
}
float v_=15.0;//��ʼĿ���ٶ�
//PID�����ṹ��
struct PID
{
 float err_00;//���ϴ�ƫ��
 float err_0;//�ϴ�ƫ��
 float err;//���
 float KP;//���������֣�΢��
 float KI;
 float KD;
 float bi_li;//������
 float ji_fen;//��������Ǵ˴����
 float wei_fen;//΢����
 float pwm_out;//���ֵ
// float v_;//Ŀ���ٶ�
};

struct PID pid=
{
 0.0,
 0.0,
 0.0,
 12.0,5.0,5.0,//���������֣�΢�֡�����ƽ�����߲�����12.0,5.0,5.0,����4/1���߲�����200.0,12.0,25.0,��
 0.0,
 0.0,
 0.0,
 0.0,//PWM��ʼֵ
// 20.0
};

//PID����
float PID_realize(float v)
{
 float U_;//����
 pid.err=v_-v;//Ŀ��ֵ����ʵֵ�Ĳ�ֵ
 pid.bi_li=pid.err-pid.err_0;//������
 pid.ji_fen=pid.err;//��������Ǵ˴����
 pid.wei_fen=pid.err-2*pid.err_0+pid.err_00;//΢����
 U_=pid.KP*pid.bi_li + pid.KI*pid.ji_fen + pid.KD*pid.wei_fen;//PID����
 pid.pwm_out+=U_;//ʵ��pwmֵ
 pid.err_00=pid.err_0;//�������ϴ�ƫ��
 pid.err_0=pid.err;//�����ϴ�ƫ��
 return pid.pwm_out;
}

//��ʱ�� 6 �жϷ������
int O_pwm;//
void TIM6_IRQHandler(void) //TIM3 �ж�
{
 if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) //��� TIM3 �����жϷ������
 {
  float v=(float)TIM3->CNT/4320.0*20/0.02;//����ʵ���ٶ�20ms
//  float v=(float)TIM3->CNT/4320.0*20/0.06;//����ʵ���ٶ�60ms
//  float v=(float)TIM3->CNT/4320.0*20;//����ʵ���ٶ�1s
  O_pwm=PID_realize(v);
  LED1=!LED1;
//  printf("60ms speed:%f\r\n",v);
  printf("%f,%f\r\n",v,v_);
  TIM3->CNT=0;
  TIM_ClearITPendingBit(TIM6, TIM_IT_Update ); //��� TIM3 �����жϱ�־
 }
}


void Encoder_Init_TIM3(u16 arr,u16 psc)
{  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //ʹ��AFIO���ù���ģ��ʱ��,��ӳ����ⲿ�ж�Ҫ����AFIO
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA6.7
 
   //��ʼ��TIM3
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //д0һ�������ǲ���Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ؼ���
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //��ʼ����ʱ��3

  TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//�������ӿ�ģʽѡ�񣬼���,���ԣ�û�з���
	
  TIM_ICStructInit(&TIM_ICInitStructure);//��TIM_ICStruct �е�ÿһ��������ȱʡֵ����
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;//ͨ��1
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//ӳ�䵽TI1��
  TIM_ICInitStructure.TIM_ICFilter = 10;  //�����˲�������
  TIM_ICInit(TIM3,&TIM_ICInitStructure);//����TIM_ICStruct�Ĳ�����ʼ������TIMx

  TIM_ICStructInit(&TIM_ICInitStructure);//��TIM_ICStruct �е�ÿһ��������ȱʡֵ����
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;//ͨ��2
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//ӳ�䵽TI1��
  TIM_ICInitStructure.TIM_ICFilter = 10;  //�����˲�������
  TIM_ICInit(TIM3,&TIM_ICInitStructure);//����TIM_ICStruct�Ĳ�����ʼ������TIMx
  
  TIM_ClearFlag(TIM3,TIM_FLAG_Update);//���TIM�ĸ��±�־λ
//  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);//ʹ�ܶ�ʱ���ж�
  
  TIM_SetCounter(TIM3,0);//CNT����
  TIM3->CNT = 0;//CNT���ֵ
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
	
}

//�����з��ŵ�16λ��������ֵ
int16_t Encoder_Get(void)
{
  int16_t Temp;
  Temp = ((short)TIM3 -> CNT);
  return Temp;
}


void TIM4_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ�ܶ�ʱ��4ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��,��ӳ����ⲿ�ж�Ҫ����AFIO
	
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //Timer4��ȫ��ӳ��  TIM4_CH->PC6789 
 
   //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOC.6789
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; //TIM_CH1234
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIO
 
   //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 
 
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM3
	
}
//void Car_Go1(int Su_Du)
//   {
//		 TIM_SetCompare1(TIM4,Su_Du);//pd12 �Ҳ�+ in1
//		 TIM_SetCompare2(TIM4,0);//pd13 �Ҳ�- in2
//		 TIM_SetCompare3(TIM4,Su_Du);//pd14 ���+ in4
//		 TIM_SetCompare4(TIM4,0);//pd15 ���-  in3
//		 
////		 TIM_SetCompare1(TIM3,Su_Du);//pc6 �Ҳ�+ in1
////		 TIM_SetCompare2(TIM3,0);//pc7 �Ҳ�- in2
////		 TIM_SetCompare3(TIM3,Su_Du);//pc8 ���+ in4
////		 TIM_SetCompare4(TIM3,0);//pc9 ���-  in3
//	 }
//void Car_R(int Su_Du)
//   {
//		 TIM_SetCompare1(TIM4,0);//pd12 �Ҳ�+ in1
//		 TIM_SetCompare2(TIM4,Su_Du);//pd13 �Ҳ�- in2
//		 TIM_SetCompare3(TIM4,Su_Du);//pd14 ���+ in4
//		 TIM_SetCompare4(TIM4,0);//pd15 ���-  in3
////		 TIM_SetCompare1(TIM4,Su_Du);//pd12 1ǰ
////		 TIM_SetCompare2(TIM4,0);//pd13 1��
////		 TIM_SetCompare3(TIM4,Su_Du);//pd14 2ǰ
////		 TIM_SetCompare4(TIM4,0);//pd15 2��
//		 
////		 TIM_SetCompare1(TIM3,0);//pc6 3ǰ
////		 TIM_SetCompare2(TIM3,Su_Du);//pc7 3��
////		 TIM_SetCompare3(TIM3,0);//pc8 4ǰ
////		 TIM_SetCompare4(TIM3,Su_Du);//pc9 4��
//	 }
//void Car_L(int Su_Du)
//   {
//		 TIM_SetCompare1(TIM4,Su_Du);//pd12 �Ҳ�+ in1
//		 TIM_SetCompare2(TIM4,0);//pd13 �Ҳ�- in2
//		 TIM_SetCompare3(TIM4,0);//pd14 ���+ in4
//		 TIM_SetCompare4(TIM4,Su_Du);//pd15 ���-  in3
////		 TIM_SetCompare1(TIM4,0);//pd12 1ǰ
////		 TIM_SetCompare2(TIM4,Su_Du);//pd13 1��
////		 TIM_SetCompare3(TIM4,0);//pd14 2ǰ
////		 TIM_SetCompare4(TIM4,Su_Du);//pd15 2��
//		 
////		 TIM_SetCompare1(TIM3,Su_Du);//pc6 3ǰ
////		 TIM_SetCompare2(TIM3,0);//pc7 3��
////		 TIM_SetCompare3(TIM3,Su_Du);//pc8 4ǰ
////		 TIM_SetCompare4(TIM3,0);//pc9 4��
//	 }
//void Car_S(int Su_Du)
//   {
//		 TIM_SetCompare1(TIM4,0);//pd12 �Ҳ�+ in1
//		 TIM_SetCompare2(TIM4,Su_Du);//pd13 �Ҳ�- in2
//		 TIM_SetCompare3(TIM4,0);//pd14 ���+ in4
//		 TIM_SetCompare4(TIM4,Su_Du);//pd15 ���-  in3
//	 }
