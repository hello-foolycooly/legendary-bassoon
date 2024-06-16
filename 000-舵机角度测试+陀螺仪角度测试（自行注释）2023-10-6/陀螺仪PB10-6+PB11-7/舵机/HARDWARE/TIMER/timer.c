#include "timer.h"
#include "usart.h"
#include "stdio.h"
#include "led.h"

//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数

void TIM2_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //定义一个定时中断的结构体	
	TIM_OCInitTypeDef TIM_OCInitTypeStrue; //定义一个PWM输出的结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIOA时钟，在STM32中使用IO口前都要使能对应时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //使能通用定时器2时钟，A0引脚对应TIM2CHN1
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;//引脚0
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //复用推挽输出模式，定时器功能为A0引脚复用功能
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //定义该引脚输出速度为50MHZ
  GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化引脚GPIOA0
	 
	TIM_TimeBaseInitStrue.TIM_Period=arr; //计数模式为向上计数时，定时器从0开始计数，计数超过到arr时触发定时中断服务函数
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //预分频系数，决定每一个计数的时长
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //计数模式：向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=0; //一般不使用，默认TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStrue); //根据TIM_TimeBaseInitStrue的参数初始化定时器TIM2
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1; //PWM模式1，当定时器计数小于TIM_Pulse时，定时器对应IO输出有效电平
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCNPolarity_High; //输出有效电平为高电平
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable; //使能PWM输出
	TIM_OCInitTypeStrue.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OC1Init(TIM2, &TIM_OCInitTypeStrue); //根TIM_OCInitTypeStrue参数初始化定时器2通道1
	TIM_OC2Init(TIM2, &TIM_OCInitTypeStrue);


	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable); //CH1预装载使能
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

	
	TIM_ARRPreloadConfig(TIM2, ENABLE); //CH1预装载使能
	
	TIM_Cmd(TIM2, ENABLE); //使能定时器TIM2
}

void TIM6_Int_Init(u16 arr,u16 psc)
{
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 NVIC_InitTypeDef NVIC_InitStructure;
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //①时钟 TIM3 使能
 
 //定时器 TIM3 初始化
 TIM_TimeBaseStructure.TIM_Period = arr; //设置自动重装载寄存器周期的值
 TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置时钟频率除数的预分频值
 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数

 TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //②初始化 TIM3
 TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //③允许更新中断
 //中断优先级 NVIC 设置
 NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn; //TIM3 中断
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //先占优先级 0 级
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //从优先级 3 级
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
 NVIC_Init(&NVIC_InitStructure); //④初始化 NVIC 寄存器
 TIM_Cmd(TIM6, ENABLE); //⑤使能 TIM3
}
float v_=15.0;//初始目标速度
//PID变量结构体
struct PID
{
 float err_00;//上上次偏差
 float err_0;//上次偏差
 float err;//误差
 float KP;//比例，积分，微分
 float KI;
 float KD;
 float bi_li;//比例项
 float ji_fen;//积分项，就是此次误差
 float wei_fen;//微分项
 float pwm_out;//输出值
// float v_;//目标速度
};

struct PID pid=
{
 0.0,
 0.0,
 0.0,
 12.0,5.0,5.0,//比例，积分，微分。缓增平稳曲线参数（12.0,5.0,5.0,）、4/1曲线参数（200.0,12.0,25.0,）
 0.0,
 0.0,
 0.0,
 0.0,//PWM初始值
// 20.0
};

//PID函数
float PID_realize(float v)
{
 float U_;//增量
 pid.err=v_-v;//目标值与真实值的差值
 pid.bi_li=pid.err-pid.err_0;//比例项
 pid.ji_fen=pid.err;//积分项，就是此次误差
 pid.wei_fen=pid.err-2*pid.err_0+pid.err_00;//微分项
 U_=pid.KP*pid.bi_li + pid.KI*pid.ji_fen + pid.KD*pid.wei_fen;//PID增量
 pid.pwm_out+=U_;//实际pwm值
 pid.err_00=pid.err_0;//更新上上次偏差
 pid.err_0=pid.err;//更新上次偏差
 return pid.pwm_out;
}

//定时器 6 中断服务程序
int O_pwm;//
void TIM6_IRQHandler(void) //TIM3 中断
{
 if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) //检查 TIM3 更新中断发生与否
 {
  float v=(float)TIM3->CNT/4320.0*20/0.02;//计算实际速度20ms
//  float v=(float)TIM3->CNT/4320.0*20/0.06;//计算实际速度60ms
//  float v=(float)TIM3->CNT/4320.0*20;//计算实际速度1s
  O_pwm=PID_realize(v);
  LED1=!LED1;
//  printf("60ms speed:%f\r\n",v);
  printf("%f,%f\r\n",v,v_);
  TIM3->CNT=0;
  TIM_ClearITPendingBit(TIM6, TIM_IT_Update ); //清除 TIM3 更新中断标志
 }
}


void Encoder_Init_TIM3(u16 arr,u16 psc)
{  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //使能AFIO复用功能模块时钟,重映射和外部中断要开启AFIO
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA6.7
 
   //初始化TIM3
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //写0一样，都是不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //边沿计数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //初始化定时器3

  TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//编码器接口模式选择，极性,极性（没有反向）
	
  TIM_ICStructInit(&TIM_ICInitStructure);//把TIM_ICStruct 中的每一个参数按缺省值填入
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;//通道1
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//映射到TI1上
  TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
  TIM_ICInit(TIM3,&TIM_ICInitStructure);//根据TIM_ICStruct的参数初始化外设TIMx

  TIM_ICStructInit(&TIM_ICInitStructure);//把TIM_ICStruct 中的每一个参数按缺省值填入
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;//通道2
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//映射到TI1上
  TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
  TIM_ICInit(TIM3,&TIM_ICInitStructure);//根据TIM_ICStruct的参数初始化外设TIMx
  
  TIM_ClearFlag(TIM3,TIM_FLAG_Update);//清除TIM的更新标志位
//  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);//使能定时器中断
  
  TIM_SetCounter(TIM3,0);//CNT清零
  TIM3->CNT = 0;//CNT设初值
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
	
}

//返回有符号的16位计数器的值
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
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能定时器4时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟,重映射和外部中断要开启AFIO
	
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //Timer4完全重映射  TIM4_CH->PC6789 
 
   //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOC.6789
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; //TIM_CH1234
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO
 
   //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 
 
	TIM_Cmd(TIM4, ENABLE);  //使能TIM3
	
}
//void Car_Go1(int Su_Du)
//   {
//		 TIM_SetCompare1(TIM4,Su_Du);//pd12 右侧+ in1
//		 TIM_SetCompare2(TIM4,0);//pd13 右侧- in2
//		 TIM_SetCompare3(TIM4,Su_Du);//pd14 左侧+ in4
//		 TIM_SetCompare4(TIM4,0);//pd15 左侧-  in3
//		 
////		 TIM_SetCompare1(TIM3,Su_Du);//pc6 右侧+ in1
////		 TIM_SetCompare2(TIM3,0);//pc7 右侧- in2
////		 TIM_SetCompare3(TIM3,Su_Du);//pc8 左侧+ in4
////		 TIM_SetCompare4(TIM3,0);//pc9 左侧-  in3
//	 }
//void Car_R(int Su_Du)
//   {
//		 TIM_SetCompare1(TIM4,0);//pd12 右侧+ in1
//		 TIM_SetCompare2(TIM4,Su_Du);//pd13 右侧- in2
//		 TIM_SetCompare3(TIM4,Su_Du);//pd14 左侧+ in4
//		 TIM_SetCompare4(TIM4,0);//pd15 左侧-  in3
////		 TIM_SetCompare1(TIM4,Su_Du);//pd12 1前
////		 TIM_SetCompare2(TIM4,0);//pd13 1后
////		 TIM_SetCompare3(TIM4,Su_Du);//pd14 2前
////		 TIM_SetCompare4(TIM4,0);//pd15 2后
//		 
////		 TIM_SetCompare1(TIM3,0);//pc6 3前
////		 TIM_SetCompare2(TIM3,Su_Du);//pc7 3后
////		 TIM_SetCompare3(TIM3,0);//pc8 4前
////		 TIM_SetCompare4(TIM3,Su_Du);//pc9 4后
//	 }
//void Car_L(int Su_Du)
//   {
//		 TIM_SetCompare1(TIM4,Su_Du);//pd12 右侧+ in1
//		 TIM_SetCompare2(TIM4,0);//pd13 右侧- in2
//		 TIM_SetCompare3(TIM4,0);//pd14 左侧+ in4
//		 TIM_SetCompare4(TIM4,Su_Du);//pd15 左侧-  in3
////		 TIM_SetCompare1(TIM4,0);//pd12 1前
////		 TIM_SetCompare2(TIM4,Su_Du);//pd13 1后
////		 TIM_SetCompare3(TIM4,0);//pd14 2前
////		 TIM_SetCompare4(TIM4,Su_Du);//pd15 2后
//		 
////		 TIM_SetCompare1(TIM3,Su_Du);//pc6 3前
////		 TIM_SetCompare2(TIM3,0);//pc7 3后
////		 TIM_SetCompare3(TIM3,Su_Du);//pc8 4前
////		 TIM_SetCompare4(TIM3,0);//pc9 4后
//	 }
//void Car_S(int Su_Du)
//   {
//		 TIM_SetCompare1(TIM4,0);//pd12 右侧+ in1
//		 TIM_SetCompare2(TIM4,Su_Du);//pd13 右侧- in2
//		 TIM_SetCompare3(TIM4,0);//pd14 左侧+ in4
//		 TIM_SetCompare4(TIM4,Su_Du);//pd15 左侧-  in3
//	 }
