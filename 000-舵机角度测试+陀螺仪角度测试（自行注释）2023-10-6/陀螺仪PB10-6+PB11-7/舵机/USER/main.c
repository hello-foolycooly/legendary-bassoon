#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "mpu6050.h"
#include "usmart.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "timer.h"
 
/************************************************
 ALIENTEK战舰STM32开发板实验32
 MPU6050六轴传感器 实验
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/
//串口1发送1个字符 
////c:要发送的字符
//void usart1_send_char(u8 c)
//{   	
//	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
//	USART_SendData(USART1,c);  
//} 
//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
//void usart1_niming_report(u8 fun,u8*data,u8 len)
//{
//	u8 send_buf[32];
//	u8 i;
//	if(len>28)return;	//最多28字节数据 
//	send_buf[len+3]=0;	//校验数置零
//	send_buf[0]=0X88;	//帧头
//	send_buf[1]=fun;	//功能字
//	send_buf[2]=len;	//数据长度
//	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
//	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
//	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
//}
//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
//{
//	u8 tbuf[12]; 
//	tbuf[0]=(aacx>>8)&0XFF;
//	tbuf[1]=aacx&0XFF;
//	tbuf[2]=(aacy>>8)&0XFF;
//	tbuf[3]=aacy&0XFF;
//	tbuf[4]=(aacz>>8)&0XFF;
//	tbuf[5]=aacz&0XFF; 
//	tbuf[6]=(gyrox>>8)&0XFF;
//	tbuf[7]=gyrox&0XFF;
//	tbuf[8]=(gyroy>>8)&0XFF;
//	tbuf[9]=gyroy&0XFF;
//	tbuf[10]=(gyroz>>8)&0XFF;
//	tbuf[11]=gyroz&0XFF;
//	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
//}	
//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
//void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
//{
//	u8 tbuf[28]; 
//	u8 i;
//	for(i=0;i<28;i++)tbuf[i]=0;//清0
//	tbuf[0]=(aacx>>8)&0XFF;
//	tbuf[1]=aacx&0XFF;
//	tbuf[2]=(aacy>>8)&0XFF;
//	tbuf[3]=aacy&0XFF;
//	tbuf[4]=(aacz>>8)&0XFF;
//	tbuf[5]=aacz&0XFF; 
//	tbuf[6]=(gyrox>>8)&0XFF;
//	tbuf[7]=gyrox&0XFF;
//	tbuf[8]=(gyroy>>8)&0XFF;
//	tbuf[9]=gyroy&0XFF;
//	tbuf[10]=(gyroz>>8)&0XFF;
//	tbuf[11]=gyroz&0XFF;	
//	tbuf[18]=(roll>>8)&0XFF;
//	tbuf[19]=roll&0XFF;
//	tbuf[20]=(pitch>>8)&0XFF;
//	tbuf[21]=pitch&0XFF;
//	tbuf[22]=(yaw>>8)&0XFF;
//	tbuf[23]=yaw&0XFF;
//	usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
//}  	


 int main(void)
 {	 
  u8 i=8;
	u8 key;

//	u16 PWM=750;
	u16 PWM=1850;
//	u8  Direction=1;
	float X,Y;//舵机角度
	float pitch,roll,yaw; 		//欧拉角
  float pitch_avg=0,roll_avg=0,yaw_avg=0;//欧拉角平均值
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据	
//	short temp;					//温度 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(500000);	 	//串口初始化为500000
	delay_init();	//延时初始化 
	//usmart_dev.init(72);		//初始化USMART中使能了TIM4，不注释会与PWM的TIM4冲突
	LED_Init();		  			//初始化与LED连接的硬件接口
	KEY_Init();					//初始化按键 
	MPU_Init();					//初始化MPU6050
  mpu_dmp_init();	


//  TIM2_PWM_Init(9999, 143); //TIM2_Int_Init(u16 arr, u16 psc)，初始化定时器TIM2
//	TIM2_PWM_Init(19999, 71); //TIM2_Int_Init(u16 arr, u16 psc)，初始化定时器TIM2
	TIM2_PWM_Init(1999, 719);
  Encoder_Init_TIM3(65535,0);
  TIM4_PWM_Init(2879,0);
  TIM6_Int_Init(1439,999);//20ms
//  TIM6_Int_Init(2159,1999);//60ms 
 	while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
//			PWM=250;//0度
			PWM=45;//pwm 50修正  0度 0.5ms
//			PWM=1950;//0度
		}
		if(key==KEY1_PRES)
		{
//			PWM=750;//90度
			PWM=145;//pwm 150修正  90度 1.5ms
//			PWM=1850;//90度
		}
		if(key==KEY2_PRES)
		{
//			PWM=1250;//180度
			PWM=245;//pwm 250修正  180度（舵机7135到不了180，小于180°） 2500/20000=12.5% 2.5ms
//			PWM=1750;//180度
		}
		printf("舵机度数:%3.2f\r\n", 0.18 * (PWM - 250));
    TIM_SetCompare1(TIM2, PWM); //占空比比例，设置待装入捕获比较寄存器的脉冲值，相当于不断设置TIM_Pulse
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
//			temp=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
      pitch+=pitch;//累加值
      roll+=roll;
      yaw+=yaw;
      if(i==20)
      {
        pitch_avg=pitch/20.0;
        roll_avg=roll/20.0;
        yaw_avg=yaw/20.0;
        pitch=0;
        roll=0;
        yaw=0;
        i=0;
      }
      i++;
////      printf("平均俯仰角=%3.2f,平均翻滚角=%3.2f,平均偏航角=%3.2f\r\n",pitch_avg,roll_avg,yaw_avg);
		}

		X=roll;
		Y=pitch;//舵机获得陀螺仪角度

////		X=roll_avg;
////		Y=pitch_avg;//舵机获得陀螺仪角度

////////		if(X<0)
////////		{
//////////			X=-X;
//////////			TIM_SetCompare2(TIM2,145+(int)X);

			printf("X=%3.2f\r\n",X);
////////		}
////////		else
////////		{
//////////      TIM_SetCompare2(TIM2,145-(int)X);

////////			printf("X=%3.2f\r\n",X);
////////		}	
////////		
////////		if(Y<0)
////////		{
//////////			Y=-Y;
//////////			TIM_SetCompare1(TIM2,145+(int)Y);
			printf("Y=%3.2f\r\n",Y);
////////		}
////////		else
////////		{
//////////      TIM_SetCompare1(TIM2,145-(int)Y);
////////			printf("Y=%3.2f\r\n",Y);
////////		}	
////    if(yaw_avg>+0.5)
////    {
////      Car_R(2599);
////		}
////    if(yaw_avg<-0.5)
////    {
////			Car_L(2599);
////		}
////    if((pitch_avg>+0.05)&&(yaw_avg>=-0.5)&&(yaw_avg<=0.5))
////    {
////			v_=25.0;
////			Car_Go1(O_pwm);
////		}
////    if((pitch_avg<-0.05)&&(yaw_avg>=-0.5)&&(yaw_avg<=0.5))
////    {
////			v_=15.0;
////			Car_Go1(O_pwm);
////		}
////    if((pitch_avg>=-0.05)&&(pitch_avg<=0.05)&&(yaw_avg>=-0.5)&&(yaw_avg<=0.5)) 
////    {
////			v_=0.0;
////			Car_Go1(O_pwm);
////		}

//		delay_ms(100);


	} 	
}
 


