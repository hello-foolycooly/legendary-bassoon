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
 ALIENTEKս��STM32������ʵ��32
 MPU6050���ᴫ���� ʵ��
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/
//����1����1���ַ� 
////c:Ҫ���͵��ַ�
//void usart1_send_char(u8 c)
//{   	
//	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
//	USART_SendData(USART1,c);  
//} 
//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
//void usart1_niming_report(u8 fun,u8*data,u8 len)
//{
//	u8 send_buf[32];
//	u8 i;
//	if(len>28)return;	//���28�ֽ����� 
//	send_buf[len+3]=0;	//У��������
//	send_buf[0]=0X88;	//֡ͷ
//	send_buf[1]=fun;	//������
//	send_buf[2]=len;	//���ݳ���
//	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
//	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���	
//	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
//}
//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
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
//	usart1_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
//}	
//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
//void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
//{
//	u8 tbuf[28]; 
//	u8 i;
//	for(i=0;i<28;i++)tbuf[i]=0;//��0
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
//	usart1_niming_report(0XAF,tbuf,28);//�ɿ���ʾ֡,0XAF
//}  	


 int main(void)
 {	 
  u8 i=8;
	u8 key;

//	u16 PWM=750;
	u16 PWM=1850;
//	u8  Direction=1;
	float X,Y;//����Ƕ�
	float pitch,roll,yaw; 		//ŷ����
  float pitch_avg=0,roll_avg=0,yaw_avg=0;//ŷ����ƽ��ֵ
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����	
//	short temp;					//�¶� 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(500000);	 	//���ڳ�ʼ��Ϊ500000
	delay_init();	//��ʱ��ʼ�� 
	//usmart_dev.init(72);		//��ʼ��USMART��ʹ����TIM4����ע�ͻ���PWM��TIM4��ͻ
	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();					//��ʼ������ 
	MPU_Init();					//��ʼ��MPU6050
  mpu_dmp_init();	


//  TIM2_PWM_Init(9999, 143); //TIM2_Int_Init(u16 arr, u16 psc)����ʼ����ʱ��TIM2
//	TIM2_PWM_Init(19999, 71); //TIM2_Int_Init(u16 arr, u16 psc)����ʼ����ʱ��TIM2
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
//			PWM=250;//0��
			PWM=45;//pwm 50����  0�� 0.5ms
//			PWM=1950;//0��
		}
		if(key==KEY1_PRES)
		{
//			PWM=750;//90��
			PWM=145;//pwm 150����  90�� 1.5ms
//			PWM=1850;//90��
		}
		if(key==KEY2_PRES)
		{
//			PWM=1250;//180��
			PWM=245;//pwm 250����  180�ȣ����7135������180��С��180�㣩 2500/20000=12.5% 2.5ms
//			PWM=1750;//180��
		}
		printf("�������:%3.2f\r\n", 0.18 * (PWM - 250));
    TIM_SetCompare1(TIM2, PWM); //ռ�ձȱ��������ô�װ�벶��ȽϼĴ���������ֵ���൱�ڲ�������TIM_Pulse
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
//			temp=MPU_Get_Temperature();	//�õ��¶�ֵ
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
      pitch+=pitch;//�ۼ�ֵ
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
////      printf("ƽ��������=%3.2f,ƽ��������=%3.2f,ƽ��ƫ����=%3.2f\r\n",pitch_avg,roll_avg,yaw_avg);
		}

		X=roll;
		Y=pitch;//�����������ǽǶ�

////		X=roll_avg;
////		Y=pitch_avg;//�����������ǽǶ�

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
 


