


#include "motor_driver.h"
#include "stdint.h"
#include "can.h"
#include "main.h"
#include "math.h"

uint16_t Speed;
uint16_t Current;
uint16_t Position;
float P=90;//70
float D=350;//210
float Pv=10;//3.5
float Dv=200;//17 55
float P_pitch=5;
float D_pitch=0;
float data_limit=32767;
float qmax=6.28;
float vmax=22;
float imax=33;
float Current_limit=29;

reporter report;

void Motor_Drive(float I_command[3])
{
	uint16_t n=0;
	while(n<8)
	{
		TxData_1[n++]=0;
	}
	
	for(int i=0;i<3;i++)
	{
	if (I_command[i]<0)
	{
		I_command[i]=I_command[i]*data_limit/imax+2*data_limit;
	}
	else
	{
		I_command[i]=I_command[i]*data_limit/imax;
	}
  }
	TxData_1[0]=((uint16_t)I_command[0]>>8);
	TxData_1[1]=(uint16_t)I_command[0] & 0xff;
	TxData_1[2]=((uint16_t)I_command[1]>>8);
	TxData_1[3]=(uint16_t)I_command[1] & 0xff;
	TxData_1[4]=((uint16_t)I_command[2]>>8);
	TxData_1[5]=(uint16_t)I_command[2] & 0xff;
	TxHeader_1.StdId=0x1FF;
	USER_CAN_Send();			
}

void Motor_Drive_2(float I_command[3])
{
	uint16_t n=0;
	while(n<8)
	{
		TxData_2[n++]=0;
	}
	
	for(int i=0;i<3;i++)
	{
	if (I_command[i]<0)
	{
		I_command[i]=I_command[i]*data_limit/imax+2*data_limit;
	}
	else
	{
		I_command[i]=I_command[i]*data_limit/imax;
	}
  }
	TxData_2[0]=((uint16_t)I_command[0]>>8);
	TxData_2[1]=(uint16_t)I_command[0] & 0xff;
	TxData_2[2]=((uint16_t)I_command[1]>>8);
	TxData_2[3]=(uint16_t)I_command[1] & 0xff;
	TxData_2[4]=((uint16_t)I_command[2]>>8);
	TxData_2[5]=(uint16_t)I_command[2] & 0xff;
	TxHeader_2.StdId=0x1FF;
	USER_CAN_Send_2();			
}


reporter Feedback(uint8_t motor_data[8])
{
  //if (RxHeader.StdId==motorID)
	//
		Speed=(motor_data[0]<<8)+motor_data[1];
		Current=(motor_data[2]<<8)+motor_data[3];
		Position=(motor_data[4]<<8)+motor_data[5];
		report.Speed=(float)Speed;
		report.Current=(float)Current;
		report.Position=(float)Position;
		if ((report.Speed)>data_limit)
		{
			report.Speed=(report.Speed-2*data_limit);
		}
		if ((report.Current)>data_limit)
		{
			report.Current=(report.Current-2*data_limit);
		}
		if ((report.Position)>data_limit)
   	{
			report.Position=(report.Position-data_limit);
		}
		report.Position=report.Position/(data_limit)*qmax;
		report.Current=report.Current/(data_limit)*imax;
		report.Speed=report.Speed/(data_limit)*vmax;
		return report;
}


float PID(float qdes,float qcur,float dqcur)
{
   float I=0;
//	if (fabs(qdes-qcur)>0.05)
//	{
//	if (qdes>6.18)
//	{
//		qdes=6.18;
//	}
//	else if (qdes<0.1)
//	{
//		qdes=0.1;
//	}
  if (I>Current_limit)
	{
		I=Current_limit;
	}
	else if (I<-Current_limit)
	{
		I=-Current_limit;
	}
	 I=P*(qdes-qcur)+D*dqcur;
//  }
	 return I;
}


float PID_wheel(float dqdes,float dqcur,float ddqcur)
{
	 float I=0;
//	if (fabs(dqdes-dqcur)>0.08)
//	{
//	if (dqdes>6)
//	{
//		dqdes=6;
//	}
//	else if (dqdes<-6)
//	{
//		dqdes=-6;
//	}
	
  if (I>Current_limit)
	{
		I=Current_limit;
	}
	else if (I<-Current_limit)
	{
		I=-Current_limit;
	}
	 I=Pv*(dqdes-dqcur)+Dv*ddqcur;
//  }
	 return I;	
	
}

float PID_pitch(float pitch_des,float pitch_curr,float dpitch)
{
	 float I=0;
	//if (fabs(qdes-qcur)>0.01)
	//{
	 I=P_pitch*(pitch_des-pitch_curr)+D_pitch*dpitch;
  //}
	 return I;	
	
}


void Motor_SetMode(uint8_t Mode1,uint8_t Mode2,uint8_t Mode3)
{
	TxData_1[0]=Mode1;
	TxData_1[1]=Mode2;
	TxData_1[2]=Mode3;
	TxHeader_1.StdId=0x3FF;
	USER_CAN_Send();
}

void Motor_SetMode_2(uint8_t Mode1,uint8_t Mode2,uint8_t Mode3)
{
	TxData_2[0]=Mode1;
	TxData_2[1]=Mode2;
	TxData_2[2]=Mode3;
	TxHeader_2.StdId=0x3FF;
	USER_CAN_Send_2();
}


void Motor_Calibration(void)
{
	uint16_t i=0;
	while(i<8)
	{
		TxData_1[i++]=0;
	}
	TxHeader_1.StdId=0x3FF;
	
	i=0;
	while(i<8)
	{
		TxData_2[i++]=0;
	}
	TxHeader_2.StdId=0x3FF;
	
	USER_CAN_Send();
	USER_CAN_Send_2();
}


