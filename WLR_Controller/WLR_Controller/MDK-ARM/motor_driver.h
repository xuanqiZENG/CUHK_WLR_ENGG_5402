

#ifndef __MOTOR_H__
#define __MOTOR_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "can.h"
#include "main.h"
//typedef struct
//{
	//float Speed;//FeedBack Speed of motor
	//float Position;//FeedBack Position
//	float Current;//FeedBack Current
//}reporter;

typedef struct reporter_
{
	float Speed;//FeedBack Speed of motor
	float Position;//FeedBack Position
  float Current;//FeedBack Current
} reporter;



extern CAN_TxHeaderTypeDef   TxHeader_1;
extern CAN_RxHeaderTypeDef   RxHeader_1;
extern uint8_t               TxData_1[8];
extern uint8_t               RxData_1[8];

extern CAN_TxHeaderTypeDef   TxHeader_2;
extern CAN_RxHeaderTypeDef   RxHeader_2;
extern uint8_t               TxData_2[8];
extern uint8_t               RxData_2[8];

float PID(float qdes,float qcur,float dqcur);

//struct reporter Feedback(uint32_t motorID);
reporter Feedback(uint8_t motor_data[8]);
float PID_wheel(float dqdes,float dqcur,float ddqcur);
float PID_pitch(float pitch_des,float pitch_curr,float dpitch);
void Motor_Drive(float I_command[3]);

void Motor_SetMode(uint8_t Mode1,uint8_t Mode2,uint8_t Mode3);
void Motor_SetMode_2(uint8_t Mode1,uint8_t Mode2,uint8_t Mode3);
void Motor_Calibration(void);
void Motor_Drive_2(float I_command[3]);
#ifdef __cplusplus
}
#endif
#endif /* __MOTRO_H__ */