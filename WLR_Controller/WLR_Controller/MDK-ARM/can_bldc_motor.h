#ifndef CAN_BLDC_H_
#define CAN_BLDC_H_

#include "struct_typedef.h"
#include "main.h"
#include "bsp_delay.h"

#define MOTOR_LEG1_ABAD 0x01
#define MOTOR_LEG1_HIP 0x02
#define MOTOR_LEG1_KNEE 0x03
#define MOTOR_LEG2_ABAD 0x01
#define MOTOR_LEG2_HIP 0x02
#define MOTOR_LEG2_KNEE 0x03

#define REST_MODE 0
#define MOTOR_MODE 1
#define ZERO_POSITION 2

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
 
#define KI_MIN -48.0f
#define KI_MAX 48.0f
#define KI_MIN_1_16 -75.f
#define KI_MAX_1_16 75.f

typedef enum reverse_forward{
	forward = 0,
	reverse = 1,
} RF;

typedef struct _BLDC_TypeDef
{
	int32_t ID[6];
	fp32 Position[6];
	fp32 Velocity[6];
	fp32 Current[6];
	uint8_t zero_flag[6];
	uint8_t enable_flag[6];
	uint8_t error_zero_flag[6];
} BLDC_Measure_TypeDef;

void CAN_SetMsg_Leg1_ABAD(void);
void CAN_SetMsg_Leg1_HIP(void);
void CAN_SetMsg_Leg1_KNEE(void);
void CAN_SetMsg_Leg2_ABAD(void);
void CAN_SetMsg_Leg2_HIP(void);
void CAN_SetMsg_Leg2_KNEE(void);
void Receive_BLDC_Data(BLDC_Measure_TypeDef* mot,uint8_t* data, uint8_t id);
const BLDC_Measure_TypeDef* get_BLDC_Measure(void);

void CAN_BLDC_cmd(CAN_HandleTypeDef *hcan ,uint8_t* BLDC_tx_message_data, float p_des, float v_des, float kp, float kd, float t_ff, void(*CAN_Mesg)(void));
void SET_MOTOR_MODE(CAN_HandleTypeDef *hcan, uint8_t M_MODE, void(*CAN_Mesg)(void));
void BLDC_MOTORS_DATA_INIT(void);

void DISABLE_LEGS(void);
void ENABLE_LEGS(void);
void Set_Zero_Position(void);
void Control_motors(RF if_reverse);

void Check_Motor_Status(void);
extern void (*CAN_SETMESSAGES[6])(void);
void control_zero(void);
void prepare_to_test(void);
#endif
