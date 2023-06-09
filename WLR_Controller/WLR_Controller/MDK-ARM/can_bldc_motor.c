#include "can_bldc_motor.h"
#include "main.h"
#include "can.h"
#include "bsp_can.h"
#include "string.h"
#include "bsp_led.h"
#include "stored_data.h"
#include "tim.h"

CAN_TxHeaderTypeDef  BLDC_tx_message;
BLDC_Measure_TypeDef BLDC_Motor;
uint8_t BLDC_tx_message_data[8];
void (*CAN_SETMESSAGES[6])(void);
uint8_t enable = 0;
const uint16_t CAN_DELAY_TIME = 250;
const uint16_t CAN_ENABLE_DELAY = 10;
const float initial_position[6] = {0, 0, 0, 0, 0, 0}; 
const float position_vari = 0.01; //abs
int line_iter = 0;
int data_index = 0;
fp32 kp = 50.;
fp32 kd = 5.;
fp32 positive_or_negative = 1.;
int call_loop_1;
int call_loop_2;
int8_t first_run = 1;
fp32 offset_knee = 0.7;
//设置位置

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint8_t motor_status_flag;

void BLDC_MOTORS_DATA_INIT(void)
{
	memset(&BLDC_tx_message, 0, sizeof(BLDC_tx_message));
	memset(&BLDC_Motor, 0, sizeof(BLDC_Motor));
	memset(BLDC_tx_message_data, 0, sizeof(BLDC_tx_message_data));

	CAN_SETMESSAGES[0] = CAN_SetMsg_Leg1_ABAD;
	CAN_SETMESSAGES[1] = CAN_SetMsg_Leg1_HIP;
	CAN_SETMESSAGES[2] = CAN_SetMsg_Leg1_KNEE;
	CAN_SETMESSAGES[3] = CAN_SetMsg_Leg2_ABAD;
	CAN_SETMESSAGES[4] = CAN_SetMsg_Leg2_HIP;
	CAN_SETMESSAGES[5] = CAN_SetMsg_Leg2_KNEE;
}


void CAN_SetMsg_Leg1_ABAD(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG1_ABAD;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
}

void CAN_SetMsg_Leg1_HIP(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG1_HIP;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
	//BLDC_tx_message.TransmitGlobalTime = DISABLE;
}
 
void CAN_SetMsg_Leg1_KNEE(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG1_KNEE;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
	//BLDC_tx_message.TransmitGlobalTime = DISABLE;
}

void CAN_SetMsg_Leg2_ABAD(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG2_ABAD;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
	//BLDC_tx_message.TransmitGlobalTime = DISABLE;
}

void CAN_SetMsg_Leg2_HIP(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG2_HIP;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
	//BLDC_tx_message.TransmitGlobalTime = DISABLE;
}
 
void CAN_SetMsg_Leg2_KNEE(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG2_KNEE;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
	//BLDC_tx_message.TransmitGlobalTime = DISABLE;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	call_loop_1++;
	CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[6];
	static uint8_t id_1;
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	switch((rx_header.StdId) & 0x0f)
	{
		case 0x00:
			id_1 = rx_data[0] - 1;
			Receive_BLDC_Data(&BLDC_Motor, rx_data, id_1);
		  BLDC_Motor.zero_flag[id_1] = 1;
			break;
		case 0x0E:
			id_1 = rx_data[0] - 1;
			BLDC_Motor.enable_flag[id_1] = 1;
		  break;
		case 0x0A:
			id_1 = rx_data[0] - 1;
			Receive_BLDC_Data(&BLDC_Motor, rx_data, id_1);
			BLDC_Motor.zero_flag[id_1] = 1;
		  break;
		case 0x0F:
			id_1 = rx_data[0] - 1;
			Receive_BLDC_Data(&BLDC_Motor, rx_data, id_1);
			BLDC_Motor.zero_flag[id_1] = 1;
			break;
	}
	//第一位为电机id 123 123

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	call_loop_2++;
	CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[6];
	static uint8_t id_2;
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
	switch((rx_header.StdId) & 0x0f)
	{
		case 0x00:
			id_2 = rx_data[0] + 2;
			Receive_BLDC_Data(&BLDC_Motor, rx_data, id_2);
			break;
		case 0x0E:
			id_2 = rx_data[0] + 2;
			BLDC_Motor.enable_flag[id_2] = 1;
		  break;
		case 0x0A:
			id_2 = rx_data[0] + 2;
			Receive_BLDC_Data(&BLDC_Motor, rx_data, id_2);
			BLDC_Motor.zero_flag[id_2] = 1;
		  break;
		case 0x0F:
			id_2 = rx_data[0] + 2;
			Receive_BLDC_Data(&BLDC_Motor, rx_data, id_2);
			BLDC_Motor.error_zero_flag[id_2] = 1;
			break;
	}
}

void Receive_BLDC_Data(BLDC_Measure_TypeDef* mot,uint8_t* data, uint8_t id)
{
	
	int8_t id_temp =  data[0];
	uint16_t p_int = ( data[1]<<8)| data[2];
  uint16_t v_int = ( data[3]<<4)|( data[4]>>4);
  uint16_t i_int = ((data[4]&0xF)<<8)| data[5];
  /// convert ints to floats ///
  float p_temp = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v_temp = uint_to_float(v_int, V_MIN, V_MAX, 12);
	float i_temp = 0;
	if((id_temp == 3) || (id_temp == 6))
		i_temp = uint_to_float(i_int, KI_MIN_1_16, KI_MAX_1_16, 12);
	else
		i_temp = uint_to_float(i_int, KI_MIN, KI_MAX, 12);
	
	mot->ID[id] = id_temp;
	mot->Position[id] = p_temp;
	mot->Velocity[id] = v_temp;
	mot->Current[id] = i_temp;
}

void CAN_BLDC_cmd(CAN_HandleTypeDef *hcan ,uint8_t* BLDC_tx_message_data, float p_des, float v_des, float kp, float kd, float t_ff, void(*CAN_Mesg)(void))
{
	uint32_t send_mail_box;
  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);                    
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
  kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
  t_ff = fminf(fmaxf(KI_MIN, t_ff), KI_MAX);
  /// convert floats to unsigned ints ///
  uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
  uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
	uint16_t t_int = 0;
	if((CAN_Mesg == CAN_SETMESSAGES[2]) || (CAN_Mesg == CAN_SETMESSAGES[5]))
		t_int = float_to_uint(t_ff, KI_MIN_1_16, KI_MAX_1_16, 12);
	else
		t_int = float_to_uint(t_ff, KI_MIN, KI_MAX, 12);
  /// pack ints into the can buffer ///
  BLDC_tx_message_data[0] = p_int>>8;                                       
  BLDC_tx_message_data[1] = p_int&0xFF;
  BLDC_tx_message_data[2] = v_int>>4;
  BLDC_tx_message_data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
  BLDC_tx_message_data[4] = kp_int&0xFF;
  BLDC_tx_message_data[5] = kd_int>>4;
  BLDC_tx_message_data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
  BLDC_tx_message_data[7] = t_int&0xff;
	CAN_Mesg();
	HAL_CAN_AddTxMessage(hcan, &BLDC_tx_message, BLDC_tx_message_data, &send_mail_box);
}

int loop_for = 0;
int loop_reverse = 0;
float test_torque = 0;

void Control_motors(RF if_reverse)
{
	if(first_run)
	{
		prepare_to_test();
		delay_ms(2000);
		first_run = 0;
		return;
	}
	
	if(if_reverse == forward)
	{
		data_index = target_status_column * line_iter;
		CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, positive_or_negative * target_status[data_index], 
																						 positive_or_negative * target_status[data_index + 3], 
																						 kp,
																						 kd,
																						 positive_or_negative * target_status[data_index + 6],
																						 CAN_SETMESSAGES[0]);	
		delay_us(CAN_DELAY_TIME);
		CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, positive_or_negative * target_status[data_index + 1],  
																						 positive_or_negative * target_status[data_index + 4],
																						 kp,
																						 kd, 
																						 positive_or_negative * target_status[data_index + 7], 
																						 CAN_SETMESSAGES[1]);
		delay_us(CAN_DELAY_TIME);
		CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, positive_or_negative * target_status[data_index + 2] + offset_knee, 
																							 positive_or_negative * target_status[data_index + 5], 
																							 kp,
																							 kd, 
																							 positive_or_negative * target_status[data_index + 8], 
																							 CAN_SETMESSAGES[2]);									 
		delay_us(CAN_DELAY_TIME);
		line_iter++;
		if(line_iter == target_status_line)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
		}
	}
	else if (if_reverse == reverse)
	{
		line_iter--;
		if(line_iter == -1)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
			line_iter = 0;
			return;
		}
		data_index = target_status_column * line_iter;
		CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, positive_or_negative * target_status[data_index], 
																						 positive_or_negative * target_status[data_index + 3], 
																						 kp,
																						 kd,
																						 positive_or_negative * target_status[data_index + 6],
																						 CAN_SETMESSAGES[0]);	
		delay_us(CAN_DELAY_TIME);
		CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, positive_or_negative * target_status[data_index + 1],  
																						 positive_or_negative * target_status[data_index + 4],
																						 kp,
																						 kd, 
																						 positive_or_negative * target_status[data_index + 7], 
																						 CAN_SETMESSAGES[1]);
		delay_us(CAN_DELAY_TIME);
		CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, positive_or_negative * target_status[data_index + 2] + offset_knee, 
																							 positive_or_negative * target_status[data_index + 5], 
																							 kp,
																							 kd, 
																							 positive_or_negative * target_status[data_index + 8], 
																							 CAN_SETMESSAGES[2]);									 
		delay_us(CAN_DELAY_TIME);
		
	}
	else
	{
		Error_Handler();
	}
}


void SET_MOTOR_MODE(CAN_HandleTypeDef *hcan, uint8_t M_MODE, void(*CAN_Mesg)(void))
{
	uint32_t send_mail_box;
	switch(M_MODE)
	{
		case REST_MODE:
		BLDC_tx_message_data[0]=0XFF; 
		BLDC_tx_message_data[1]=0XFF; 
		BLDC_tx_message_data[2]=0XFF;
		BLDC_tx_message_data[3]=0XFF;
		BLDC_tx_message_data[4]=0XFF;
		BLDC_tx_message_data[5]=0XFF;
		BLDC_tx_message_data[6]=0XFF;
		BLDC_tx_message_data[7]=0XFD;
		CAN_Mesg();
		HAL_CAN_AddTxMessage(hcan, &BLDC_tx_message, BLDC_tx_message_data, &send_mail_box);
		break;
		
		case MOTOR_MODE:
		BLDC_tx_message_data[0]=0XFF; 
		BLDC_tx_message_data[1]=0XFF; 
		BLDC_tx_message_data[2]=0XFF;
		BLDC_tx_message_data[3]=0XFF;
		BLDC_tx_message_data[4]=0XFF;
		BLDC_tx_message_data[5]=0XFF;
		BLDC_tx_message_data[6]=0XFF;
		BLDC_tx_message_data[7]=0XFC;
		CAN_Mesg();
		HAL_CAN_AddTxMessage(hcan, &BLDC_tx_message, BLDC_tx_message_data, &send_mail_box);
		break;
	
		case ZERO_POSITION:
		BLDC_tx_message_data[0] = 0xFF;
		BLDC_tx_message_data[1] = 0xFF;
		BLDC_tx_message_data[2] = 0xFF;
		BLDC_tx_message_data[3] = 0xFF;
		BLDC_tx_message_data[4] = 0xFF;
		BLDC_tx_message_data[5] = 0xFF;
		BLDC_tx_message_data[6] = 0xF1;
		BLDC_tx_message_data[7] = 0x00;  
		CAN_Mesg();
		HAL_CAN_AddTxMessage(hcan, &BLDC_tx_message, BLDC_tx_message_data, &send_mail_box);
		break;
	};
}


void ENABLE_LEGS()
{
	SET_MOTOR_MODE(&hcan1, MOTOR_MODE, CAN_SETMESSAGES[0]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan1, MOTOR_MODE, CAN_SETMESSAGES[1]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan1, MOTOR_MODE, CAN_SETMESSAGES[2]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan2, MOTOR_MODE, CAN_SETMESSAGES[3]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan2, MOTOR_MODE, CAN_SETMESSAGES[4]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan2, MOTOR_MODE, CAN_SETMESSAGES[5]);
	delay_ms(CAN_ENABLE_DELAY);
}

void DISABLE_LEGS()
{
	SET_MOTOR_MODE(&hcan1, REST_MODE, CAN_SETMESSAGES[0]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan1, REST_MODE, CAN_SETMESSAGES[1]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan1, REST_MODE, CAN_SETMESSAGES[2]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan2, REST_MODE, CAN_SETMESSAGES[3]);
	delay_ms(CAN_ENABLE_DELAY);	
	SET_MOTOR_MODE(&hcan2, REST_MODE, CAN_SETMESSAGES[5]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan2, REST_MODE, CAN_SETMESSAGES[4]);
	delay_ms(CAN_ENABLE_DELAY);
}

void Set_Zero_Position()
{
	SET_MOTOR_MODE(&hcan1, ZERO_POSITION, CAN_SETMESSAGES[0]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan1, ZERO_POSITION, CAN_SETMESSAGES[1]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan1, ZERO_POSITION, CAN_SETMESSAGES[2]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan2, ZERO_POSITION, CAN_SETMESSAGES[3]);
	delay_ms(CAN_ENABLE_DELAY);	
	SET_MOTOR_MODE(&hcan2, ZERO_POSITION, CAN_SETMESSAGES[4]);
	delay_ms(CAN_ENABLE_DELAY);
	SET_MOTOR_MODE(&hcan2, ZERO_POSITION, CAN_SETMESSAGES[5]);
	delay_ms(CAN_ENABLE_DELAY);
}

void Check_Motor_Status()
{
	uint8_t check_zero = 0;
	while(1)
	{
		check_zero = 0;
		for(uint8_t i = 0; i < 3; i++)
		{
//			if((BLDC_Motor.error_zero_flag[i]) == 1)
//			{
//				Error_Handler();
//			}
			
			if(BLDC_Motor.zero_flag[i] == 1)
			{
				check_zero++;
			}
		}
		
		if(3 == check_zero)
			break;
	}
	
	while(1)
	{
		motor_status_flag = 0;
		for(uint8_t i = 0; i < 3; i++)
		{
			if((BLDC_Motor.Position[i] < (initial_position[i] + position_vari)) && (BLDC_Motor.Position[i] > (initial_position[i] - position_vari)))
			{
				motor_status_flag++;
			}
		}
		
		if(motor_status_flag == 3) //ready
		{
			turn_on_all_leds();
			break;
		}
	}
	
}

const BLDC_Measure_TypeDef* get_BLDC_Measure()
{
	return &BLDC_Motor;
}

void control_zero()
{
	CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, 0, 
																						 0, 
																						 0,
																						 0,
																						 0,
																						 CAN_SETMESSAGES[0]);	
		delay_us(CAN_DELAY_TIME);
		CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, 0, 
																						 0, 
																						 0,
																						 0,
																						 0 ,
																						 CAN_SETMESSAGES[1]);
		delay_us(CAN_DELAY_TIME);
		CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, 0, 
																						 0, 
																						 0,
																						 0,
																						 0, 
																							 CAN_SETMESSAGES[2]);									 
		delay_us(CAN_DELAY_TIME);
}

void prepare_to_test()
{
	CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, positive_or_negative * target_status[0], 
																						 positive_or_negative * target_status[3], 
																						 kp,
																						 kd,
																						 0,
																						 CAN_SETMESSAGES[0]);	
	delay_us(CAN_DELAY_TIME);
	CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, positive_or_negative * target_status[1],  
																						 positive_or_negative * target_status[4],
																						 kp,
																						 kd, 
																						 0, 
																						 CAN_SETMESSAGES[1]);
	delay_us(CAN_DELAY_TIME);
	CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, positive_or_negative * target_status[2] + offset_knee, 
																							 positive_or_negative * target_status[5], 
																							 kp,
																							 kd, 
																							 0, 
																							 CAN_SETMESSAGES[2]);									 
	delay_us(CAN_DELAY_TIME);
}

	
