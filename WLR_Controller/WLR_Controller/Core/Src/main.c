/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_driver.h"
#include "remote_control.h"
#include "string.h"
#include "WLR_controller.h"
#include "bsp_delay.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
reporter report1;
reporter report2;
reporter report3;
reporter report4;
reporter report5;
reporter report6;
reporter report10;
reporter report20;
reporter report30;
reporter report40;
reporter report50;
reporter report60;
invkin_q joint_1_q;
invkin_q joint_2_q;
invkin_q joint_q0;
kin_xy body_1_p;
kin_xy body_2_p;
kin_xy body_1_p_r;
kin_xy body_2_p_r;
char Kd_receive[3];
char *endptr;

uint8_t               RxData_1_1[8];
uint8_t               RxData_1_2[8];
uint8_t               RxData_1_3[8];
uint8_t               RxData_2_1[8];
uint8_t               RxData_2_2[8];
uint8_t               RxData_2_3[8];
const RC_ctrl_t* Rc_command;
float Icommand[3];
float Icommand_2[3];
char Kp_receive[3];
float check_en;
float IMU_check_cont;
uint8_t IMU_data[44];
uint32_t IMU[11];
typedef struct imu{
	float q[4];
	float gyro[3];
	float accel[3];
	uint32_t check;
} imu_handle;
r_p_y rpy_t;
r_p_y rpy_t0;
float* q[4];
float* gyro[3];
float* acc[3];
float yy=0.14;
float err[4];
float PI=3.1415926;
float yaw=0;
float pitch_des=0;
float ch[4];
float Kp_pitch=18; //12 20
float Kd_pitch=110; //15 120
uint8_t Kp_Kd[8];
uint8_t control_motor_flag = 0;
imu_handle imu;
float i=0;
uint8_t ifok=0;
uint8_t tail_flag = 0;
float Kp_pre;
float Kd_pre;
float tt=0;
float x_offset;
int counter=0;
float pitch_ctrl=0;
float pitch_ctrl0=0;
float yaw_ctrl=0;
float yaw_ctrl0=0;
float sum_pitch;
float sum_yaw;
int update_period=10;//25 10
float pitch_recent[10];
float yaw_recent[10];
float left_right=0;
float position_left=0;
float position_right=0;
float position_robot=0;
float position_robot_old=0;
float wheel_radius=0.187/2.0;
float P_command=0;
float Kp_position=3;
float Kd_position=9;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //pid_init(motor_pid);
	//motor_pid->f_param_init(motor_pid,0,3,0.1,0.01,0.01,0.01,0,1,0,0);
	//motor_pid->f_pid_reset(motor_pid,1,0,0);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart6, IMU_data, 44);
//	__HAL_UART_ENABLE_IT(&huart8,UART_IT_IDLE);
//	HAL_UART_Receive_DMA(&huart8, Kp_Kd, 8);
	tail_flag = 1;
	delay_init();
	for (int i=0;i<4;i++){
	 q[i]=(float*)&imu.q[i];
	}
	for (int i=0;i<3;i++){
	 gyro[i]=(float*)&imu.gyro[i];
	 acc[i]=(float*)&imu.accel[i];
	}
	memset(&imu, 0, sizeof(imu));
	memset(&TxData_1, 0, sizeof(TxData_1));
	memset(&TxData_2, 0, sizeof(TxData_2));
   //Motor_SetMode(0x03);
	 //ID_Set(1);
	Encoder_Check();
	IMU_check();
	
	//Motor_Calibration();
	remote_control_init();
	
	HAL_Delay(100);
	Rc_command=get_remote_control_point();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_Delay(2000);
	Motor_SetMode(0x01,0x01,0x01);
	HAL_Delay(2000);
	Motor_SetMode_2(0x01,0x01,0x01);
	HAL_Delay(2000);
	i=i+0.1;
	report10=report1;
	report20=report2;
	report30=report3;
	report40=report4;
	report50=report5;
	report60=report6;
	x_offset=-0.009;
	body_1_p=kinematic(report1.Position, report2.Position);
	body_1_p_r.x=x_offset;
	body_1_p.y=0.05;
	//joint_1_q=invkin(0,body_1_p.y);
	body_2_p=kinematic_2(report4.Position, report5.Position);
	//body_2_p_r.x=x_offset;
	body_2_p.y=0.05;
  //joint_2_q=invkin_2(0, body_2_p.y);
	yaw=rpy_t.yaw;
	pitch_des=rpy_t.pitch;
	ch[3]=yaw;
	//HAL_UART_Transmit_DMA(&huart8,RxData_1_2,8);
	
	//for vofa+
	//HAL_UART_Transmit_DMA(&huart8,(uint8_t*)ch,16);
	
	
	pitch_ctrl=rpy_t.pitch;
	pitch_ctrl0=rpy_t.pitch;
	yaw_ctrl=rpy_t.yaw;
	yaw_ctrl0=rpy_t.yaw;
  while (1)
  {
		counter++;
		if (*acc[2] > 30 || *acc[2] < -30 )
		{
			Error_Handler();
		}
		
		//forward and backward
	 if (Rc_command->rc.ch[1]>10)
		{
			body_1_p_r.x=x_offset+(float)Rc_command->rc.ch[1]/60000; 
			body_2_p_r.x=x_offset+(float)Rc_command->rc.ch[1]/60000;
		}
		else if (Rc_command->rc.ch[1]<-10)
		{
			body_1_p_r.x=x_offset+(float)Rc_command->rc.ch[1]/60000;
			body_2_p_r.x=x_offset+(float)Rc_command->rc.ch[1]/60000;
	 }
		
		//tilt left and right
	 	 if (Rc_command->rc.ch[2]>10)
		{
			left_right=(float)Rc_command->rc.ch[2]/20000;
		}
		else if (Rc_command->rc.ch[2]<-10)
		{
			left_right=(float)Rc_command->rc.ch[2]/20000;
	  }
		
		//turn left or right
		if (Rc_command->rc.ch[0]>15)
		{
			yaw=yaw-(float)Rc_command->rc.ch[0]/8000000;
		}
		else if (Rc_command->rc.ch[0]<-15)
		{
			yaw=yaw-(float)Rc_command->rc.ch[0]/8000000;
	  }
    
		//body up and down
		 if (Rc_command->rc.ch[3]>30)
		{
			body_1_p.y=body_1_p.y+0.0000001*Rc_command->rc.ch[3];//0.000001667
			body_2_p.y=body_2_p.y+0.0000001*Rc_command->rc.ch[2];
		}
		else if (Rc_command->rc.ch[3]<-30)
		{
			body_1_p.y=body_1_p.y+0.0000001*Rc_command->rc.ch[3];
			body_2_p.y=body_2_p.y+0.0000001*Rc_command->rc.ch[2];
	  }
		
		if (yaw>=3.14)
		{
		  yaw=-3.14;
		}
		else if (yaw<-3.14)
		{
			yaw=3.14;
		}
		
    //calculate inverkinematic for joint angel
		joint_1_q=invkin(body_1_p_r.x, body_1_p.y+left_right);
		joint_2_q=invkin_2(body_2_p_r.x,body_1_p.y-left_right);

		 if (rpy_t.yaw-yaw>1.4)
		 {
			 yaw=rpy_t.yaw+0.001;
		 }
		 else if (rpy_t.yaw-yaw<-1.4)
		 {
			 yaw=rpy_t.yaw-0.001;
		 }
		 
     //calculate current command for all six motors
	   Icommand[0]=PID(joint_1_q.q1,report1.Position,report10.Position-report1.Position);
     Icommand[1]=PID(joint_1_q.q2,report2.Position,report20.Position-report2.Position);
		 Icommand[2]=-(Kp_pitch*(pitch_ctrl-pitch_des)-Kd_pitch*(pitch_ctrl0-pitch_ctrl)-4*(yaw_ctrl-yaw));
		 Icommand_2[0]=PID(joint_2_q.q1,report4.Position,report40.Position-report4.Position);
     Icommand_2[1]=PID(joint_2_q.q2,report5.Position,report50.Position-report5.Position);
		 Icommand_2[2]=(Kp_pitch*(pitch_ctrl-pitch_des)-Kd_pitch*(pitch_ctrl0-pitch_ctrl)+4*(yaw_ctrl-yaw));
		Motor_Drive(Icommand);
		Motor_Drive_2(Icommand_2);
		
	 	report10=report1;
		report20=report2;
		report30=report3;
		report40=report4;
		report50=report5;
		report60=report6;
		rpy_t0=rpy_t;
		ch[3]=yaw;
		
//		for (int n=0;n<(update_period-1);n++)
//	  {
//		pitch_recent[n]=pitch_recent[n+1];
//		yaw_recent[n]=yaw_recent[n+1];
//	  }
//		pitch_recent[update_period-1]=rpy_t.pitch;
//		yaw_recent[update_period-1]=rpy_t.yaw;
		
		
		if (counter>update_period)
		{
//			sum_pitch=0;
//			sum_yaw=0;
//			for (int n=0;n<update_period;n++)
//			{
//				sum_pitch=sum_pitch+pitch_recent[n];
//				sum_yaw=sum_yaw+yaw_recent[n];		
//			}
//			pitch_ctrl0=pitch_ctrl;
//			pitch_ctrl=sum_pitch/(float)update_period;
//			yaw_ctrl0=yaw_ctrl;
//			yaw_ctrl=sum_yaw/(float)update_period;
			pitch_ctrl0=pitch_ctrl;
			pitch_ctrl=rpy_t.pitch;
			yaw_ctrl0=yaw_ctrl;
			yaw_ctrl=rpy_t.yaw;
			counter=0;
			position_robot_old=position_robot;
		
		//get robot position by wheel encoder 
		if (fabs(report30.Position-report3.Position)>1.5)
		{
		position_left=0;
		}
		else 
		{
			position_left=report3.Position-report30.Position;
		}
		
		if (fabs(report60.Position-report6.Position)>1.5)
		{
		position_right=0;
		}
		else 
		{
			position_right=-(report6.Position-report60.Position);
		}
		
		position_robot=position_robot+wheel_radius*(position_left+position_right)/2;
		}
		
		delay_us(100);
	  
		//for vofa+
		ch[0]=rpy_t.pitch;
	  ch[1]=rpy_t.roll;
	  ch[2]=rpy_t.yaw;
//	  ch[3]=yaw;
//		ch[0]=report1.Position;
//	  ch[1]=report2.Position;
//	  ch[2]=rpy_t.yaw;
//	  ch[3]=yaw;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void uint8touint32(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof32)
{
	for(int i = 0; i < lengthof32; i++)
	{
		vector_32[i] = (vector_8[4 * i + 3] << 24) + (vector_8[4 * i + 2] << 16) + (vector_8[4 * i + 1] << 8) + vector_8[4 * i];
	}
}

uint32_t checksum_t;
uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
//for vofa+
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)	
{
	if(huart == &huart8)
	{
		if(tail_flag)
		{
			tail_flag = 0;
			HAL_UART_Transmit_DMA(&huart8,tail,4);
		}
		else if (!tail_flag)
		{
			//HAL_UART_Transmit_DMA(&huart8,RxData_1_2,8);
			HAL_UART_Transmit_DMA(&huart8,(uint8_t*)ch,16);
			tail_flag = 1;
		}
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8touint32((uint32_t*)(&imu), (uint8_t*)IMU_data, 11);
  checksum_t = data_checksum((uint32_t*)(&imu),10);
	if(checksum_t == imu.check)
	{
    control_motor_flag = 1;
	}
	else{
		control_motor_flag = 0;
		//reset imu;
	}
	HAL_UART_Receive_DMA(&huart6, IMU_data, 44);
	rpy_t=q2rpy(*q[0],*q[1],*q[2],*q[3]);
	ch[0]=rpy_t.pitch;
	ch[1]=rpy_t.roll;
	ch[2]=rpy_t.yaw;
	
//	HAL_UART_Receive_DMA(&huart8, Kp_Kd, 8);
//	if (Kp_Kd[3]==',' && Kp_Kd[4]==' ')
//	{
//		for (int n=0;n<3;n++)
//		{
//		Kp_receive[n]=Kp_Kd[n];
//		Kd_receive[n]=Kp_Kd[n+5];
//		}
//		if (fabs(strtof(Kp_receive,&endptr))<50 && fabs(strtof(Kd_receive,&endptr))<1500)
//		{
//		Kp_pre=strtof(Kp_receive,&endptr);
//		Kd_pre=strtof(Kd_receive,&endptr);
//		}
//	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get RX message */
	if(hcan1.Instance==CAN1)
	{
		//RxHeader.StdId=0x203;
		
		HAL_CAN_GetRxMessage(CanHandle,CAN_FILTER_FIFO0,&RxHeader_1,RxData_1);
		data_process();
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get RX message */
		if(hcan2.Instance==CAN2)
	{
		//RxHeader.StdId=0x203;
		HAL_CAN_GetRxMessage(CanHandle,CAN_FILTER_FIFO1,&RxHeader_2,RxData_2);
		data_process_2();
	}
}

void data_process(void)
{
	if (RxHeader_1.StdId==0x201)
	{  
		memcpy(RxData_1_1,RxData_1,8);
		report1=Feedback(RxData_1_1);
		
	}
	else if (RxHeader_1.StdId==0x202)
	{
		memcpy(RxData_1_2,RxData_1,8);
		report2=Feedback(RxData_1_2);
	}
	else if (RxHeader_1.StdId==0x203)
	{
		memcpy(RxData_1_3,RxData_1,8);
		report3=Feedback(RxData_1_3);
	}
	
	ifok = ifok | (0x01 << (RxHeader_1.StdId-0x201));
}

void data_process_2(void)
{
	if (RxHeader_2.StdId==0x201)
	{  
		memcpy(RxData_2_1,RxData_2,8);
		report4=Feedback(RxData_2_1);
	}
	else if (RxHeader_2.StdId==0x202)
	{
		memcpy(RxData_2_2,RxData_2,8);
		report5=Feedback(RxData_2_2);
	}
	else if (RxHeader_2.StdId==0x203)
	{
		memcpy(RxData_2_3,RxData_2,8);
		report6=Feedback(RxData_2_3);
	}
	
	ifok = ifok | (0x01 << ((RxHeader_2.StdId-0x201) + 3));
}

void Encoder_Check(void)
{
//	while (fabs(report1.Position)<0.01 || fabs(report2.Position)<0.01 || fabs(report3.Position)<0.01 || fabs(report4.Position)<0.01 || fabs(report5.Position)<0.01 || fabs(report6.Position)<0.01)
//	{
//		//check_en=check_en+0.00000001;
//	}
	while(ifok != 0x3f)
	{
	}
}

uint32_t data_checksum(uint32_t* data_to_check, uint32_t check_length)
{
	uint32_t t = 0;
	for(int i = 0; i < check_length; i++)
	{
		t = t ^ data_to_check[i];
	}
	
	return t;
}

void IMU_check(void)
{
	while (fabs(*q[1])<0.001)
	{
		IMU_check_cont=IMU_check_cont+0.0000000000001;
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
		
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
