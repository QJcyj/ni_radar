/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stmflash.h"
#include "delay.h"
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//?д??STM32 FLASH???????????
//3米 0x01,0x2C  1.5米 0x00,0x96,0.9米  0x00,0x5A, 
//0.6米0x00,0x3C, 0.3米0x00,0x1E,
uint8_t param[28]={
	0x01,0x2C, 0x00,0x5A,  0x00,0x1E,
	0x00,0x96, 0x00,0x5A,  0x00,0x3C,
	0x00,0x1E, 0x00,0x3C,  0x00,0x5A,
	0x00,0x1E, 0x00,0x96,  0x00,0x5A,
	0x00,0x3C, 0x00,0x1E,
};

//#define Con1A_12		((float)(param[0]*256+param[1])/100)
//#define Con1A_3_max		((float)(param[2]*256+param[3])/100)
//#define Con1A_3_min		((float)(param[4]*256+param[5])/100)
//#define Con1B_12		((float)(param[6]*256+param[7])/100)
//#define Con1B_3			((float)(param[8]*256+param[9])/100)
//#define Con1B_4			((float)(param[10]*256+param[11])/100)
//#define Con1B_5			((float)(param[12]*256+param[13])/100)

//#define Con2A_12		((float)(param[14]*256+param[15])/100)
//#define Con2A_3_max		((float)(param[16]*256+param[17])/100)
//#define Con2A_3_min		((float)(param[18]*256+param[19])/100)
//#define Con2B_12		((float)(param[20]*256+param[21])/100)
//#define Con2B_3			((float)(param[22]*256+param[23])/100)
//#define Con2B_4			((float)(param[24]*256+param[25])/100)
//#define Con2B_5			((float)(param[26]*256+param[27])/100)


float Con1A_12;		
float Con1A_3_max;		
float Con1A_3_min;		
float Con1B_12;	
float Con1B_3;		
float Con1B_4;		
float Con1B_5;	

float Con2A_12	;	
float Con2A_3_max;	
float Con2A_3_min;	
float Con2B_12;	
float Con2B_3;	
float Con2B_4;	
float Con2B_5;	

uint32_t NOW1_R_A_2 = 0;
uint32_t NOW1_R_B_2 = 0;
uint32_t NOW1_R_B_3 = 0;
uint32_t NOW1_R_B_4 = 0;

uint32_t NOW2_R_A_2 = 0;
uint32_t NOW2_R_B_2 = 0;
uint32_t NOW2_R_B_3 = 0;
uint32_t NOW2_R_B_4 = 0;

uint32_t NOW3_R = 0;

uint8_t CMD_Buf[5]={0,0,0,0,0};
#define Cmd_size	5

#define SIZE sizeof(param)		//???鳤??


uint8_t	Radar1_Rec_Buf[27];
#define Radar1_size	27

uint8_t	Radar2_Rec_Buf[27];
#define Radar2_size	27

#define Control_Mode 	PBin(0)
#define LED_Mode   		PBin(15)
#define LED_1 	   		PAout(15)
#define LED_2      		PBout(3)
#define LED_3      		PAout(11)
#define ExCon1			PBout(4)
#define ExCon2			PBout(5)

#define One_Ctrl_Mode	PBin(2)	//  PIN 20
													//户外
#define Out_Ger_led		PAout(15)	// PIN 38   	 绿灯
#define Out_Yellow_led	PBout(4)	// PIN 40		 琥珀
#define Out_Red_led		PBout(3)	// PIN 39		 红灯
															//户内
#define In_Red_led		PBout(14)	// PIN 27		 红灯
#define In_Ger_led		PBout(13)	// PIN 26		 绿灯

#define _100ms			1U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
bool Radar_Flag = false;
enum  Radar
{
	FIX1,
	FIX2,
	DIS_L,
	DIS_H,
	STR_1,
	STR_2,
	MODE,
	SAVE,
	ChceckSum
};
enum Radar state1 = FIX1;
enum Radar state2 = FIX1;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;
uint32_t time_100ms = 0;

bool Radar1_ok = false;
bool Radar2_ok = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void RadarB_hand(void);
void RadarA_hand(void);
void One_Radar_hand(void);
void Radar_Data_Set(uint8_t *cmd);
void ParamLoad(void);
void get_time(uint32_t *time);
uint32_t hrt_time(uint32_t time);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  delay_init(72);
  delay_ms(500);
  HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_DMA(&huart1,CMD_Buf,Cmd_size);
	HAL_UART_Receive_DMA(&huart2,Radar1_Rec_Buf,Radar1_size);
	HAL_UART_Receive_DMA(&huart3,Radar2_Rec_Buf,Radar2_size);
//	uint8_t start = 0;
//	start = STMFLASH_ReadHalfWord(FLASH_SAVE_ADDR + LOAD_PARAM);
//	printf("start=%d\n",start);
//	if(start==0xFF){
//		STMFLASH_WriteHalfWord(FLASH_SAVE_ADDR + LOAD_PARAM,(uint16_t)(0xAA));
//		STMFLASH_Write(FLASH_SAVE_ADDR,(uint16_t*)param,SIZE);
//		ParamLoad();
//	}
//	
//	Con1A_12	=	((float)(param[0]*256+param[1])/100);
//	Con1A_3_max	=	((float)(param[2]*256+param[3])/100);
//	Con1A_3_min	=	((float)(param[4]*256+param[5])/100);
//	Con1B_12	=	((float)(param[6]*256+param[7])/100);
//	Con1B_3		=	((float)(param[8]*256+param[9])/100);
//	Con1B_4		=	((float)(param[10]*256+param[11])/100);
//	Con1B_5		=	((float)(param[12]*256+param[13])/100);

//	Con2A_12	=	((float)(param[14]*256+param[15])/100);
//	Con2A_3_max =((float)(param[16]*256+param[17])/100);
//	Con2A_3_min	=	((float)(param[18]*256+param[19])/100);
//	Con2B_12	=	((float)(param[20]*256+param[21])/100);
//	Con2B_3		=	((float)(param[22]*256+param[23])/100);
//	Con2B_4		=	((float)(param[24]*256+param[25])/100);
//	Con2B_5		=	((float)(param[26]*256+param[27])/100);
	
//	STMFLASH_Read(FLASH_SAVE_ADDR,(uint16_t*)datatemp,SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	if(!Radar_Flag){
//		RadarA_hand();
//	}
//	else{
//		RadarB_hand();
//	}
	 One_Radar_hand();
	 delay_ms(40); 
//	if((time_100ms%20) == 0)
//	{
//		STMFLASH_Read(FLASH_SAVE_ADDR,(uint16_t*)param,SIZE);
//		Con1A_12	=	((float)(param[0]*256+param[1])/100);
//		Con1A_3_max	=	((float)(param[2]*256+param[3])/100);
//		Con1A_3_min	=	((float)(param[4]*256+param[5])/100);
//		Con1B_12	=	((float)(param[6]*256+param[7])/100);
//		Con1B_3		=	((float)(param[8]*256+param[9])/100);
//		Con1B_4		=	((float)(param[10]*256+param[11])/100);
//		Con1B_5		=	((float)(param[12]*256+param[13])/100);

//		Con2A_12	=	((float)(param[14]*256+param[15])/100);
//		Con2A_3_max =	((float)(param[16]*256+param[17])/100);
//		Con2A_3_min	=	((float)(param[18]*256+param[19])/100);
//		Con2B_12	=	((float)(param[20]*256+param[21])/100);
//		Con2B_3		=	((float)(param[22]*256+param[23])/100);
//		Con2B_4		=	((float)(param[24]*256+param[25])/100);
//		Con2B_5		=	((float)(param[26]*256+param[27])/100);
//	}

//	if((time_100ms%3) == 0)
//	{
////		HAL_UART_Transmit_DMA(&huart1, param, SIZE);
//	}
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	if(huart == &huart2){
//		Radar_Flag = true;
//	}
//	if(huart == &huart3){

//	}
//	if(huart == &huart1){
////		HAL_UART_Transmit_DMA(&huart1, CMD_Buf, 5);
//		Radar_Data_Set(CMD_Buf);
//	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance)
	{
		time_100ms ++;
		if(time_100ms >= 0xFFFFFFFE){
			time_100ms = 0;
			NOW1_R_A_2 = 0;
			NOW1_R_B_2 = 0;
			NOW1_R_B_3 = 0;
			NOW1_R_B_4 = 0;

			NOW2_R_A_2 = 0;
			NOW2_R_B_2 = 0;
			NOW2_R_B_3 = 0;
			NOW2_R_B_4 = 0;

			NOW3_R = 0;
		}
	}

}

uint8_t check_sum(uint8_t *data)
{
	uint8_t sum = 0;
	int crc_sum = 0;
	for(uint8_t i=0;i<8;i++){
//		printf(" %02x",data[i]);
		crc_sum += data[i];
	}
	sum = crc_sum&0xff;
	return sum;
}
void RadarB_hand(void)
{
	uint8_t c;
	uint8_t buffer[9];
	Radar2_ok = false;
	for(uint8_t i=0;i<9;i++)
	{
		c = state2;
//		printf("in to radar2\n");
//		printf(" %02x",Radar1_Rec_Buf[i]);
		switch(c)
		{
			case FIX1:
				if(Radar1_Rec_Buf[i] == 0x59){
					state2 = FIX2;
					buffer[0] = Radar1_Rec_Buf[i];
				}
				else{
					state2 = FIX1;
				}
				break;
			case FIX2:
				if(Radar1_Rec_Buf[i] == 0x59){
					buffer[1] = Radar1_Rec_Buf[i];
					state2 = DIS_L;
				}
				else{
					state2 = FIX1;
				}
				break;
			case DIS_L:
				buffer[2] = Radar1_Rec_Buf[i];
				state2 = DIS_H;
				break;
				
			case DIS_H:
				buffer[3] = Radar1_Rec_Buf[i];
				state2 = STR_1;
				break;
			case STR_1:
				buffer[4] = Radar1_Rec_Buf[i];
				state2 = STR_2;
				break;
			case STR_2:
				buffer[5] = Radar1_Rec_Buf[i];
				state2 = MODE;
				break;
			case MODE:
				buffer[6] = Radar1_Rec_Buf[i];
				state2 = SAVE;
				break;
			case SAVE:
				buffer[7] = Radar1_Rec_Buf[i];
				state2 = ChceckSum;
				break;
			case ChceckSum:
//				printf("buf=%02x crc=%02x\n",Radar1_Rec_Buf[i],check_sum(buffer));
				if( Radar1_Rec_Buf[i] == check_sum(buffer)){
					Radar2_ok = true;
//					printf("sum ok\n");
				}
				break;
			default:
				break;
		}
//		if(Radar2_ok ){
//			state2 = FIX1;
//			break;
//		}
	}
	state2 = FIX1;
	
//control A
//	printf("Radar_Mode=%d 2_ok=%d\n",(uint16_t)Radar_Mode ,Radar2_ok );
	if(Radar2_ok){
		Radar2_ok = false;
		float distance = 0;
		distance = (float)(buffer[2] + buffer[3]*256)/100;
//		printf("distance B=%f \n",distance);
		if(LED_Mode){
//			printf("mode A \n\n");
			if((distance>(float)Con2A_3_min 
				&& distance<=(float)Con2A_3_max) 
				|| (distance<=(float)Con2A_12)){
				if((float)Con2A_3_max < (float)Con2A_12){
					LED_1 = 1; LED_2 = 1; LED_3 = 0;
//					printf("distance0.3 - 0.9\n\n");
				}else{
					if((hrt_time(NOW2_R_A_2) > _100ms*10) && (distance<=(float)Con2A_12)){
						get_time(&NOW2_R_A_2);
						LED_1 ^= 1; LED_2 ^= 1; LED_3 = 0;
//						printf("distance<0.6 time=%d\n\n",time_100ms);
					}
					else if(distance <= (float)Con2A_3_max){
						LED_1 = 1; LED_2 = 1; LED_3 = 0;
//						printf("distance<0.9\n\n");
					}
				}
			}
			else if(distance > (float)Con2A_12){
				LED_1 = 0; LED_2 = 0; LED_3 = 1;
//				printf("distance>0.6\n\n");
			}				
		}
		else{ //control B
//			printf("mode B \n\n");
			
			if(distance <= (float)Con2B_12){
//				printf("distance B=%f \n",distance);
				if(distance <= (float)Con2B_5){
					LED_1 = 0; LED_2 = 1; LED_3 = 0;
//					printf("distance<0.3\n\n");
				}
				else if(distance <= (float)Con2B_4){
					if(hrt_time(NOW2_R_B_4) > _100ms*2){
						get_time(&NOW2_R_B_4);
						LED_1 ^= 1; LED_2 = 0; LED_3 = 0;
//					printf("distance<0.6\n\n");
					}
				}
				else if(distance <= (float)Con2B_3){
					if(hrt_time(NOW2_R_B_3) > _100ms*5){
						get_time(&NOW2_R_B_3);
						LED_1 ^= 1; LED_2 = 0; LED_3 = 0;
//					printf("distance<0.9\n\n");
					}
				}
				else if(distance<= (float)Con2B_12){
					if(hrt_time(NOW2_R_B_2) > _100ms*10){
						get_time(&NOW2_R_B_2);
						LED_1 ^= 1; LED_2 = 0; LED_3 = 0;
//						printf("distance<1.5\n\n");
					}
				}
			}
			else{
				LED_1=0; LED_2=0; LED_3=1;
//				printf("distance>1.5\n\n");
			}
		}
	}
}



void RadarA_hand(void)
{
	uint8_t c;
	uint8_t buffer[9];
	Radar1_ok = false;
//	printf("in radar1 fun\n");
	for(uint8_t i=0;i<9;i++)
	{
//		printf(" %02x\n",Radar1_Rec_Buf[i]);
		c = state1;
		switch(c)
		{
			case FIX1:
				if(Radar2_Rec_Buf[i] == 0x59){
					state1 = FIX2;
					buffer[0] = Radar2_Rec_Buf[i];
				}
				else{
					state1 = FIX1;
				}
				break;
			case FIX2:
				if(Radar2_Rec_Buf[i] == 0x59){
					buffer[1] = Radar2_Rec_Buf[i];
					state1 = DIS_L;
				}
				else{
					state1 = FIX1;
				}
				break;
			case DIS_L:
				buffer[2] = Radar2_Rec_Buf[i];
				state1 = DIS_H;
				break;
				
			case DIS_H:
				buffer[3] = Radar2_Rec_Buf[i];
				state1 = STR_1;
				break;
			case STR_1:
				buffer[4] = Radar2_Rec_Buf[i];
				state1 = STR_2;
				break;
			case STR_2:
				buffer[5] = Radar2_Rec_Buf[i];
				state1 = MODE;
				break;
			case MODE:
				buffer[6] = Radar2_Rec_Buf[i];
				state1 = SAVE;
				break;
			case SAVE:
				buffer[7] = Radar2_Rec_Buf[i];
				state1 = ChceckSum;
				break;
			case ChceckSum:
//				printf("buf=%02x crc=%02x\n",Radar2_Rec_Buf[i],check_sum(buffer));
				if( Radar2_Rec_Buf[i] == check_sum(buffer)){
					Radar1_ok = true;
//					printf("Radar1_ok=%d\n",Radar1_ok);
				}
//				printf("Radar1 sum ok\n");
				break;
			default:
				break;
		}
	}
	state1 = FIX1;
	
//control A
	if(Radar1_ok==true){
//		printf("dis jisuan \n");
		float distance = 0;
		Radar1_ok = false;
		distance = (float)(buffer[2] + buffer[3]*256)/100;
//		printf("distance A=%f \n",distance);
		if(Control_Mode && LED_Mode){
			if(distance <= 0.6f){
				LED_1=1; LED_2=1; LED_3=0; ExCon1=0; ExCon2=1;
			}
			else if(distance <= 3.0f){
				if(hrt_time(NOW3_R) > _100ms*10){
					get_time(&NOW3_R);
					LED_1 ^= 1; LED_2 ^= 1; LED_3=0; ExCon1=1; ExCon2=0;
				}
			}
			else if(distance > 3.0f){
				LED_1 = 0; LED_2 = 0; LED_3=1; ExCon1=0; ExCon2=0;
			}
		}
		else{
			if(LED_Mode){
				if(((distance > (float)Con1A_3_min) 
					&& (distance<=(float)Con1A_3_max)) 
					|| (distance <= (float)Con1A_12)){
					if( distance <= (float)Con1A_3_max){
						LED_1 = 1; LED_2 = 1; LED_3 = 0; ExCon1=0; ExCon2=1;
					}
					else{
						if((hrt_time(NOW1_R_A_2) > _100ms*10) && (distance<=(float)Con1A_3_min)){
							get_time(&NOW1_R_A_2);
							LED_1 ^= 1; LED_2 ^= 1; LED_3 = 0; ExCon1=1; ExCon2=0;
						}
					}
				}
				else if(distance > (float)Con1A_12){
					LED_1 = 0; LED_2 = 0; LED_3 = 1; ExCon1=0; ExCon2=0;
				}					
			}
			else{ //control B
//				printf("MODE B  %3.2f\n",Con1B_12);
				if(distance <= (float)Con1B_12){
					if(distance <= (float)Con1B_5){
						LED_1 = 0; LED_2 = 1; LED_3 = 0;
//						printf("distance <0.3\n\n");
					}
					else if(distance < (float)Con1B_4){
						if(hrt_time(NOW1_R_B_4) > _100ms*2){
							get_time(&NOW1_R_B_4);
							LED_1 ^= 1; LED_2 = 0; LED_3 = 0;
//							printf("distance <0.6\n\n");
						}
					}
					else if(distance <= (float)Con1B_3){
						if(hrt_time(NOW1_R_B_3) > _100ms*5){
							get_time(&NOW1_R_B_3);
							LED_1 ^= 1; LED_2 = 0; LED_3 = 0;
//							printf("distance <0.9\n\n");
						}
					}
					else if(distance <= (float)Con1B_12){
						if(hrt_time(NOW1_R_B_2) > _100ms*10){
							get_time(&NOW1_R_B_2);
							LED_1 ^= 1; LED_2 = 0; LED_3 = 0;
//							printf("time_100ms=%d\n",time_100ms);
//							printf("distance < 1.5\n\n");
						}
					}
				}
				else{
//					printf("distance > 1.5\n\n");
					LED_1=0; LED_2=0; LED_3=1;
				}
			}
		}
	}
	
}


void One_Radar_hand(void)
{
	uint8_t c;
	uint8_t buffer[9];
	Radar1_ok = false;
	for(uint8_t i=0;i<9;i++)
	{
		c = state1;
		switch(c)
		{
			case FIX1:
				if(Radar2_Rec_Buf[i] == 0x59){
					state1 = FIX2;
					buffer[0] = Radar2_Rec_Buf[i];
				}
				else{
					state1 = FIX1;
				}
				break;
			case FIX2:
				if(Radar2_Rec_Buf[i] == 0x59){
					buffer[1] = Radar2_Rec_Buf[i];
					state1 = DIS_L;
				}
				else{
					state1 = FIX1;
				}
				break;
			case DIS_L:
				buffer[2] = Radar2_Rec_Buf[i];
				state1 = DIS_H;
				break;
				
			case DIS_H:
				buffer[3] = Radar2_Rec_Buf[i];
				state1 = STR_1;
				break;
			case STR_1:
				buffer[4] = Radar2_Rec_Buf[i];
				state1 = STR_2;
				break;
			case STR_2:
				buffer[5] = Radar2_Rec_Buf[i];
				state1 = MODE;
				break;
			case MODE:
				buffer[6] = Radar2_Rec_Buf[i];
				state1 = SAVE;
				break;
			case SAVE:
				buffer[7] = Radar2_Rec_Buf[i];
				state1 = ChceckSum;
				break;
			case ChceckSum:
//				printf("buf=%02x crc=%02x\n",Radar2_Rec_Buf[i],check_sum(buffer));
				if( Radar2_Rec_Buf[i] == check_sum(buffer)){
					Radar1_ok = true;
//					printf("Radar1_ok=%d\n",Radar1_ok);
				}
//				printf("Radar1 sum ok\n");
				break;
			default:
				break;
		}
	}
	state1 = FIX1;
	
	if(Radar1_ok==false){
		if(One_Ctrl_Mode){
			Out_Ger_led=1; Out_Yellow_led=0; Out_Red_led=0; In_Red_led=1; In_Ger_led=0;
		}
		else{
			Out_Ger_led=0; Out_Yellow_led=0; Out_Red_led=1; In_Red_led=0; In_Ger_led=1;
		}
	}
	if(Radar1_ok==true){
		float distance = 0;
		Radar1_ok = false;
		distance = (float)(buffer[2] + buffer[3]*256)/100;
//		printf("distance A=%f \n",distance);
		if(One_Ctrl_Mode){
			if(distance <= 0.31f){
				Out_Ger_led=0; Out_Yellow_led=1; Out_Red_led=0; In_Red_led=1; In_Ger_led=0;
			}
			else if((distance > 0.31f) && (distance < 0.9f)){
				if(hrt_time(NOW3_R) > _100ms*6){
					get_time(&NOW3_R);
					Out_Ger_led=0; Out_Yellow_led^=1; Out_Red_led=0; In_Red_led=1; In_Ger_led=0;
				}
			}
		}
		else{
			Out_Ger_led=0; Out_Yellow_led=0; Out_Red_led=1; In_Red_led=0; In_Ger_led=1;
		}
	}

}
void Radar_Data_Set(uint8_t *cmd)
{
	if((cmd[0]==0xA5) && (cmd[1]==0x5A)){
//		printf("cmd=%02x\n",cmd[2]);
		switch(cmd[2])
		{	
			case RADAR1_A_STEP12_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_A_STEP12_DIS,(uint16_t*)(&cmd[3]),1);
				param[0]=cmd[3]; param[1]=cmd[4];
				break;
			case RADAR1_A_STEP3_BIG:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_A_STEP3_BIG,(uint16_t*)(&cmd[3]),1);
				param[2]=cmd[3]; param[3]=cmd[4];
				break;
			case RADAR1_A_STEP3_MIN:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_A_STEP3_MIN,(uint16_t*)(&cmd[3]),1);
				param[4]=cmd[3]; param[5]=cmd[4];
				break;
			case RADAR1_B_STEP12_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_B_STEP12_DIS,(uint16_t*)(&cmd[3]),2);
				param[6]=cmd[3]; param[7]=cmd[4];
				break;
			case RADAR1_B_STEP3_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_B_STEP3_DIS,(uint16_t*)(&cmd[3]),2);
				param[8]=cmd[3]; param[9]=cmd[4];
				break;
			case RADAR1_B_STEP4_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_B_STEP4_DIS,(uint16_t*)(&cmd[3]),2);
				param[10]=cmd[3]; param[11]=cmd[4];
				break;
			case RADAR1_B_STEP5_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_B_STEP5_DIS,(uint16_t*)(&cmd[3]),2);
				param[12]=cmd[3]; param[13]=cmd[4];
				break;
			case RADAR2_A_STEP12_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_A_STEP12_DIS,(uint16_t*)(&cmd[3]),2);
				param[14]=cmd[3]; param[15]=cmd[4];
				break;
			case RADAR2_A_STEP3_BIG:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_A_STEP3_BIG,(uint16_t*)(&cmd[3]),2);
				param[16]=cmd[3]; param[17]=cmd[4];
				break;
			case RADAR2_A_STEP3_MIN:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_A_STEP3_MIN,(uint16_t*)(&cmd[3]),2);
				param[18]=cmd[3]; param[19]=cmd[4];
				break;
			case RADAR2_B_STEP12_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_B_STEP12_DIS,(uint16_t*)(&cmd[3]),2);
				param[20]=cmd[3]; param[21]=cmd[4];
				break;
			case RADAR2_B_STEP3_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_B_STEP3_DIS,(uint16_t*)(&cmd[3]),2);
				param[22]=cmd[3]; param[23]=cmd[4];
				break;
			case RADAR2_B_STEP4_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_B_STEP4_DIS,(uint16_t*)(&cmd[3]),2);
				param[24]=cmd[3]; param[25]=cmd[4];
				break;
			case RADAR2_B_STEP5_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_B_STEP5_DIS,(uint16_t*)(&cmd[3]),2);
				param[26]=cmd[3]; param[27]=cmd[4];
				break;
			default:
				break;
		}				
	}
}

void ParamLoad(void)
{
	STMFLASH_Read(FLASH_SAVE_ADDR,(uint16_t*)param,SIZE);
	
}


void get_time(uint32_t *time)
{
	*time = time_100ms;
}

uint32_t hrt_time(uint32_t time)
{
	return (time_100ms - time);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
