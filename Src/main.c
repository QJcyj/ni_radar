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

//?§Õ??STM32 FLASH???????????

uint8_t param[28]={0};

uint8_t CMD_Buf[5]={0,0,0,0,0};
#define Cmd_size	5

#define SIZE sizeof(param)		//???øA??


uint8_t	Radar1_Rec_Buf[27];
#define Radar1_size	27

uint8_t	Radar2_Rec_Buf[27];
#define Radar2_size	27

#define Radar_Mode PBin(0)
#define LED_Mode   PBin(15)
#define LED_1 	   PAout(15)
#define LED_2      PBout(3)
#define LED_3      PBout(4)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
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
uint16_t time_10ms = 0;

bool Radar1_ok = false;
bool Radar2_ok = false;

bool Radar_siwch = false;
bool LED_control = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Radar1_hand(void);
void Radar2_hand(void);
void Radar_Data_Set(uint8_t *cmd);
void ParamLoad(void);
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
	HAL_UART_Receive_DMA(&huart1,CMD_Buf,Cmd_size);
	HAL_UART_Receive_DMA(&huart2,Radar1_Rec_Buf,Radar1_size);
	HAL_UART_Receive_DMA(&huart3,Radar2_Rec_Buf,Radar2_size);
	ParamLoad();
//	STMFLASH_Read(FLASH_SAVE_ADDR,(uint16_t*)datatemp,SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  printf("RA1:%s\n",Radar1_Rec_Buf);
	  
//	  LED_1 = 0;
//	  LED_2 = 0;
//	  printf("LED_1=%d LED_2=%d \n",(uint16_t)LED_1,(uint16_t)LED_2);
	  Radar1_hand();
	  Radar2_hand();
//	  	STMFLASH_Read(FLASH_SAVE_ADDR,(uint16_t*)datatemp,SIZE);
//		HAL_UART_Transmit_DMA(&huart1, param, 28);

//	  delay_ms(1000);
//	  LED_1 = 1;
//	  LED_2 = 1;
//	  printf("LED_1=%d LED_2=%d \n",(uint16_t)LED_1,(uint16_t)LED_2);
//	  delay_ms(100);
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
	if(huart == &huart2){
//		HAL_UART_Transmit_DMA(&huart1, Radar1_Rec_Buf, 9);
//		Radar_Data_Set(CMD_Buf);
//		Radar1_hand();
	}
	if(huart == &huart3){
//		printf("Radar2 data \n");
//		printf("Radar2 %s \n",Radar2_Rec_Buf);
	}
	if(huart == &huart1){
		HAL_UART_Transmit_DMA(&huart1, CMD_Buf, 5);
		Radar_Data_Set(CMD_Buf);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance)
	{
		time_10ms ++;
		if(time_10ms >= 60000){
			time_10ms = 0;
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
void Radar2_hand(void)
{
	uint8_t c;
	uint8_t buffer[9];
	Radar2_ok = false;
//	printf("in radar2 fun\n");
	for(uint8_t i=0;i<9;i++)
	{
		c = state2;
//		printf("in to radar2\n");
//		printf(" %02x",Radar2_Rec_Buf[i]);
		switch(c)
		{
			case FIX1:
				if(Radar2_Rec_Buf[i] == 0x59){
					state2 = FIX2;
					buffer[0] = Radar2_Rec_Buf[i];
				}
				else{
					state2 = FIX1;
				}
				break;
			case FIX2:
				if(Radar2_Rec_Buf[i] == 0x59){
					buffer[1] = Radar2_Rec_Buf[i];
					state2 = DIS_L;
				}
				else{
					state2 = FIX1;
				}
				break;
			case DIS_L:
				buffer[2] = Radar2_Rec_Buf[i];
				state2 = DIS_H;
				break;
				
			case DIS_H:
				buffer[3] = Radar2_Rec_Buf[i];
				state2 = STR_1;
				break;
			case STR_1:
				buffer[4] = Radar2_Rec_Buf[i];
				state2 = STR_2;
				break;
			case STR_2:
				buffer[5] = Radar2_Rec_Buf[i];
				state2 = MODE;
				break;
			case MODE:
				buffer[6] = Radar2_Rec_Buf[i];
				state2 = SAVE;
				break;
			case SAVE:
				buffer[7] = Radar2_Rec_Buf[i];
				state2 = ChceckSum;
				break;
			case ChceckSum:
				printf("buf=%02x crc=%02x\n",Radar2_Rec_Buf[i],check_sum(buffer));
				if( Radar2_Rec_Buf[i] == check_sum(buffer)){
					Radar2_ok = true;
					printf("sum ok\n");
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
	printf("Radar_Mode=%d 2_ok=%d\n",(uint16_t)Radar_Mode ,Radar2_ok );
	if(Radar2_ok && (Radar_Mode == 0)){
		Radar2_ok = false;
		float distance = 0;
		distance = (float)(buffer[2] + buffer[3]*256)/100;
		printf("distance2=%f \n",distance);
		if(LED_Mode){
			if((time_10ms%200==0) && (distance<0.6)){
				LED_1 ^= 1;
				LED_2 ^= 1;
				LED_3 ^= 0;
			}
			else if(distance>0.3 && distance<0.9){
				LED_1 = 1;
				LED_2 = 1;
				LED_3 = 0;
			}
			if(distance>0.6){
				LED_1 = 0;
				LED_2 = 0;
				LED_3 = 1;
			}
		}
		else{ //control B
			if(distance <= 0.3){
				LED_1 = 0;
				LED_2 = 1;
				LED_3 = 0;
			}
			else if(distance>0.3 && distance<=0.6){
				LED_2 = 0;
				LED_3 = 0;
				if((time_10ms%100==0) && (distance<0.6)){
					LED_1 ^= 1;
				}
			}
			else if(distance>0.6 && distance<=0.9){
				LED_2 = 0;
				LED_3 = 0;
				if((time_10ms%100==0) && (distance<0.6)){
					LED_1 ^= 1;
				}
			}
			else if(distance>0.9 && distance<=1.5){
				LED_2 = 0;
				LED_3 = 0;
				if((time_10ms%50==0) && (distance<0.6)){
					LED_1 ^= 1;
				}
			}
			else{
				LED_1 = 0;
				LED_2 = 0;
				LED_3 = 1;
			}
		}
	}
}



void Radar1_hand(void)
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
				if(Radar1_Rec_Buf[i] == 0x59){
					state1 = FIX2;
					buffer[0] = Radar1_Rec_Buf[i];
				}
				else{
					state1 = FIX1;
				}
				break;
			case FIX2:
				if(Radar1_Rec_Buf[i] == 0x59){
					buffer[1] = Radar1_Rec_Buf[i];
					state1 = DIS_L;
				}
				else{
					state1 = FIX1;
				}
				break;
			case DIS_L:
				buffer[2] = Radar1_Rec_Buf[i];
				state1 = DIS_H;
				break;
				
			case DIS_H:
				buffer[3] = Radar1_Rec_Buf[i];
				state1 = STR_1;
				break;
			case STR_1:
				buffer[4] = Radar1_Rec_Buf[i];
				state1 = STR_2;
				break;
			case STR_2:
				buffer[5] = Radar1_Rec_Buf[i];
				state1 = MODE;
				break;
			case MODE:
				buffer[6] = Radar1_Rec_Buf[i];
				state1 = SAVE;
				break;
			case SAVE:
				buffer[7] = Radar1_Rec_Buf[i];
				state1 = ChceckSum;
				break;
			case ChceckSum:
//				printf("buf=%02x crc=%02x\n",Radar1_Rec_Buf[i],check_sum(buffer));
				if( Radar1_Rec_Buf[i] == check_sum(buffer)){
					Radar1_ok = true;
//					printf("Radar1_ok=%d\n",Radar1_ok);
				}
//				printf("Radar1 sum ok\n");
				break;
			default:
				break;
		}
//		if(Radar1_ok){
//			state1 = FIX1;
//			printf("Radar1_ok\n");
//			return;
//		}
	}
	state1 = FIX1;
	
//control A
//	printf("Radar_Mode=%d ok=%d\n",(uint16_t)Radar_Mode ,Radar1_ok );
	if(Radar1_ok==true && (Radar_Mode == 1)){
//		printf("dis jisuan \n");
		float distance = 0;
		Radar1_ok = false;
		distance = (float)(buffer[2] + buffer[3]*256)/100;
		
//		printf("distance1=%f \n",distance);
		if(LED_Mode){
			if((time_10ms%200==0) && (distance<3.0)){
				LED_1 ^= 1;
				LED_2 ^= 1;
				LED_3 = 0;
			}
			else if(distance>0.3 && distance<0.9){
				LED_1 = 1;
				LED_2 = 1;
				LED_3 = 0;
			}
			if(distance>3.0){
				LED_1 = 0;
				LED_2 = 0;
				LED_3 = 1;
			}
		}
		else{ //control B
			if(distance <= 0.3){
				LED_1 = 0;
				LED_2 = 1;
			}
			else if(distance>0.3 && distance<=0.6){
				LED_2 = 0;
				if((time_10ms%200==0) && (distance<0.6)){
					LED_1 ^= 1;
				}
			}
			else if(distance>0.6 && distance<=0.9){
				LED_2 = 0;
				if((time_10ms%50==0) && (distance<0.6)){
					LED_1 ^= 1;
				}
			}
			else if(distance>0.9 && distance<=1.5){
				LED_2 = 0;
				if((time_10ms%100==0) && (distance<0.6)){
					LED_1 ^= 1;
				}
			}
			else{
				LED_1 = 0;
				LED_2 = 0;
				LED_3 = 1;
			}
		}
	}
	
}

void Radar_Data_Set(uint8_t *cmd)
{
	if((cmd[0]==0xA5) && (cmd[1]==0x5A)){
		switch(cmd[2])
		{	
			case RADAR1_A_STEP12_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_A_STEP12_DIS,(uint16_t*)(&cmd[3]),1);
				break;
			case RADAR1_A_STEP3_BIG:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_A_STEP3_BIG,(uint16_t*)(&cmd[3]),1);
				break;
			case RADAR1_A_STEP3_MIN:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_A_STEP3_MIN,(uint16_t*)(&cmd[3]),1);
				break;
			case RADAR1_B_STEP12_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_B_STEP12_DIS,(uint16_t*)(&cmd[3]),2);
				break;
			case RADAR1_B_STEP3_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_B_STEP3_DIS,(uint16_t*)(&cmd[3]),2);
				break;
			case RADAR1_B_STEP4_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_B_STEP4_DIS,(uint16_t*)(&cmd[3]),2);
				break;
			case RADAR1_B_STEP5_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR1_B_STEP5_DIS,(uint16_t*)(&cmd[3]),2);
				break;
			case RADAR2_A_STEP12_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_A_STEP12_DIS,(uint16_t*)(&cmd[3]),2);
				break;
			case RADAR2_A_STEP3_BIG:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_A_STEP3_BIG,(uint16_t*)(&cmd[3]),2);
				break;
			case RADAR2_A_STEP3_MIN:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_A_STEP3_MIN,(uint16_t*)(&cmd[3]),2);
				break;
			case RADAR2_B_STEP12_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_B_STEP12_DIS,(uint16_t*)(&cmd[3]),2);
				break;
			case RADAR2_B_STEP3_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_B_STEP3_DIS,(uint16_t*)(&cmd[3]),2);
				break;
			case RADAR2_B_STEP4_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_B_STEP4_DIS,(uint16_t*)(&cmd[3]),2);
				break;
			case RADAR2_B_STEP5_DIS:
				STMFLASH_Write(FLASH_SAVE_ADDR + RADAR2_B_STEP5_DIS,(uint16_t*)(&cmd[3]),2);
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
