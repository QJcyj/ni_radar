#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"  

#define u8 unsigned char	

//////////////////////////////////////////////////////////////////////////////////////////////////////
//用户根据自己的需要设置
#define STM32_FLASH_SIZE 	128 	 		//所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN 	1              	//使能FLASH写入(0，不是能;1，使能)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 		//STM32 FLASH的起始地址

#define FLASH_SAVE_ADDR  0X0800F000		//数据保存地址   0x08000000 +  设定区   60K处  F000

#define	RADAR1_A_STEP12_DIS	0x00		//雷达1 模式A 步骤1 2 数据存储地址
#define	RADAR1_A_STEP3_BIG	0x02		//雷达1 模式A 步骤3 大距离数据存储地址
#define	RADAR1_A_STEP3_MIN	0x04		//雷达1 模式A 步骤3 小距离数据存储地址

#define	RADAR1_B_STEP12_DIS	0x06
#define	RADAR1_B_STEP3_DIS	0x08
#define	RADAR1_B_STEP4_DIS	0x0A
#define	RADAR1_B_STEP5_DIS	0x0C


#define	RADAR2_A_STEP12_DIS	0x0E
#define	RADAR2_A_STEP3_BIG	0x10
#define	RADAR2_A_STEP3_MIN	0x12

#define	RADAR2_B_STEP12_DIS	0x14
#define	RADAR2_B_STEP3_DIS	0x16
#define	RADAR2_B_STEP4_DIS	0x18
#define	RADAR2_B_STEP5_DIS	0x1A
#define LOAD_PARAM			0x1E


void STMFLASH_Unlock(void);					  //FLASH解锁
void STMFLASH_Lock(void);					  //FLASH上锁
uint8_t STMFLASH_GetStatus(void);				  //获得状态
uint8_t STMFLASH_WaitDone(uint16_t time);				  //等待操作结束
uint8_t STMFLASH_ErasePage(uint32_t paddr);			  //擦除页
uint8_t STMFLASH_WriteHalfWord(uint32_t faddr, uint16_t dat);//写入半字
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr);		  //读出半字  
void STMFLASH_WriteLenByte(uint32_t WriteAddr,uint32_t DataToWrite,uint16_t Len);	//指定地址开始写入指定长度的数据
uint32_t STMFLASH_ReadLenByte(uint32_t ReadAddr,uint16_t Len);						//指定地址开始读取指定长度数据
void STMFLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint16_t NumToRead);   		//从指定地址开始读出指定长度的数据

//测试写入
void Test_Write(uint32_t WriteAddr,uint16_t WriteData);								   
#endif

















