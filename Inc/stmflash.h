#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"  

#define u8 unsigned char	

//////////////////////////////////////////////////////////////////////////////////////////////////////
//�û������Լ�����Ҫ����
#define STM32_FLASH_SIZE 	128 	 		//��ѡSTM32��FLASH������С(��λΪK)
#define STM32_FLASH_WREN 	1              	//ʹ��FLASHд��(0��������;1��ʹ��)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 0x08000000 		//STM32 FLASH����ʼ��ַ

#define FLASH_SAVE_ADDR  0X0800F000		//���ݱ����ַ   0x08000000 +  �趨��   60K��  F000

#define	RADAR1_A_STEP12_DIS	0x00		//�״�1 ģʽA ����1 2 ���ݴ洢��ַ
#define	RADAR1_A_STEP3_BIG	0x02		//�״�1 ģʽA ����3 ��������ݴ洢��ַ
#define	RADAR1_A_STEP3_MIN	0x04		//�״�1 ģʽA ����3 С�������ݴ洢��ַ

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


void STMFLASH_Unlock(void);					  //FLASH����
void STMFLASH_Lock(void);					  //FLASH����
uint8_t STMFLASH_GetStatus(void);				  //���״̬
uint8_t STMFLASH_WaitDone(uint16_t time);				  //�ȴ���������
uint8_t STMFLASH_ErasePage(uint32_t paddr);			  //����ҳ
uint8_t STMFLASH_WriteHalfWord(uint32_t faddr, uint16_t dat);//д�����
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr);		  //��������  
void STMFLASH_WriteLenByte(uint32_t WriteAddr,uint32_t DataToWrite,uint16_t Len);	//ָ����ַ��ʼд��ָ�����ȵ�����
uint32_t STMFLASH_ReadLenByte(uint32_t ReadAddr,uint16_t Len);						//ָ����ַ��ʼ��ȡָ����������
void STMFLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint16_t NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����

//����д��
void Test_Write(uint32_t WriteAddr,uint16_t WriteData);								   
#endif

















