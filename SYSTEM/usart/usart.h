#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��

#define FLASH_SAVE_ADDR  0X0800C004 	//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.
										                  //����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.	
#define USART_REC_LEN  	  200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

extern u8 runing_mode;
extern u8 lock_wavelength;			
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
extern u8 RX_flag;
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void Data_Send_Status(int32_t VAL_1,int32_t VAL_2,int32_t VAL_3);
void complay_command(u8 flag);
u8 string_pipei(char *a,char *b,u8 c);         //�Ƚ��ַ������Ⱥ���
void JSON_REPLY_WRITE_OK(void);
void JSON_REPLY_BD_ARRAY_TEM(void);
void JSON_REPLY_DATA_TEM(void);   //�ظ�����
//void JSON_REPLY_TEC_TEM(void);   //�ظ�TEC���¶�
#endif


