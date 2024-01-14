#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持

#define FLASH_SAVE_ADDR  0X0800C004 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.
										                  //否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.	
#define USART_REC_LEN  	  200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

extern u8 runing_mode;
extern u8 lock_wavelength;			
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
extern u8 RX_flag;
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void Data_Send_Status(int32_t VAL_1,int32_t VAL_2,int32_t VAL_3);
void complay_command(u8 flag);
u8 string_pipei(char *a,char *b,u8 c);         //比较字符串长度函数
void JSON_REPLY_WRITE_OK(void);
void JSON_REPLY_BD_ARRAY_TEM(void);
void JSON_REPLY_DATA_TEM(void);   //回复数据
//void JSON_REPLY_TEC_TEM(void);   //回复TEC的温度
#endif


