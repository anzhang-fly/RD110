
#ifndef __MCP4017_H
#define __MCP4017_H
#include "sys.h" 		   
//IO方向设置

#define SDA1_IN()  {GPIOC->CRL&=0XFFF0FFFF;GPIOC->CRL|=8<<16;}
#define SDA1_OUT() {GPIOC->CRL&=0XFFF0FFFF;GPIOC->CRL|=3<<16;}
//IO操作函数	 
#define IIC1_SCL    PCout(5) //SCL
#define IIC1_SDA    PCout(4) //SDA	 
#define READ1_SDA   PCin(4)  //输入SDA 

//IIC所有操作函数
void IIC1_Init(void);                //初始化IIC的IO口				 
void IIC1_Start(void);				//发送IIC开始信号
void IIC1_Stop(void);	  			//发送IIC停止信号
void IIC1_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC1_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC1_Wait_Ack(void); 				//IIC等待ACK信号
void IIC1_Ack(void);					//IIC发送ACK信号
void IIC1_NAck(void);				//IIC不发送ACK信号
void I2C1_init(void);
uint16_t MCP4017_Read_IV(void);
void MCP4017_Write_IV(uint16_t info);
#endif

