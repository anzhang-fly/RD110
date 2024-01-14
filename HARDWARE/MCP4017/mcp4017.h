
#ifndef __MCP4017_H
#define __MCP4017_H
#include "sys.h" 		   
//IO��������

#define SDA1_IN()  {GPIOC->CRL&=0XFFF0FFFF;GPIOC->CRL|=8<<16;}
#define SDA1_OUT() {GPIOC->CRL&=0XFFF0FFFF;GPIOC->CRL|=3<<16;}
//IO��������	 
#define IIC1_SCL    PCout(5) //SCL
#define IIC1_SDA    PCout(4) //SDA	 
#define READ1_SDA   PCin(4)  //����SDA 

//IIC���в�������
void IIC1_Init(void);                //��ʼ��IIC��IO��				 
void IIC1_Start(void);				//����IIC��ʼ�ź�
void IIC1_Stop(void);	  			//����IICֹͣ�ź�
void IIC1_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC1_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC1_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC1_Ack(void);					//IIC����ACK�ź�
void IIC1_NAck(void);				//IIC������ACK�ź�
void I2C1_init(void);
uint16_t MCP4017_Read_IV(void);
void MCP4017_Write_IV(uint16_t info);
#endif

