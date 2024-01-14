#include "mcp4017.h"
#include "delay.h"

//��ʼ��IIC
void IIC1_Init(void)
{			
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��
    //GPIOC4,C5��ʼ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
    IIC1_SCL=1;
    IIC1_SDA=1;
}

//����IIC��ʼ�ź�
void IIC1_Start(void)
{
	SDA1_OUT();     //sda�����
	IIC1_SDA=1;	  	  
	IIC1_SCL=1;
	delay_us(4);
 	IIC1_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC1_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	

//����IICֹͣ�ź�
void IIC1_Stop(void)
{
	SDA1_OUT();//sda�����
	IIC1_SCL=0;
	IIC1_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC1_SCL=1; 
	IIC1_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC1_Wait_Ack(void)
{
    u8 ucErrTime = 0;	
    SDA1_IN();      //SDA����Ϊ����   
    IIC1_SDA = 1;
    delay_us(1);	   
    IIC1_SCL = 1;
    delay_us(1);	
	while (READ1_SDA)
	{
        ucErrTime++;
        if (ucErrTime>250)
        {
            IIC1_Stop();
            return 1;
        }
	}
	IIC1_SCL=0;//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
void IIC1_Ack(void)
{
	IIC1_SCL=0;
	SDA1_OUT();
	IIC1_SDA=0;
	delay_us(2);
	IIC1_SCL=1;
	delay_us(2);
	IIC1_SCL=0;
}

//������ACKӦ��		    
void IIC1_NAck(void)
{
	IIC1_SCL=0;
	SDA1_OUT();
	IIC1_SDA=1;
	delay_us(2);
	IIC1_SCL=1;
	delay_us(2);
	IIC1_SCL=0;
}	

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC1_Send_Byte(u8 txd)
{                        
    u8 t;   
    SDA1_OUT(); 	    
    IIC1_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0; t<8; t++)
    {              
        IIC1_SDA=(txd&0x80)>>7;
        txd <<= 1; 	  
        delay_us(2);   //��TEA5767��������ʱ���Ǳ����
        IIC1_SCL=1;
        delay_us(2); 
        IIC1_SCL=0;	
        delay_us(2);
    }	 
}

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC1_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA1_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
    {
        IIC1_SCL=0; 
        delay_us(2);
        IIC1_SCL=1;
        receive<<=1;
        if(READ1_SDA)
        {
            receive++;
        }   
        delay_us(1); 
    }					 
    if (!ack)
        IIC1_NAck();//����nACK
    else
        IIC1_Ack(); //����ACK   
    return receive;
}

void I2C1_init(void)
{
	IIC1_Init();
}

uint16_t MCP4017_Read_IV(void)
{
	uint16_t val;	
	IIC1_Start();           //����IIC
	IIC1_Send_Byte(0x5f);   //���Ͷ������ź�
	IIC1_Wait_Ack();        //�ȴ���Ӧ
	val=IIC1_Read_Byte(0);  //��ȡ����
	IIC1_Stop();            //ֹͣIIC
	return val;
}

void MCP4017_Write_IV(uint16_t info)
{
	IIC1_Start();           //����IIC
	IIC1_Send_Byte(0x5e);   //����д�����ź�
	IIC1_Wait_Ack();        //�ȴ���Ӧ	
	IIC1_Send_Byte(info);   //����Ҫд������
	IIC1_Wait_Ack();
	IIC1_Stop();            //ֹͣIIC
}

