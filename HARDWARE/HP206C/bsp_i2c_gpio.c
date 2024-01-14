#include "bsp_i2c_gpio.h"
#include "sensor.h"
#include "sensor303.h"
#include "sys.h"


/************************************************
��ʼ��PB10��PB11���ţ����ں�hp203b��I2CͨѶ
*************************************************/
void Sensor_I2C_Init( )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_I2C_PORT, ENABLE);	/* ��GPIOʱ�� */
	
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�������	
	GPIO_Init(GPIO_PORT_I2C, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,I2C_SCL_PIN|I2C_SDA_PIN); 	//PB10,PB11 �����
}

/************************************************
�������� �� I2C_Delay
��    �� �� I2C��ʱ(�Ǳ�׼��ʱ,�����MCU�ٶ� ���ڴ�С)
��    �� �� ��
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
 void I2C_Delay(void)
{
  uint16_t  i;
   for(i=0;i<8;i++)
    {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }
}
/************************************************
�������� �� I2C_SDA_SetOutput
��    �� �� I2C_SDA����Ϊ���
��    �� �� ��
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void I2C_SDA_SetOutput(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
	
    GPIO_InitStructure.GPIO_Pin  = I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  	/* ��� */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/************************************************
�������� �� I2C_SDA_SetInput
��    �� �� I2C_SDA����Ϊ����
��    �� �� ��
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void I2C_SDA_SetInput(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Pin  = I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  //HAL_GPIO_Init(PORT_I2C_SDA, &GPIO_InitStructure);
}
void I2C_Start(void)
{
  I2C_SCL_HIGH;                                  //SCL��
  I2C_Delay();

  I2C_SDA_HIGH;                                  //SDA�� -> ��
  I2C_Delay();
  I2C_SDA_LOW;                                   //SDA��
  I2C_Delay();

  I2C_SCL_LOW;                                   //SCL��(��д��֕/����)
  I2C_Delay();
}

/************************************************
�������� �� I2C_Stop
��    �� �� I2Cֹͣ
��    �� �� ��
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void I2C_Stop(void)
{
  I2C_SDA_LOW;                                   //SDA�� -> ��
  I2C_Delay();

  I2C_SCL_HIGH;                                  //SCL��
  I2C_Delay();

  I2C_SDA_HIGH;                                  //SDA��
  I2C_Delay();
}

/************************************************
�������� �� I2C_PutAck
��    �� �� I2C��������Ӧ��(���Ӧ��)λ
��    �� �� I2C_ACK ----- Ӧ��
            I2C_NOACK --- ��Ӧ��
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void I2C_PutAck(uint8_t Ack)
{
  I2C_SCL_LOW;                                   //SCL��
  I2C_Delay();

  if(I2C_ACK == Ack)
    I2C_SDA_LOW;                                 //Ӧ��
  else
    I2C_SDA_HIGH;                                //��Ӧ��
  I2C_Delay();

  I2C_SCL_HIGH;                                  //SCL�� -> ��
  I2C_Delay();
  I2C_SCL_LOW;                                   //SCL��
  I2C_Delay();
}


/************************************************
�������� �� I2C_GetAck
��    �� �� I2C������ȡӦ��(���Ӧ��)λ
��    �� �� ��
�� �� ֵ �� I2C_ACK ----- Ӧ��
            I2C_NOACK --- ��Ӧ��
��    �� �� strongerHuang
*************************************************/
uint8_t I2C_GetAck(void)
{
  uint8_t ack;

  I2C_SCL_LOW;                                   //SCL�� -> ��
  I2C_Delay();

  I2C_SDA_SetInput();                            //SDA����Ϊ����ģʽ(��©ģʽ���Բ����л�����)

  I2C_SCL_HIGH;                                  //SCL��(��ȡӦ��λ)
  I2C_Delay();

  if(I2C_SDA_READ)
    ack = I2C_NOACK;                             //��Ӧ��
  else
    ack = I2C_ACK;                               //Ӧ��

  I2C_SCL_LOW;                                   //SCL��
  I2C_Delay();

  I2C_SDA_SetOutput();                           //SDA����Ϊ���ģʽ

  return ack;                                    //����Ӧ��λ
}
/************************************************
�������� �� I2C_WriteByte
��    �� �� I2Cдһ�ֽ�
��    �� �� Data -------- ����
�� �� ֵ �� I2C_ACK ----- Ӧ��
            I2C_NOACK --- ��Ӧ��
��    �� �� strongerHuang
*************************************************/
uint8_t I2C_WriteByte(uint8_t Data)
{
  uint8_t cnt;
	for(cnt=0; cnt<8; cnt++)
      {
        I2C_SCL_LOW;                                 //SCL?(SCL???????SDA??)
        I2C_Delay();

        if(Data & 0x80)
          I2C_SDA_HIGH;                              //SDA?
        else
          I2C_SDA_LOW;                               //SDA?
        Data <<= 1;
        I2C_Delay();

        I2C_SCL_HIGH;                                //SCL?(????)
        I2C_Delay();
      }
     I2C_SCL_LOW;                                   //SCL?(??????)
     I2C_Delay();

   return I2C_GetAck();                           //?????                       //����Ӧ��λ
}
/************************************************
�������� �� I2C_ReadByte
��    �� �� I2C��һ�ֽ�
��    �� �� ack --------- ����Ӧ��(���ߕ�Ӧ��)λ
�� �� ֵ �� data -------- ��ȡ��һ�ֽ�����
��    �� �� strongerHuang
*************************************************/
uint8_t I2C_ReadByte()
{
  uint8_t cnt;
  uint8_t data;

  I2C_SCL_LOW;                                   //SCL?
  I2C_Delay();
  //I2C_Delay();
  I2C_SDA_SetInput();                            //SDA???????
  for(cnt=0; cnt<8; cnt++)
  {
    I2C_SCL_HIGH;                                //SCL?(????)
    I2C_Delay();

    data <<= 1;
    if(I2C_SDA_READ)
      data |= 0x01;                              //SDA??(????)
	I2C_Delay();
    I2C_SCL_LOW;                                 //SCL?
    I2C_Delay();
  }
  I2C_SDA_SetOutput();                           //SDA???????
  return data;                                   //????                              //��������
}


