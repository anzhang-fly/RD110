#include "bsp_i2c_gpio.h"
#include "sensor.h"
#include "sensor303.h"
#include "sys.h"


/************************************************
初始化PB10、PB11引脚，用于和hp203b的I2C通讯
*************************************************/
void Sensor_I2C_Init( )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_I2C_PORT, ENABLE);	/* 打开GPIO时钟 */
	
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出	
	GPIO_Init(GPIO_PORT_I2C, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,I2C_SCL_PIN|I2C_SDA_PIN); 	//PB10,PB11 输出高
}

/************************************************
函数名称 ： I2C_Delay
功    能 ： I2C延时(暻标准延时,请根据MCU速度 调节大小)
参    数 ： 无
暤 回 值 ： 无
作    者 ： strongerHuang
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
函数名称 ： I2C_SDA_SetOutput
功    能 ： I2C_SDA设置为输出
参    数 ： 无
暤 回 值 ： 无
作    者 ： strongerHuang
*************************************************/
void I2C_SDA_SetOutput(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
	
    GPIO_InitStructure.GPIO_Pin  = I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  	/* 输出 */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/************************************************
函数名称 ： I2C_SDA_SetInput
功    能 ： I2C_SDA设置为输入
参    数 ： 无
暤 回 值 ： 无
作    者 ： strongerHuang
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
  I2C_SCL_HIGH;                                  //SCL高
  I2C_Delay();

  I2C_SDA_HIGH;                                  //SDA高 -> 低
  I2C_Delay();
  I2C_SDA_LOW;                                   //SDA低
  I2C_Delay();

  I2C_SCL_LOW;                                   //SCL低(待写地謺/数据)
  I2C_Delay();
}

/************************************************
函数名称 ： I2C_Stop
功    能 ： I2C停止
参    数 ： 无
暤 回 值 ： 无
作    者 ： strongerHuang
*************************************************/
void I2C_Stop(void)
{
  I2C_SDA_LOW;                                   //SDA低 -> 高
  I2C_Delay();

  I2C_SCL_HIGH;                                  //SCL高
  I2C_Delay();

  I2C_SDA_HIGH;                                  //SDA高
  I2C_Delay();
}

/************************************************
函数名称 ： I2C_PutAck
功    能 ： I2C主机产生应答(或暻应答)位
参    数 ： I2C_ACK ----- 应答
            I2C_NOACK --- 暻应答
暤 回 值 ： 无
作    者 ： strongerHuang
*************************************************/
void I2C_PutAck(uint8_t Ack)
{
  I2C_SCL_LOW;                                   //SCL低
  I2C_Delay();

  if(I2C_ACK == Ack)
    I2C_SDA_LOW;                                 //应答
  else
    I2C_SDA_HIGH;                                //暻应答
  I2C_Delay();

  I2C_SCL_HIGH;                                  //SCL高 -> 低
  I2C_Delay();
  I2C_SCL_LOW;                                   //SCL低
  I2C_Delay();
}


/************************************************
函数名称 ： I2C_GetAck
功    能 ： I2C主机读取应答(或暻应答)位
参    数 ： 无
暤 回 值 ： I2C_ACK ----- 应答
            I2C_NOACK --- 暻应答
作    者 ： strongerHuang
*************************************************/
uint8_t I2C_GetAck(void)
{
  uint8_t ack;

  I2C_SCL_LOW;                                   //SCL低 -> 高
  I2C_Delay();

  I2C_SDA_SetInput();                            //SDA配置为输入模式(开漏模式可以不用切换暯向)

  I2C_SCL_HIGH;                                  //SCL高(读取应答位)
  I2C_Delay();

  if(I2C_SDA_READ)
    ack = I2C_NOACK;                             //暻应答
  else
    ack = I2C_ACK;                               //应答

  I2C_SCL_LOW;                                   //SCL低
  I2C_Delay();

  I2C_SDA_SetOutput();                           //SDA配置为输出模式

  return ack;                                    //暤回应答位
}
/************************************************
函数名称 ： I2C_WriteByte
功    能 ： I2C写一字节
参    数 ： Data -------- 数据
暤 回 值 ： I2C_ACK ----- 应答
            I2C_NOACK --- 暻应答
作    者 ： strongerHuang
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

   return I2C_GetAck();                           //?????                       //暤回应答位
}
/************************************************
函数名称 ： I2C_ReadByte
功    能 ： I2C读一字节
参    数 ： ack --------- 产生应答(或者暻应答)位
暤 回 值 ： data -------- 读取的一字节数据
作    者 ： strongerHuang
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
  return data;                                   //????                              //暤回数据
}


