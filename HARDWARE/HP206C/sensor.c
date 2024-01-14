#include "bsp_i2c_gpio.h"
#include "sensor.h"
#include "sensor303.h"
#include "sys.h"


/************************************************
函数名称 ： HP303B_IIC_Write
功    能 ： 向HP303B写n字节
参    数 ： IIC_Address -------- 写入地址
            buffer -------- 写入数据
            count -------- 数据长度
作    者 ： strongerHuang
*************************************************/
uint8_t HP303B_IIC_Write(uint8_t IIC_Address,uint8_t *buffer,uint16_t count)
{
	//add your own code which send data via iic
  uint16_t i;

  I2C_Start();
  if(I2C_WriteByte(IIC_Address)==0)
	{
		for(i=0;i<count;i++)
		{
			I2C_WriteByte(buffer[i]);
		}			
		I2C_Stop();
		
		return 1;
	}
	else
		return 0;
}


/************************************************
函数名称 ： HP303B_IIC_Read
功    能 ： 从HP303B读取n字节
参    数 ： IIC_Address -------- 写入地址
            buffer -------- 写入数据
            count -------- 数据长度
作    者 ： strongerHuang
*************************************************/
uint8_t HP303B_IIC_Read(uint8_t IIC_Address,uint8_t *buffer,uint16_t count)
{
	//add your own code which receive data via iic
	uint16_t i = 0;
	
	count = count - 1;
	I2C_Start();
	if(I2C_WriteByte(IIC_Address|0x01)==0)
	{
		for(i=0;i<count;i++)
		{
			buffer[i] = I2C_ReadByte();
			I2C_PutAck(0);
		}
		buffer[i] = I2C_ReadByte();
		I2C_PutAck(1);
		
		//I2C_ReadByte();
		I2C_Stop();
		return 1;
	}
	return 0;
}


/************************************************
函数名称 ： HP303B_IIC_WriteSingle
功    能 ： 向HP303B写入1个字节
参    数 ： IIC_Address -------- 写入地址
            buffer -------- 写入数据
作    者 ： strongerHuang
*************************************************/
uint8_t  HP303B_IIC_WriteSingle(uint8_t IIC_Address,uint8_t buffer)
{
	if(HP303B_IIC_Write(IIC_Address,&buffer,1))
        return 1;
	else 
        return 0;
}


/**********************************************************
**Name: 	HP303B_bIICBurstWrite
**Func: 	IIC wirte some byte 
**input: 	chip---------chip address 芯片地址（从机地址）
			addr---------regesisiter address 待写入的芯片寄存器地址
            ptr----------write buffer pointer  写入数据  
            length-------buffer length  数据长度
**Output:	true---------write success
            false--------write faild
**********************************************************/
uint8_t HP303B_bIICBurstWrite(uint8_t chip, uint8_t addr, uint8_t ptr[], uint8_t length)
{
	uint8_t i;	
	
	I2C_Start();
    I2C_WriteByte(chip);
	if(I2C_WriteByte(addr)==0)
	{
		for(i=0;i<length;i++)
		{
			I2C_WriteByte(ptr[i]);
		}			
		I2C_Stop();
        return 1;
	}
	else
		return 0;
}


/**********************************************************
**Name: 	P303B_bIICBurstRead
**Func: 	IIC Read some byte 
**input: 	chip---------chip address 芯片地址（从机地址）
			addr---------regesisiter address 待读取芯片的寄存器地址
            ptr----------write buffer pointer  读取数据缓存  
            length-------buffer length  读取数据长度
**Output:	true---------write success
            false--------write faild
**********************************************************/
uint8_t HP303B_bIICBurstRead(uint8_t chip, uint8_t addr, uint8_t ptr[], uint8_t length)
{
	HP303B_IIC_Write(chip, &addr,1);	
	if(HP303B_IIC_Read(chip, ptr,length))
        return 1 ;
	else 
        return 0 ;
}
