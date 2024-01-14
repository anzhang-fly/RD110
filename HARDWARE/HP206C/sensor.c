#include "bsp_i2c_gpio.h"
#include "sensor.h"
#include "sensor303.h"
#include "sys.h"


/************************************************
�������� �� HP303B_IIC_Write
��    �� �� ��HP303Bдn�ֽ�
��    �� �� IIC_Address -------- д���ַ
            buffer -------- д������
            count -------- ���ݳ���
��    �� �� strongerHuang
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
�������� �� HP303B_IIC_Read
��    �� �� ��HP303B��ȡn�ֽ�
��    �� �� IIC_Address -------- д���ַ
            buffer -------- д������
            count -------- ���ݳ���
��    �� �� strongerHuang
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
�������� �� HP303B_IIC_WriteSingle
��    �� �� ��HP303Bд��1���ֽ�
��    �� �� IIC_Address -------- д���ַ
            buffer -------- д������
��    �� �� strongerHuang
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
**input: 	chip---------chip address оƬ��ַ���ӻ���ַ��
			addr---------regesisiter address ��д���оƬ�Ĵ�����ַ
            ptr----------write buffer pointer  д������  
            length-------buffer length  ���ݳ���
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
**input: 	chip---------chip address оƬ��ַ���ӻ���ַ��
			addr---------regesisiter address ����ȡоƬ�ļĴ�����ַ
            ptr----------write buffer pointer  ��ȡ���ݻ���  
            length-------buffer length  ��ȡ���ݳ���
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
