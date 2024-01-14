#include "bsp_i2c_gpio.h"
#include "sensor.h"
#include "sensor303.h"
#include "sys.h"
long Hp303b_C0;//12
long Hp303b_C1;//12
double Hp303b_C00;//20
double Hp303b_C10;//20
double Hp303b_C01;//16
double Hp303b_C11;//16
double Hp303b_C20;//16
double Hp303b_C21;//16
double Hp303b_C30;//16

uint8_t ID[4];
double Hp203bPressures = 0,Hp203bAltitudes = 0,Hp203bTemp;     // ÆøÑ¹ º£°Î ÎÂ¶È  

void HAL_Delay( uint8_t  time)
{
    uint8_t  i;
	u16 j;
    for(i=0;i<time;i++)
    {
      for(j=0;j<500;j++);
    }
}

#define Osr_Value 1024;
//#define Osr_Cfg  HP20X_CONVERT_OSR1024;
#define Osr_ConvertTime 16;

/**********************************************************
**Name:      HP203_relocation
**Function:  
**Input:    none
**Output:   none
**********************************************************/
void HP203_relocation(void)
{ 
  HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,HP20X_SOFT_RST);	 
  HAL_Delay(20);
}

/**********************************************************
**Name:      Hp203b_read_id
**Function: 
**Input:    none
**Output:   none
**********************************************************/
void Hp203b_read_id(uint8_t buf[])
{
	u8 ptr[]={0x00};
  if(HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,0x20))
  {
      HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,0x20);
      HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,0x20);
      HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,0x04);
      HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,0x2A);
      HAL_Delay(10);
      
    //  uint8_t 
        
      HP303B_bIICBurstWrite(HP20X_ADDRESSCMD,0xDF, ptr,1);
      HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,0x26);
      HP303B_bIICBurstWrite(HP20X_ADDRESSCMD,0xDE, ptr,1);
      ptr[0]=0x41;
      HP303B_bIICBurstWrite(HP20X_ADDRESSCMD,0xDF, ptr,1);
      
      ptr[0]=0x05;
      HP303B_bIICBurstWrite(HP20X_ADDRESSCMD,0xDE, ptr,1);
      ptr[0]=0x43;
      HP303B_bIICBurstWrite(HP20X_ADDRESSCMD,0xDF, ptr,1);
      HAL_Delay(1);
      HP303B_bIICBurstRead(HP20X_ADDRESSCMD,0x9D, &buf[1],1);

      ptr[0]=0x06;
      HP303B_bIICBurstWrite(HP20X_ADDRESSCMD,0xDE, ptr,1);
      ptr[0]=0x43;
      HP303B_bIICBurstWrite(HP20X_ADDRESSCMD,0xDF, ptr,1);
      HAL_Delay(1);
      HP303B_bIICBurstRead(HP20X_ADDRESSCMD,0x9D, &buf[2],1);

      ptr[0]=0x07;
      HP303B_bIICBurstWrite(HP20X_ADDRESSCMD,0xDE, ptr,1);
      ptr[0]=0x43;
      HP303B_bIICBurstWrite(HP20X_ADDRESSCMD,0xDF, ptr,1);
      HAL_Delay(1);
      HP303B_bIICBurstRead(HP20X_ADDRESSCMD,0x9D, &buf[3],1);
       
      ptr[0]=0x7F;
      HP303B_bIICBurstWrite(HP20X_ADDRESSCMD,0xDE, ptr,1);
      ptr[0]=0x43;
      HP303B_bIICBurstWrite(HP20X_ADDRESSCMD,0xDF, ptr,1);
      HAL_Delay(1);
      HP303B_bIICBurstRead(HP20X_ADDRESSCMD,0x9D, &buf[0],1);
      
      HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,0x2C);
      HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,0x22);
  }
}

/**********************************************************
**Name:     Hp203bReadPressureTemperature
**Function: 
**Input:    none
**Output:   none
**********************************************************/
uint8_t  Hp203bReadPressureTemperature(double *Hp203bPressure,double *Hp203bTemp,double *Hp203bAltitude,uint8_t pressure_Type)
{
  uint8_t DataBuf[6];
  uint8_t Osr_Cfg=HP20X_CONVERT_OSR1024;
//	long Hp203b_Pressure= DataBuf[3];
//	long Hp203b_Altitude = DataBuf[0];
//	long Hp203bPressure_temp = DataBuf[0];
  long Hp203bPressure_temp,Hp203b_Pressure,Hp203b_Altitude;
	
  if(HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,(HP20X_WR_CONVERT_CMD|Osr_Cfg)))
    {
      HAL_Delay((16*4));
      
      HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,HP20X_READ_PT);
     
      HP303B_IIC_Read(HP20X_ADDRESSCMD,DataBuf,6);
      
      Hp203bPressure_temp = DataBuf[0];
      Hp203bPressure_temp <<= 8;
      Hp203bPressure_temp |= DataBuf[1];
      Hp203bPressure_temp <<= 8;
      Hp203bPressure_temp |= DataBuf[2];
      if(Hp203bPressure_temp & 0x800000)
        Hp203bPressure_temp |= 0xff000000;
        
      *Hp203bTemp=(double)Hp203bPressure_temp/100;
        
      Hp203b_Pressure= DataBuf[3];
      Hp203b_Pressure <<= 8;
      Hp203b_Pressure |= DataBuf[4];
      Hp203b_Pressure <<= 8;
      Hp203b_Pressure |= DataBuf[5];
      *Hp203bPressure=(double)Hp203b_Pressure/100;
    //  if(pressure_Type==4)
    //		*Hp203bPressure=((double)Hp203b_Pressure-52428.8)/419430.4*10000;//10pa
    //  
      HP303B_IIC_WriteSingle(HP20X_ADDRESSCMD,HP20X_READ_A);
      
      HP303B_IIC_Read(HP20X_ADDRESSCMD,DataBuf,3);
      
      Hp203b_Altitude = DataBuf[0];
      Hp203b_Altitude <<= 8;
      Hp203b_Altitude |= DataBuf[1];
      Hp203b_Altitude <<= 8;
      Hp203b_Altitude |= DataBuf[2];
      if(Hp203b_Altitude & 0x800000)
        Hp203b_Altitude |= 0xff000000;
      
      *Hp203bAltitude=(double)Hp203b_Altitude/100;
      
      return(1);    
    }
    else 
        return(0);
}

/**********************************************************
**Name:     HP203B_Get_Data_Press_Temp_Altitude
**Function: 
**Input:    none
**Output:   none
**********************************************************/
void HP203B_Get_Data_Press_Temp_Altitude(double *Hp203bPressure,double *Hp203bAltitude,double *Hp203bTemperature,uint8_t ID[])
{
	Hp203b_read_id(ID);
	Hp203bReadPressureTemperature( Hp203bPressure, Hp203bTemperature, Hp203bAltitude,4);
	
}
