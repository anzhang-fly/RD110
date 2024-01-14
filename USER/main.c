#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "math.h"	 
#include "led.h"
#include "dma.h" 
#include "dac.h"
#include "timer.h"
#include "OURS.h"
#include "sensor.h"
#include "sensor303.h"
#include "bsp_i2c_gpio.h"
#include "SHT2X.h"
#include "cJSON.h"
#include "mcp4017.h"
#include "myiic.h"
//   ��   saw_f_1     ��ʾ�ɵ�·�汾��ɨ�跶ΧPY100-FZ200��
//   ��   saw_f_2     ��ʾ�µ�·�汾��ɨ�跶ΧPY130-FZ200��

u8 flag_lock_wavelength = 0;
u8 ERROR_CODE = 0;
u8 ad_sam_ok = 0,ad_sam_adgin = 0;
double nongdu_huancun_val[3];
double nongdu_A;
u8 count = 1;
u16 i = 0;
u16 a=0, b=0, c=0;

int main(void)
{	
    //SCB->VTOR = FLASH_BASE | 0x10000; /* Vector Table Relocation in Internal FLASH. */
    syetem_init();
    mcp4017_step = hex_to_doc(mcp4017_value);
    while(1)
    {			
        ad_sam_ok = 0;
        ad_sam_adgin = 0;
        flag_lock_wavelength = lock_wavelength;
        while(ad_sam_ok == 0);
        
        for (a = 0 ; a < 8000; a++)
        {
            b = (ad_val[a] & 0xFFFF0000) >> 16;
            printf("%d\r\n", b);
            c = (ad_val[a] & 0xFFFF);
            printf("%d\r\n", c);
        }
        
//        for  (i=0; i<ad_cnt; i++)
//        {
//            printf("%d\r\n",adc1_val[i]);
//        }
//         for  (i=0; i<ad_cnt; i++)
//        {
//            printf("%d\r\n",adc2_val[i]);
//        }
        
        ad_math();
        
        //��mcp4017_valueԽС��IV_signal_MaxԽС��
        if (IV_signal_Max > 4000)//�Ѿ����ͣ��µ�����
        {
            if (mcp4017_value < 6)
            {
                mcp4017_value = 0;
            }			
            else
            {							
                mcp4017_value-= 5;
            }
            MCP4017_Write_IV(mcp4017_value);
            Parameter_saving();						
            adc_ff_flag = 1;			
        }	
        
        if ((mcp4017_value == 0) && (adc_ff > 3100))    //�Ѿ����ͣ����������޶ȣ�mcp4017_value�Ѿ����ڵ���С����Ȼ��ǿ��ǿ��
        {
            adc_ff_flag = 2;
        }
        //��mcp4017_valueԽ��IV_signal_MaxԽ��
        else if ((mcp4017_value==128) && (adc_ff < 1000))   //mcp4017_value�Ѿ����ڵ������Ȼ��ǿ����
        { 
            adc_ff_flag = 3;
        }	
        else	
        {				
            if (adc_ff < 2400)
            {
                mcp4017_value+= 5;
                if (mcp4017_value > 127)
                {
                    mcp4017_value = 127;
                }
                 MCP4017_Write_IV(mcp4017_value);
                 Parameter_saving();
                 adc_ff_flag = 1;
            }
            
            else if(adc_ff > 2800)
            {
                if(mcp4017_value < 6)
                {
                    mcp4017_value = 0;
                }			
                else
                {							
                    mcp4017_value-=5;
                }
                 MCP4017_Write_IV(mcp4017_value);
                 Parameter_saving();						
                 adc_ff_flag = 1;
            }
            else
            {
                 adc_ff_flag = 0;
            }		
        }
			
        if (count++ >= count_cum)   // count_cum = 15
        {		
            if(ND_integral > biaoding[0])    
                BIAODING_H(ND_integral);	
            else 							 
                BIAODING_L(nongdu);
            tem_buchang();
        }
                    
        if(count > 6)
        {
            PID_point();					
        }	
                
        if(flag_lock_wavelength == 0)
        {
            if(flag_lock_wavelength == 0)  
                Saw_Generation_init();						
            if(RX_flag == 15)  
            {
                JSON_REPLY_WRITE_OK();
                RX_flag = 0;
                delay_ms(1000);
            }	
            else if(RX_flag == 1)	 
            {
                JSON_REPLY_BD_ARRAY_TEM();
                RX_flag = 0;
                delay_ms(1000);
            }
            
            if(runing_mode == 1)     
                JSON_REPLY_DATA_TEM();
            else if(runing_mode == 2) 
                printf("H:%.3f\r\n", ND_integral);
            else if(runing_mode == 2) 
                printf("H:%.3f  %.3f  %.3f\r\n", ND_integral,S_wuxishou,integral);				
            else                  
            {
                if(count>=count_cum+1)
                {
                    data_send();	
                    if(count >= count_cum+3)   
                        count = count_cum;						
                }
            }				
        }
        LED = !LED;			
    }
    
    if(flag_lock_wavelength == 0)  
    {
        Saw_Generation_init(); 
    }
}

