#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "SHT2X.h"

extern float temperatureC;
extern float humidityRH;
 
int main(void)
{	
	delay_init();	    //��ʱ������ʼ��	  
	uart_init(115200);
	LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
	I2C_Configuration(); 
	 
	SHT2X_TEST(); 
	
	printf("Init Over!\r\n");
	
	while(1)
	{
		LED0=0;
		delay_ms(500);	 //��ʱ500ms
		LED0=1;
		delay_ms(500);	//��ʱ500ms
		
		SHT2X_TEST(); 
	}
}
