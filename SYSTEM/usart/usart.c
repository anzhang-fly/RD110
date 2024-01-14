#include "sys.h"
#include "usart.h"	 
#include "ours.h"
#include "delay.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "stmflash.h"
#include "cJSON.h"
#include "string.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 
#include "dac.h"
u8 lock_wavelength = 0;
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
  
void uart_init(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
    //USART1_RX	  GPIOA.10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
    //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}

u8 string_pipei(char *a,char *b,u8 c)         //�Ƚ��ַ������Ⱥ���
{
    u16 i=0;

    for(i=0;i<c;i++)
    {
        if(*(a+i) != *(b+i))  
            return 0;
    }
    
    return 1;
}

char *tcp_server_sendbuf;

void JSON_REPLY_BD_ARRAY_TEM(void)   //�ظ��궨ϵ��
{
	u16 i,size;
	cJSON *root=cJSON_CreateObject();
	cJSON *REPLY=cJSON_CreateObject();
	
	cJSON *C_BD_ARRAY = cJSON_CreateArray();
	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber(BD_ARRAY[0]));
	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber(BD_ARRAY[1]));
	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber(BD_ARRAY[2]));
	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber(BD_ARRAY[3]));
	cJSON_AddItemToObject(root,"BD_ARRAY",C_BD_ARRAY);
	cJSON_AddItemToObject(REPLY,"REPLY",root);

	tcp_server_sendbuf = cJSON_PrintUnformatted(REPLY);
	size=strlen(tcp_server_sendbuf);
	*(tcp_server_sendbuf+size)=0x0A;
	for (i=0;i<size+1;i++)    
	{
		USART_SendData(USART1,*(tcp_server_sendbuf+i));
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���	
	}	
	free(tcp_server_sendbuf);    
	cJSON_Delete(REPLY);
}

extern float tem_V;

void JSON_REPLY_DATA_TEM(void)   //�ظ�����
{
	u16 i,size;
	cJSON *root=cJSON_CreateObject();
	cJSON *REPLY=cJSON_CreateObject();
	
	cJSON *C_BD_ARRAY = cJSON_CreateArray();
	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber(nongdu));
	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber(bc_point));
	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber((u32)adc_ff));
	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber((u16)((wavelength + zhi + tem_V)*10000)));
	cJSON_AddItemToObject(root,"DATA",C_BD_ARRAY);
	cJSON_AddItemToObject(REPLY,"REPLY",root);

	tcp_server_sendbuf = cJSON_PrintUnformatted(REPLY);
	size=strlen(tcp_server_sendbuf);
	*(tcp_server_sendbuf+size)=0x0A;
	for (i=0;i<size+1;i++)    
	{
		USART_SendData(USART1,*(tcp_server_sendbuf+i));
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���	
	}
	free(tcp_server_sendbuf);   
	cJSON_Delete(REPLY);
}

//void JSON_REPLY_TEC_TEM(void)   //�ظ�TEC���¶�
//{
//	u16 i,size;
//	cJSON *root=cJSON_CreateObject();
//	cJSON *REPLY=cJSON_CreateObject();
//	
//	cJSON *C_BD_ARRAY = cJSON_CreateArray();
//	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber(9999));
//	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber(9999));
//	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber(9999));
//	cJSON_AddItemToArray(C_BD_ARRAY, cJSON_CreateNumber((u16)(wavelength*10000)));
//	cJSON_AddItemToObject(root,"TEC",C_BD_ARRAY);
//	cJSON_AddItemToObject(REPLY,"REPLY",root);

//	tcp_server_sendbuf = cJSON_PrintUnformatted(REPLY);
//	size=strlen(tcp_server_sendbuf);
//	*(tcp_server_sendbuf+size)=0x0A;
//	for (i=0;i<size+1;i++)    
//	{
//		USART_SendData(USART1,*(tcp_server_sendbuf+i));
//		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���	
//	}
//	free(tcp_server_sendbuf);   
//	cJSON_Delete(REPLY);
//}

void JSON_REPLY_WRITE_OK(void)
{
	Parameter_saving();
	printf("OK\r\n");
	delay_ms(1000);	
}

u8 command = 0;
u8 RX_flag = 0;
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 i,Res;
	cJSON *json, *DATA, *AA, *bb;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
    {
		Res = USART_ReceiveData(USART1);  //(USART1->DR);	//��ȡ���յ�������
		if ((USART_RX_STA & 0x8000) == 0)     //����δ���
		{ 
			if(USART_RX_STA & 0x4000)      //���յ���0x0d
			{
				if (Res != 0x0a)
                {
                    USART_RX_STA = 0;    //���մ���,���¿�ʼ
                }   
				else 
				{
					USART_RX_STA = 0;	//���������
					if(USART_RX_BUF[0] == 'A' && USART_RX_BUF[8]=='F') 	
					{	
						if(USART_RX_BUF[1] == 0x31)           command = 1;          //��λ ȥУ׼
						else if(USART_RX_BUF[1] == 0x32)      command = 2;         //��ȡ����
						else if(USART_RX_BUF[1] == 0x33)	  command = 3;         //ģʽ�л�
						else if(USART_RX_BUF[1] == 0x34)	  command = 4;         //�Ҳ���
						else if(USART_RX_BUF[1] == 0x35)	  command = 5;         //У׼
						else if(USART_RX_BUF[1] == 0x36)	  command = 6;         //��������
						else if(USART_RX_BUF[1] == 0x37)	  command = 7;         //TEC++	
                        else if(USART_RX_BUF[1] == 0x38)	  command = 8;         //TEC--	
						else if(USART_RX_BUF[1] == 0x39)	  command = 9;         //����ģʽ2							
						complay_command(command);	
						command = 0;
					}
                    
					if (USART_RX_BUF[0] == 'B' && USART_RX_BUF[8] == 'F') 	
					{	
						if(USART_RX_BUF[1] == 0x31)         command = 11;         //��λ ȥУ׼
						else if(USART_RX_BUF[1] == 0x32)    command = 12;         //��ȡ����
						else if(USART_RX_BUF[1] == 0x33)    command = 13;         //��λ ȥУ׼
						else if(USART_RX_BUF[1] == 0x34)    command = 14;         //��ȡ����
                        else if(USART_RX_BUF[1] == 0x35)    command = 15;         //����TECֵ
                        else if(USART_RX_BUF[1] == 0x36)    command = 16;         //����TECֵ		
                        else if(USART_RX_BUF[1] == 0x37)    command = 17;						
						complay_command(command);	
						command = 0;
					}
                    
					if (USART_RX_BUF[0] == 'C' && USART_RX_BUF[8] == 'F') 	
					{	
						if(USART_RX_BUF[1] == 0x32)         command = 22;         //��ȡ����
						else if(USART_RX_BUF[1] == 0x34)    command = 24;         //��ȡ����
                        else if(USART_RX_BUF[1] == 0x36)    command = 26;         //����TECֵ		
                        else if(USART_RX_BUF[1] == 0x38)    command = 28;
                        else if(USART_RX_BUF[1] == 0x30)    command = 30;						
						complay_command(command);	
						command = 0;
					}
                    
                    if (USART_RX_BUF[0] == 0x46 && USART_RX_BUF[1] == 0x69 && USART_RX_BUF[2] == 0x72 && USART_RX_BUF[3] == 0x5F && USART_RX_BUF[4] == 0x75 &&
                       USART_RX_BUF[5] == 0x70 && USART_RX_BUF[6] == 0x64 && USART_RX_BUF[7] == 0x61 && USART_RX_BUF[8] == 0x74 && USART_RX_BUF[9] == 0x61)
					{
						printf("OK");
						NVIC_SystemReset(); 
					}
         
					json = cJSON_Parse((const char *)USART_RX_BUF);
					
					DATA = cJSON_GetObjectItem(json,"READ");
					if (DATA  != NULL)                         // READ
					{
                        AA = cJSON_GetObjectItem(DATA,"SET");
                        if (AA != NULL)     
                        {
                            if (string_pipei(AA->valuestring,"BD_ARRAY", 8) == 1)        
                                RX_flag = 1;      //���궨ϵ��
                        }
					}
					
					DATA = cJSON_GetObjectItemCaseSensitive(json,"WRITE");
					if (DATA  != NULL)                       //WRITE
					{
                        AA=cJSON_GetObjectItem(DATA,"BD_ARRAY");        
                        if (AA != NULL)    
                            RX_flag = 15;       //д�궨ϵ��
					}
                    
					if (RX_flag == 1)            //���궨ϵ��
					{
						BD_ARRAY[0] = intercept;
						BD_ARRAY[1] = B1;
						BD_ARRAY[2] = B2;
						BD_ARRAY[3] = B3;						
					}
                    
                    else if(RX_flag == 15)       //д�궨����
					{
                        DATA = cJSON_GetObjectItemCaseSensitive(json,"WRITE");
                        AA = cJSON_GetObjectItem(DATA,"BD_ARRAY");
                        for(i = 0; i < 4; i++)
                        {																							   	
                            bb = cJSON_GetArrayItem(AA, i);
                            BD_ARRAY[i] = bb -> valuedouble;
                        }	
                            intercept = BD_ARRAY[0];
                            B1 = BD_ARRAY[1];
                            B2 = BD_ARRAY[2];
                            B3 = BD_ARRAY[3];									
					}
					
                    for(i=0; i < USART_RX_STA; i++)	   
                        USART_RX_BUF[i] = 0;	
                    cJSON_Delete(json);						
		 		}					
			}
			else //��û�յ�0X0d
			{	
				if(Res==0x0d)
                    USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))
                        USART_RX_STA=0;         //�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
    } 
}
#endif

extern F32_save a9;
u8 runing_mode = 0;
void complay_command(u8 flag)
{
	if(flag == 1)    //ȥУ׼
	{
        Calibration_coefficient = 1.0000;	
	}
	else if(flag == 2)
	{
        Parameter_read();
        printf("TEC=%.4f,  Tem_C_H=%.4f,  Tem_C_L=%.4f\r\n",wavelength,Tem_coefficient_h,Tem_coefficient_l);
        delay_ms(1000);		
	}
	else if(flag == 3)
	{
        if(runing_mode == 0)                   
            runing_mode = 1;
        else if(runing_mode == 1)              
            runing_mode = 0;	
	}
	else if(flag == 4)
	{
		lock_wavelength = 1;	
	}	
	else if(flag == 5)    //У׼
	{
		Calibration_coefficient = (u16)(((double)((USART_RX_BUF[2] * 256+USART_RX_BUF[3]) * 100) / nongdu_val) * 10000);
		Parameter_saving();
        printf("Calibration_coefficient=%.4f\r\n", (float)Calibration_coefficient/10000.0f);
        delay_ms(1000);			
	}
	else if(flag == 6)      //
	{
		Parameter_saving();
		printf("Parameter_saving_ok\r\n");
		delay_ms(1000);
	}		
	else if(flag == 7)      //
	{
		Tem_coefficient_h += 0.0002;
        printf("Tem_coefficient=%.4f\r\n",Tem_coefficient_h);
		delay_ms(1000);
	}
	else if(flag == 8)      //
	{
		Tem_coefficient_h -= 0.0002;
		printf("Tem_coefficient=%.4f\r\n",Tem_coefficient_h);
		delay_ms(1000);
	}	
	else if(flag == 9)
	{
		if(runing_mode == 0)                   
            runing_mode= 2;
		else if(runing_mode == 2)              
            runing_mode= 0;
	}		
	
	else if(flag == 11)      //
	{
		wavelength += 0.02;  //0.002
		Dac2_Set_Vol(wavelength + zhi);
		Parameter_saving();		
        printf("Tem=%.4f\r\n",wavelength + zhi);
	}
	else if(flag == 12)      //
	{
		wavelength -= 0.02;  //0.002
		Dac2_Set_Vol(wavelength + zhi);
		Parameter_saving();		
		printf("Tem=%.4f\r\n",wavelength + zhi);
	}
	else if(flag == 13)      //
	{
		Tem_coefficient_l += 0.0002;
        printf("Tem_coefficient=%.4f\r\n",Tem_coefficient_l);
		delay_ms(1000);
	}
	else if(flag == 14)      //
	{
		Tem_coefficient_l -= 0.0002;
		printf("Tem_coefficient=%.4f\r\n",Tem_coefficient_l);
		delay_ms(1000);
	}	
	else if(flag == 15)      //
	{
		wavelength = wavelength+tem_V;
		tem_V = 0;
		Parameter_saving();
		printf("wavelength=%.4f\r\n",wavelength);
		delay_ms(1000);
	}	
	else if(flag == 16)      //
	{
        a9.s1 = 0x00001234;
		Parameter_saving();
		printf("У׼��\r\n");
		delay_ms(1000);
	}	
	else if(flag == 17)      //
	{
        a9.s1 = 0x00000000;
		Parameter_saving();
		printf("У׼�ر�\r\n");
		delay_ms(1000);
	}		
	else if(flag == 22)            //20%
	{	
		flag_H_D = 2;
	}
	else if(flag == 24)            //30%
	{	
		flag_H_D = 4;
	}
	else if(flag == 26)            //50%
	{	
		flag_H_D = 6;
	}
	else if(flag == 28)            //80%
	{	
		flag_H_D = 8;
	}
	else if(flag == 30)            //100%
	{	
		flag_H_D = 10;
	}	
}


u8 data_to_send[100];
#define    BYTE0(dwTemp)     (*((char *)(&dwTemp)))
#define    BYTE1(dwTemp)     (*((char *)(&dwTemp) + 1))
#define    BYTE2(dwTemp)     (*((char *)(&dwTemp) + 2))
#define    BYTE3(dwTemp)     (*((char *)(&dwTemp) + 3))
void Data_Send_Status(int32_t VAL_1, int32_t VAL_2, int32_t VAL_3)
{
	u8 _cnt=0, i=0;
	u8 sum=0;
	u8 data_to_send[16];
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0xF1;
	data_to_send[_cnt++]=0;

	data_to_send[_cnt++]=BYTE3(VAL_1);
	data_to_send[_cnt++]=BYTE2(VAL_1);	
	data_to_send[_cnt++]=BYTE1(VAL_1);
	data_to_send[_cnt++]=BYTE0(VAL_1);

	data_to_send[_cnt++]=BYTE3(VAL_2);
	data_to_send[_cnt++]=BYTE2(VAL_2);	
	data_to_send[_cnt++]=BYTE1(VAL_2);
	data_to_send[_cnt++]=BYTE0(VAL_2);

	data_to_send[_cnt++]=BYTE3(VAL_3);
	data_to_send[_cnt++]=BYTE2(VAL_3);	
	data_to_send[_cnt++]=BYTE1(VAL_3);
	data_to_send[_cnt++]=BYTE0(VAL_3);

	data_to_send[_cnt++]=0xA1;
	
	data_to_send[4] = _cnt-5;
	
	for(i=0; i < _cnt; i++)   
        sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
    for(i=0; i < _cnt; i++)
    {
        USART_SendData(USART1, data_to_send[i]);         //�򴮿�1��������
        while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���				
    }	
	
}

