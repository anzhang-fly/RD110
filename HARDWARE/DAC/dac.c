#include "dac.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//DAC ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/8
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  

//DACͨ��1�����ʼ��,DAC_Signal
void Dac1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );	   //ʹ��PORTAͨ��ʱ��
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );	  //ʹ��DACͨ��ʱ�� 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				  // �˿�����
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 		     //ģ������
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);                       //PA.4 �����
					
	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	                    //��ʹ�ô������� TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;            //��ʹ�ò��η���
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;  //���Ρ���ֵ����
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	        //DAC1�������ر� BOFF1=1
  	DAC_Init(DAC_Channel_1,&DAC_InitType);	                            //��ʼ��DACͨ��1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //ʹ��DAC1
  
  	DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12λ�Ҷ������ݸ�ʽ����DACֵ
}

//����ͨ��1�����ѹ
//vol:0~3300,����0~3.3V
void Dac1_Set_Vol(u16 vol)
{
	float temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12λ�Ҷ������ݸ�ʽ����DACֵ
}


//DACͨ��2�����ʼ��,DAC_TEST
void DAC2_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    DAC_InitTypeDef  DAC_InitStructure;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );	    //ʹ��PORTAͨ��ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );	    //ʹ��DACͨ��ʱ��
	
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;                  //����GPIOA4 �� A5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;               //����Ϊģ�����루������ע��㣬��Ҫ��
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                      //д��ṹ��
	
	DAC_InitStructure.DAC_Trigger=DAC_Trigger_None;	                //��ʹ�ô������� TEN1=0
	DAC_InitStructure.DAC_WaveGeneration=DAC_WaveGeneration_None;   //��ʹ�ò��η���
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//���Ρ���ֵ����
	DAC_InitStructure.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1�������ر� BOFF1=1
    DAC_Init(DAC_Channel_2,&DAC_InitStructure);	                    //��ʼ��DACͨ��2

	DAC_Cmd(DAC_Channel_2, ENABLE);  //ʹ��DACͨ��2
  
    DAC_SetChannel2Data(DAC_Align_12b_R,0);  //12λ�Ҷ������ݸ�ʽ����DACֵ
}

//vol:0~3300,����0~3.3V
void Dac2_Set_Vol(float vol)
{
	u16 temp;
//	if(vol > 0.9f)     vol = 0.9;
//	if(vol < 0.45f)    vol = 0.45;
	
  temp = (u16)((4096.0f / 3.3f) * vol);
	DAC_SetChannel2Data(DAC_Align_12b_R,temp);//12λ�Ҷ������ݸ�ʽ����DACֵ
}





