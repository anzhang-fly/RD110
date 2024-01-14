#include "dma.h"
#include "delay.h"
#include "timer.h"
#include "OURS.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//DMA ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/8
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
extern u8 ad_sam_ok,ad_sam_adgin;

DMA_InitTypeDef DMA_InitStructure;

u16 DMA1_MEM_LEN;  //����DMAÿ�����ݴ��͵ĳ��� 

//DMA1�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_CHx:DMAͨ��CHx
//cpar:�����ַ
//cmar:�洢����ַ
//cndtr:���ݴ����� 
void DAC_DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
	NVIC_InitTypeDef NVIC_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
	
    DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ

	DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;                    //DMA�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;                        //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = cndtr;                           //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;          //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //������ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                       //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
    
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Channel3,DMA_IT_TC,ENABLE);
} 

//����һ��DMA����
void MYDMA_Enable(DMA_Channel_TypeDef *DMA_CHx)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //�ر�USART1 TX DMA1 ��ָʾ��ͨ��      
 	DMA_SetCurrDataCounter(DMA_CHx, DMA1_MEM_LEN);//DMAͨ����DMA����Ĵ�С
 	DMA_Cmd(DMA_CHx, ENABLE);  //ʹ��USART1 TX DMA1 ��ָʾ��ͨ�� 
}


// DMA1_Channel1����ADC����
void ADC1_DMA1_Config(DMA_Channel_TypeDef* DMAy_CHx, u16 SAMPLS_NUM)
{
	NVIC_InitTypeDef NVIC_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
	
    DMA_DeInit(DMAy_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ

	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;                    //DMA�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ad_val;                        //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = SAMPLS_NUM;                           //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;  //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;          //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //������ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                       //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMAy_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    DMA_ClearITPendingBit(DMA1_IT_TC1);
    DMA_ITConfig(DMAy_CHx, DMA1_IT_TC1, ENABLE);
    
    /*==================ʹ��ADC1DMA����===================*/
	// ADC_DMACmd(ADC1,ENABLE);
	
	/*==================ʹ��DMA1ͨ��1 ��������===================*/
	DMA_Cmd(DMAy_CHx,ENABLE);
} 

void DMA1_Channel3_IRQHandler()
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC3))
    {
        if(ad_sam_ok == 0 && ad_sam_adgin == 0)
        {
            delay_us(50);
            //����û�в����ⲿ���� ������ʹ�����ADCת��
            //ADC ����ģʽҪ����Ϊͬ������ģʽ������ ADC ��ͨ���Ĳ���ʱ����Ҫһ�£�
            //ADC1����Ϊ���������ADC2 ����Ϊ�ⲿ���� 
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            ad_sam_adgin = 1;
        }
    }
    DMA_ClearITPendingBit(DMA1_FLAG_TC3);
}


void DMA1_Channel1_IRQHandler()
{
    //u16 a=0;
	if(DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
//        for (a = 0 ; a < 2000; a++)
//        {
//            adc2_val[a] = (ad_val[a] & 0xFFFF0000) >> 16;
//            adc1_val[a] = (ad_val[a] & 0xFFFF);
//        }
        ad_sam_ok = 1;
        ad_sam_adgin = 0; 
    }
    DMA_ClearITPendingBit(DMA1_FLAG_TC1);
}






