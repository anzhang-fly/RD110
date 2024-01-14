#include "dma.h"
#include "delay.h"
#include "timer.h"
#include "OURS.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//DMA 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/8
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
extern u8 ad_sam_ok,ad_sam_adgin;

DMA_InitTypeDef DMA_InitStructure;

u16 DMA1_MEM_LEN;  //保存DMA每次数据传送的长度 

//DMA1的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_CHx:DMA通道CHx
//cpar:外设地址
//cmar:存储器地址
//cndtr:数据传输量 
void DAC_DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
	NVIC_InitTypeDef NVIC_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	
    DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值

	DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;                    //DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;                        //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = cndtr;                           //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;          //数据宽度为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //工作在循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                       //DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //DMA通道x没有设置为内存到内存传输
    
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Channel3,DMA_IT_TC,ENABLE);
} 

//开启一次DMA传输
void MYDMA_Enable(DMA_Channel_TypeDef *DMA_CHx)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //关闭USART1 TX DMA1 所指示的通道      
 	DMA_SetCurrDataCounter(DMA_CHx, DMA1_MEM_LEN);//DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA_CHx, ENABLE);  //使能USART1 TX DMA1 所指示的通道 
}


// DMA1_Channel1传输ADC数据
void ADC1_DMA1_Config(DMA_Channel_TypeDef* DMAy_CHx, u16 SAMPLS_NUM)
{
	NVIC_InitTypeDef NVIC_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	
    DMA_DeInit(DMAy_CHx);   //将DMA的通道1寄存器重设为缺省值

	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;                    //DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ad_val;                        //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = SAMPLS_NUM;                           //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;  //数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;          //数据宽度为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //工作在循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                       //DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMAy_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    DMA_ClearITPendingBit(DMA1_IT_TC1);
    DMA_ITConfig(DMAy_CHx, DMA1_IT_TC1, ENABLE);
    
    /*==================使能ADC1DMA发送===================*/
	// ADC_DMACmd(ADC1,ENABLE);
	
	/*==================使能DMA1通道1 开启传输===================*/
	DMA_Cmd(DMAy_CHx,ENABLE);
} 

void DMA1_Channel3_IRQHandler()
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC3))
    {
        if(ad_sam_ok == 0 && ad_sam_adgin == 0)
        {
            delay_us(50);
            //由于没有采用外部触发 ，所以使用软件ADC转换
            //ADC 工作模式要设置为同步规则模式；两个 ADC 的通道的采样时间需要一致；
            //ADC1设置为软件触发；ADC2 设置为外部触发 
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






