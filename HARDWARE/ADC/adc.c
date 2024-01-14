 #include "adc.h"
 #include "delay.h"
 #include "OURS.h"


void Adc_Init(void)
{ 	
    GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure; 
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC2, ENABLE);	 
    
    //设置ADC分频因子2 16M/2=8, ADC最大时间不能超过14M
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);   
                     
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 配置ADC1通道、分辨率
	// ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;    // 同步规则模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;        // 启用扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  // 连续转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);

    // 配置ADC1采样通道
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    // 配置ADC1采样通道
    ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_28Cycles5);
    
    // 使能ADC DMA 请求
    ADC_DMACmd(ADC1, ENABLE);
    
    /* 使能 ADC2 的外部触发转换 */
    ADC_ExternalTrigConvCmd(ADC2, ENABLE);

    ADC_Cmd(ADC1, ENABLE);                      // 启用ADC1

    // 校准ADC1
    ADC_ResetCalibration(ADC1);                    
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    
    ADC_Cmd(ADC2, ENABLE);                      // 启用ADC2
    // 校准ADC2
    ADC_ResetCalibration(ADC2);                    
    while(ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while(ADC_GetCalibrationStatus(ADC2));
}

















