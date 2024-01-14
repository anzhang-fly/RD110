 #include "adc.h"
 #include "delay.h"
 #include "OURS.h"


void Adc_Init(void)
{ 	
    GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure; 
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC2, ENABLE);	 
    
    //����ADC��Ƶ����2 16M/2=8, ADC���ʱ�䲻�ܳ���14M
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);   
                     
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // ����ADC1ͨ�����ֱ���
	// ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;    // ͬ������ģʽ
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;        // ����ɨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  // ����ת��
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //�������
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);

    // ����ADC1����ͨ��
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    // ����ADC1����ͨ��
    ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_28Cycles5);
    
    // ʹ��ADC DMA ����
    ADC_DMACmd(ADC1, ENABLE);
    
    /* ʹ�� ADC2 ���ⲿ����ת�� */
    ADC_ExternalTrigConvCmd(ADC2, ENABLE);

    ADC_Cmd(ADC1, ENABLE);                      // ����ADC1

    // У׼ADC1
    ADC_ResetCalibration(ADC1);                    
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    
    ADC_Cmd(ADC2, ENABLE);                      // ����ADC2
    // У׼ADC2
    ADC_ResetCalibration(ADC2);                    
    while(ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while(ADC_GetCalibrationStatus(ADC2));
}

















