#include "timer.h"
#include "led.h"
#include "OURS.h"
 
extern u8 ad_sam_ok, ad_sam_adgin;

void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
    
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx					 
}


void TIM2_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
    
    /*==============2.配置TIM2PWM模式=============*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//设置输出模式：PWM模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//设置比较输出使能：
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//设置ref为有效电平时 输出高电平
	TIM_OCInitStructure.TIM_Pulse = arr/2;							//输出占空比：50%的PWM波
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    
	TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设							 
}










