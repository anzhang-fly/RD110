#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "math.h"	 
#include "dma.h" 
#include "dac.h"
#include "timer.h"
#include "OURS.h"
#include "adc.h"
#include "fdacoefs.h"
#include "sensor.h"
#include "sensor303.h"
#include "bsp_i2c_gpio.h"
#include "stmflash.h"
#include "myiic.h"
#include "SHT2X.h"
#include "mcp4017.h"


#define GQ_POINT        380
#define pi              3.1415926f
#define DAC_SIN_SIZE    1000 	


#ifdef  saw_f_1 
#define DAC_PIANYI      100   //ƫ��
#define DAC_FUZHI       200   //��ֵ
#endif

#ifdef  saw_f_2 
#define DAC_PIANYI     200       // 400   //ƫ��
#define DAC_FUZHI      350       // 400   //��ֵ
#endif  

//#if saw_f_2 
//#define DAC_PIANYI      130
//#define DAC_FUZHI       200
//#endif

u8 flag_H_D = 0;

float biaoding[5];
u32 biaoding_val[5]={200000,300000,500000,800000,1000000};

u32 ad_val[ad_cnt];                    //ad_cnt = 8000������źŲɼ���·����(ADC1ͨ��1)
// u16 ad_val_2[ad_cnt];                  //ad_cnt = 8000����Ųο��źŲɼ���·����(ADC2ͨ��8)

//u16 adc1_val[2000];
//u16 adc2_val[2000];

float Calibration_coefficient=1;   //У׼ϵ��
float wavelength=0;                //TEC�¶ȵ�ѹ
float intercept=0;                 //�궨������
float B1=0;                        //�궨һ����ϵ��
float B2=0;                        //�궨������ϵ��
float B3=0;	                       //�궨������ϵ��
float Tem_coefficient_h=0;           //�¶Ȳ���ϵ��
float Tem_coefficient_l=0;           //�¶Ȳ���ϵ��
float tem_V=0;                       //�¶Ȳ���ֵ

u32 mcp4017_value=64;
u16 mcp4017_step=0;
u8  adc_ff_flag=0;

F32_save a1;
F32_save a2;
F32_save a3;
F32_save a4;
F32_save a5;
F32_save a6;
F32_save a7;
F32_save a8;
F32_save a9;
F32_save a10;
F32_save a11;
F32_save a12;
F32_save a13;
F32_save a14;
F32_save a15;

//double		intercept = 97.51665;
//double		B1 =  10.5764;
//double    B2 =  7.34948E-4;
//double    B3 =  -7.38218E-8;
//double    B4 =  8.01216E-12;
double   nongdu_val;

u8 flag_json_rx = 0;
double BD_ARRAY[4];
extern u8 ERROR_CODE;                    
u16 DAC_SIN_BUF[DAC_SIN_SIZE];         //DAC�������Ҳ����ε�����   DAC_SIN_SIZE=1000
//float wavelength = 0.6000;           //����λ��0.700v

u16 Parameter_save[30];
float sanjiao[san_num];               //san_num = 150
float AD_sanjiao[ad_point];			  //ad_point = 500		

extern u8 flag_lock_wavelength;
u32 AHT20_temp=0;
u32 AHT20_wet=0;
void Parameter_read(void)
{
    u8 i;
    STMFLASH_Read(FLASH_SAVE_ADDR, Parameter_save, 30);
    a1.s1 = Parameter_save[0] << 16 | Parameter_save[1];
    a2.s1 = Parameter_save[2] << 16 | Parameter_save[3];
    a3.s1 = Parameter_save[4] << 16 | Parameter_save[5];
    a4.s1 = Parameter_save[6] << 16 | Parameter_save[7];
    a5.s1 = Parameter_save[8] << 16 | Parameter_save[9];
    a6.s1 = Parameter_save[10] << 16 | Parameter_save[11];
    a7.s1 = Parameter_save[12] << 16 | Parameter_save[13];
    a8.s1 = Parameter_save[14] << 16 | Parameter_save[15];
    a9.s1 = Parameter_save[16] << 16 | Parameter_save[17];
    a10.s1 = Parameter_save[18] << 16 | Parameter_save[19];
    a11.s1 = Parameter_save[20] << 16 | Parameter_save[21];
    a12.s1 = Parameter_save[22] << 16 | Parameter_save[23];
    a13.s1 = Parameter_save[24] << 16 | Parameter_save[25];
    a14.s1 = Parameter_save[26] << 16 | Parameter_save[27];	
    mcp4017_value= Parameter_save[28] << 16 | Parameter_save[29];	
    //	  Calibration_coefficient = a1.f1;

    wavelength = a2.f1;
    intercept = a3.f1;
    B1 = a4.f1;
    B2 = a5.f1;
    B3 = a6.f1;
    Tem_coefficient_h = a7.f1;
    Tem_coefficient_l = a8.f1;
    
    biaoding[0] = a10.f1;
    biaoding[1] = a11.f1;
    biaoding[2] = a12.f1;
    biaoding[3] = a13.f1;
    biaoding[4] = a14.f1;
	 
    if ((a4.s1 == 0xffffffff || a4.s1 == 0x0) && (a5.s1 == 0xffffffff || a5.s1 == 0x0) && (a6.s1 == 0xffffffff || a6.s1 == 0x0))
    {
        intercept = 97.51665;
        B1 =  10.5764;
        B2 =  7.34948E-4;
        B3 =  -7.38218E-8;		
    }
	
//	if(Calibration_coefficient <= 0.5000 || Calibration_coefficient >= 2.0000)     Calibration_coefficient = 1.0000;
//	if(wavelength>=0.9500 || wavelength<=0.4000)    wavelength = 0.5000;	
//  if(Tem_coefficient_h>0.008 || Tem_coefficient_h<0.0001)	  Tem_coefficient_h = 0.0035;	
//	if(Tem_coefficient_l>0.008 || Tem_coefficient_l<0.001)	  Tem_coefficient_l = 0.0035;	

    if (Calibration_coefficient <= 0.5000 || Calibration_coefficient >= 2.0000)
    {
        Calibration_coefficient = 1.0000;
    }
        
    //	if(wavelength>=0.9500 || wavelength<=0.4000)    
    //      wavelength = 0.5000;	
    if (wavelength>=2.500 || wavelength<=1.0000)
    {
        wavelength = 1.0000;
    }        
        	
    if (Tem_coefficient_h>0.008 || Tem_coefficient_h<0.0001)
    {
        Tem_coefficient_h = 0.0035;	
    }
       
    if (Tem_coefficient_l>0.008 || Tem_coefficient_l<0.001)
    {
         Tem_coefficient_l = 0.0035;	
    }		
}

void syetem_init(void)
{
    RCC_Configuration(); 	    //������ƵΪ16M
    delay_init();	    	    //��ʱ������ʼ
    IO_Init();                //PA3 PB12���ų�ʼ��
    uart_init(115200);	    //���ڳ�ʼ��Ϊ115200	
    Sensor_I2C_Init();        //��ʼ��PB10��PB11���ţ����ں�hp203b��I2C2ͨѶ
    HP203_relocation();	    //��hp203bĳ����ַд��ĳ�����ݣ�ʵ��ĳ������
    Parameter_read();         //��flash�ж�ȡ���ݣ�������ax.s1��
    I2C1_init();              //��ʼ��PC4��PC5���ţ����ں�MCP4017��I2C1ͨѶ
    MCP4017_Write_IV(mcp4017_value); //��mcp4017̽����д����ֵ
    //��ʪ�ȴ�����AHT20
    AHT20_Init();             //��ʼ��PB6��PB7���ţ����ں�SHT20��I2C1ͨѶ,�����һ������ͨ��
    sys_int();
}

float zhi=0;
void sys_int(void)
{
	u8 count=0;
    //��Hp203b�ж�ȡ�¶ȡ���ѹ���������ݣ��ֱ����Hp203bPressures,Hp203bAltitudes,Hp203bTemp��
    //ID��ֵ��ͨ��Hp203b_read_id��ID����ȡ��
	HP203B_Get_Data_Press_Temp_Altitude(&Hp203bPressures,&Hp203bAltitudes,&Hp203bTemp,ID);  
	
	while(Hp203bPressures == 0 && Hp203bTemp == 0)   //�������쳣
	{
		HP203B_Get_Data_Press_Temp_Altitude(&Hp203bPressures,&Hp203bAltitudes,&Hp203bTemp,ID);
		if(count++ >=3)
		{
			Hp203bTemp = 25.0f;
			break;
		}
	}
	
    TEM_math();                         //�����ȡ���¶�ֵ
	DAC2_Config();                      //��ʼ��PA5��ΪDAC��ͨ��2���---DAC2��ӦDAC_TEST
	Dac2_Set_Vol(wavelength + zhi);     //����wavelength + zhi������ΪDAC2�����
	LAS_EN = 1;	                        //��ӦPA3���򿪵�Դ
    Adc_Init();                       
    Saw_Generation_init();	            //�������ǲ��ĳ�ʼ��(�õ�DAC1��TIM3��)
    ADC1_DMA1_Config(DMA1_Channel1, ad_cnt);
}

// �����ȡ���¶�ֵ
void TEM_math(void)
{
	int tem = 0;
	tem = (int)(Hp203bTemp * 10);
	if(tem < 250)    	
        zhi = ((250 - tem)/10.0f)/5000.0f;	
//	else              
//    zhi = ((250 - tem)/10.0f)/3000.0f;		
}

void CLOSE_LD(void)
{
	TIM_Cmd(TIM3, DISABLE);
	DAC_Cmd(DAC_Channel_1, DISABLE);  
	DMA_DeInit(DMA1_Channel3);
}

void Saw_Generation_init(void)
{
    TIM3_Int_Init(16-1,3-1);      // ��ʼ����ʱ��3,����Ϊ500Khz
    saw_Generation();             // �����������ǲ�������DAC_SIN_BUF
    Dac1_Init();                  // ��ʼ��PA4��ΪDAC��ͨ��1���---DAC1��ӦDAC_Signal
    /* ����DMA1ͨ��3�����˷���Ϊ���ڴ浽����,����ΪDAC�Ҷ���12λ�Ĵ���(DAC->DHR12R1)��
    ��ַ�̶�Ϊ���������ڴ�ΪDAC_SIN_BUF������DAC_SIN_SIZE */
   	DAC_DMA_Config(DMA1_Channel3,(u32)&DAC->DHR12R1,(u32)DAC_SIN_BUF,DAC_SIN_SIZE);
 	TIM3->DIER|=1<<8;             // ������µ�DMA����
    MYDMA_Enable(DMA1_Channel3);  // �����ڲ��õ�DAC_SIN_SIZE������һ���԰��˵�������
}

void saw_Generation(void)
{
	u16 n;
	for(n=0;n<DAC_SIN_SIZE;n++)
	{
		DAC_SIN_BUF[n] = ((float)(n)/DAC_SIN_SIZE )*DAC_FUZHI +DAC_PIANYI;
	}
}

void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;
    RCC_DeInit();  									//��ʼ��Ϊȱʡֵ
    RCC_HSEConfig(RCC_HSE_ON);  					//ʹ���ⲿ�ĸ���ʱ�� 
    HSEStartUpStatus = RCC_WaitForHSEStartUp();  
    if(HSEStartUpStatus == SUCCESS)  				//�ⲿ����ʱ��ʹ�ܾ���
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div1); 			//HCLK = SYSCLK
        RCC_PCLK2Config(RCC_HCLK_Div1);  			//PCLK2 =  HCLK
        RCC_PCLK1Config(RCC_HCLK_Div2);  			//PCLK1 = HCLK/2
        RCC_ADCCLKConfig(RCC_PCLK2_Div6);
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_2);   //PLLCLK = 8MHZ * 9 =72MHZ������
        RCC_PLLCmd(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while(RCC_GetSYSCLKSource() != 0x08)
        {
        } 
    }
}


int jun[ad_point + san_num]; 	//ad_point=500  san_num=150
u16 nongdu;
u16 bc_point,bc_point_i;   		//������λ��    ��С��λ��
u32 adc_ff;                     //��ǿ
u32 IV_signal_Max=0;			
float ND_integral = 0;
float integral = 0;   			//����
float S_wuxishou = 0;   		//������ʱ�ź������	
void ad_math(void)
{
    u8 flag1=0, flag2;
    u16 i, j, k, max_point=0, min_point=0;
    u16 FF_MAX_I=0, FF_MIN_I=0;
    static u8 frist_in = 0;
    long int FF_VAL, FF_MAX_VAL=0, FF_MIN_VAL=10000, MAX=0, MIN=0;
    long int ADC_ic_ff;
    int	buchang_X;
    u16 max = 0, min = 65535;
    u32 add, IV_signal_guangqiang=0;
    //	  float integral = 0;   //����
    //	  float S_wuxishou = 0;   //������ʱ�ź������	
    float jun_f[ad_point + san_num];
    
    // ����150�����ݣ�����sanjiao������
    if (frist_in == 0)
    {
        frist_in = 1;
        for(i=0;i<san_num;i++)  
            sanjiao[i] = sin((2 * pi * ((float)(i+1)/san_num)) + (0.5f * pi));
    }	

    // ad_val������8000��ֵ���ȼ�������Ϊ500������16��ֵȡƽ������jun���飬ѭ��500�Σ�
    for (i=0; i<ad_point; i++)
    {
        add = 0;
        for(j=0; j<AD_CY; j++)
        {
            add += ad_val[j*(ad_point) + i];
        }
        jun[i] = add / (AD_CY);
    }	

    // jun���鹲650��������jun�����ǰ150����������jun����ĺ�150������
    for (i=ad_point; i<ad_point + san_num; i++)
    {
        jun[i]=jun[i-ad_point];
    }	

    // ��jun[450 550]֮��������ֵ����Сֵ���Լ����Ӧ�ĵ�
    for(i=ad_point - 50; i<ad_point +50; i++)
    {
        if(jun[i] > max)    
        {  
            max = jun[i]; 
            max_point=i; 
        }
        if(jun[i] < min)    
        {  
            min = jun[i];  
            min_point=i; 
        }
    }					
    IV_signal_Max = max;
    IV_signal_guangqiang = max-min;
    adc_ff = (u16)(((3.3 / 4095) * (float)max)*1000) ;   //��ǿ	

    // û���׸ú���������
    if ((max_point >(ad_point-10)) && (max_point < ad_point) && (min_point >( ad_point-1)) && (min_point <( ad_point+10)))
    {
        // IV_signal_guangqiang = jun[ad_point-1]-jun[0];
        // adc_ff = (u16)(((3.3 / 4095) * (float)max)*1000) ;   //��ǿ	
        for(i=0; i<ad_point + san_num; i++) //ȡ���㣬��һ��
        {
            jun[i] -= min;
            if(jun[i] <= 0)  
                jun[i] = 0;
            jun_f[i] = (float)jun[i]*100 / (float)(max - min);
        }
        integral=0;
        for(i=2; i<ad_point; i++)
        {
            integral += jun_f[i];	
        }
        ND_integral = 50000 - integral;					
    }
    else
    {
        // adc_ff = 9999;
        // IV_signal_guangqiang = 0;
        S_wuxishou = 0;
        ND_integral = biaoding[0];
        // ND_integral = 0;
    }
		
    if (flag_H_D != 0)            //��Ũ�ȶ���
    {
        switch(flag_H_D)
        {
            case 2:  biaoding[0] = ND_integral;
                     a10.f1 = biaoding[0];
                     printf("%f\r\n",biaoding[0]);
                        break;               
            case 4:  biaoding[1] = ND_integral;
                     a11.f1 = biaoding[1];
                     printf("%f\r\n",biaoding[1]);
                        break;               
            case 6:  biaoding[2] = ND_integral;
                     a12.f1 = biaoding[2];
                     printf("%f\r\n",biaoding[2]);
                        break;     
            case 8:  biaoding[3] = ND_integral;
                     a13.f1 = biaoding[3];
                     printf("%f\r\n",biaoding[3]);
                        break;             
            case 10: biaoding[4] = ND_integral;
                     a14.f1 = biaoding[4];
                     printf("%f\r\n",biaoding[4]);
                        break;	
        }
        
        flag_H_D = 0;
        Parameter_saving(); //������Parameter_saving��������Ҫ����ʲô����
        printf("Parameter_saving_ok\r\n");
    }
		
    //ģ�������洢AD_sanjiao[]������ֵ�����Ǻ�ȷ��AD_sanjiaoֵ�ĺ��壩
    for (i=0; i<ad_point; i++)	            
    {
        AD_sanjiao[i] = 0;
        k = 0;
        for(j=i; j<san_num+i; j++, k++)
        {
            AD_sanjiao[i] += (jun[j] * sanjiao[k]);
        }
    }
    
//  for(i=0;i<ad_point;i++)                             //ģ����
//  {
//	  printf("%f\n", AD_sanjiao[i]);
//	}
		
//		for(i=0;i<ad_point;i++)    Data_Send_Status((int32_t)AD_sanjiao[i],(int32_t)jun[i],0);
//		for(i=0;i<100;i++)         Data_Send_Status(0,0,0);
	
    //��AD_sanjiao[380 500]�в���������Сֵ���ֱ�ֵ��MAX��MIN
    for (i=GQ_POINT; i<ad_point; i++)             //2f���ֵ
    {
        if(AD_sanjiao[i] > MAX)  
        {
            MAX = (long int)(AD_sanjiao[i]);
        }
        if(AD_sanjiao[i] < MIN)
        {
            MIN = (long int)(AD_sanjiao[i]);
        } 
    }
    ADC_ic_ff = MAX - MIN;                      //2f���ֵ	   ��ǿ
    if (ADC_ic_ff <= 0)   
    {
        ADC_ic_ff = 0;
    }
    //		adc_ff = (u32)ADC_ic_ff;
    //   	adc_ff = (u16)(((3.3 / 4095) * (float)ADC_ic_ff)*1000) ;   //��ǿ		

    // ��AD_sanjiao[100 380]�в������ֵ����ֵ��FF_MAX_VAL, ��Ӧ��λ�ø�ֵ��FF_MAX_I
    for(i=100; i<GQ_POINT; i++)                //��Ч����2f���ֵ����Сֵ
    {
        if (AD_sanjiao[i] > FF_MAX_VAL)  
        {
            FF_MAX_VAL = AD_sanjiao[i];
            FF_MAX_I = i;
        } 
    }
    bc_point = FF_MAX_I;        //���Ĳ�����λ��

    // ��AD_sanjiao[100 FF_MAX_I]�в�����Сֵ����ֵ��FF_MIN_VAL, ��Ӧ��λ�ø�ֵ��FF_MAX_I
    for(i=100; i<FF_MAX_I; i++)
    {
        if(AD_sanjiao[i] < FF_MIN_VAL)
        {
            FF_MIN_VAL = AD_sanjiao[i];
            FF_MIN_I = i;
        } 
    }
    bc_point_i = FF_MIN_I;
    FF_VAL = FF_MAX_VAL - FF_MIN_VAL;

    if (FF_VAL<=0)
    {
         FF_VAL = 0;
    }
     
    
    //�Զ��Ҳ���	
    if (flag_lock_wavelength == 1)
    {	
        tem_V = 0;			//�ر��¶Ȳ���
    //	if((((float)FF_VAL/(float)ADC_ic_ff)*50000) >= 1000 && FF_MAX_I >=290 && FF_MAX_I <= 310)  //�ҵ�����λ��
        if ((((float)FF_VAL / (float)IV_signal_guangqiang)*180) >= 400 && FF_MAX_I >=290 && FF_MAX_I <= 310)  //�ҵ�����λ��
        {
            Parameter_saving();
            runing_mode = 1;
            delay_ms(1000);
            lock_wavelength =0;
        }
        else                                        
        {	
            wavelength += 0.005;            //0.002
            Dac2_Set_Vol(wavelength + zhi + tem_V );	
    //		nongdu = 9999;
    //		adc_ff = 9999;
            JSON_REPLY_DATA_TEM();
            delay_us(1);
            if(wavelength >=2.500)           //0.900
            {
                wavelength = 1.0000;        //0.4
                Dac2_Set_Vol(wavelength + zhi);
                delay_ms(1000);
            }
        }
    }	

    if (FF_MAX_I - FF_MIN_I >= 45 && FF_MAX_I - FF_MIN_I <= 70)                      
    {
        flag1 = 1;	    //��������ƥ��
    }
    
    if (FF_MAX_I >= 100 && FF_MAX_I <= 380)
    {
        flag2 = 1;      //����λ��ƥ�� 
    }
        
    if (flag2 == 1)                               
    {
        //nongdu =(u16)(((float)FF_VAL/(float)ADC_ic_ff)*50000);
        nongdu =(u16)(((float)FF_VAL*180/(float)IV_signal_guangqiang));			

        //���Բ���
        buchang_X = (FF_MAX_I - 300);
        if(buchang_X > 0)    
            nongdu = nongdu * (1-(buchang_X * 0.003));
        else               
            nongdu = nongdu * (1-(buchang_X * 0.0045));
    }
    else    
        nongdu = 0;
}


double    nongdu_val;
u16 BIAODING_H(float num)
{
	u32 return_val;

	if(num <= biaoding[1])   return_val = (u32)((float)((num - biaoding[0]) * (biaoding_val[1] - biaoding_val[0])) / (float)(biaoding[1] - biaoding[0])) + biaoding_val[0];
	else if(num <= biaoding[2])   return_val = (u32)((float)((num - biaoding[1]) * (biaoding_val[2] - biaoding_val[1])) / (float)(biaoding[2] - biaoding[1])) + biaoding_val[1];
	else if(num <= biaoding[3])   return_val = (u32)((float)((num - biaoding[2]) * (biaoding_val[3] - biaoding_val[2])) / (float)(biaoding[3] - biaoding[2])) + biaoding_val[2];
	else if(num <= biaoding[4])   return_val = (u32)((float)((num - biaoding[3]) * (biaoding_val[4] - biaoding_val[3])) / (float)(biaoding[4] - biaoding[3])) + biaoding_val[3];
    else                          return_val = 1000000;           
	nongdu_val = (double)return_val;
	return return_val;
}

u32 BIAODING_L(u16 val)
{
	nongdu_val = (intercept + (B1* val) + (B2*val*val) + (B3*val*val*val))*Calibration_coefficient;
	if(nongdu_val < 1000)     
    {
        nongdu_val = 0;
    }
//	nongdu_val = nongdu_val / 10000.0f; 
	return (u32)nongdu_val;
}


// �¶Ȳ���
void tem_buchang(void)
{
    if(Tem_coefficient_h > 0.008 || Tem_coefficient_h < 0.0001)	    
    {
        Tem_coefficient_h = 0.0035;
    }
    if(Tem_coefficient_l > 0.008 || Tem_coefficient_l < 0.001)	  
    {
        Tem_coefficient_l = 0.0035;
    }
	
	HP203B_Get_Data_Press_Temp_Altitude(&Hp203bPressures,&Hp203bAltitudes,&Hp203bTemp,ID);  // ��ȡ�¶� ѹǿ
	AHT20_Read_CTdata_crc();     //CRCУ��󣬶�ȡAHT20���¶Ⱥ�ʪ������
	if(Hp203bPressures != 0)    //��⴫��������
	{
        TEM_math();		
	}
	
	if((Hp203bTemp - 25) >= 0)          
    {
        nongdu_val = nongdu_val *(1.0 + ((Hp203bTemp - 25.0f) * Tem_coefficient_h)); //���²���
    }
	else if((25 - Hp203bTemp) >= 0)        
    {
        nongdu_val = nongdu_val *(1.0-((25.0f - Hp203bTemp) * Tem_coefficient_l));   //���²���
    }	
}

// 
void PID_point(void)
{
	if (nongdu >= 600 && lock_wavelength == 0 )
	{
		if (bc_point < 290 && bc_point > 100)
		{
			tem_V -= 0.0008;
		}
		else if (bc_point > 310 && bc_point < 400)
		{
			tem_V += 0.0008;		
		}
	}
	Dac2_Set_Vol(wavelength + zhi + tem_V);		
}


int hex_to_doc(u16 num)
{
	int count1=0;
	int count2=0;
	num=(u8)num;
	switch(num >> 4)
	{
		case 0:count1=0; break;
		case 1:count1= 1; break;
		case 2:count1= 2; break;
		case 3:count1= 3; break;
		case 4:count1= 4; break;
		case 5:count1= 5; break;
		case 6:count1= 6; break;
		case 7:count1= 7; break;
		case 8:count1= 8; break;
		case 9:count1= 9; break;
		case 10:count1=10; break;
		case 11:count1=11; break;
		case 12:count1=12; break;
		case 13:count1=13; break;
		case 14:count1=14; break;
		case 15:count1=15; break;	
	}
	switch(num&0x0f)
	{
		case 0:count2=0; break;
		case 1:count2= 1; break;
		case 2:count2= 2; break;
		case 3:count2= 3; break;
		case 4:count2= 4; break;
		case 5:count2= 5; break;
		case 6:count2= 6; break;
		case 7:count2= 7; break;
		case 8:count2= 8; break;
		case 9:count2= 9; break;
		case 10:count2=10; break;
		case 11:count2=11; break;
		case 12:count2=12; break;
		case 13:count2=13; break;
		case 14:count2=14; break;
		case 15:count2=15; break;	
	}	
	
	return (count1*16+count2);
}


u16 num_to_res(int num)
{
	u16 count_res=0;
	count_res=num*390;
	return count_res;
}


u8 XOR_VAL;
u8 send_buf[51];
extern u8 count;	
void data_send(void)
{
    u8 i;
    int TEM1,TEM2,HUM,PRE;
    u32 ch4_val;
    u8 con= 0;
    u8 mcp4017_iv=0;
    u16 mcp4017_res_iv=0;
    ch4_val =(u32)nongdu_val;	 
    if (ch4_val > 1000000)	   
    {
        ch4_val = 1000000;
    }
	
	TEM1 = (int)(Hp203bTemp * 10);
	PRE = (int)(Hp203bPressures * 100);
	
	TEM2 = (int)(AHT_temp * 10);
	HUM  = (int)(AHT_wet * 10);

    if (adc_ff <= 7000)
    {
        ERROR_CODE = 1;
    }
    else if((TEM1==0 && PRE==0)||(HUM==0 && TEM2==0))
    {
        ERROR_CODE = 2;	
    }   
    else 
    {
        ERROR_CODE = 0;
    }
    
	send_buf[con++] = 'A';  //֡ͷ
	send_buf[con++] = '+';  //Ũ������
	send_buf[con++] = '0'; 	
//	send_buf[con++] = (ch4_val / 1000000) + 0x30;               //ʮλ��
	send_buf[con++] = (ch4_val / 100000) + 0x30;               //ʮλ��
	send_buf[con++] = (ch4_val / 10000) % 10 + 0x30;             //��λ��
	send_buf[con++] = '.';
	send_buf[con++] = (ch4_val / 1000) % 10 + 0x30;	             //ʮ��λ
    send_buf[con++] = (ch4_val / 100) % 10 + 0x30;
    
	send_buf[con++] = ' ';  //�ո�
	
	if(TEM1<0)   //�¶�1
    {
        send_buf[con++]  = '-';
        TEM1 = -TEM1;
    }  
	else         
    {
        send_buf[con++]  = '+';
    }
	send_buf[con++] = (TEM1 / 100) + 0x30;
	send_buf[con++] = (TEM1 / 10) % 10 + 0x30;
	send_buf[con++] = '.';
	send_buf[con++] = (TEM1 % 10) + 0x30;
	
	send_buf[con++] = ' ';  //�ո� 
	
    send_buf[con++] = (PRE / 100000) + 0x30;   //ѹ��
	send_buf[con++] = (PRE / 10000) % 10 + 0x30;
	send_buf[con++] = (PRE / 1000) % 10 + 0x30;
	send_buf[con++] = (PRE / 100) % 10 + 0x30;
	send_buf[con++] = '.';
	send_buf[con++] = (PRE / 10) % 10 + 0x30;
	send_buf[con++] = (PRE % 10) + 0x30;
	
	send_buf[con++] = ' ';  //�ո�

	if(TEM2<0)    //�¶�2
    {
        send_buf[con++]  = '-';
        TEM2 = -TEM2;
    }  
	else         
    {
        send_buf[con++]  = '+';
    }
	send_buf[con++] = (TEM2 / 100) + 0x30;
	send_buf[con++] = (TEM2 / 10) % 10 + 0x30;
	send_buf[con++] = '.';
	send_buf[con++] = (TEM2 % 10) + 0x30;	
	
	send_buf[con++] = ' ';  //�ո�
	send_buf[con++] = 'B';

	if(HUM < 0)    //ѹ��
    {
        send_buf[con++] = '-';
        HUM = -HUM;
    } 
	else           
    {
        send_buf[30] = '+';
    }                  
	send_buf[con++] = (HUM / 1000) + 0x30;
	send_buf[con++] = (HUM / 100) % 10 + 0x30;
	send_buf[con++] = (HUM / 10) % 10 + 0x30;
	send_buf[con++] = '.';
	send_buf[con++] = (HUM % 10) + 0x30;
	
	send_buf[con++] = ' ';  //�ո�
	
	send_buf[con++] = '0';  //������
	send_buf[con++] = ERROR_CODE + 0x30;  
	
	send_buf[con++] = ' ';  //�ո�	
	
	XOR_VAL = XOR_check(send_buf, con);
	switch(XOR_VAL >> 4)
	{
		case 0:send_buf[con++] = 0 +0x30; break;
		case 1:send_buf[con++] = 1 +0x30; break;
		case 2:send_buf[con++] = 2 +0x30; break;
		case 3:send_buf[con++] = 3 +0x30; break;
		case 4:send_buf[con++] = 4 +0x30; break;
		case 5:send_buf[con++] = 5 +0x30; break;
		case 6:send_buf[con++] = 6 +0x30; break;
		case 7:send_buf[con++] = 7 +0x30; break;
		case 8:send_buf[con++] = 8 +0x30; break;
		case 9:send_buf[con++] = 9 +0x30; break;
		case 10:send_buf[con++] = 0x41; break;
		case 11:send_buf[con++] = 0x42; break;
		case 12:send_buf[con++] = 0x43; break;
		case 13:send_buf[con++] = 0x44; break;
		case 14:send_buf[con++] = 0x45; break;
		case 15:send_buf[con++] = 0x46; break;
        default:send_buf[con++] = 0x2F; break;		
	}
	switch(XOR_VAL & 0x0f)
	{
		case 0:send_buf[con++] = 0 +0x30; break;
		case 1:send_buf[con++] = 1 +0x30; break;
		case 2:send_buf[con++] = 2 +0x30; break;
		case 3:send_buf[con++] = 3 +0x30; break;
		case 4:send_buf[con++] = 4 +0x30; break;
		case 5:send_buf[con++] = 5 +0x30; break;
		case 6:send_buf[con++] = 6 +0x30; break;
		case 7:send_buf[con++] = 7 +0x30; break;
		case 8:send_buf[con++] = 8 +0x30; break;
		case 9:send_buf[con++] = 9 +0x30; break;
		case 10:send_buf[con++] = 0x41; break;
		case 11:send_buf[con++] = 0x42; break;
		case 12:send_buf[con++] = 0x43; break;
		case 13:send_buf[con++] = 0x44; break;
		case 14:send_buf[con++] = 0x45; break;
		case 15:send_buf[con++] = 0x46; break;	
        default:send_buf[con++] = 0x2F; break;		
	}	
	
//	send_buf[con++] = 0X0D; //�س�����
//	send_buf[con++] = 0X0A;		
	send_buf[42] = ' ';  //�ո�
	mcp4017_iv = hex_to_doc(MCP4017_Read_IV());
	mcp4017_res_iv = num_to_res(mcp4017_iv);
	send_buf[43] = mcp4017_res_iv/10000+0x30;
	send_buf[44] = mcp4017_res_iv%10000/1000+0x30;
	send_buf[45] = mcp4017_res_iv%1000/100+0x30;
	send_buf[46] = mcp4017_res_iv%100/10+0x30;
	send_buf[47] = mcp4017_res_iv%10+0x30;
	send_buf[48] = ' ';  //�ո�
	send_buf[49] = 0X0D; //�س�����
	send_buf[50] = 0X0A;		
    USART_GetFlagStatus(USART1, USART_FLAG_TC);	
	for(i=0; i<51; i++)
	{
		USART_SendData(USART1, send_buf[i]);                    //�򴮿�1��������
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);  //�ȴ����ͽ���	
	}
	
	ERROR_CODE = 0;	
	Hp203bTemp=0;
	Hp203bPressures=0;
	AHT_temp=0;
	AHT_wet=0;	
}


u8 XOR_check(u8 *buff,u8 length)
{
	u8 i;
	u8 val;
    val = *(buff) ^ *(buff+1);
	for(i=2; i<length-1 ;i++)
	{
		val ^= *(buff+i);
	}
	return val;
}


void Parameter_saving(void)
{
	u16 time[2];
    
	mcp4017_step = hex_to_doc(mcp4017_value);
	STMFLASH_Read(FLASH_SAVE_ADDR, time, 2);
	a1.s1 = time[0] << 16 | time[1];

	a2.f1 = wavelength;
    a3.f1 = intercept;
	a4.f1 = B1;
    a5.f1 = B2;
	a6.f1 = B3;	
	a7.f1 = Tem_coefficient_h;
	a8.f1 = Tem_coefficient_l;
	
	Parameter_save[0] = a1.s1 >> 16;
	Parameter_save[1] = a1.s1;
	Parameter_save[2] = a2.s1 >> 16;
	Parameter_save[3] = a2.s1;
	Parameter_save[4] = a3.s1 >> 16;
	Parameter_save[5] = a3.s1;
	Parameter_save[6] = a4.s1 >> 16;
	Parameter_save[7] = a4.s1;
	Parameter_save[8] = a5.s1 >> 16;
	Parameter_save[9] = a5.s1;
	Parameter_save[10] = a6.s1 >> 16;
	Parameter_save[11] = a6.s1;	
	Parameter_save[12] = a7.s1 >> 16;
	Parameter_save[13] = a7.s1;	
	Parameter_save[14] = a8.s1 >> 16;
	Parameter_save[15] = a8.s1;		
	Parameter_save[16] = a9.s1 >> 16;
	Parameter_save[17] = a9.s1;
	Parameter_save[18] = a10.s1 >> 16;
	Parameter_save[19] = a10.s1;
	Parameter_save[20] = a11.s1 >> 16;
	Parameter_save[21] = a11.s1;
	Parameter_save[22] = a12.s1 >> 16;
	Parameter_save[23] = a12.s1;
	Parameter_save[24] = a13.s1 >> 16;
	Parameter_save[25] = a13.s1;
	Parameter_save[26] = a14.s1 >> 16;
	Parameter_save[27] = a14.s1;	
	Parameter_save[28] = mcp4017_step>>16;
	Parameter_save[29] = mcp4017_step;
	STMFLASH_Write(FLASH_SAVE_ADDR, (u16*)Parameter_save, 30);
}

