#ifndef __OURS_H
#define __OURS_H	 
#include "sys.h"


#define     AD_CY         16
#define     ad_point      500
#define     ad_cnt        8000            //总采集点数

#define     san_num       150 
#define     count_cum     15


typedef union
{
	float f1;
	u32  s1;
}F32_save;

extern u32 ad_val[ad_cnt];                    //ad_cnt = 8000，存放信号采集电路数据
//extern u16 ad_val_2[ad_cnt];

//extern u16 adc1_val[2000];
//extern u16 adc2_val[2000];

extern float    Calibration_coefficient;  
extern float    wavelength;    
extern float	intercept;
extern float	B1;
extern float    B2;
extern float    B3;
extern float 	Tem_coefficient_h;           //温度补偿系数
extern float 	Tem_coefficient_l;           //温度补偿系数  
           
extern u8       flag_H_D;
extern float    ND_integral;
extern float    integral;                    //积分
extern float    S_wuxishou;                  //无吸收时信号面积。	
extern float    biaoding[5];
extern u32      biaoding_val[5];
extern double   BD_ARRAY[4];
extern float    zhi;

extern u8 flag_json_rx;
extern u16 Parameter_save[30];
extern u16 FIT_VAL;
extern u32 adc_ff,IV_signal_Max;
extern u16 bc_point,bc_point_i;
extern double   nongdu_val;
extern u16 nongdu;
extern u32 mcp4017_value;
extern u8 adc_ff_flag;                          //判断光强值反馈标志
extern u16 mcp4017_step;

void Saw_Generation_init(void);
void saw_Generation(void);
void syetem_init(void);
void ad_math(void);
void RCC_Configuration(void);
u8 XOR_check(u8 *buff,u8 length);
void Parameter_saving(void);
u16 BIAODING_H(float num);
u32 BIAODING_L(u16 val);
void data_send(void);
void CLOSE_LD(void);
void sys_int(void);
void tem_buchang(void);
void PID_point(void);
void close_Peripherals(void);
void Parameter_read(void);
void TEM_math(void);
int hex_to_doc(u16 num);
#endif

