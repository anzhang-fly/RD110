#ifndef __SENSOR303_H
#define	__SENSOR303_H

#include "sys.h"

extern uint8_t ID[4];
extern double Hp203bPressures,Hp203bAltitudes,Hp203bTemp;     // ÎÂ¶È Êª¶È ÆøÑ¹

#define HP20X_SOFT_RST    0x06
#define HP20X_ADDRESSCMD  0xec      //csb = VCC

#define HP20XX_RDY        0x8d

#define HP20X_READ_P      0x30      //read_p command
#define HP20X_READ_A      0x31      //read_a command
#define HP20X_READ_T      0x32      //read_t command
#define HP20X_READ_PT     0x10      //read_pt command
#define HP20X_READ_AT     0x11      //read_at command

#define HP20X_WR_CONVERT_CMD   0x40
#define HP20X_CONVERT_OSR4096  0<<2
#define HP20X_CONVERT_OSR2048  1<<2
#define HP20X_CONVERT_OSR1024  2<<2
#define HP20X_CONVERT_OSR512   3<<2
#define HP20X_CONVERT_OSR256   4<<2
#define HP20X_CONVERT_OSR128   5<<2


void HP203_relocation(void);
void Hp203b_read_id(uint8_t buf[]);
void HP203B_Get_Data_Press_Temp_Altitude(double *Hp203bPressure,double *Hp203bAltitude,double *Hp203bTemperature,uint8_t ID[]);

#endif
