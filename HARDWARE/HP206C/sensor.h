#ifndef __SENSOR_H
#define	__SENSOR_H

#include "sys.h"  

#define HP303B_DEV_ADDR           0xee                     //地址(设备地址:与A2、A1、A0有关)
#define EEPROM_WR                 0x00                     //写
#define EEPROM_RD                 0x01                     //读

/* 函数申明 ------------------------------------------------------------------*/
uint8_t HP303B_IIC_Write(uint8_t IIC_Address,uint8_t *buffer,uint16_t count);
uint8_t HP303B_IIC_Read(uint8_t IIC_Address,uint8_t *buffer,uint16_t count);
uint8_t HP303B_IIC_WriteSingle(uint8_t IIC_Address,uint8_t buffer);
uint8_t HP303B_bIICBurstWrite(uint8_t chip, uint8_t addr, uint8_t ptr[], uint8_t length);
uint8_t HP303B_bIICBurstRead(uint8_t chip, uint8_t addr, uint8_t ptr[], uint8_t length);

#endif
