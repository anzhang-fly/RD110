#ifndef _BSP_I2C_GPIO_H
#define _BSP_I2C_GPIO_H
#include "sys.h"
#define GPIO_PORT_I2C	GPIOB			/* GPIO端口 */
#define RCC_I2C_PORT 	RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */
#define I2C_SCL_PIN		GPIO_Pin_10			/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		GPIO_Pin_11			/* 连接到SDA数据线的GPIO */
#define I2C_SCL_HIGH    GPIO_SetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 1 */
#define I2C_SCL_LOW     GPIO_ResetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 0 */
#define I2C_SDA_HIGH    GPIO_SetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 1 */
#define I2C_SDA_LOW     GPIO_ResetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 0 */
#define I2C_SDA_READ    GPIO_ReadInputDataBit(GPIO_PORT_I2C, I2C_SDA_PIN)	/* 读SDA口线状态 */
#define I2C_ACK                   0                        //应答
#define I2C_NOACK                 1                        //非应答 

extern void Sensor_I2C_Init(void);
extern void I2C_SDA_SetOutput(void);
extern void I2C_SDA_SetInput(void);
extern void I2C_Start(void);
extern void I2C_Stop(void);
extern void I2C_PutAck(uint8_t Ack);
extern uint8_t I2C_GetAck(void);
extern uint8_t I2C_WriteByte(uint8_t Data);
extern uint8_t I2C_ReadByte(void );
extern void I2C_Delay(void);
#endif
