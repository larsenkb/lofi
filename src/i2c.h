#ifndef __I2C_H__
#define __I2C_H__

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include <avr/eeprom.h>
#include <avr/cpufunc.h>
#include <stdbool.h>


//extern void I2C_WriteBit(unsigned char c);
//extern unsigned char I2C_ReadBit(void);
extern void I2C_Init(void);
//extern void I2C_Start(void);
//extern void I2C_Stop(void);
//extern bool I2C_Write(unsigned char c);
//extern uint8_t I2C_Read(bool ack);
extern uint32_t readAHT10Temp(void);
extern uint32_t readAHT10Humd(bool humdDone);


#endif // __I2C_H__ 
