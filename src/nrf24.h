/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* This library is based on this library: 
*   https://github.com/aaronds/arduino-nrf24l01
* Which is based on this library: 
*   http://www.tinkerer.eu/AVRLib/nRF24L01
* -----------------------------------------------------------------------------
*  Modified by: Kent B. Larsen
*/
#ifndef __NRF24_H__
#define __NRF24_H__

#include "nRF24L01.h"
#include <stdint.h>
#include "lofi.h"

#define LOW 0
#define HIGH 1

#undef MOSI_PIN
#undef MISO_PIN

#define CE_PIN			0
#define CSN_PIN			1
#define SCK_PIN			4
#define MOSI_PIN		6
#define MISO_PIN		5

#define CE				(1<<CE_PIN)		/* PORTA */
#define CSN				(1<<CSN_PIN)	/* PORTA */
#define SCK				(1<<SCK_PIN)	/* PORTA */
#define MOSI			(1<<MOSI_PIN)	/* PORTA */
#define MISO			(1<<MISO_PIN)	/* PORTA */
#define ASSERT_CE()     (PORTA |= CE)
#define DEASSERT_CE()   (PORTA &= ~CE)
#define DEASSERT_CSN()  (PORTA |= CSN)
#define ASSERT_CSN()    (PORTA &= ~CSN)
#define ASSERT_SCK()	(PORTA |= SCK)
#define DEASSERT_SCK()	(PORTA &= ~SCK)
#define ASSERT_MOSI()	(PORTA |= MOSI)
#define DEASSERT_MOSI()	(PORTA &= ~MOSI)


#define nrf24_ADDR_LEN 5

void    nrfInit(void);
void    nrfConfig(config_t *config, uint8_t pay_length);
uint8_t nrfIsSending(void);
uint8_t nrfGetStatus(void);
void	nrfFlushTx(void);
void    nrfFillTxFifo(config_t *config, uint8_t *buf, uint8_t buf_length);
void    nrfPulseCE(void);
void    nrfPowerUpTx(void);
void    nrfPowerDown(void);
void    spi_init(void);
void    nrfReadRegs(uint8_t reg, uint8_t* value, uint8_t len);
uint8_t nrfWriteRegs(uint8_t reg, uint8_t* value, uint8_t len);
uint8_t nrfWriteReg(uint8_t reg, uint8_t value);
uint8_t nrfReadReg(uint8_t reg);
uint8_t nrfRetransmit(void);

#endif /* __NRF24_H__ */
