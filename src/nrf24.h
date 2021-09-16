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

#define CE				0		/* PORTA */
#define CSN				1		/* PORTA */
#define SCK				4		/* PORTA */
#define MOSI			6		/* PORTA */
#define MISO			5		/* PORTA */
#define ASSERT_CE()     (PORTA |= (1<<CE))
#define DEASSERT_CE()   (PORTA &= ~(1<<CE))
#define DEASSERT_CSN()  (PORTA |= (1<<CSN))
#define ASSERT_CSN()    (PORTA &= ~(1<<CSN))

#define nrf24_ADDR_LEN 5
#define nrf24_CONFIG ((1<<MASK_RX_DR) | (1<<MASK_TX_DS) | (1<<MASK_MAX_RT) | (1<<EN_CRC) | (1<<CRCO))


void    nrfInit(void);
void    nrfConfig(config_t *config, uint8_t pay_length);
uint8_t nrfIsSending(void);
uint8_t nrfGetStatus(void);
void	nrfFlushTx(void);
void	nrfClearStatus(void);
void    nrfFillTxFifo(config_t *config, uint8_t *buf, uint8_t buf_length);
void    nrfPulseCE(void);
void    nrfPowerUpTx(void);
void    nrfPowerDown(void);
void    spi_init(void);
uint8_t spi_transfer(uint8_t tx);
void    nrfReadRegs(uint8_t reg, uint8_t* value, uint8_t len);
void    nrfWriteReg(uint8_t reg, uint8_t value);
uint8_t nrfReadReg(uint8_t reg);

#endif /* __NRF24_H__ */
