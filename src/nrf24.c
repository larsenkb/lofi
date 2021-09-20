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
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <util/delay_basic.h>
#include "nrf24.h"

extern uint8_t gstatus;


void spi_init(void)
{
	DDRA |= (1<<CSN);	// OUTPUT
    DDRA |= (1<<SCK);	// OUTPUT
    DDRA |= (1<<MOSI);	// OUTPUT
    DDRA &= ~(1<<MISO);	// INPUT
    DDRA |= (1<<CE);	// OUTPUT
	DEASSERT_CE();
	DEASSERT_CSN();
	PORTA &= ~(1<<SCK);

}

/* software spi routine */
uint8_t spi_transfer(uint8_t tx)
{
	register uint8_t i = 0;
    register uint8_t rx = 0;    

    for (i=0x80; i; i>>=1) {

        if (tx & i) {
			PORTA |= (1<<MOSI);
        } else {
			PORTA &= ~(1<<MOSI);
        }

	    PORTA |= (1<<SCK);

		if (PINA & (1<<MISO)) {
            rx |= i;
        }

	    PORTA &= ~(1<<SCK);

    }
    return rx;
}


void nrfInit(void) 
{
    spi_init();
}


/* configure the module */
void nrfConfig(config_t *config, uint8_t pay_length)
{
	uint8_t tval;

    // Set RF channel
    nrfWriteReg(RF_CH, config->rf_chan);

	// Set length of incoming payload 
    // Use static payload length ...
	///KBL TODO: only write next reg if en_aa set???
	nrfWriteReg(RX_PW_P0, pay_length); // Auto-ACK pipe ...
    nrfWriteReg(RX_PW_P1, pay_length); // Data payload pipe

    // configure RF speed and power gain
	tval = (config->rf_gain & 0x3) << 1;
	if (config->spd_1M) {
		// do nothing
	} else if (config->spd_250K) {
		tval |= 0x20;
	} else {
		tval |= 0x08;
	}
	nrfWriteReg(RF_SETUP, tval | 0);

    // Auto Acknowledgment
	if (config->en_aa) {
		nrfWriteReg(EN_AA,(1<<ENAA_P0)|(0<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));
		tval = config->nodeId & 0x0F;
		if (tval == 0) tval = 3;
		tval <<= 4;
		tval |= config->setup_retr & 0x0F;
		//nrfWriteReg(SETUP_RETR, tval);
		nrfWriteReg(SETUP_RETR,config->setup_retr);
	} else {
		nrfWriteReg(EN_AA, 0);
		nrfWriteReg(SETUP_RETR, 0);
	}

    // Enable RX addresses
    nrfWriteReg(EN_RXADDR,(1<<ERX_P0)|(1<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));

	// Dynamic NO ACK feature
	if (config->en_aa && config->en_dyn_ack) {
		nrfWriteReg(FEATURE, 0x01);
	} else {
		nrfWriteReg(FEATURE, 0x00);
	}
}

/* Clocks only one byte into the given nrf24 register */
void nrfWriteReg(uint8_t reg, uint8_t value)
{
	ASSERT_CSN();
	gstatus = spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
	spi_transfer(value);
	DEASSERT_CSN();
}

void nrfFlushTx(void)
{
	// Write cmd to flush transmit FIFO
	ASSERT_CSN();
	gstatus = spi_transfer(FLUSH_TX);     
	DEASSERT_CSN();
}

// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void nrfFillTxFifo(config_t *config, uint8_t *buf, uint8_t buf_length) 
{    
	uint8_t i;

	// Write payload cmd and write payload
	ASSERT_CSN();
	if (config->en_aa && config->en_dyn_ack) {
	    gstatus = spi_transfer(W_TX_PAYLOAD_NOACK);
	} else {
		gstatus = spi_transfer(W_TX_PAYLOAD);
	}
	spi_transfer(config->nodeId);
	for (i=0; i<buf_length; i++) {
		spi_transfer(buf[i]);
	}
	DEASSERT_CSN();
}


void nrfPulseCE(void)
{
	// Start the transmission
	ASSERT_CE();
//	_NOP();
//	_NOP();
//	_NOP();
//	_NOP();
//	_NOP();
//	_NOP();
//	_NOP();
//	_NOP();
//	_NOP();
//	_NOP();
	_delay_loop_1(5);
	DEASSERT_CE();
}

// returns 1 (true) if still sending; i.e. TX_DS nor MAX_RT has been asserted
// or IRQn has not been asserted, if polling
uint8_t nrfIsSending()
{
#if EN_IRQ_POLL
	return ((1<<2) == (PINB & (1<<2)));
#else
	// read the current status
	gstatus = nrfGetStatus();
	return (0 == (gstatus & ((1 << TX_DS) | (1 << MAX_RT))));        
#endif
}


uint8_t nrfGetStatus(void)
{
	ASSERT_CSN();
	gstatus = spi_transfer(NOP);
	DEASSERT_CSN();
	return gstatus;
}

void nrfPowerUpTx(void)
{
	nrfWriteReg(CONFIG, nrf24_CONFIG | ((1<<PWR_UP) | (0<<PRIM_RX)));
}

/* Read single register from nrf24 */
uint8_t nrfReadReg(uint8_t reg)
{
	uint8_t rv = 0;

	ASSERT_CSN();
	gstatus = spi_transfer(reg & REGISTER_MASK);
	rv = spi_transfer(rv);
	DEASSERT_CSN();
	return rv;
}

/* Read multiple register(s) from nrf24 */
void nrfReadRegs(uint8_t reg, uint8_t *value, uint8_t len)
{
	uint8_t i;

	ASSERT_CSN();
	gstatus = spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
	for (i=0; i<len; i++) {
		value[i] = spi_transfer(value[i]);
	}
	DEASSERT_CSN();
}

void nrfPowerDown()
{
	nrfWriteReg(CONFIG, nrf24_CONFIG);
}
