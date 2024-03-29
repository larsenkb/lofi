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

static uint8_t nrf24_CONFIG;

static uint8_t gstatus;

static uint8_t (*spi_xfer)(uint8_t tx);

/* software spi routine */
uint8_t spi_transfer_USI(uint8_t tx)
{
	register uint8_t i = (uint8_t)0x11;
    register uint8_t o = (uint8_t)0x13;    

	USIDR = (uint8_t)tx;

	USICR = i;
	USICR = o;
	USICR = i;
	USICR = o;
	USICR = i;
	USICR = o;
	USICR = i;
	USICR = o;
	USICR = i;
	USICR = o;
	USICR = i;
	USICR = o;
	USICR = i;
	USICR = o;
	USICR = i;
	USICR = o;
	return USIDR;

}

uint8_t spi_transfer_BB(uint8_t tx)
{
	register uint8_t i;
    register uint8_t rx = tx;    

	//USIDR = (uint8_t)tx;
    for (i = 0x80; i; i >>= 1) {

        if (tx & i) {
			ASSERT_MOSI();
        } else {
			DEASSERT_MOSI();
        }

		ASSERT_SCK();

		if (PINA & MISO) {
            rx |= i;
        }

		DEASSERT_SCK();

    }
    return rx;
}

void spi_init(void)
{
	DDRA |= CSN;		// OUTPUT
    DDRA |= SCK;		// OUTPUT
    DDRA |= CE;			// OUTPUT
	DEASSERT_CE();
	DEASSERT_CSN();
	DEASSERT_SCK();
	if (PWB_REV == 5 || PWB_REV == 6) {
		spi_xfer = spi_transfer_USI;
		DDRA |= (1<<5);		// OUTPUT
		DDRA &= ~(1<<6);		// INPUT
		USICR = 0x10;
		DDRA &= ~(1<<2);	// IRQ
	} else {
		spi_xfer = spi_transfer_BB;
		DDRA |= MOSI;		// OUTPUT
		DDRA &= ~MISO;		// INPUT
	}
}

void nrfInit(void) 
{
    spi_init();

	if (PWB_REV == 5) {
		nrf24_CONFIG = ((1<<MASK_RX_DR) | (0<<MASK_TX_DS) | (0<<MASK_MAX_RT) | (1<<EN_CRC) | (1<<CRCO));
	} else {
		nrf24_CONFIG = ((1<<MASK_RX_DR) | (1<<MASK_TX_DS) | (1<<MASK_MAX_RT) | (1<<EN_CRC) | (1<<CRCO));
	}
}


/* configure the module */
void nrfConfig(config_t *config, uint8_t pay_length)
{
	uint8_t tval;
	uint8_t pipe;

	// calculate pipe number this node is...
	pipe = config->nodeId % 6;  // remainder 0-5
#if 1
	// initialize pipe addresses in config if not done already
	if (config->p0Addr[0] == 0xFF) {
		config->p0Addr[0] = 0xE7;
		config->p0Addr[1] = 0xE7;
		config->p0Addr[2] = 0xE7;
		config->p0Addr[3] = 0xE7;
		config->p0Addr[4] = 0xE7;
	}
	if (config->p15Addr[0] == 0xFF) {
		config->p15Addr[0] = 0xC2;
		config->p15Addr[1] = 0xC2;
		config->p15Addr[2] = 0xC2;
		config->p15Addr[3] = 0xC2;
		config->p15Addr[4] = 0xC2;
		config->pxAddr[0] = 0xC3;
		config->pxAddr[1] = 0xC4;
		config->pxAddr[2] = 0xC5;
		config->pxAddr[3] = 0xC6;
	}
#endif
	nrfWriteRegs(RX_ADDR_P1, config->p15Addr, 5); 
    nrfWriteReg(RX_ADDR_P2, config->pxAddr[0]);
    nrfWriteReg(RX_ADDR_P3, config->pxAddr[1]);
    nrfWriteReg(RX_ADDR_P4, config->pxAddr[2]);
    nrfWriteReg(RX_ADDR_P5, config->pxAddr[3]);

	if (pipe > 1)
		config->p15Addr[4] = config->pxAddr[pipe-2];

	if (pipe == 0) {
		nrfWriteRegs(TX_ADDR, config->p0Addr, 5); 
		nrfWriteRegs(RX_ADDR_P0, config->p0Addr, 5); 
	} else {
		nrfWriteRegs(TX_ADDR, config->p15Addr, 5); 
		nrfWriteRegs(RX_ADDR_P0, config->p15Addr, 5); 
	}

    // Set RF channel
    nrfWriteReg(RF_CH, config->rf_chan);

	// Set length of incoming payload 
    // Use static payload length ...
	///KBL TODO: only write next reg if en_aa set???
	nrfWriteReg(RX_PW_P0, pay_length); // Auto-ACK pipe ...
	nrfWriteReg(RX_PW_P1, 0); // Auto-ACK pipe ...
	nrfWriteReg(RX_PW_P2, 0); // Auto-ACK pipe ...
	nrfWriteReg(RX_PW_P3, 0); // Auto-ACK pipe ...
	nrfWriteReg(RX_PW_P4, 0); // Auto-ACK pipe ...
	nrfWriteReg(RX_PW_P5, 0); // Auto-ACK pipe ...
//    nrfWriteReg(RX_PW_P1, 0); //pay_length); // Data payload pipe

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
#if 1
//		nrfWriteReg(EN_AA,(1<<ENAA_P0));
		nrfWriteReg(EN_AA, 1);
		tval = (config->nodeId & 0xF);
		tval ^= ((config->nodeId>>4) & 0x0F);
		if (tval == 0) tval = 5;
		tval |= (config->setup_retr & 0xF0);
		//nrfWriteReg(SETUP_RETR, tval);
		nrfWriteReg(SETUP_RETR,config->setup_retr);
#else
		nrfWriteReg(EN_AA,(1<<ENAA_P0)|(0<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));
		tval = config->nodeId & 0x0F;
		if (tval == 0) tval = 3;
		tval <<= 4;
		tval |= config->setup_retr & 0x0F;
		//nrfWriteReg(SETUP_RETR, tval);
		nrfWriteReg(SETUP_RETR,config->setup_retr);
#endif
	} else {
		nrfWriteReg(EN_AA, 0);
		nrfWriteReg(SETUP_RETR, 0);
	}

    // Enable RX addresses
//    nrfWriteReg(EN_RXADDR,(1<<ERX_P0)|(1<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));
    nrfWriteReg(EN_RXADDR, 1);

	// Dynamic NO ACK feature
	if (config->en_aa && config->en_dyn_ack) {
		nrfWriteReg(FEATURE, 0x01);
	} else {
		nrfWriteReg(FEATURE, 0x00);
	}
}

/* Clocks only one byte into the given nrf24 register */
uint8_t nrfWriteReg(uint8_t reg, uint8_t value)
{
	ASSERT_CSN();
	gstatus = (*spi_xfer)(W_REGISTER | (REGISTER_MASK & reg));
	(*spi_xfer)(value);
	DEASSERT_CSN();
	return gstatus;
}

// Write to a multi-byte register
uint8_t nrfWriteRegs(uint8_t reg, uint8_t *buf, uint8_t buf_length)
{
	int8_t i;

	ASSERT_CSN();
	gstatus = (*spi_xfer)(W_REGISTER | (REGISTER_MASK & reg));
	for (i=4; i >= 0; i--) 
		(*spi_xfer)(buf[i]);
	DEASSERT_CSN();
	return gstatus;
}

void nrfFlushTx(void)
{
	// Write cmd to flush transmit FIFO
	ASSERT_CSN();
	gstatus = (*spi_xfer)(FLUSH_TX);     
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
	    gstatus = (*spi_xfer)(W_TX_PAYLOAD_NOACK);
	} else {
		gstatus = (*spi_xfer)(W_TX_PAYLOAD);
	}
	(*spi_xfer)(config->nodeId);
	for (i=0; i<buf_length; i++) {
		(*spi_xfer)(buf[i]);
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
	if (PWB_REV == 5) {
		return ((1<<2) == (PINA & (1<<2)));
	} else {
		// read the current status
		gstatus = nrfGetStatus();
		return (0 == (gstatus & ((1<<TX_DS) | (1<<MAX_RT))));
	}
}


uint8_t nrfGetStatus(void)
{
	ASSERT_CSN();
	gstatus = (*spi_xfer)(NOP);
	DEASSERT_CSN();
	return gstatus;
}

uint8_t nrfRetransmit(void)
{
	ASSERT_CSN();
	gstatus = (*spi_xfer)(REUSE_TX_PL);
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
	gstatus = (*spi_xfer)(reg & REGISTER_MASK);
	rv = (*spi_xfer)(rv);
	DEASSERT_CSN();
	return rv;
}

/* Read multiple register(s) from nrf24 */
void nrfReadRegs(uint8_t reg, uint8_t *value, uint8_t len)
{
	uint8_t i;

	ASSERT_CSN();
	gstatus = (*spi_xfer)(R_REGISTER | (REGISTER_MASK & reg));
	for (i=0; i<len; i++) {
		value[i] = (*spi_xfer)(value[i]);
	}
	DEASSERT_CSN();
}

void nrfPowerDown()
{
	nrfWriteReg(CONFIG, nrf24_CONFIG);
}
