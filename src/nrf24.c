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


void nrf24_init(void) 
{
    spi_init();
}


/* configure the module */
void nrf24_config(config_t *config, uint8_t pay_length)
{
	uint8_t tval;

    // Set RF channel
    nrf24_configRegister(RF_CH, config->rf_chan);

	// Set length of incoming payload 
    // Use static payload length ...
	///KBL TODO: only write next reg if en_aa set???
	nrf24_configRegister(RX_PW_P0, pay_length); // Auto-ACK pipe ...
    nrf24_configRegister(RX_PW_P1, pay_length); // Data payload pipe

    // configure RF speed and power gain
	tval = (config->rf_gain & 0x3) << 1;
	if (config->spd_1M) {
		// do nothing
	} else if (config->spd_250K) {
		tval |= 0x20;
	} else {
		tval |= 0x08;
	}
	nrf24_configRegister(RF_SETUP, tval | 0);

    // CRC enable, 1 byte CRC length
//    nrf24_configRegister(CONFIG, nrf24_CONFIG);

    // Auto Acknowledgment
	if (config->en_aa) {
		nrf24_configRegister(EN_AA,(1<<ENAA_P0)|(1<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));
		nrf24_configRegister(SETUP_RETR,config->setup_retr);
	} else {
		nrf24_configRegister(EN_AA, 0);
		nrf24_configRegister(SETUP_RETR, 0);
	}

    // Enable RX addresses
    nrf24_configRegister(EN_RXADDR,(1<<ERX_P0)|(1<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));

	// Dynamic NO ACK feature
	if (config->en_aa && config->en_dyn_ack) {
		nrf24_configRegister(FEATURE, 0x01);
	} else {
		nrf24_configRegister(FEATURE, 0x00);
	}
}

/* Clocks only one byte into the given nrf24 register */
void nrf24_configRegister(uint8_t reg, uint8_t value)
{
	ASSERT_CSN();
	spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
	spi_transfer(value);
	DEASSERT_CSN();
}

void nrf24_flush_tx(void)
{
	// Write cmd to flush transmit FIFO
	ASSERT_CSN();
	spi_transfer(FLUSH_TX);     
	DEASSERT_CSN();
}

// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void nrf24_send(config_t *config, uint8_t *buf, uint8_t buf_length) 
{    
	// Write payload cmd and write payload
	ASSERT_CSN();
	if (config->en_aa && config->en_dyn_ack) {
	    spi_transfer(W_TX_PAYLOAD_NOACK);
	} else {
		spi_transfer(W_TX_PAYLOAD);
	}
	spi_transfer(config->nodeId);
	nrf24_transmitSync(buf, buf_length);   
	DEASSERT_CSN();
}


void nrf24_pulseCE(void)
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


uint8_t nrf24_isSending()
{
	// read the current status
	gstatus = nrf24_getStatus();
 
	// if sending successful (TX_DS) or max retries exceded (MAX_RT).
	if (gstatus & ((1 << TX_DS) | (1 << MAX_RT))) {        
		return 0; // false
	}

	return 1; // true
}


uint8_t nrf24_getStatus(void)
{
	uint8_t rv;

	ASSERT_CSN();
	rv = spi_transfer(NOP);
	DEASSERT_CSN();
	return rv;
}

void nrf24_clearStatus(void)
{
	nrf24_configRegister(STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT)); 
}

void nrf24_powerUpTx(void)
{
//	nrf24_configRegister(STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT)); 
	nrf24_configRegister(CONFIG, nrf24_CONFIG | ((1<<PWR_UP) | (0<<PRIM_RX)));
}


/* send multiple bytes over SPI */
void nrf24_transmitSync(uint8_t *dataout, uint8_t len)
{
	uint8_t i;
 
	for (i=0; i<len; i++) {
		spi_transfer(dataout[i]);
	}
}


/* Read single register from nrf24 */
uint8_t nrf24_rdReg(uint8_t reg)
{
	uint8_t spiBuf[2];

	spiBuf[0] = reg & 0x1f;
	spiBuf[1] = 0;
	ASSERT_CSN();
	nrf24_transferSync(spiBuf, spiBuf, 2);
	DEASSERT_CSN();
	return spiBuf[1];
}

/* Read multiple register(s) from nrf24 */
void nrf24_readRegister(uint8_t reg, uint8_t *value, uint8_t len)
{
	ASSERT_CSN();
	spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
	nrf24_transferSync(value,value,len);
	DEASSERT_CSN();
}

/* send and receive multiple bytes over SPI */
void nrf24_transferSync(uint8_t *dataout, uint8_t *datain, uint8_t len)
{
	uint8_t i;

	for (i=0; i<len; i++) {
		datain[i] = spi_transfer(dataout[i]);
	}
}

void nrf24_powerDown()
{
//	nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 
	nrf24_configRegister(CONFIG, nrf24_CONFIG);
}
