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
#include <util/delay.h>
#include "nrf24.h"
#include "lofi.h"


#if 1
void spi_init(void)
{
#undef MOSI
#undef MISO
#define MOSI 5
#define MISO 6
    DDRA |= (1<<CSN);	// OUTPUTs
    DDRA |= (1<<SCK);	// OUTPUTs
    DDRA |= (1<<MOSI);	// OUTPUTs
    DDRA |= (1<<CE);	// OUTPUTs
    DDRA &= ~(1<<MISO);	// INPUT
	DEASSERT_CE();
	DEASSERT_CSN();
	PORTA &= ~(1<<SCK);

//	USICR = (1<<USIWM0) | (1<<USICS1) | (1<<USICLK);
}
#if 1
uint8_t spi_transfer(uint8_t _data)
{
	USIDR = _data;
	USISR = (1<<USIOIF);

	while ((USISR & (1<<USIOIF)) == 0) {
		USICR = (1<<USIWM0) | (1<<USICS1) | (1<<USICLK) | (1<<USITC);
	}
	return USIDR;
}
#endif
#if 0
uint8_t spi_transfer(uint8_t _data)
{
	USIDR = _data;
	USISR = (1<<USIOIF);

	do {
		USICR = (1<<USIWM0) | (1<<USICS1) | (1<<USICLK) | (1<<USITC);
	} while ((USISR & (1<<USIOIF)) == 0);
	return USIDR;
}
#endif
#if 0
uint8_t spi_transfer(uint8_t _data)
{
	register uint8_t r16 = (1<<USIWM0) | (0<<USICS0) | (1<<USITC);
	register uint8_t r17 = (1<<USIWM0) | (0<<USICS0) | (1<<USICLK) | (1<<USITC);
	USIDR = _data;

	// Set Divide by 8 for 8MHz RC oscillator 
//	CLKPR = (1<<CLKPCE);
//	CLKPR = 1;

	USICR = r16;
	USICR = r17;
	USICR = r16;
	USICR = r17;
	USICR = r16;
	USICR = r17;
	USICR = r16;
	USICR = r17;
	USICR = r16;
	USICR = r17;
	USICR = r16;
	USICR = r17;
	USICR = r16;
	USICR = r17;
	USICR = r16;
	USICR = r17;
	// Set Divide by 8 for 8MHz RC oscillator 
//	CLKPR = (1<<CLKPCE);
//	CLKPR = 3;
	return USIDR;
}
#endif
#endif

#if 0  // this bit-bang method works
void spi_init(void)
{
#undef MOSI
#undef MISO
#define MOSI 5
#define MISO 6
    DDRA |= (1<<CSN);	// OUTPUT
    DDRA |= (1<<SCK);	// OUTPUT
    DDRA |= (1<<MOSI);	// OUTPUT
    DDRA &= ~(1<<MISO);	// INPUT
    DDRA |= (1<<CE);	// OUTPUT
	DEASSERT_CE();
	DEASSERT_CSN();
	PORTA &= ~(1<<SCK);

//	USICR = (1<<USIWM0) | (1<<USICS1) | (1<<USICLK);
}
/* software spi routine */
#if 0
uint8_t spi_transfer(uint8_t tx)
{
    uint8_t i = 0;
    uint8_t rx = 0;    

	PORTA &= ~(1<<SCK);

    for (i=0; i<8; i++) {

        if (tx & (1<<(7-i))) {
			PORTA |= (1<<MOSI);
        } else {
			PORTA &= ~(1<<MOSI);
        }

	    PORTA |= (1<<SCK);

        rx = rx << 1;
#if 0
		rx |= (PINA>>MISO) & 1;
#else
		if (PINA & (1<<MISO)) {
            rx |= 0x01;
        }
#endif
	    PORTA &= ~(1<<SCK);

    }
    return rx;
}
#else
uint8_t spi_transfer(uint8_t tx)
{
#if 0
//	register uint8_t i = 0x80;
//    register uint8_t rx = 0;    
	USICR = 0x10;

	USIDR = tx;

	PORTA |= (1<<SCK);
	if (PINA & (1<<MISO)) USIDR |= 1;
    PORTA &= ~(1<<SCK);
	USIDR <<= 1;

	PORTA |= (1<<SCK);
	if (PINA & (1<<MISO)) USIDR |= 1;
    PORTA &= ~(1<<SCK);
	USIDR <<= 1;

	PORTA |= (1<<SCK);
	if (PINA & (1<<MISO)) USIDR |= 1;
    PORTA &= ~(1<<SCK);
	USIDR <<= 1;

	PORTA |= (1<<SCK);
	if (PINA & (1<<MISO)) USIDR |= 1;
    PORTA &= ~(1<<SCK);
	USIDR <<= 1;

	PORTA |= (1<<SCK);
	if (PINA & (1<<MISO)) USIDR |= 1;
    PORTA &= ~(1<<SCK);
	USIDR <<= 1;

	PORTA |= (1<<SCK);
	if (PINA & (1<<MISO)) USIDR |= 1;
    PORTA &= ~(1<<SCK);
	USIDR <<= 1;

	PORTA |= (1<<SCK);
	if (PINA & (1<<MISO)) USIDR |= 1;
    PORTA &= ~(1<<SCK);
	USIDR <<= 1;

	PORTA |= (1<<SCK);
	if (PINA & (1<<MISO)) USIDR |= 1;
    PORTA &= ~(1<<SCK);

    return USIDR;
//    return rx;

#else
	register uint8_t i = 0;
    register uint8_t rx = 0;    

//	PORTA &= ~(1<<SCK);

    for (i=0x80; i; i>>=1) {

        if (tx & i) {
			PORTA |= (1<<MOSI);
        } else {
			PORTA &= ~(1<<MOSI);
        }

	    PORTA |= (1<<SCK);

//        rx <<= 1;

		if (PINA & (1<<MISO)) {
//            rx |= 0x01;
            rx |= i;
        }

	    PORTA &= ~(1<<SCK);

    }
    return rx;
#endif
}
#endif
#endif


void nrf24_init(void) 
{
    spi_init();
}


/* configure the module */
void nrf24_config(uint8_t channel, uint8_t pay_length, uint8_t speed_1M, uint8_t rf_gain)
{

    // Set RF channel
    nrf24_configRegister(RF_CH, channel);

#if 1
	// Set length of incoming payload 
    // Use static payload length ...
	nrf24_configRegister(RX_PW_P0, pay_length); // Auto-ACK pipe ...
    nrf24_configRegister(RX_PW_P1, pay_length); // Data payload pipe
#endif
#if 0
    nrf24_configRegister(RX_PW_P2, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P3, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P4, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P5, 0x00); // Pipe not used 
#endif

    // configure RF speed and power gain
	if (speed_1M)
		nrf24_configRegister(RF_SETUP, 0x00 | ((rf_gain & 0x3) << 1));
	else
		nrf24_configRegister(RF_SETUP, 0x08 | ((rf_gain & 0x3) << 1));

    // CRC enable, 1 byte CRC length
//    nrf24_configRegister(CONFIG, nrf24_CONFIG);

    // Auto Acknowledgment
	// I think this is for PTX only
//    nrf24_configRegister(EN_AA,0);
#if EN_ENH_SWAVE
	nrf24_configRegister(EN_AA,(1<<ENAA_P0)|(1<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));
    nrf24_configRegister(SETUP_RETR,(0x00<<ARD)|(0x01<<ARC));
#else
	nrf24_configRegister(EN_AA, 0);
    nrf24_configRegister(SETUP_RETR, 0);
#endif

    // Enable RX addresses
    nrf24_configRegister(EN_RXADDR,(1<<ERX_P0)|(1<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));

#if 0
	// turn OFF auto retransmit
    nrf24_configRegister(SETUP_RETR,0);
	// Auto retransmit delay: 1000 us and Up to 15 retransmit trials
    nrf24_configRegister(SETUP_RETR,(0x01<<ARD)|(0x02<<ARC));
//    nrf24_configRegister(SETUP_RETR,(0x04<<ARD)|(0x0F<<ARC));
#endif

	// enable W_TX_PAYLOAD_NOACK feature
    //nrf24_configRegister(0x1d,1);

    // Dynamic length configurations: No dynamic length
//    nrf24_configRegister(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));
}


/* Clocks only one byte into the given nrf24 register */
void nrf24_configRegister(uint8_t reg, uint8_t value)
{
    ASSERT_CSN();
    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_transfer(value);
	DEASSERT_CSN();
}


void nrf24_powerUpRx()
{     
	ASSERT_CSN();
    spi_transfer(FLUSH_RX);
	DEASSERT_CSN();

    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 

    DEASSERT_CE();
    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(1<<PRIM_RX)));    
    ASSERT_CE();
}


// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void nrf24_send(uint8_t* value, uint8_t pay_length) 
{    
     
    /* Set to transmitter mode , Power up if needed */
    nrf24_powerUpTx();

    /* Do we really need to flush TX fifo each time ? */
#if 1
    /* Write cmd to flush transmit FIFO */
    ASSERT_CSN();
    spi_transfer(FLUSH_TX);     
    DEASSERT_CSN();
#endif 

    /* Write payload cmd and write payload */
	ASSERT_CSN();
    spi_transfer(W_TX_PAYLOAD);
    nrf24_transmitSync(value, pay_length);   
    DEASSERT_CSN();

}

void nrf24_pulseCE(void)
{
	/* Start the transmission */
	ASSERT_CE();
	_NOP();
	_NOP();
//	_NOP();
//	_NOP();
//	_NOP();
//	_NOP();
//	_NOP();
	_NOP();
	_NOP();
	_NOP();
	DEASSERT_CE();
}

uint8_t nrf24_isSending()
{
    uint8_t status;

    /* read the current status */
    status = nrf24_getStatus();
                
    /* if sending successful (TX_DS) or max retries exceded (MAX_RT). */
    if ((status & ((1 << TX_DS)  | (1 << MAX_RT)))) {        
        return 0; /* false */
    }

    return 1; /* true */

}

uint8_t nrf24_getStatus(void)
{
    uint8_t rv;

	ASSERT_CSN();
    rv = spi_transfer(NOP);
	DEASSERT_CSN();
    return rv;
}


void nrf24_powerUpTx(void)
{
    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 
    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(0<<PRIM_RX)));
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
#if 1
	uint8_t spiBuf[2];

	spiBuf[0] = reg & 0x1f;
	spiBuf[1] = 0;
    ASSERT_CSN();
	nrf24_transferSync(spiBuf, spiBuf, 2);
    DEASSERT_CSN();
	return spiBuf[1];
#else
	uint8_t val;

    ASSERT_CSN();
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    val = spi_transfer(0);
    DEASSERT_CSN();
    return val;
#endif
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
    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 
    nrf24_configRegister(CONFIG, nrf24_CONFIG);
}


#if 0
/* Returns the number of retransmissions occured for the last message */
uint8_t nrf24_retransmissionCount()
{
    uint8_t rv;
    nrf24_readRegister(OBSERVE_TX, &rv, 1);
    rv = rv & 0x0F;
    return rv;
}

uint8_t nrf24_lastMessageStatus()
{
    uint8_t rv;

    rv = nrf24_getStatus();

    /* Transmission went OK */
    if ((rv & ((1 << TX_DS)))) {
        return NRF24_TRANSMISSON_OK;
    }
    /* Maximum retransmission count is reached */
    /* Last message probably went missing ... */
    else if ((rv & ((1 << MAX_RT)))) {
        return NRF24_MESSAGE_LOST;
    }  
    /* Probably still sending ... */
    else {
        return 0xFF;
    }
}
#endif



