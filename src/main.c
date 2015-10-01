#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
//#include "softuart.h"
#include "nRF24L01.h"

#undef F_CPU
#define F_CPU 1000000UL

#include <util/delay.h>

#define EN_WD			1

#define LED_RED			0		/* PORTB */
#define LED_GREEN		1		/* PORTB */

#define CE				0		/* PORTA */
#define CSN				1		/* PORTA */
#define SCK				4		/* PORTA */
#define MOSI			6		/* PORTA */
#define MISO			5		/* PORTA */
#define ASSERT_CE()     (PORTA |= (1<<CE))
#define DEASSERT_CE()   (PORTA &= ~(1<<CE))
#define DEASSERT_CSN()  (PORTA |= (1<<CSN))
#define ASSERT_CSN()    (PORTA &= ~(1<<CSN))
#define ASSERT_GRNLED()	(PORTB |= (1<<LED_GREEN))
#define DEASSERT_GRNLED() (PORTB &= ~(1<<LED_GREEN))
#define ASSERT_REDLED()	(PORTB |= (1<<LED_RED))
#define DEASSERT_REDLED() (PORTB &= ~(1<<LED_RED))

#define nrf24_CONFIG   (1<<EN_CRC)
//#define nrf24_CONFIG   ((1<<EN_CRC) | (1<<CRCO))


/* ------------------------------------------------------------------------- */
volatile uint8_t wdInt;
//uint8_t temp;
uint8_t q = 0;
uint8_t data_array[4];
//uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
//uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
/* ------------------------------------------------------------------------- */


void spi_init(void);
uint8_t spi_transfer(uint8_t _data);
void nrf24_init(void);
void nrf24_config(uint8_t channel, uint8_t pay_length);
void nrf24_configRegister(uint8_t reg, uint8_t value);
void nrf24_powerUpRx(void);
void nrf24_send(uint8_t* value, uint8_t pay_length);
uint8_t nrf24_isSending(void);
uint8_t nrf24_getStatus(void);
void nrf24_powerUpTx(void);
void nrf24_transmitSync(uint8_t* dataout,uint8_t len);
void nrf24_readRegister(uint8_t reg, uint8_t* value, uint8_t len);
void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len);

void uartbb_init(void);
void uartbb_puthex( uint8_t data );
void uartbb_puts( const char *s );
void uartbb_putchar(char val);

//****************************************************************  
// set system into the sleep state 
// system wakes up when watchdog times out
void system_sleep(void)
{

//  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out 
//  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON

}

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii)
{
	uint8_t bb;

    wdInt = 0;

	if (ii > 9 )
		ii=9;
	bb=ii & 7;
	if (ii > 7)
		bb |= (1<<5);
	bb |= (1<<WDCE);


	MCUSR &= ~(1<<WDRF);
	// start timed sequence
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	// set new watchdog timeout value
	WDTCSR = bb;
	WDTCSR |= _BV(WDIE);
}

//This runs each time the watch dog wakes us up from sleep
ISR(WDT_vect)
{
    wdInt = 1;
//  watchdog_counter++;
}

int main(void)
{


	/* Perform system initialization */
	uartbb_init();

	/* init LED pins as OUTPUT */
//	DDRB |= (1<<LED_RED) | (1<<LED_GREEN);
	DDRB |= (1<<LED_GREEN);


	/* init SPI */
	spi_init();

	/* init hardware pins for talking to radio */
    DDRA |= (1<<CE);	// OUTPUT
	DEASSERT_CE();
	nrf24_init();
    
	/* Set the device addresses */
//	nrf24_tx_address(tx_address);
//	nrf24_rx_address(rx_address);    

	/* Channel 0x4c , payload length: 4 */
	nrf24_config(2, 4);

#if EN_WD
	setup_watchdog(7);
#endif

  /* Enable interrupts */
#if 1 //EN_WD
    sei();
#endif

    uartbb_puts("\r\n");

	while (1) {
        uint8_t rv;
  
#if EN_WD
        system_sleep();
#endif

        if (wdInt) {
            wdInt = 0;
#if 1
            nrf24_readRegister(0,&rv,1);
            uartbb_puthex(rv);
//	        uartbb_puthex(nrf24_getStatus());
#else
		    uartbb_putchar('a');
		    uartbb_putchar('b');
		    uartbb_putchar('c');
#endif

            /* Fill the data buffer */
		    data_array[0] = 0x00;
		    data_array[1] = 0xAA;
		    data_array[2] = 0x55;
		    data_array[3] = q++;                                    
#if 1
		    /* Automatically goes to TX mode */
		    nrf24_send(data_array, 4);        
        
		    /* Wait for transmission to end */
//		    while (nrf24_isSending());

		    /* Make analysis on last tranmission attempt */
//		    temp = nrf24_lastMessageStatus();

		    /* Retranmission count indicates the tranmission quality */
//		    temp = nrf24_retransmissionCount();
//		    xprintf("> Retranmission count: %d\r\n",temp);

		    /* Optionally, go back to RX mode ... */
//		    nrf24_powerUpRx();

		    /* Or you might want to power down after TX */
		    // nrf24_powerDown();            
#endif

            ASSERT_GRNLED();
            _delay_ms(1);
            DEASSERT_GRNLED();
        }
		/* Wait a little ... */
#if EN_WD==0
		_delay_ms(1000);
#endif

    }
    return 0;
}


void spi_init(void)
{
    DDRA |= (1<<CSN);	// OUTPUT
    DDRA |= (1<<SCK);	// OUTPUT
    DDRA |= (1<<MOSI);	// OUTPUT
    DDRA &= ~(1<<MISO);	// INPUT
	DEASSERT_CSN();
	PORTA &= ~(1<<SCK);

	USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK);
}

#if 0
#if 1
uint8_t spi_transfer(uint8_t _data)
{
  USIDR = _data;
  uint8_t i;
  
  for (i=0; i<16; i++) {
    USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
  }
  return USIDR;
}
#else
uint8_t spi_transfer(uint8_t _data)
{
  USIDR = _data;
  USISR = _BV(USIOIF);
  
  while((USISR & _BV(USIOIF)) == 0){
    USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
  }
  return USIDR;
}
#endif
#else
/* software spi routine */
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
        if (PINA & (1<<MISO)) {
            rx |= 0x01;
        }

	    PORTA &= ~(1<<SCK);

    }
    return rx;
}
#endif



void nrf24_init() 
{
//    nrf24_setupPins();
//    nrf24_ce_digitalWrite(LOW);
//    nrf24_csn_digitalWrite(HIGH);    
}


/* configure the module */
void nrf24_config(uint8_t channel, uint8_t pay_length)
{
    /* Use static payload length ... */
//    payload_len = pay_length;

    // Set RF channel
//    nrf24_configRegister(RF_CH, channel);

    // Set length of incoming payload 
#if 0
	nrf24_configRegister(RX_PW_P0, 0x00); // Auto-ACK pipe ...
    nrf24_configRegister(RX_PW_P1, payload_len); // Data payload pipe
    nrf24_configRegister(RX_PW_P2, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P3, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P4, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P5, 0x00); // Pipe not used 
#else
	nrf24_configRegister(RX_PW_P0, pay_length); // Auto-ACK pipe ...
    nrf24_configRegister(RX_PW_P1, pay_length); // Data payload pipe
    nrf24_configRegister(RX_PW_P2, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P3, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P4, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P5, 0x00); // Pipe not used 
#endif

    // 250Kbps, TX gain: 0dbm
//    nrf24_configRegister(RF_SETUP, (2<<RF_DR) | ((0x03)<<RF_PWR));

    // CRC enable, 1 byte CRC length
//    nrf24_configRegister(CONFIG, nrf24_CONFIG);

    // Auto Acknowledgment
//    nrf24_configRegister(EN_AA,(1<<ENAA_P0)|(1<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));

    // Enable RX addresses
//    nrf24_configRegister(EN_RXADDR,(1<<ERX_P0)|(1<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));

    // Auto retransmit delay: 1000 us and Up to 15 retransmit trials
    nrf24_configRegister(SETUP_RETR,(0x04<<ARD)|(0x0F<<ARC));

    // Dynamic length configurations: No dynamic length
//    nrf24_configRegister(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));

    // Start listening
//    nrf24_powerUpRx();
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
//	int i;

    /* Go to Standby-I first */
	DEASSERT_CE();
     
    /* Set to transmitter mode , Power up if needed */
    nrf24_powerUpTx();

    /* Do we really need to flush TX fifo each time ? */
#if 1
        /* Pull down chip select */
    ASSERT_CSN();

    /* Write cmd to flush transmit FIFO */
    spi_transfer(FLUSH_TX);     

    /* Pull up chip select */
    DEASSERT_CSN();
#endif 

    /* Pull down chip select */
	ASSERT_CSN();

    /* Write cmd to write payload */
    spi_transfer(W_TX_PAYLOAD);

    /* Write payload */
    nrf24_transmitSync(value, pay_length);   

    /* Pull up chip select */
	DEASSERT_CSN();

    /* Start the transmission */
	ASSERT_CE();

//	DEASSERT_CE();
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
void nrf24_transmitSync(uint8_t* dataout, uint8_t len)
{
    uint8_t i;
    
    for (i=0; i<len; i++) {
        spi_transfer(dataout[i]);
    }
}


/* Read single register from nrf24 */
void nrf24_readRegister(uint8_t reg, uint8_t* value, uint8_t len)
{
    ASSERT_CSN();
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    nrf24_transferSync(value,value,len);
    DEASSERT_CSN();
}

/* send and receive multiple bytes over SPI */
void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len)
{
    uint8_t i;

    for (i=0; i<len; i++) {
        datain[i] = spi_transfer(dataout[i]);
    }
}
#if 0

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
#include "nrf24.h"

uint8_t payload_len;

/* init the hardware pins */
void nrf24_init() 
{
    nrf24_setupPins();
    nrf24_ce_digitalWrite(LOW);
    nrf24_csn_digitalWrite(HIGH);    
}

/* configure the module */
void nrf24_config(uint8_t channel, uint8_t pay_length)
{
    /* Use static payload length ... */
    payload_len = pay_length;

    // Set RF channel
    nrf24_configRegister(RF_CH, channel);

    // Set length of incoming payload 
#if 0
	nrf24_configRegister(RX_PW_P0, 0x00); // Auto-ACK pipe ...
    nrf24_configRegister(RX_PW_P1, payload_len); // Data payload pipe
    nrf24_configRegister(RX_PW_P2, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P3, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P4, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P5, 0x00); // Pipe not used 
#else
	nrf24_configRegister(RX_PW_P0, payload_len); // Auto-ACK pipe ...
    nrf24_configRegister(RX_PW_P1, payload_len); // Data payload pipe
    nrf24_configRegister(RX_PW_P2, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P3, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P4, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P5, 0x00); // Pipe not used 
#endif

    // 250Kbps, TX gain: 0dbm
//    nrf24_configRegister(RF_SETUP, (2<<RF_DR) | ((0x03)<<RF_PWR));

    // CRC enable, 1 byte CRC length
//    nrf24_configRegister(CONFIG, nrf24_CONFIG);

    // Auto Acknowledgment
//    nrf24_configRegister(EN_AA,(1<<ENAA_P0)|(1<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));

    // Enable RX addresses
//    nrf24_configRegister(EN_RXADDR,(1<<ERX_P0)|(1<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));

    // Auto retransmit delay: 1000 us and Up to 15 retransmit trials
    nrf24_configRegister(SETUP_RETR,(0x04<<ARD)|(0x0F<<ARC));

    // Dynamic length configurations: No dynamic length
//    nrf24_configRegister(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));

    // Start listening
    nrf24_powerUpRx();
}

/* Set the RX address */
void nrf24_rx_address(uint8_t * adr) 
{
    nrf24_ce_digitalWrite(LOW);
    nrf24_writeRegister(RX_ADDR_P1,adr,nrf24_ADDR_LEN);
    nrf24_ce_digitalWrite(HIGH);
}

/* Returns the payload length */
uint8_t nrf24_payload_length()
{
    return payload_len;
}

/* Set the TX address */
void nrf24_tx_address(uint8_t* adr)
{
    /* RX_ADDR_P0 must be set to the sending addr for auto ack to work. */
    nrf24_writeRegister(RX_ADDR_P0,adr,nrf24_ADDR_LEN);
    nrf24_writeRegister(TX_ADDR,adr,nrf24_ADDR_LEN);
}

/* Checks if data is available for reading */
/* Returns 1 if data is ready ... */
uint8_t nrf24_dataReady() 
{
    // See note in getData() function - just checking RX_DR isn't good enough
    uint8_t status = nrf24_getStatus();

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << RX_DR) ) {
        return 1;
    }

    return !nrf24_rxFifoEmpty();;
}

/* Checks if receive FIFO is empty or not */
uint8_t nrf24_rxFifoEmpty()
{
    uint8_t fifoStatus;

    nrf24_readRegister(FIFO_STATUS,&fifoStatus,1);
    
    return (fifoStatus & (1 << RX_EMPTY));
}

/* Returns the length of data waiting in the RX fifo */
uint8_t nrf24_payloadLength()
{
    uint8_t status;
    nrf24_csn_digitalWrite(LOW);
    spi_transfer(R_RX_PL_WID);
    status = spi_transfer(0x00);
    nrf24_csn_digitalWrite(HIGH);
    return status;
}

/* Reads payload bytes into data array */
void nrf24_getData(uint8_t* data) 
{
    /* Pull down chip select */
    nrf24_csn_digitalWrite(LOW);                               

    /* Send cmd to read rx payload */
    spi_transfer( R_RX_PAYLOAD );
    
    /* Read payload */
    nrf24_transferSync(data,data,payload_len);
    
    /* Pull up chip select */
    nrf24_csn_digitalWrite(HIGH);

    /* Reset status register */
    nrf24_configRegister(STATUS,(1<<RX_DR));   
}

/* Returns the number of retransmissions occured for the last message */
uint8_t nrf24_retransmissionCount()
{
    uint8_t rv;
    nrf24_readRegister(OBSERVE_TX,&rv,1);
    rv = rv & 0x0F;
    return rv;
}

// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void nrf24_send(uint8_t* value, uint8_t pay_length) 
{    
    /* Go to Standby-I first */
    nrf24_ce_digitalWrite(LOW);
     
    /* Set to transmitter mode , Power up if needed */
    nrf24_powerUpTx();

    /* Do we really need to flush TX fifo each time ? */
#if 1
        /* Pull down chip select */
        nrf24_csn_digitalWrite(LOW);           

        /* Write cmd to flush transmit FIFO */
        spi_transfer(FLUSH_TX);     

        /* Pull up chip select */
        nrf24_csn_digitalWrite(HIGH);                    
#endif 

    /* Pull down chip select */
    nrf24_csn_digitalWrite(LOW);

    /* Write cmd to write payload */
    spi_transfer(W_TX_PAYLOAD);

    /* Write payload */
    nrf24_transmitSync(value, pay_length);   

    /* Pull up chip select */
    nrf24_csn_digitalWrite(HIGH);

    /* Start the transmission */
    nrf24_ce_digitalWrite(HIGH);    
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

uint8_t nrf24_getStatus()
{
    uint8_t rv;
    nrf24_csn_digitalWrite(LOW);
    rv = spi_transfer(NOP);
    nrf24_csn_digitalWrite(HIGH);
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

void nrf24_powerUpRx()
{     
    nrf24_csn_digitalWrite(LOW);
    spi_transfer(FLUSH_RX);
    nrf24_csn_digitalWrite(HIGH);

    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 

    nrf24_ce_digitalWrite(LOW);    
    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(1<<PRIM_RX)));    
    nrf24_ce_digitalWrite(HIGH);
}

void nrf24_powerUpTx()
{
    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 

    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(0<<PRIM_RX)));
}

void nrf24_powerDown()
{
    nrf24_ce_digitalWrite(LOW);
    nrf24_configRegister(CONFIG,nrf24_CONFIG);
}

/* software spi routine */
uint8_t spi_transfer(uint8_t tx)
{
    uint8_t i = 0;
    uint8_t rx = 0;    

    nrf24_sck_digitalWrite(LOW);

    for (i=0; i<8; i++) {

        if (tx & (1<<(7-i))) {
            nrf24_mosi_digitalWrite(HIGH);            
        } else {
            nrf24_mosi_digitalWrite(LOW);
        }

        nrf24_sck_digitalWrite(HIGH);        

        rx = rx << 1;
        if (nrf24_miso_digitalRead()) {
            rx |= 0x01;
        }

        nrf24_sck_digitalWrite(LOW);                

    }
    return rx;
}

/* send and receive multiple bytes over SPI */
void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len)
{
    uint8_t i;

    for (i=0; i<len; i++) {
        datain[i] = spi_transfer(dataout[i]);
    }
}

/* send multiple bytes over SPI */
void nrf24_transmitSync(uint8_t* dataout,uint8_t len)
{
    uint8_t i;
    
    for (i=0; i<len; i++) {
        spi_transfer(dataout[i]);
    }
}

/* Clocks only one byte into the given nrf24 register */
void nrf24_configRegister(uint8_t reg, uint8_t value)
{
    nrf24_csn_digitalWrite(LOW);
    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_transfer(value);
    nrf24_csn_digitalWrite(HIGH);
}

/* Read single register from nrf24 */
void nrf24_readRegister(uint8_t reg, uint8_t* value, uint8_t len)
{
    nrf24_csn_digitalWrite(LOW);
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    nrf24_transferSync(value,value,len);
    nrf24_csn_digitalWrite(HIGH);
}

/* Write to a single register of nrf24 */
void nrf24_writeRegister(uint8_t reg, uint8_t* value, uint8_t len) 
{
    nrf24_csn_digitalWrite(LOW);
    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    nrf24_transmitSync(value,len);
    nrf24_csn_digitalWrite(HIGH);
}
#endif




#if 0
disable BOD to save power prior to sleep
disable ADC prior to sleep
first conversion after disable/enable will be an extended conversion
#endif


#define UARTBB_BAUD_RATE		9600

#if 0
#define UARTBB_TXPORT  PORTB
#define UARTBB_TXDDR   DDRB
#define UARTBB_TXBIT   PB2
#define UARTBB_T_COMP_REG        OCR0A
#define UARTBB_T_CONTR_REGA      TCCR0A
#define UARTBB_T_CONTR_REGB      TCCR0B
#define UARTBB_T_CNT_REG         TCNT0
#define UARTBB_T_INTCTL_REG      TIMSK0

#define UARTBB_CMPINT_EN_MASK    (1 << OCIE0A)

#define UARTBB_CTC_MASKA         (1 << WGM01)
#define UARTBB_CTC_MASKB         (0)

    /* "A timer interrupt must be set to interrupt at three times 
       the required baud rate." */
#define UARTBB_PRESCALE (1)

#if (UARTBB_PRESCALE == 8)
    #define UARTBB_PRESC_MASKA         (0)
    #define UARTBB_PRESC_MASKB         (1 << CS01)
#elif (UARTBB_PRESCALE==1)
    #define UARTBB_PRESC_MASKA         (0)
    #define UARTBB_PRESC_MASKB         (1 << CS00)
#else 
    #error "prescale unsupported"
#endif
#endif
#define UARTBB_TXPORT  PORTB
#define UARTBB_TXDDR   DDRB
#define UARTBB_TXBIT   PB2
#define UARTBB_PRESCALE (1)
#define UARTBB_TIMERTOP ( F_CPU/UARTBB_PRESCALE/UARTBB_BAUD_RATE - 1)

#define CLR_TX()  (UARTBB_TXPORT &= ~(1<<UARTBB_TXBIT))
#define SET_TX()  (UARTBB_TXPORT |= (1<<UARTBB_TXBIT))

#define UARTBB_TX_PWR		5

enum { UARTBB_STATE_IDLE = 0,
	   UARTBB_STATE_START,
	   UARTBB_STATE_CHAR,
	   UARTBB_STATE_STOP,
	   UARTBB_STATE_POST
} uartbb_state_t;

char uartbb_tx[1<<UARTBB_TX_PWR];
uint8_t		uartbb_state;
uint8_t		uartbb_cnt;
uint8_t		uartbb_val;
volatile uint8_t		uartbb_tx_ridx;
volatile uint8_t		uartbb_tx_widx;

void uartbb_init(void)
{

	// TX-Pin as output
	UARTBB_TXDDR |=  (1<<UARTBB_TXBIT);
	uartbb_state = UARTBB_STATE_IDLE;
	uartbb_tx_ridx = uartbb_tx_widx = 0;
    SET_TX();

	/* initialize timer */
    OCR0A = UARTBB_TIMERTOP;
    TCNT0 = 0;

    TCCR0A = 0x02;
    TCCR0B = 0x00;

    TIFR0 |= (1<<OCF0A);
    TIMSK0 |= (1<<OCIE0A);
}

ISR( TIM0_COMPA_vect )
{
	switch (uartbb_state) {
	case UARTBB_STATE_START:
		CLR_TX();
		uartbb_state = UARTBB_STATE_CHAR;
		uartbb_cnt = 8;
		uartbb_val = uartbb_tx[uartbb_tx_ridx];
		uartbb_tx_ridx = (uartbb_tx_ridx + 1) & ((1<<UARTBB_TX_PWR) - 1);
		break;
	case UARTBB_STATE_CHAR:
		if (uartbb_val & 0x01) {
			SET_TX();
		} else {
			CLR_TX();
		}
		uartbb_val >>= 1;
		if (--uartbb_cnt == 0) {
			uartbb_state = UARTBB_STATE_STOP;
		}
		break;
	case UARTBB_STATE_STOP:
		SET_TX();
		if (uartbb_tx_ridx != uartbb_tx_widx) {
			uartbb_state = UARTBB_STATE_START;
		} else {
            uartbb_state = UARTBB_STATE_IDLE;
		}
        break;
    case UARTBB_STATE_IDLE:
		/* disable interrupt */
//        TIMSK0 &= ~(1<<OCIE0A);
//	    UARTBB_T_INTCTL_REG &= ~UARTBB_CMPINT_EN_MASK;
		/* stop timer */
        TCCR0B = 0x00;
        break;
	default:
		break;
	}
}

void uartbb_putchar(char val)
{
	uint8_t tidx;


	tidx = (uartbb_tx_widx + 1) & ((1<<UARTBB_TX_PWR)-1);
	if (tidx == uartbb_tx_ridx)
		return;

	uartbb_tx[uartbb_tx_widx] = val;
	uartbb_tx_widx = tidx;

    /* get the ball rolling if we are in idle state */
	if (uartbb_state == UARTBB_STATE_IDLE) {
		uartbb_state = UARTBB_STATE_START;
		/* clear timer */
        TCNT0 = 0;
		/* start timer */
        TCCR0B = 0x01;
	}
}


void uartbb_puts( const char *s )
{
	while ( *s ) {
		uartbb_putchar( *s++ );
	}
}


const char hex[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
void uartbb_puthex( uint8_t data )
{
	uint8_t val = data;

	data >>= 4;
	uartbb_putchar(hex[data]);
	val &= 0xF;
	uartbb_putchar(hex[val]);
}
