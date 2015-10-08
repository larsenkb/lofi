#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <string.h>

#include "lofi.h"
#include "nrf24.h"
#include "uartbb.h"
#include "xprintf.h"

#undef F_CPU
#define F_CPU 1000000UL

#define EN_WD			1
#define WD_TO           9
#define WD_TO_SHORT     3
#define EN_SWITCH       1

#define PAYLOAD_LENGTH  8

#define SWITCH_1        2       /* PORTB bit2 */
#define SWITCH_MSK      2
#define SWITCH_GMSK     5


#define LED_RED			0		/* PORTB  bit0 */
#define LED_GREEN		1		/* PORTB  bit1 */


#define ASSERT_GRNLED()	(PORTB |= (1<<LED_GREEN))
#define DEASSERT_GRNLED() (PORTB &= ~(1<<LED_GREEN))
#define ASSERT_REDLED()	(PORTB |= (1<<LED_RED))
#define DEASSERT_REDLED() (PORTB &= ~(1<<LED_RED))


/* ------------------------------------------------------------------------- */
volatile uint8_t wdInt;
uint8_t nodeId;
//uint8_t temp;
//uint16_t q = 0;
uint8_t data_array[PAYLOAD_LENGTH];
//uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
//uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
#if EN_SWITCH
volatile uint8_t switchFlag;
volatile uint8_t switchFlagWd;
volatile uint8_t debounceFlag;
#endif
uint8_t xmitFlagWd;
uint8_t xmitFlagPc;

sensor_ctr_t    sens_ctr;
sensors_t       sensors;

/* ------------------------------------------------------------------------- */



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

// This runs each time the watch dog wakes us up from sleep
// The system wakes up when any interrupt occurs. Setting this
// flag lets background (main loop) know that the watchdog
// interrupt occured so that we can xmit.
ISR(WATCHDOG_vect)
{
    if (debounceFlag) {
        debounceFlag = 0;
        switchFlagWd = 1;
    } else {
        wdInt++;
    }
}

#if EN_SWITCH
ISR(PCINT1_vect)
{
    switchFlag = 1;
}
#endif

int main(void)
{
    uint8_t rv;


	/* Perform system initialization */
	uartbb_init();
    xfunc_out = uartbb_putchar;

	/* init LED pins as OUTPUT */
	DDRB |= (1<<LED_GREEN);
	DDRB |= (1<<LED_RED);

#if EN_SWITCH
    switchFlag = 0;
    switchFlagWd = 0;
    debounceFlag = 0;
    DDRB &= ~(1<<2);
    PORTB |= (1<<2);
    GIMSK = (1<<SWITCH_GMSK);
    PCMSK1 = (1<<SWITCH_MSK);
#endif

    nodeId = eeprom_read_byte((uint8_t *)0);
    *(uint8_t *)&sensors   = eeprom_read_byte((uint8_t *)1);
//    xprintf("nodeId: %02X\n", nodeId);

    if (sensors.ctr) {
        sens_ctr.ctr_lo = 0;
        sens_ctr.ctr_hi = 0;
        sens_ctr.sensorId = SENS_ID_CTR;
    }

	/* init SPI */
//	spi_init();

	/* init hardware pins for talking to radio */
	nrf24_init();
    
	/* Set the device addresses */
//	nrf24_tx_address(tx_address);
//	nrf24_rx_address(rx_address);    

	/* Channel 2 , payload length: 4 */
	nrf24_config(2, PAYLOAD_LENGTH);
    data_array[0] = nodeId;

#if EN_WD
	setup_watchdog(WD_TO);
#endif

  /* Enable interrupts */
#if 1 //EN_WD
    sei();
#endif

    xprintf("\nHello\n");
    nrf24_readRegister(0,&rv,1);
    xprintf("00:%02X\n", rv);
    xprintf("00:%02X\n", nrf24_rdReg(0));
    nrf24_readRegister(1,&rv,1);
    xprintf("01:%02X\n", rv);
    nrf24_readRegister(2,&rv,1);
    xprintf("02:%02X\n", rv);
    nrf24_readRegister(3,&rv,1);
    xprintf("03:%02X\n", rv);
    nrf24_readRegister(4,&rv,1);
    xprintf("04:%02X\n", rv);
    nrf24_readRegister(5,&rv,1);
    xprintf("05:%02X\n", rv);
    nrf24_readRegister(6,&rv,1);
    xprintf("06:%02X\n", rv);
    nrf24_readRegister(7,&rv,1);
    xprintf("07:%02X\n", rv);
    nrf24_readRegister(8,&rv,1);
    xprintf("08:%02X\n", rv);
    nrf24_readRegister(9,&rv,1);
    xprintf("09:%02X\n", rv);
    _delay_ms(100);


	while (1) {
  
#if EN_WD
        system_sleep();
#endif

#if EN_SWITCH
        if (switchFlag) {
            setup_watchdog(WD_TO_SHORT);
            switchFlag = 0;
            debounceFlag = 1;
            continue;
        }
        if (switchFlagWd) {
            switchFlagWd = 0;
            setup_watchdog(WD_TO);
            ASSERT_REDLED();
            _delay_ms(1);
            DEASSERT_REDLED();
            /* set flag to xmit */
            xmitFlagPc = 1;
        }
#endif

        if (wdInt > 3) {
            wdInt = 0;
            xmitFlagWd = 1;
        }
#if 0
            xprintf("%02X ", nrf24_rdReg(8));
#endif
        if (xmitFlagWd || xmitFlagPc) {
            /* Fill the data buffer */
//		    data_array[0] = nodeId;
            if (sensors.ctr) {
                memcpy(&data_array[1], &sens_ctr, sizeof(sens_ctr));
                if (++sens_ctr.ctr_lo == 0)
                    sens_ctr.ctr_hi++;
            }
//		    data_array[1] = (SENID_CTR<<3) | ((q>>8) & 0x03);
//		    data_array[2] = q & 0xFF;
//            if (xmitFlagPc) {
//                q = 0;
//            }
//		    data_array[3] = q++;                                    
#if 1
		    /* Automatically goes to TX mode */
		    nrf24_send(data_array, PAYLOAD_LENGTH);        
        
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
            xmitFlagWd = 0;
            xmitFlagPc = 0;

        }

#if 1
        ASSERT_GRNLED();
        _delay_ms(1);
        DEASSERT_GRNLED();
#endif

		/* Wait a little ... */
#if EN_WD==0
		_delay_ms(1000);
#endif

    }
    return 0;
}


#if 0
disable BOD to save power prior to sleep
disable ADC prior to sleep
first conversion after disable/enable will be an extended conversion
#endif
