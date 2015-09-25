#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "nrf24.h"

#undef F_CPU
#define F_CPU 1000000L

#include <util/delay.h>


#define LED_RED   0x01
#define LED_GREEN 0x02

/* ------------------------------------------------------------------------- */
uint8_t temp;
uint8_t q = 0;
uint8_t data_array[4];
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
//uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
/* ------------------------------------------------------------------------- */

//****************************************************************  
// set system into the sleep state 
// system wakes up when wtchdog is timed out
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
//  watchdog_counter++;
}

int main(void)
{


//	setup_watchdog(9);

	/* Perform system initialization */

	/* init hardware pins */
	nrf24_init();
    
	/* Set the device addresses */
//	nrf24_tx_address(tx_address);
//	nrf24_rx_address(rx_address);    

	/* Channel 0x4c , payload length: 4 */
	nrf24_config(2,4);

	DDRB |= (LED_RED | LED_GREEN);

  /* Enable interrupts */
//	sei();

	while (1) {
  
//		system_sleep();

		/* Fill the data buffer */
		data_array[0] = 0x00;
		data_array[1] = 0xAA;
		data_array[2] = 0x55;
		data_array[3] = q++;                                    
#if 1
		/* Automatically goes to TX mode */
		nrf24_send(data_array);        
        
		/* Wait for transmission to end */
		while(nrf24_isSending());

		/* Make analysis on last tranmission attempt */
//		temp = nrf24_lastMessageStatus();

		/* Retranmission count indicates the tranmission quality */
//		temp = nrf24_retransmissionCount();
//		xprintf("> Retranmission count: %d\r\n",temp);

		/* Optionally, go back to RX mode ... */
//		nrf24_powerUpRx();

		/* Or you might want to power down after TX */
		// nrf24_powerDown();            
#endif
    PORTB |= (LED_GREEN);
    _delay_ms(1);
    PORTB &= ~(LED_GREEN);
#if 0
	_delay_ms(180);
    
    PORTB |= (LED_RED);
    _delay_ms(1);
    PORTB &= ~(LED_RED);
#endif
		/* Wait a little ... */
		_delay_ms(1000);


  }
#if 0
  while(1)
    {                
        /* Fill the data buffer */
        data_array[0] = 0x00;
        data_array[1] = 0xAA;
        data_array[2] = 0x55;
        data_array[3] = q++;                                    

        /* Automatically goes to TX mode */
        nrf24_send(data_array);        
        
        /* Wait for transmission to end */
        while(nrf24_isSending());

        /* Make analysis on last tranmission attempt */
        temp = nrf24_lastMessageStatus();

        if(temp == NRF24_TRANSMISSON_OK)
        {                    
            xprintf("> Tranmission went OK\r\n");
        }
        else if(temp == NRF24_MESSAGE_LOST)
        {                    
            xprintf("> Message is lost ...\r\n");    
        }
        
		/* Retranmission count indicates the tranmission quality */
		temp = nrf24_retransmissionCount();
		xprintf("> Retranmission count: %d\r\n",temp);

		/* Optionally, go back to RX mode ... */
		nrf24_powerUpRx();

		/* Or you might want to power down after TX */
		// nrf24_powerDown();            

		/* Wait a little ... */
		_delay_ms(10);
    }
#endif
  return 0;
}

#if 0
disable BOD to save power prior to sleep
disable ADC prior to sleep
first conversion after disable/enable will be an extended conversion


#endif
