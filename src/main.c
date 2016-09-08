//-------------------------------------------------------------
// lofi + nrf24
//-------------------------------------------------------------
// This is software that runs on a LoFi board designed by David Cook.
// See https://hackaday.io/project/1552-lofi
//
// I have mated the lofi board with a nrf24l01+ instead of using
// the 433 MHz transmitter described at the above link.
//
// I'm using the nrf24 in its very basic form, using miminal
// features.
//
// I am using parts of xprintf.[ch] by ChaN (see xprintf.c for copyright)
// I am using parts of nrf24.[ch] by <ihsan@ehribar.me> (see nrf24.c for license)
//
// The lofi+nrf24 combo draws ~6 uA in the idle mode.
// It can monitor two reed switches, Vcc and internal temperature.
// It uses the internal RC oscillator so timing is not very accurate.
// The switches can be configured to generate an interrupt on Pin Change
// which will trigger a transmission. Otherwise, they are only polled
// and switch state will be transmitted at the polling frequency.
// The polling period is configured in eeprom.
// A 10-bit incrementing count is sent on polled transmissions.
//
// Configuration is stored in the eeprom starting at addr 0.
// See config_t structure for details.
//
//---------------------------------------------------------

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/cpufunc.h>
#include <string.h>

#include "lofi.h"
#include "nrf24.h"
#include "uartbb.h"
#include "xprintf.h"

#undef F_CPU
#define F_CPU 1000000UL

#define SWITCH_1        2       /* PORTB bit2 */
#define SWITCH_1_MSK    2
#define SWITCH_1_GMSK   5
#define SWITCH_2        2       /* PORTA bit2 */
#define SWITCH_2_MSK    2
#define SWITCH_2_GMSK   4

#define EEPROM_NODEID_ADR        ((uint8_t *)0)

#define NRF24_CHANNEL            2
#define NRF24_PAYLOAD_LEN        8

// ---------  LED MACROS  ----------
#define LED_RED					(1<<0)  // PORTB bit0
#define LED_GRN					(1<<1)  // PORTB bit1

#define LED_INIT()				{(DDRB |= LED_RED); (DDRB |= LED_GRN);}
#define LED_ASSERT(x)			(PORTB |= (x))
#define LED_DEASSERT(x)			(PORTB &= ~(x))

#define NRF_VCC_PIN				(1<<3)
#define NRF_VCC_INIT()			(DDRA |= NRF_VCC_PIN)
#define NRF_VCC_ASSERT()		(PORTA |= NRF_VCC_PIN)
#if 0
#define NRF_VCC_DEASSERT()		
#else
#define NRF_VCC_DEASSERT()		(PORTA &= ~NRF_VCC_PIN)
#endif
#define NRF_VCC_DLY_MS(x)		_delay_ms((x))

/* ------------------------------------------------------------------------- */
volatile uint8_t wdTick;
uint8_t wdSec, wdMin, wdHour;
uint16_t wdDay;
uint8_t data_array[NRF24_PAYLOAD_LEN];
volatile uint8_t sw1Flag;
volatile uint8_t sw2Flag;
volatile uint8_t wdFlag;
speed_t speed = speed_2M;
config_t		config;
sensor_ctr_t    sens_ctr;
sensors_t       sensors;
sensor_switch_t sens_sw1;
sensor_switch_t sens_sw2;
sensor_vcc_t    sens_vcc;
sensor_temp_t   sens_temp;


/* FORWARD DECLARATIONS ---------------------------------------- */

uint16_t readVccVoltage(void);
uint16_t readTemperature(void);
void printConfig(void);
void blinkLed(void);
void getSw1(uint8_t *payload);
void getSw2(uint8_t *payload);


//****************************************************************  
// system_sleep - go to sleep
//
// set system into the sleep state 
// system wakes up when watchdog times out
void system_sleep(void)
{
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
	sleep_enable();

	sleep_bod_disable();

//  PRR |= 0x0D;

	sleep_mode();                        // System sleeps here

//  sleep_disable();                     // System continues execution here when watchdog timed out 
}

//****************************************************************
// setup_watchdog - configure watchdog timeout
//
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int val)
{
	uint8_t bb;

	if (val > 9 )
		return;		//val = 9;
	bb = val & 7;
	if (val > 7)
		bb |= (1<<5);
	bb |= (1<<WDCE);

	MCUSR &= ~(1<<WDRF);
	// start timed sequence
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	// set new watchdog timeout value
	WDTCSR = bb;
	WDTCSR |= _BV(WDIE);
}

//****************************************************************
// watchdog_isr
//
// This runs each time the watch dog wakes us up from sleep
// The system wakes up when any interrupt occurs. Setting this
// flag lets background (main loop) know that the watchdog
// interrupt occured so that we can xmit.
ISR(WATCHDOG_vect)
{
	if (config.wdCnts) {
		if (++wdTick >= config.wdCnts) {
			wdTick = 0;
			wdFlag = 1;
		}
	}
}


//****************************************************************
// pinChange_isr
//
ISR(PCINT1_vect)
{
	uint8_t pinState;

	pinState = (PINB>>SWITCH_1) & 1;
    sw1Flag = (sens_sw1.lastState == pinState)?0:1;
}


ISR(PCINT0_vect)
{
	uint8_t pinState;

	pinState = (PINA>>SWITCH_2) & 1;
    sw2Flag = (sens_sw2.lastState == pinState)?0:1;
}


//****************************************************************
// main
//
int main(void)
{
	uint8_t pay_idx;

	// Set Divide by 8 for 8MHz RC oscillator 
	CLKPR = (1<<CLKPCE);
	CLKPR = 3;

	// disable PUD
	MCUCR &= ~(1<<PUD);

	// turn off analog comparator
	ACSR = 0x80;

	// reduce power on TIMER1
	PRR |= (1<<PRTIM1);

	// read the config params from eeprom
	eeprom_read_block(&config, 0, sizeof(config));

    // init LED pins as OUTPUT
	LED_INIT();				// set as output even if not used
	LED_DEASSERT(LED_GRN);	// turn them both off
	LED_DEASSERT(LED_RED);	// turn them both off

	if (config.spd_1M)
		speed = speed_1M;
	else if (config.spd_250K)
		speed = speed_250K;

	// init hardware pins for talking to radio
	nrf24_init();
    
    // initialize uart if eeprom configured
//	config.txDbg = 0;
	if (config.txDbg) {
		uartbb_init();
		xfunc_out = uartbb_putchar;
	} else {
		PRR |= (1<<PRTIM0);
	}

	// Initialize counter capability/structure if eeprom configured
    if (config.ctr) {
        sens_ctr.sensorId = SENID_CTR;
        sens_ctr.ctr_lo = 0;
        sens_ctr.ctr_hi = 0;
    }

	// Initialize Vcc capability/structure if eeprom configured
    if (config.vcc) {
        sens_vcc.sensorId = SENID_VCC;
        sens_vcc.vcc_lo = 0;
        sens_vcc.vcc_hi = 0;
    }

	// Initialize switch 1 capability/structure if eeprom configured
    if (config.sw1_enb) {
		uint8_t pinState;

		DDRB &= ~(1<<SWITCH_1);
		sens_sw1.sensorId = SENID_SW1;
		pinState = (PINB>>SWITCH_1) & 1;
		sens_sw1.closed = config.sw1_rev ^ pinState;
        sens_sw1.changed = pinState;
        sens_sw1.lastState = pinState;
		if (config.sw1_pc) {
			GIMSK = (1<<SWITCH_1_GMSK);
			PCMSK1 = (1<<SWITCH_1_MSK);
			sw1Flag = 0;
		}
    } else {
		// if we leave pin as input, it will draw more current if it oscillates
		// but if we pgm pin as output and there REALLY is a switch connected
		// we are going to do damage
		DDRB |= (1<<SWITCH_1);
		PORTB &= ~(1<<SWITCH_1);
	}

	// Initialize switch 2 capability/structure if eeprom configured
    if (config.sw2_enb) {
		uint8_t pinState;

		DDRA &= ~(1<<SWITCH_2);
        sens_sw2.sensorId = SENID_SW2;
		pinState = (PINA>>SWITCH_2) & 1;
		sens_sw2.closed = config.sw2_rev ^ pinState;
        sens_sw2.changed = pinState;
        sens_sw2.lastState = pinState;
		if (config.sw2_pc) {
			GIMSK |= (1<<SWITCH_2_GMSK);
			PCMSK0 = (1<<SWITCH_2_MSK);
			sw2Flag = 0;
		}
    } else {
		// if we leave pin as input, it will draw more current if it oscillates
		// but if we pgm pin as output and there REALLY is a switch connected
		// we are going to do damage
		// Suggest that I install a series resistor on input pin...
		DDRA |= (1<<SWITCH_2);
		PORTA &= ~(1<<SWITCH_2);
    }

	// initialize pin and apply power to NRF
	NRF_VCC_INIT();
	NRF_VCC_ASSERT();
	NRF_VCC_DLY_MS(100);

	CLKPR = (1<<CLKPCE);
	CLKPR = 4;
	// Initialize radio channel and payload length
	nrf24_config(config.rf_chan, NRF24_PAYLOAD_LEN, speed, config.rf_gain);

	// Power off radio as we go into our first sleep
	nrf24_powerDown();            
//	NRF_VCC_DEASSERT();
	CLKPR = (1<<CLKPCE);
	CLKPR = 3;

    // initialize watchdog and associated variables
    wdTick = 0;
    wdSec = 0;
    wdMin = 0;
    wdHour = 0;
    wdDay = 0;
	setup_watchdog(config.wd_timeout + 5);

	// Enable interrupts
    sei();

	// must be called after global interrupts are enabled
	if (config.txDbg) {
		printConfig();
		_delay_ms(100);
	}

	// Clear the payload buffer and setup for next xmit
	memset(data_array, 0, NRF24_PAYLOAD_LEN);
	data_array[0] = config.nodeId;
	pay_idx = 1;

	//
	// Start of main loop
	//
	while (1) {

		// go to sleep and wait for interrupt (watchdog or pin change)
		system_sleep();

		if (wdFlag) {

			if (config.sw1_enb) {
				getSw1(&data_array[pay_idx++]);
			}

			if (config.sw2_enb) {
				getSw2(&data_array[pay_idx++]);
			}

            if (config.ctr) {
                memcpy(&data_array[pay_idx], &sens_ctr, sizeof(sens_ctr));
                pay_idx += sizeof(sens_ctr);
                if (++sens_ctr.ctr_lo == 0)
                    sens_ctr.ctr_hi++;
            }

            if (config.vcc) {
                uint16_t vcc = readVccVoltage();
                sens_vcc.sensorId = SENID_VCC;
                sens_vcc.vcc_lo = vcc & 0xFF;
                sens_vcc.vcc_hi = (vcc>>8) & 0x3;
                memcpy(&data_array[pay_idx], &sens_vcc, sizeof(sens_vcc));
                pay_idx += sizeof(sens_vcc);
            }

            if (config.temp) {
                uint16_t temp = readTemperature();
                sens_temp.sensorId = SENID_TEMP;
                sens_temp.temp_lo = temp & 0xFF;
                sens_temp.temp_hi = (temp>>8) & 0x3;
                memcpy(&data_array[pay_idx], &sens_temp, sizeof(sens_temp));
                pay_idx += sizeof(sens_temp);
            }

		} else {

			if (sw1Flag && config.sw1_enb) {
				getSw1(&data_array[pay_idx++]);
			}

			if (sw2Flag && config.sw2_enb) {
				getSw2(&data_array[pay_idx++]);
			}
		}

		if (wdFlag || sw1Flag || sw2Flag) {
			int i;

	// Set Divide by 8 for 8MHz RC oscillator 
	CLKPR = (1<<CLKPCE);
	CLKPR = 4;

			if (wdFlag) wdFlag = 0;
			if (sw1Flag) sw1Flag = 0;
			if (sw2Flag) sw2Flag = 0;

		    /* Automatically goes to TX mode */
			nrf24_send(data_array, NRF24_PAYLOAD_LEN);        

			/* Start the transmission */
			nrf24_pulseCE();

			i = 0;
			while(nrf24_isSending() && i < 100)
				i++;

		    nrf24_powerDown();            

	CLKPR = (1<<CLKPCE);
	CLKPR = 3;

#if 1
	extern uint8_t gstatus;
    if (gstatus & (1 << MAX_RT)) {        
			if (config.enLed) { LED_ASSERT(LED_RED); }
	}
			if (config.enLed) { LED_ASSERT(LED_GRN); }
			_delay_us(100);
    if (gstatus & (1 << MAX_RT)) {        
			if (config.enLed) { LED_DEASSERT(LED_RED); }
	}
			if (config.enLed) { LED_DEASSERT(LED_GRN); }
#endif
            // Clear the payload buffer and setup for next xmit
            memset(data_array, 0, NRF24_PAYLOAD_LEN);
		    data_array[0] = config.nodeId;
			pay_idx = 1;

        } //endof: if (wdFlag || sw1Flag || sw2Flag) {

    }

    return 0;
}

void getSw1(uint8_t *payload)
{
	uint8_t pinState;

	pinState = (PINB>>SWITCH_1) & 1;
	sens_sw1.changed = (sens_sw1.lastState == pinState)?0:1;
	sens_sw1.closed = config.sw1_rev ^ pinState;
	sens_sw1.lastState = pinState;
	*payload = *(uint8_t *)&sens_sw1;
}

void getSw2(uint8_t *payload)
{
	uint8_t pinState;

	pinState = (PINA>>SWITCH_2) & 1;
	sens_sw2.changed = (sens_sw2.lastState == pinState)?0:1;
	sens_sw2.closed = config.sw2_rev ^ pinState;
	sens_sw2.lastState = pinState;
	*payload = *(uint8_t *)&sens_sw2;
}

void blinkLed(void)
{
	if (config.enLed) { LED_ASSERT(LED_GRN); }
	_delay_us(100);
	if (config.enLed) { LED_DEASSERT(LED_GRN); }
}

void printConfig(void)
{
	uint8_t  ta[8];

    xprintf("\nHello\n");
	xprintf("00:%02x", nrf24_rdReg(0));
    xprintf("  01:%02x", nrf24_rdReg(1));
    xprintf("  02:%02x", nrf24_rdReg(2));
    xprintf("  03:%02X", nrf24_rdReg(3));
	xprintf("  04:%02X", nrf24_rdReg(4));
    xprintf("  05:%02X", nrf24_rdReg(5));
    xprintf("  06:%02X", nrf24_rdReg(6));
    xprintf("  07:%02X", nrf24_rdReg(7));
    xprintf("  08:%02X", nrf24_rdReg(8));
    xprintf("  09:%02X\n", nrf24_rdReg(9));
	memset(ta, 0, 8);
	nrf24_readRegister(0x0a, ta, 5);
	xprintf("0A:%02X %02X %02X %02X %02X\n", ta[4], ta[3], ta[2], ta[1], ta[0]);
	memset(ta, 0, 8);
	nrf24_readRegister(0x0b, ta, 5);
	xprintf("0B:%02X %02X %02X %02X %02X", ta[4], ta[3], ta[2], ta[1], ta[0]);
    xprintf("  0C:%02X", nrf24_rdReg(0x0c));
    xprintf("  0D:%02X", nrf24_rdReg(0x0d));
    xprintf("  0E:%02X", nrf24_rdReg(0x0e));
    xprintf("  0F:%02X\n", nrf24_rdReg(0x0f));
	memset(ta, 0, 8);
	nrf24_readRegister(0x10, ta, 5);
	xprintf("10:%02X %02X %02X %02X %02X\n", ta[4], ta[3], ta[2], ta[1], ta[0]);
    xprintf("11:%02X", nrf24_rdReg(0x11));
    xprintf("  12:%02X", nrf24_rdReg(0x12));
    xprintf("  13:%02X", nrf24_rdReg(0x13));
    xprintf("  14:%02X", nrf24_rdReg(0x14));
    xprintf("  15:%02X", nrf24_rdReg(0x15));
    xprintf("  16:%02X", nrf24_rdReg(0x16));
    xprintf("  17:%02X\n", nrf24_rdReg(0x17));
//	xprintf("              ?\n");
    xprintf("1C:%02X", nrf24_rdReg(0x1c));
    xprintf("  1D:%02X\n", nrf24_rdReg(0x1d));
}


//------------------------------------------------------------------
// readVccVoltage - read Vcc (indirectly) through ADC subsystem
//
// Returns the 10-bit ADC value.
// On each reading we: enable the ADC, take the measurement, and then disable the ADC for power savings.
// This takes >1ms becuase the internal reference voltage must stabilize each time the ADC is enabled.
// For faster readings, you could initialize once, and then take multiple fast readings, just make sure to
// disable the ADC before going to sleep so you don't waste power. 
// I got this technique from:
//   http://wp.josh.com/2014/11/06/battery-fuel-guage-with-zero-parts-and-zero-pins-on-avr/
//
uint16_t readVccVoltage(void)
{
	
	PRR &= ~(1<<PRADC);

	// Select ADC inputs
	// bit    76543210 
	// REFS = 00       = Vcc used as Vref
	// MUX  =   100001 = Single ended, 1.1V (Internal Ref) as Vin
	
	ADMUX = 0b00100001;
	
	// By default, the successive approximation circuitry requires an input clock frequency between 50
	// kHz and 200 kHz to get maximum resolution.
				
	// Enable ADC, set prescaller to /8 which will give a ADC clock of 1MHz/8 = 125KHz
	
	ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);
	
	// After switching to internal voltage reference the ADC requires a settling time of 1ms before
	// measurements are stable. Conversions starting before this may not be reliable. The ADC must
	// be enabled during the settling time.
		
	_delay_ms(1);
				
	// The first conversion after switching voltage source may be inaccurate, and the user is advised to discard this result.
	// first conversion after disable/enable will be an extended conversion
		
	ADCSRA |= _BV(ADSC);				// Start a conversion
	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...
										//..and ignore the result
	ADCSRA |= _BV(ADSC);				// Start a conversion
	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...
										//..and ignore the result
	ADCSRA |= _BV(ADSC);				// Start a conversion
	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...
						
		
	// After the conversion is complete (ADIF is high), the conversion result can be found in the ADC
	// Result Registers (ADCL, ADCH).		
		
	// When an ADC conversion is complete, the result is found in these two registers.
	// When ADCL is read, the ADC Data Register is not updated until ADCH is read.		
	
	// Note we could have used ADLAR left adjust mode and then only needed to read a single byte here
		
	uint8_t low  = ADCL;
	uint8_t high = ADCH;

	uint16_t adc = (high << 8) | low;		// 0<= result <=1023
			
	// Compute a fixed point with 1 decimal place (i.e. 5v= 50)
	//
	// Vcc   =  (1.1v * 1024) / ADC
	// Vcc10 = ((1.1v * 1024) / ADC ) * 10				->convert to 1 decimal fixed point
	// Vcc10 = ((11   * 1024) / ADC )				->simplify to all 16-bit integer math
				
//	uint8_t vccx10 = (uint8_t) ( (11 * 1024) / adc); 
	
	// Note that the ADC will not automatically be turned off when entering other sleep modes than Idle
	// mode and ADC Noise Reduction mode. The user is advised to write zero to ADEN before entering such
	// sleep modes to avoid excessive power consumption.
	
	ADCSRA &= ~_BV( ADEN );			// Disable ADC to save power
	PRR |= (1<<PRADC);
	
	return ( adc );
	
}


//------------------------------------------------------------------
// readTemperature - read internal temperature through ADC subsystem
//
// Returns the 10-bit ADC value.
// On each reading we: enable the ADC, take the measurement, and then disable the ADC for power savings.
// This takes >1ms becuase the internal reference voltage must stabilize each time the ADC is enabled.
// For faster readings, you could initialize once, and then take multiple fast readings, just make sure to
// disable the ADC before going to sleep so you don't waste power. 

uint16_t readTemperature(void)
{
	
	PRR &= ~(1<<PRADC);

	// Select ADC inputs
	// bit    76543210 
	// REFS = 10       = 1.1V used as Vref
	// MUX  =   100010 = Single ended, chan 8 (internal Temp sensor) as Vin
	
	ADMUX = 0b10100010;
	
	// By default, the successive approximation circuitry requires an input clock frequency between 50
	// kHz and 200 kHz to get maximum resolution.
				
	// Enable ADC, set prescaller to /8 which will give a ADC clock of 1MHz/8 = 125KHz
	
	ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);
	
	// After switching to internal voltage reference the ADC requires a settling time of 1ms before
	// measurements are stable. Conversions starting before this may not be reliable. The ADC must
	// be enabled during the settling time.
		
	_delay_ms(1);
				
	// The first conversion after switching voltage source may be inaccurate, and the user is advised to discard this result.
	// first conversion after disable/enable will be an extended conversion
		
	ADCSRA |= _BV(ADSC);				// Start a conversion
	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...
										//..and ignore the result
	ADCSRA |= _BV(ADSC);				// Start a conversion
	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...
										//..and ignore the result
	ADCSRA |= _BV(ADSC);				// Start a conversion
	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...
						
		
	// After the conversion is complete (ADIF is high), the conversion result can be found in the ADC
	// Result Registers (ADCL, ADCH).		
		
	// When an ADC conversion is complete, the result is found in these two registers.
	// When ADCL is read, the ADC Data Register is not updated until ADCH is read.		
	
	// Note we could have used ADLAR left adjust mode and then only needed to read a single byte here
		
	uint8_t low  = ADCL;
	uint8_t high = ADCH;

	uint16_t adc = (high << 8) | low;		// 0<= result <=1023
			
	// Compute a fixed point with 1 decimal place (i.e. 5v= 50)
	//
	// Vcc   =  (1.1v * 1024) / ADC
	// Vcc10 = ((1.1v * 1024) / ADC ) * 10				->convert to 1 decimal fixed point
	// Vcc10 = ((11   * 1024) / ADC )				->simplify to all 16-bit integer math
				
	
	// Note that the ADC will not automatically be turned off when entering other sleep modes than Idle
	// mode and ADC Noise Reduction mode. The user is advised to write zero to ADEN before entering such
	// sleep modes to avoid excessive power consumption.
	
	ADCSRA &= ~_BV( ADEN );			// Disable ADC to save power
	PRR |= (1<<PRADC);
	
	return ( adc );
	
}
