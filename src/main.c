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
// A 10-bit incrementing count is sent on each transmission.
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
#include <string.h>

#include "lofi.h"
#include "nrf24.h"
#include "uartbb.h"
#include "xprintf.h"

#undef F_CPU
#define F_CPU 1000000UL

#define EN_NRF			1
//#define EN_WD			1
//#define WD_TO           9
//#define WD_TO_SHORT     3
#define EN_SWITCH       1


#define SWITCH_1        2       /* PORTB bit2 */
#define SWITCH_MSK      2
#define SWITCH_GMSK     5

#define EEPROM_NODEID_ADR        ((uint8_t *)0)
//#define EEPROM_CAPABILITY_ADR    ((uint8_t *)1)

#define NRF24_CHANNEL            2
#define NRF24_PAYLOAD_LEN        8

// ---------  LED MACROS  ----------
#define LED_RED                  (1<<0)  // PORTB bit0
#define LED_GRN                  (1<<1)  // PORTB bit1

#define LED_INIT(x)              (DDRB |= (x))
#define LED_ASSERT(x)            (PORTB |= (x))
#define LED_DEASSERT(x)          (PORTB &= ~(x))

#define NRF_VCC_PIN				((1<<3) | (1<<7))
#define NRF_VCC_INIT()			(DDRA |= NRF_VCC_PIN)
#define NRF_VCC_ASSERT()		(PORTA |= NRF_VCC_PIN)
#define NRF_VCC_DEASSERT()		(PORTA &= ~NRF_VCC_PIN)
#define NRF_VCC_DLY_MS(x)		_delay_ms((x))

/* ------------------------------------------------------------------------- */
volatile uint8_t wdInt;
volatile uint8_t wdTick;
uint8_t wdSec, wdMin, wdHour;
uint16_t wdDay;
//uint8_t nodeId;
uint8_t data_array[NRF24_PAYLOAD_LEN];
#if EN_SWITCH
volatile uint8_t sw1Flag;
volatile uint8_t sw2Flag;
volatile uint8_t debounceFlag;
#endif
uint8_t xmitFlagWd;
uint8_t xmitFlagPc;

uint8_t ta[8];

config_t		config;
sensor_ctr_t    sens_ctr;
sensors_t       sensors;
sensor_switch_t sens_sw1;
sensor_switch_t sens_sw2;
sensor_vcc_t    sens_vcc;
sensor_temp_t   sens_temp;


/* ------------------------------------------------------------------------- */

uint16_t readVccVoltage(void);
uint16_t readTemperature(void);
void printConfig(void);


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

    wdInt = 0;

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
    wdTick = 1;
}


//****************************************************************
// pinChange_isr
//
ISR(PCINT1_vect)
{
    sw1Flag = 1;
}


ISR(PCINT0_vect)
{
    sw2Flag = 1;
}


//****************************************************************
// main
//
int main(void)
{
	uint16_t i;

	// Set Divide by 8 for 8MHz RC oscillator 
	CLKPR = (1<<CLKPCE);
	CLKPR = 3;

	// disable PUD
	MCUCR &= ~(1<<PUD);

	// turn off analog comparator
	ACSR = 0x80;

	// read the config params from eeprom
	eeprom_read_block(&config, 0, sizeof(config));

    // init LED pins as OUTPUT
	LED_INIT(LED_RED | LED_GRN);		// set as output even if not used
	LED_DEASSERT(LED_RED | LED_GRN);	// turn them both off

    // initialize uart if eeprom configured
	if (config.txDbg) {
		uartbb_init();
		xfunc_out = uartbb_putchar;
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
		DDRB &= ~(1<<2);
		sens_sw1.sensorId = SENID_SW1;
		if (config.sw1_nc)
			sens_sw1.switch_closed = (PINB & (1<<2))?(1):(0);
		else
			sens_sw1.switch_closed = (PINB & (1<<2))?(0):(1);
        sens_sw1.swtich_changed = 0;
        sens_sw1.rsvd = 0;
		if (config.sw1_pc) {
			// enable pin change functionality
			GIMSK = (1<<SWITCH_GMSK);
			PCMSK1 = (1<<SWITCH_MSK);
			sw1Flag = 0;
			debounceFlag = 0;
		}
    } else {
		// if we leave pin as input, it will draw more current if it oscillates
		// but if we pgm pin as output and there REALLY is a switch connected
		// we are going to do damage
	}

	// Initialize switch 2 capability/structure if eeprom configured
    if (config.sw2_enb) {
		DDRA &= ~(1<<2);
        sens_sw2.sensorId = SENID_SW2;
		if (config.sw2_nc)
			sens_sw2.switch_closed = (PINA & (1<<2))?(1):(0);
		else
			sens_sw2.switch_closed = (PINA & (1<<2))?(0):(1);
        sens_sw2.swtich_changed = 0;
        sens_sw2.rsvd = 0;
		if (config.sw2_pc) {
			// enable pin change functionality
			GIMSK |= (1<<4);
			PCMSK0 = (1<<SWITCH_MSK);
			sw2Flag = 0;
			debounceFlag = 0;
		}
    } else {
		// if we leave pin as input, it will draw more current if it oscillates
		// but if we pgm pin as output and there REALLY is a switch connected
		// we are going to do damage
    }

	// initialize pin and apply power to NRF
	NRF_VCC_INIT();
	NRF_VCC_ASSERT();
	NRF_VCC_DLY_MS(10);


#if EN_NRF
	// init hardware pins for talking to radio
	nrf24_init();
    
	// Initialize radio channel and payload length
	nrf24_config(config.rf_chan, NRF24_PAYLOAD_LEN, config.spd_1M, config.rf_gain);
#endif

//    data_array[0] = config.nodeId;

    // initialize watchdog and associated variables
    wdTick = 0;
    wdSec = 0;
    wdMin = 0;
    wdHour = 0;
    wdDay = 0;
	setup_watchdog(config.wd_timeout + 5);
//	setup_watchdog(3);

	// Enable interrupts
    sei();

	if (config.txDbg) {
		printConfig();
		_delay_ms(100);
	}

	// Power off radio as we go into our first sleep
	nrf24_powerDown();            
	DEASSERT_CE();
	NRF_VCC_DEASSERT();


	//
	// Start of main loop
	//
	while (1) {

		// clear MOSI
		PORTA &= ~(1<<6);
  
		// go to sleep and wait for interrupt (watchdog or pin change)
		system_sleep();


        if (sw1Flag) {
			sw1Flag = 0;
            xmitFlagPc = 1;
		}

        if (sw2Flag) {
			sw2Flag = 0;
            xmitFlagPc = 1;
		}


		if (wdTick) {
			wdTick = 0;
			if (config.wdCnts) {
				if (++wdInt >= config.wdCnts) {
					wdInt = 0;
					xmitFlagWd = 1;
				}
			}
		}

		if (config.txDbg) {
            xprintf("%02X ", nrf24_rdReg(8));
		}

        if (xmitFlagWd || xmitFlagPc) {
            uint8_t pay_idx = 1;

            if (xmitFlagWd) xmitFlagWd = 0;
            if (xmitFlagPc) xmitFlagPc = 0;

            // Clear the payload buffer
            memset(data_array, 0, 8);
            // Fill the payload buffer
		    data_array[0] = config.nodeId;
            if (config.ctr) {
                memcpy(&data_array[pay_idx], &sens_ctr, sizeof(sens_ctr));
                pay_idx += sizeof(sens_ctr);
                if (++sens_ctr.ctr_lo == 0)
                    sens_ctr.ctr_hi++;
            }

            if (config.sw1_enb) {
                sens_sw1.swtich_changed = 0; //xmitFlagPc;
				if (config.sw1_nc)
					sens_sw1.switch_closed = (PINB & (1<<2))?(1):(0);
				else
					sens_sw1.switch_closed = (PINB & (1<<2))?(0):(1);
                data_array[pay_idx] = *(uint8_t *)&sens_sw1;
                pay_idx++;
            }

            if (config.sw2_enb) {
                sens_sw2.swtich_changed = 0; //xmitFlagPc;
				if (config.sw2_nc)
					sens_sw2.switch_closed = (PINA & (1<<2))?(1):(0);
				else
					sens_sw2.switch_closed = (PINA & (1<<2))?(0):(1);
                data_array[pay_idx] = *(uint8_t *)&sens_sw2;
                pay_idx++;
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


			NRF_VCC_ASSERT();
			NRF_VCC_DLY_MS(10);
			ASSERT_CE();

		    /* Automatically goes to TX mode */
		    nrf24_send(data_array, NRF24_PAYLOAD_LEN);        
        
		    /* Wait for transmission to end */
			i = 0;
		    while (nrf24_isSending() && i++ < 10000);

		    /* Make analysis on last tranmission attempt */
//		    temp = nrf24_lastMessageStatus();

		    /* Retranmission count indicates the tranmission quality */
//		    temp = nrf24_retransmissionCount();
//		    xprintf("> Retranmission count: %d\r\n",temp);


		    /* Or you might want to power down after TX */
		    nrf24_powerDown();            
			DEASSERT_CE();
			NRF_VCC_DEASSERT();


			if (config.enLed) {
				LED_ASSERT(LED_GRN);
        		_delay_us(500);
				LED_DEASSERT(LED_GRN);
			}

        } //endof: if (xmitFlagWd || xmitFlagPc) {

    }

    return 0;
}


void printConfig(void)
{
    xprintf("\nHello\n");
    xprintf("00:%02X", nrf24_rdReg(0));
    xprintf("  01:%02X", nrf24_rdReg(1));
    xprintf("  02:%02X", nrf24_rdReg(2));
    xprintf("  03:%02X", nrf24_rdReg(3));
    xprintf("  04:%02X", nrf24_rdReg(4));
    xprintf("  05:%02X", nrf24_rdReg(5));
    xprintf("  06:%02X", nrf24_rdReg(6));
    xprintf("  07:%02X", nrf24_rdReg(7));
    xprintf("  08:%02X", nrf24_rdReg(8));
    xprintf("  09:%02X\n", nrf24_rdReg(9));
	nrf24_readRegister(0x0a, ta, 5);
	xprintf("0A:%02X %02X %02X %02X %02X\n", ta[4], ta[3], ta[2], ta[1], ta[0]);
	nrf24_readRegister(0x0b, ta, 5);
	xprintf("0B:%02X %02X %02X %02X %02X", ta[4], ta[3], ta[2], ta[1], ta[0]);
    xprintf("  0C:%02X", nrf24_rdReg(0x0c));
    xprintf("  0D:%02X", nrf24_rdReg(0x0d));
    xprintf("  0E:%02X", nrf24_rdReg(0x0e));
    xprintf("  0F:%02X\n", nrf24_rdReg(0x0f));
	nrf24_readRegister(0x10, ta, 5);
	xprintf("10:%02X %02X %02X %02X %02X\n", ta[4], ta[3], ta[2], ta[1], ta[0]);
    xprintf("11:%02X", nrf24_rdReg(0x11));
    xprintf("  12:%02X", nrf24_rdReg(0x12));
    xprintf("  13:%02X", nrf24_rdReg(0x13));
    xprintf("  14:%02X", nrf24_rdReg(0x14));
    xprintf("  15:%02X", nrf24_rdReg(0x15));
    xprintf("  16:%02X", nrf24_rdReg(0x16));
    xprintf("  17:%02X\n", nrf24_rdReg(0x17));
    xprintf("1C:%02X", nrf24_rdReg(0x1c));
    xprintf("  1D:%02X\n", nrf24_rdReg(0x1d));
		//void nrf24_transferSync(uint8_t* dataout, uint8_t* datain, uint8_t len)
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
