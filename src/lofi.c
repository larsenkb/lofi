//-------------------------------------------------------------
// lofi + nrf24
//-------------------------------------------------------------
// This project was influenced by a LoFi board designed by
// David Cook and entered in a hackaday contest.
// See https://hackaday.io/project/1552-lofi
//
// At the beginning, I used OshPark to make some of his
// boards and hacked them to work with an NRF24l01+.
//
// I have mated the lofi board with a nrf24l01+ instead of using
// the 433 MHz transmitter described at the above link.
//
// I'm using the nrf24 in its very basic form, using minimal
// features.
//
// I am using parts of xprintf.[ch] by ChaN (see xprintf.c for copyright)
// I am using parts of nrf24.[ch] by <ihsan@ehribar.me>
// (see nrf24.c for license)
//
// The lofi+nrf24 combo draws ~2 uA in the idle mode.
// It can monitor one reed switch, Vcc and internal temperature.
// It uses the internal RC oscillator so timing is not very accurate.
// The switch can be configured to generate an interrupt on Pin Change
// which will trigger a transmission. Otherwise, it is only polled
// and switch state will be transmitted at the polling frequency.
// The polling period is configured in eeprom, to be a multiple
// of the TPL5111 period.
//
// A 10-bit incrementing count can also be enabled and will be
// sent at a frequency determined by eeprom and TPL5111.
//
// I am making my own board now that uses a TPL5111 to periodically
// wake the ATtiny84. I do not power down the t84 but instead,
// use a Pin Change function to monitor the TPL5111 DRVn pin.
// So, I continuously draw about 2uA.
// I chose this method, so that my original code did not have
// to change much. I can still use SRAM between sleeps.
// If I powered down the t84 between DRVn assertions, I would
// have to send all messages each time. I don't think there
// would be a method of only sending Vcc messages every n
// DRV assertions.
//
// I'm using spin loop delays. See util/delay_basic.h.
// _delay_loop_1 loop is three cycles and takes an 8-bit
// loop counter. 3us * 256 is max delay.
// _delay_loop_2 loop is four cycles and takes a 16-bit
// loop counter. 4us * 65536 is max delay.
//
// Configuration is stored in the eeprom starting at addr 0.
// See config_t structure for details.
//
//---------------------------------------------------------

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include <avr/eeprom.h>
#include <avr/cpufunc.h>
#include <string.h>

#include "lofi.h"
#include "nrf24.h"
#include "uartbb.h"
#include "xprintf.h"

#undef F_CPU
#define F_CPU 1000000UL

const uint16_t revision = 7;	// increment this for each release

int clk_div = 3;


// GLOBAL VARIABLES --------------------------------------------
uint8_t				gstatus;

sensor_switch_t		sens_sw1;
sensor_ctr_t		sens_ctr;
sensor_ctr_t		sens_rev;
sensor_vcc_t		sens_vcc;
sensor_temp_t		sens_temp;
config_t			config;

// these hold how many TPL5111 periods to bypass before xmitting
uint16_t			swCnts;
uint16_t			vccCnts;
uint16_t			tempCnts;
uint16_t			ctrCnts;

uint8_t				txBuf[TXBUF_SIZE][NRF24_PAYLOAD_LEN-1];
uint8_t				txBufWr, txBufRd;

//****************************************************************
// setup_watchdog - configure watchdog timeout
//
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(void)
{
    uint8_t bb;
//	uint8_t val = 9;
#if 0
    if (val > 9 )
        return;     //val = 9;
    bb = val & 7;
    if (val > 7)
        bb |= (1<<5);
    bb |= (1<<WDCE);
#endif
	bb = 0x21 | (1<<WDCE);

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
	static uint8_t wdTick = 0xF0;

	if (++wdTick >= 7 ) { //config.wdCntsMax) {
		wdTick = 0;
		FLAGS |= WD_FLAG;
	}
}

//****************************************************************
// pinChange_isr
//   for PA0..PA7
//   DRVn -> PA2	// for version 2
//   SW1  -> PA7	// for version 3
//   not enabled for PWB_REV == 0
ISR(PCINT0_vect)
{
	if (PWB_REV == 2) {
		FLAGS |= WD_FLAG;
	} else if (PWB_REV == 3) {
		FLAGS |= SW_FLAG;
	}
}

//****************************************************************
// pinChange_isr
//   for PB0..PB3
//   SW1  -> PB2	// for version 2
//   DRVn -> PB0	// for version 3
ISR(PCINT1_vect)
{
	if (PWB_REV == 0) {
		FLAGS |= SW_FLAG;
	} else if (PWB_REV == 2) {
		FLAGS |= SW_FLAG;
	} else if (PWB_REV == 3) {
		FLAGS |= WD_FLAG;
	}
}

//****************************************************************
// main
//
int main(void)
{

	gstatus = 0;
    FLAGS = 0;
	PWB_REV = 0xff;		// until it can get properly initialized from eeprom

	// initialize uart buf indices
	txBufRd = 0;
	txBufWr = 0;

	// Set Divide by 8 for 8MHz RC oscillator 
	// to get a 1MHz F_CPU
	CORE_CLK_SET(CORE_FAST);

	// disable Pullups
	MCUCR &= ~(1<<PUD);

	// set unused pins to output low
	init_unused_pins();

	// turn off analog comparator
	ACSR = 0x80;

	// reduce power on TIMER1
	PRR |= (1<<PRTIM1);

	// reduce power of USI peripheral
	PRR |= (1<<PRUSI);

	// read the config params from eeprom
	eeprom_read_block(&config, 0, sizeof(config));
	PWB_REV = config.pwbRev;

	// initialze pin to reset TPL5111
	tpl_done_init();

    // init LED pin as OUTPUT
	// LED pin is a shared resource with txDbg
	led_init();		// set as output/high even if not used
	// init hardware SPI pins for talking to radio
	nrf24_init();
    
    // initialize uart if eeprom configured
	if (config.en_txDbg) {
		uartbb_init();
		xfunc_out = uartbb_putchar;
	} else {
		// disable timer0 to reduce power
		PRR |= (1<<PRTIM0);
	}

	// Initialize revision message structure
	rev_msg_init();

	// Initialize counter message structure
	ctr_msg_init();

	// Initialize Temperature message structure
	temp_msg_init();

	// Initialize Vcc message structure
	vcc_msg_init();

	// Initialize switch 1 message structure
	sw1_msg_init();

	// Enable TPL5111 DVRn pin change
	tpl_drv_init();

	// switch to a lower clock rate while reading/writing NRF. For some
	// reason I can't get anywhere near 10MHz SPI CLK rate.
	CORE_CLK_SET(CORE_SLOW);

	// Initialize radio channel and payload length
	nrf24_config(&config, NRF24_PAYLOAD_LEN);
	nrf24_flush_tx();

	// Power off radio as we go into our first sleep
	nrf24_powerDown();            

	CORE_CLK_SET(CORE_FAST);

    // initialize delay counts so that first time through loop
	// all enabled messages will be transmitted
    swCnts = config.swCntsMax - 1;
	ctrCnts = config.ctrCntsMax - 1;
	vccCnts = config.vccCntsMax - 1;
	tempCnts = config.tempCntsMax - 1;
	if (PWB_REV == 0) {
		setup_watchdog();
//		setup_watchdog(config.wd_timeout + 5);
	}

	// set sleep mode one time here
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here

	// Enable interrupts
    sei();

	// must be called after global interrupts are enabled
	if (config.en_txDbg) {
		printConfig();
		_delay_loop_2(25000); // ~100ms at 1MHz F_CPU
		// disable timer0 to reduce power
		PRR |= (1<<PRTIM0);
	}

	// fake a watchdog/TPL interrupt
	FLAGS = WD_FLAG;

	//
	// Start of main loop
	//
	while (1) {

		// can only execute this code if TPL5111 DRVn asserted
		// check to see if it is time to xmit a pkt...
		if (FLAGS & WD_FLAG)	{
			tpl_done_pulse();
			flags_update();
			// build messages to xmit
			msgs_build(0);
			FLAGS &= ~WD_FLAG;
		} else {
			// build messages to xmit
			msgs_build(1);
		}


		// Set Divide by 8 for 8MHz RC oscillator 
		CORE_CLK_SETi(CORE_SLOW);

		nrf24_powerUpTx();
		dlyMS(4);

		while (txBufRd != txBufWr) {	// enable to send all pkts
			int i;

			nrf24_clearStatus();

			/* Automatically goes to TX mode */
			nrf24_send(&config, &txBuf[txBufRd][0], NRF24_PAYLOAD_LEN-1);        

			// Bump the read index
			txBufRd = (txBufRd + 1) & (TXBUF_SIZE - 1);

			/* Start the transmission */
			nrf24_pulseCE();

			// spin waiting for xmit to complete with good or max retries set
			i = 0;
			while (nrf24_isSending() && i < 100) {
				i++;
			}
			if (i >= 100) {
				led_assert();
				dlyMS(1000);
				led_deassert();
			}

			if (config.en_led) {
				if (config.en_aa) {
					if (config.en_led_nack) {
						if (gstatus &  (1<<MAX_RT)) {
							led_assert();
							_delay_loop_1(10); // ~100us at 1MHz F_CPU
							led_deassert();
						}
					} else if (gstatus & (1<<TX_DS)) {
						led_assert();
						_delay_loop_1(10); // ~100us at 1MHz F_CPU
						led_deassert();
					}
				}
			} else if (config.en_led_nack) {
				led_assert();
				_delay_loop_1(10); // ~100us at 1MHz F_CPU
				led_deassert();
			}

        } //endof: while (txBufRd != TxBufWr) {

		nrf24_powerDown();            

		CORE_CLK_SETi(CORE_FAST);

		// go to sleep and wait for interrupt (tpl5111 DRVn or switch pin change)
		if (FLAGS == 0) {
			sleep_mode();                // System sleeps here
		}
    } //endof: while (1) {

    return 0;
}

//
// print nrf24l01+ configuration out serial port
//
void printConfig(void)
{
	uint8_t  ta[8];

    xprintf("\nnrf config:\n");
	xprintf("00:%02x", nrf24_rdReg(0));
    xprintf(" %02x", nrf24_rdReg(1));
    xprintf(" %02x", nrf24_rdReg(2));
    xprintf(" %02X", nrf24_rdReg(3));
	xprintf(" %02X", nrf24_rdReg(4));
    xprintf(" %02X", nrf24_rdReg(5));
    xprintf(" %02X", nrf24_rdReg(6));
    xprintf(" %02X", nrf24_rdReg(7));
    xprintf(" %02X\n", nrf24_rdReg(8));
    xprintf("09:%02X\n", nrf24_rdReg(9));
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
// readVccTemp - read Vcc (indirectly) or Temp through ADC subsystem
//
// Returns the 10-bit ADC value.
// On each reading we: enable the ADC, take the measurement,
// and then disable the ADC for power savings.
// This takes >1ms because the internal reference voltage must
// stabilize each time the ADC is enabled.
// For faster readings, you could initialize once, and then take
// multiple fast readings, just make sure to disable the ADC
// before going to sleep so you don't waste power. 
// I got this technique from:
//   http://wp.josh.com/2014/11/06/battery-fuel-guage-with-zero-
//   parts-and-zero-pins-on-avr/
//
uint16_t readVccTemp(uint8_t mux_select)
{
	
	PRR &= ~(1<<PRADC);

	// Select ADC inputs
	// bit    76543210 
	// REFS = 00       = Vcc used as Vref (for Vcc measurement)
	// MUX  =   100001 = Single ended, 1.1V (Internal Ref) as Vin
	// REFS = 10       = 1.1V used as Vref (for Temp measurement)
	// MUX  =   100010 = Single ended, chan 8 (internal Temp sensor) as Vin
	
	ADMUX = mux_select;
	
	// By default, the successive approximation circuitry requires an
	// input clock frequency between 50 kHz and 200 kHz to get maximum resolution.
				
	// Enable ADC, set prescaller to /8 which will give a ADC clock of 1MHz/8 = 125KHz
	
	ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);
	
	// After switching to internal voltage reference the ADC requires
	// a settling time of 1ms before measurements are stable.
	// Conversions starting before this may not be reliable. The ADC must
	// be enabled during the settling time.
		
	_delay_loop_2(250); // ~1ms at 1MHz F_CPU
				
	// The first conversion after switching voltage source may be inaccurate,
	// and the user is advised to discard this result.
	// First conversion after disable/enable will be an extended conversion.
		
	ADCSRA |= _BV(ADSC);				// Start a conversion
	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion...
										//..and ignore the result
	ADCSRA |= _BV(ADSC);				// Start a conversion
	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 2nd conversion...
						
		
	// After the conversion is complete (ADIF is high), the conversion
	// result can be found in the ADC.
	// Result Registers (ADCL, ADCH).		
		
	// When an ADC conversion is complete, the result is found in these two registers.
	// When ADCL is read, the ADC Data Register is not updated until ADCH is read.		
	
	// Note we could have used ADLAR left adjust mode and then only needed
	// to read a single byte here
		
	uint8_t low  = ADCL;
	uint8_t high = ADCH;

	uint16_t adc = (high << 8) | low;		// 0<= result <=1023
			
	// Compute a fixed point with 1 decimal place (i.e. 5v= 50)
	//
	// Vcc   =  (1.1v * 1024) / ADC
	// Vcc10 = ((1.1v * 1024) / ADC ) * 10			->convert to 1 decimal fixed point
	// Vcc10 = ((11   * 1024) / ADC )				->simplify to all 16-bit integer math
				
	// let the receiving software compute floating point value
	// uint8_t vccx10 = (uint8_t) ( (11 * 1024) / adc); 
	
	// Note that the ADC will not automatically be turned off when
	// entering other sleep modes than Idle
	// mode and ADC Noise Reduction mode. The user is advised to write
	// zero to ADEN before entering such
	// sleep modes to avoid excessive power consumption.
	
	ADCSRA &= ~_BV( ADEN );			// Disable ADC to save power
	PRR |= (1<<PRADC);
	
	return ( adc );
	
}

//
// initialze counter message structure
//
void rev_msg_init(void)
{
	// Initialize counter capability/structure if eeprom configured
    sens_rev.sensorId = SENID_REV;
    sens_rev.ctr_lo = (revision & 0xff);
    sens_rev.ctr_hi = ((revision >> 8) & 0x3);
	sens_rev.seq = 0;
}

//
// initialze counter message structure
//
void ctr_msg_init(void)
{
	// Initialize counter capability/structure if eeprom configured
    if (config.en_ctr) {
        sens_ctr.sensorId = SENID_CTR;
        sens_ctr.ctr_lo = 0;
        sens_ctr.ctr_hi = 0;
		sens_ctr.seq = 0;
    }
}

//
// initialze Temperature message structure
//
void temp_msg_init(void)
{
	// Initialize Temp capability/structure if eeprom configured
	if (config.en_temp) {
        sens_temp.sensorId = SENID_TEMP;
        sens_temp.temp_lo = 0;
        sens_temp.temp_hi = 0;
		sens_temp.seq = 0;
    }
}

//
// initialze Vcc message structure
//
void vcc_msg_init(void)
{
	// Initialize Vcc capability/structure if eeprom configured
    if (config.en_vcc) {
        sens_vcc.sensorId = SENID_VCC;
        sens_vcc.vcc_lo = 0;
        sens_vcc.vcc_hi = 0;
		sens_vcc.seq = 0;
    }
}

//
// initialze switch 1 message structure
//
void sw1_msg_init(void)
{
	// Initialize switch 1 capability/structure if eeprom configured
    if (config.en_sw1) {
		init_switch();
		sens_sw1.sensorId = SENID_SW1;
		sens_sw1.seq = 2; // init to 2 because it is called 2 times before first xmit
		getSw1(0);
		if (config.sw1_pc) {
			init_switch_PC();
		}
    } else {
		// if we leave pin as input, it will draw more current if it oscillates
		// but if we pgm pin as output and there REALLY is a switch connected
		// we are would  do damage. That is why there is a series resistor on
		// switch pin.
		safe_switch();
	}
}

//
// Update flags to know what packets to build for xmission
//
void flags_update(void)
{
	// inc count and set flag if time to xmit a message
	if (config.en_sw1) {
		if (++swCnts >= config.swCntsMax) {
			swCnts = 0;
			FLAGS |= SW_FLAG;
		}
	}
	// inc count and set flag if time to xmit a message
	if (config.en_ctr) {
		if (++ctrCnts >= config.ctrCntsMax) {
			ctrCnts = 0;
			FLAGS |= CTR_FLAG;
		}
	}
	// inc count and set flag if time to xmit a message
	if (config.en_vcc) {
		if (++vccCnts >= config.vccCntsMax) {
			vccCnts = 0;
			FLAGS |= VCC_FLAG;
		}
	}
	// inc count and set flag if time to xmit a message
	if (config.en_temp) {
		if (++tempCnts >= config.tempCntsMax) {
			tempCnts = 0;
			FLAGS |= TEMP_FLAG;
		}
	}
}

//
// read switch one and update state
//
uint8_t  getSw1(uint8_t pc_triggered)
{
	register uint8_t pin_debounce = 0xaa;

	// debounce read switch
	do {
		pin_debounce <<= 1;
//		_delay_loop_1(10); // ~100us at 1MHz F_CPU
		pin_debounce |= read_switch();
	} while ((pin_debounce != 0) && (pin_debounce != 0xff));

	pin_debounce &= 1;
	sens_sw1.lastState = pc_triggered;
	sens_sw1.closed = config.sw1_rev ^ pin_debounce;
	sens_sw1.seq++;
	return (*(uint8_t *)&sens_sw1);
}

//
// Build and queue packets for xmission
//
void msgs_build(int pc_triggered)
{

	// build a Switch message if flag set
	if (FLAGS & SW_FLAG) {
		FLAGS &= ~SW_FLAG;
		txBuf[txBufWr][0] = getSw1(pc_triggered);
		txBuf[txBufWr][1] = 0;
		txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);
	}

	// build a Vcc message if flag set and a rev msg
	//if (sens_rev.seq == 0) {
	if (FLAGS & VCC_FLAG) {
		memcpy(&txBuf[txBufWr][0], &sens_rev, sizeof(sens_rev));
		txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);
		sens_rev.seq++;
//	}
//	if (FLAGS & VCC_FLAG) {
		int16_t vcc = readVccTemp(VCC_MUX);
		vcc += config.vccFudge;
		FLAGS &= ~VCC_FLAG;
		sens_vcc.vcc_lo = vcc & 0xFF;
		sens_vcc.vcc_hi = (vcc>>8) & 0x3;
		sens_vcc.seq++;
		memcpy(&txBuf[txBufWr][0], &sens_vcc, sizeof(sens_vcc));
		txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);

	}

	// build a Temperature message if flag set
	if (FLAGS & TEMP_FLAG) {
		int16_t temp = readVccTemp(TEMP_MUX);
		temp += config.tempFudge;
		FLAGS &= ~TEMP_FLAG;
		sens_temp.temp_lo = temp & 0xFF;
		sens_temp.temp_hi = (temp>>8) & 0x3;
		sens_temp.seq++;
		memcpy(&txBuf[txBufWr][0], &sens_temp, sizeof(sens_temp));
		txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);
	}

	// build a counter message if flag set
	if (FLAGS & CTR_FLAG) {
		FLAGS &= ~CTR_FLAG;
		memcpy(&txBuf[txBufWr][0], &sens_ctr, sizeof(sens_ctr));
		sens_ctr.seq++;
		if (++sens_ctr.ctr_lo == 0)
			sens_ctr.ctr_hi++;
		txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);
	}
}

//
// a milli-second delay routine (spin loop)
//
void dlyMS(uint16_t ms)
{
	uint16_t val = 2000>>clk_div;
	val -= ms;
	while(ms--) {
		_delay_loop_2(val);
	}
}

//
// define TPL_DONE pin and macro; must be at least 0.1us wide
// and rising edge must be at least 0.1us later that rising edge of DRV
//
void tpl_done_init(void)
{
	if (PWB_REV == 2 || PWB_REV == 3) {
		DDRB |= (1<<1);
		PORTB &= ~(1<<1);
	}
	// do nothing for PWB_REV == 0
	// it is GREEN LED
}

// define TPL_DONE pin and macro; must be at least 0.1us wide
// and rising edge must be at least 0.1us later that rising edge of DRV
void tpl_done_pulse(void)
{
	if (PWB_REV == 2 || PWB_REV == 3) {
		PORTB |= (1<<1);
		PORTB &= ~(1<<1);
	}
	// don't do anything for PWB_REV == 0
}

void led_init(void)
{
	if (PWB_REV == 0) {
		DDRB |= (1<<0);
		PORTB |= (1<<0);
	} else if (PWB_REV == 2 || PWB_REV == 3) {
		DDRA |= (1<<3);
		PORTA |= (1<<3);
	}
}

void led_deassert(void)
{
	if (PWB_REV== 0) {
		PORTB &= ~(1<<0);
	} else if (PWB_REV == 2 || PWB_REV == 3) {
		PORTA |= (1<<3);
	}
}

void led_assert(void)
{
	if (PWB_REV == 0) {
		PORTB |= (1<<0);
	} else if (PWB_REV == 2  || PWB_REV == 3) {
		PORTA &= ~(1<<3);
	}
}

uint8_t read_switch(void)
{
	if (PWB_REV == 0) {
		return ((PINB >> 2) & 1);
	} else if (PWB_REV == 2) {
		return ((PINB >> 2) & 1);
	} else if (PWB_REV == 3) {
		return ((PINA >> 7) & 1);
	}
	return 0;
}

void safe_switch(void)
{
	if (PWB_REV == 0) {
		DDRB |= (1<<2);
		PORTB &= ~(1<<2);
	} else if (PWB_REV == 2) {
		DDRB |= (1<<2);
		PORTB &= ~(1<<2);
	} else if (PWB_REV == 3) {
		DDRA |= (1<<7);
		PORTA &= ~(1<<7);
	}
}

void init_switch(void)
{
	if (PWB_REV == 0) {
		DDRB &= ~(1<<2);
	} else if (PWB_REV == 2) {
		DDRB &= ~(1<<2);
	} else if (PWB_REV == 3) {
		DDRA &= ~(1<<7);
	}
}

void init_switch_PC(void)
{
	if (PWB_REV == 0) {
		GIMSK |= (1<<5);
		PCMSK1 |= (1<<2);
	} else if (PWB_REV == 2) {
		GIMSK |= (1<<5);
		PCMSK1 |= (1<<2);
	} else if (PWB_REV == 3) {
		GIMSK |= (1<<4);
		PCMSK0 |= (1<<7);
	}
}

void tpl_drv_init(void)
{
	if (PWB_REV == 0) {
	} else if (PWB_REV == 2) {
		DDRA &= ~(1<<2);
		GIMSK |= (1<<4);
		PCMSK0 |= (1<<2);
	} else if (PWB_REV == 3) {
		DDRB &= ~(1<<0);
		GIMSK |= (1<<5);
		PCMSK1 |= (1<<0);
	}
}

void init_unused_pins(void)
{
	if (PWB_REV == 0) {
		DDRB |= (1<<1);
		PORTB &= ~(1<<1);
		DDRA |= (1<<2);
		PORTA &= ~(1<<2);
	} else if (PWB_REV == 2) {
		DDRB |= (1<<0);
		PORTB &= ~(1<<0);
		DDRA |= (1<<7);
		PORTA &= ~(1<<7);
	} else if (PWB_REV == 3) {
		DDRB |= (1<<2);
		PORTB &= ~(1<<2);
		DDRA |= (1<<2);
		PORTA &= ~(1<<2);
	}
}
