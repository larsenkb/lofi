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


int clk_div = 3;


// GLOBAL VARIABLES --------------------------------------------
uint8_t				gstatus;

sensor_switch_t		sens_sw1;
//sensor_switch_t		sens_sw2;
sensor_ctr_t		sens_ctr;
sensor_vcc_t		sens_vcc;
sensor_temp_t		sens_temp;
config_t			config;

uint16_t			swCnts;
uint16_t			vccCnts;
uint16_t			tempCnts;
uint16_t			ctrCnts;

uint8_t				txBuf[TXBUF_SIZE][NRF24_PAYLOAD_LEN-1];
uint8_t				txBufWr, txBufRd;


//****************************************************************
// pinChange_isr
//   for PB0, PB1, PB2, PB3
//   DRVn -> PB0
//   SW1  -> PB2
ISR(PCINT1_vect)
{
	uint8_t pinState;
	uint8_t pinbState;

	pinbState = PINB;

	// did TPL5111 generate interrupt?
	if (PCMSK1 & 0x01) {
		if (pinbState & 0x01) {
			FLAGS |= wdFlag;
		}
	}

	// always check the reed switch state
	pinState = (pinbState >> SWITCH_1) & 1;
    if (sens_sw1.lastState == pinState)
        FLAGS &= ~swFlag;
    else
        FLAGS |= swFlag;
}


//****************************************************************
// main
//
int main(void)
{
	speed_t		speed;


	gstatus = 0;
    FLAGS = 0;

	txBufRd = 0;
	txBufWr = 0;

	// Set Divide by 8 for 8MHz RC oscillator 
	// to get a 1MHz F_CPU
	CORE_CLK_SET(CORE_FAST);

	// disable Pullups
	MCUCR &= ~(1<<PUD);

	// turn off analog comparator
	ACSR = 0x80;

	// reduce power on TIMER1
	PRR |= (1<<PRTIM1);

	// reduce power of USI peripherla
	PRR |= (1<<PRUSI);

	// read the config params from eeprom
	eeprom_read_block(&config, 0, sizeof(config));

	// initialze pin to reset TPL5111
	DONE_INIT();

    // init LED pin as OUTPUT
	// LED pin is a shared resource with txDbg
	LED_INIT(LED);		// set as output/high even if not used

	// get desired xmit speed
	///KBL TODO move this to nrf24_config???
	if (config.spd_1M)
		speed = speed_1M;
	else if (config.spd_250K)
		speed = speed_250K;
	else
		speed = speed_2M;

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

	// Initialize counter message structure
	ctr_msg_init();

	// Initialize Temperature message structure
	temp_msg_init();

	// Initialize Vcc message structure
	vcc_msg_init();

	// Initialize switch 1 message structure
	sw1_msg_init();

// is this needed???
	DDRB &= ~(1<<0);
	GIMSK |= (1<<5);
	PCMSK1 |= (1<<0);

	// switch to a lower clock rate while reading/writing NRF. For some
	// reason I can't get anywhere near 10MHz SPI CLK rate.
	CORE_CLK_SET(CORE_SLOW);

	// Initialize radio channel and payload length
	nrf24_config(&config, NRF24_PAYLOAD_LEN, speed);
	nrf24_flush_tx();

	// Power off radio as we go into our first sleep
	nrf24_powerDown();            

    // initialize watchdog and associated variables
    swCnts = config.swCntsMax - 1;
	ctrCnts = config.ctrCntsMax - 1;
	vccCnts = config.vccCntsMax - 1;
	tempCnts = config.tempCntsMax - 1;

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

	//
	// Start of main loop
	//
	while (1) {

#if 0
		// if TPL5111 woke us up, reset it
		if (FLAGS & wdFlag) {
			DONE_PULSE();
		}
#endif

		// can only execute this code if TPL5111 DRVn asserted
		// check to see if it is time to xmit a pkt...
		if (FLAGS & wdFlag)	{
			DONE_PULSE();
			FLAGS &= ~wdFlag;
			flags_update();
		}

		// build messages to xmit
		msgs_build();

		// Set Divide by 8 for 8MHz RC oscillator 
		CORE_CLK_SETi(CORE_SLOW);

		nrf24_powerUpTx();
		dlyMS(4);

		while (txBufRd != txBufWr) {	// enable to send all pkts
			int i;

			nrf24_clearStatus();

#if 0
			if (config.en_nrfVcc) {
				// Initialize radio channel and payload length
				nrf24_config(&config, NRF24_PAYLOAD_LEN, speed);
			}
#endif

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
			

//			if (config.en_aa) {
				if (0) { //(gstatus & (1 << MAX_RT)) {        
					LED_ASSERT(LED);
					_delay_loop_1(15); // ~100us at 1MHz F_CPU
					LED_DEASSERT(LED);
					dlyMS(50); // ~100us at 1MHz F_CPU
				}
				LED_ASSERT(LED);
				_delay_loop_1(10); // ~100us at 1MHz F_CPU
				LED_DEASSERT(LED);
//			}

        } //endof: while (txBufRd != TxBufWr) {

	    nrf24_powerDown();            

		CORE_CLK_SETi(CORE_FAST);

#if 0
		if (PINA & (1<<SWITCH_2)) {
			DONE_PULSE();
		}
#endif
		// go to sleep and wait for interrupt (tpl5111 DRVn, watchdog or switch pin change)
	    sleep_mode();                // System sleeps here

    } //endof: while (1) {

    return 0;
}


//
// read switch one and update state
//
uint8_t  getSw1(void)
{
	uint8_t pinState;

	pinState = (PINB>>SWITCH_1) & 1;
	sens_sw1.closed = config.sw1_rev ^ pinState;
	sens_sw1.lastState = pinState;
	sens_sw1.seq++;
	return (*(uint8_t *)&sens_sw1);
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
	
//	ADMUX = 0b00100001;
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
				
//	uint8_t vccx10 = (uint8_t) ( (11 * 1024) / adc); 
	
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
		DDRB &= ~(1<<SWITCH_1);
		sens_sw1.sensorId = SENID_SW1;
		sens_sw1.seq = 2; // init to 2 because it is called 2 times before first xmit
		getSw1();
		if (config.sw1_pc) {
			GIMSK |= (1<<SWITCH_1_GMSK);
			PCMSK1 |= (1<<SWITCH_1_MSK);
		}
    } else {
		// if we leave pin as input, it will draw more current if it oscillates
		// but if we pgm pin as output and there REALLY is a switch connected
		// we are would  do damage. That is why there is a series resistor on
		// switch pin.
		DDRB |= (1<<SWITCH_1);
		PORTB &= ~(1<<SWITCH_1);
	}
}

//
// Update flags to know what packets to build for xmission
//
void flags_update(void)
{
	// inc count and set flag if time to xmit a message
	if (++swCnts >= config.swCntsMax) {
		swCnts = 0;
		FLAGS |= swFlag;
	}
	// inc count and set flag if time to xmit a message
	if (config.en_ctr) {
		if (++ctrCnts >= config.ctrCntsMax) {
			ctrCnts = 0;
			FLAGS |= ctrFlag;
		}
	}
	// inc count and set flag if time to xmit a message
	if (config.en_vcc) {
		if (++vccCnts >= config.vccCntsMax) {
			vccCnts = 0;
			FLAGS |= vccFlag;
		}
	}
	// inc count and set flag if time to xmit a message
	if (config.en_temp) {
		if (++tempCnts >= config.tempCntsMax) {
			tempCnts = 0;
			FLAGS |= tempFlag;
		}
	}
}


//
// Build and queue packets for xmission
//
void msgs_build(void)
{

	if (FLAGS & swFlag) {
		FLAGS &= ~swFlag;
		txBuf[txBufWr][0] = getSw1();
		txBuf[txBufWr][1] = 0;
		txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);
	}

	// build a Vcc message if flag set
   	if (FLAGS & vccFlag) {
		int16_t vcc = readVccTemp(VCC_MUX);
		vcc += config.vccFudge;
		FLAGS &= ~vccFlag;
		sens_vcc.vcc_lo = vcc & 0xFF;
		sens_vcc.vcc_hi = (vcc>>8) & 0x3;
		sens_vcc.seq++;
		memcpy(&txBuf[txBufWr][0], &sens_vcc, sizeof(sens_vcc));
		txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);

	}

	// build a Temperature message if flag set
	if (FLAGS & tempFlag) {
		int16_t temp = readVccTemp(TEMP_MUX);
		temp += config.tempFudge;
		FLAGS &= ~tempFlag;
		sens_temp.temp_lo = temp & 0xFF;
		sens_temp.temp_hi = (temp>>8) & 0x3;
		sens_temp.seq++;
		memcpy(&txBuf[txBufWr][0], &sens_temp, sizeof(sens_temp));
		txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);
	}

	// build a counter message if flag set
	if (FLAGS & ctrFlag) {
		FLAGS &= ~ctrFlag;
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
