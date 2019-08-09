//-------------------------------------------------------------
// lofi + nrf24
//-------------------------------------------------------------
// This is software that runs on a LoFi board designed by David Cook.
// See https://hackaday.io/project/1552-lofi
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
// The lofi+nrf24 combo draws ~6 uA in the idle mode.
// It can monitor two reed switches, Vcc and internal temperature.
// It uses the internal RC oscillator so timing is not very accurate.
// The switches can be configured to generate an interrupt on Pin Change
// which will trigger a transmission. Otherwise, they are only polled
// and switch state will be transmitted at the polling frequency.
// The polling period is configured in eeprom.
// A 10-bit incrementing count is sent on polled transmissions.
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

#define EN_TPL5111		1
//#define HI_AMPS			1		/* pwr on/off nrf due to damaged chip */
								/* and high current drain */

#define SWITCH_1        2       /* PORTB bit2 */
#define SWITCH_1_MSK    2
#define SWITCH_1_GMSK   5
#define SWITCH_2        2       /* PORTA bit2 */
#define SWITCH_2_MSK    2
#define SWITCH_2_GMSK   4

#define EEPROM_NODEID_ADR        ((uint8_t *)0)

#define NRF24_PAYLOAD_LEN        3

// ---------  LED MACROS  ----------
#define LED_RED					(1<<0)  // PORTB bit0
#define LED_GRN					(1<<1)  // PORTB bit1

#define LED_INIT(x)				{DDRB |= (x); PORTB &= ~(x);}
#define LED_ASSERT(x)	\
	if (config.en_led) {	\
		PORTB |= (x);	\
	}
#define LED_DEASSERT(x)	\
	if (config.en_led) {	\
		PORTB &= ~(x);	\
	}

#if EN_TPL5111
#define NRF_VCC_INIT(x)
#define NRF_VCC_ASSERT(x)
#define NRF_VCC_DEASSERT(x)
#define NRF_VCC_DLY_MS(x,y)
#else
//#define NRF_VCC_PIN				((1<<3) | (1<<7))
#define NRF_VCC_PIN				((1<<7))
void NRF_VCC_INIT(config_t *config)
{
	if (config->en_nrfVcc) {
		DDRA |= NRF_VCC_PIN;
	}
}

void NRF_VCC_ASSERT(config_t *config)
{
	if (config->en_nrfVcc) {
		DDRA |= NRF_VCC_PIN;
		PORTA |= NRF_VCC_PIN;
	}
}

void NRF_VCC_DEASSERT(config_t *config)
{
	if (config->en_nrfVcc) {
		PORTA &= ~NRF_VCC_PIN;
		DDRA &= ~NRF_VCC_PIN;
	}
}

void NRF_VCC_DLY_MS(config_t *config, uint16_t ms)
{
	if (config->en_nrfVcc) {
		_delay_loop_2(ms);
	}
}
#endif


int clk_div = 3;
#define CLK_DIV			3
#define CORE_FAST		CLK_DIV
#define CORE_SLOW		(CLK_DIV+1)	
#define CORE_CLK_SET(x)  {	\
	CLKPR = (1<<CLKPCE);	\
	CLKPR = (x);			\
	clk_div = (x); }

void dlyMS(uint16_t ms)
{
	uint16_t val = 2000>>clk_div;
	val -= ms;
	while(ms--) {
		_delay_loop_2(val);
	}
}

#define TXBUF_SIZE		8	// must be a power of 2!!!


/* ------------------------------------------------------------- */
uint8_t				gstatus;

/* ------------------------------------------------------------- */
sensor_switch_t		sens_sw1;
sensor_switch_t		sens_sw2;
config_t			config;


/* FORWARD DECLARATIONS ---------------------------------------- */

uint16_t readVccTemp(uint8_t mux_select);
//uint16_t readTemperature(void);
void printConfig(void);
uint8_t getSw1(void);
uint8_t getSw2(void);


//****************************************************************
// setup_watchdog - configure watchdog timeout
//
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
#if 1
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

void watchdog_disable(void)
{
	uint8_t bb,bb2;

	bb  = (1<<WDCE) | (1<<WDE);
	bb2 = 0;
	WDTCSR = bb;
	WDTCSR = bb2;
}
#endif

//****************************************************************
// watchdog_isr
//
// This runs each time the watch dog wakes us up from sleep
// The system wakes up when any interrupt occurs. Setting this
// flag lets background (main loop) know that the watchdog
// interrupt occured so that we can xmit.
#if 1
ISR(WATCHDOG_vect)
{
	FLAGS |= wdFlag;

	       
	LED_ASSERT(LED_RED);
	LED_ASSERT(LED_GRN);
	dlyMS(40);
	LED_DEASSERT(LED_RED | LED_GRN);
}
#endif

//****************************************************************
// pinChange_isr
//
ISR(PCINT1_vect)
{
	uint8_t pinState;

	pinState = (PINB>>SWITCH_1) & 1;
    if (sens_sw1.lastState == pinState)
        FLAGS &= ~sw1Flag;
    else
        FLAGS |= sw1Flag;
}


//****************************************************************
// pinChange_isr
//
ISR(PCINT0_vect)
{
#if EN_TPL5111
//	FLAGS |= wdFlag;
#else
	uint8_t pinState;

	pinState = (PINA>>SWITCH_2) & 1;
    if (sens_sw2.lastState == pinState)
        FLAGS &= ~sw2Flag;
    else
        FLAGS |= sw2Flag;
#endif
}


//****************************************************************
// main
//
int main(void)
{
	uint8_t				txBuf[TXBUF_SIZE][NRF24_PAYLOAD_LEN-1];
	uint8_t				txBufWr, txBufRd;
	speed_t				speed;
	sensor_ctr_t		sens_ctr;
	sensor_vcc_t		sens_vcc;
	sensor_temp_t		sens_temp;
	uint16_t			swCnts;
	uint16_t			vccCnts;
	uint16_t			tempCnts;
	uint16_t			ctrCnts;
	uint8_t				jj;


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

	// read the config params from eeprom
	eeprom_read_block(&config, 0, sizeof(config));

#if EN_TPL5111
	config.en_nrfVcc = 0;
	// set DONE as output/low
	DDRA |= (1<<3);
	PORTA &= ~(1<<3);
#endif

    // init LED pins as OUTPUT
	LED_INIT(LED_GRN | LED_RED);		// set as output even if not used

	// get desired xmit speed
	///KBL TODO move this to nrf24_config???
	if (config.spd_1M)
		speed = speed_1M;
	else if (config.spd_250K)
		speed = speed_250K;
	else
		speed = speed_2M;

	// init hardware pins for talking to radio
	nrf24_init();
    
    // initialize uart if eeprom configured
	if (config.en_txDbg) {
		uartbb_init();
		xfunc_out = uartbb_putchar;
	} else {
		// disable timer0 to reduce power
		PRR |= (1<<PRTIM0);
	}

	// Initialize counter capability/structure if eeprom configured
    if (config.en_ctr) {
        sens_ctr.sensorId = SENID_CTR;
        sens_ctr.ctr_lo = 0;
        sens_ctr.ctr_hi = 0;
		sens_ctr.seq = 0;
		ctrCnts = 0;
    }

	// Initialize Temp capability/structure if eeprom configured
	if (config.en_temp) {
        sens_temp.sensorId = SENID_TEMP;
        sens_temp.temp_lo = 0;
        sens_temp.temp_hi = 0;
		sens_temp.seq = 0;
		tempCnts = 0;
    }

	// Initialize Vcc capability/structure if eeprom configured
    if (config.en_vcc) {
        sens_vcc.sensorId = SENID_VCC;
        sens_vcc.vcc_lo = 0;
        sens_vcc.vcc_hi = 0;
		sens_vcc.seq = 0;
		vccCnts = 0;
    }

	// Initialize switch 1 capability/structure if eeprom configured
    if (config.en_sw1) {
		DDRB &= ~(1<<SWITCH_1);
		sens_sw1.sensorId = SENID_SW1;
		sens_sw1.seq = 2; // init to 2 because it is called 2 times before first xmit
		getSw1();
		if (config.sw1_pc) {
			GIMSK = (1<<SWITCH_1_GMSK);
			PCMSK1 = (1<<SWITCH_1_MSK);
		}
    } else {
		// if we leave pin as input, it will draw more current if it oscillates
		// but if we pgm pin as output and there REALLY is a switch connected
		// we are would  do damage. That is why there is a series resistor on
		// switch pin.
		DDRB |= (1<<SWITCH_1);
		PORTB &= ~(1<<SWITCH_1);
	}

#if EN_TPL5111
	config.en_sw2 = 0;
	DDRA &= ~(1<<SWITCH_2);
	GIMSK |= (1<<SWITCH_2_GMSK);
	PCMSK0 = (1<<SWITCH_2_MSK);
#else

	// Initialize switch 2 capability/structure if eeprom configured
    if (config.en_sw2) {
		DDRA &= ~(1<<SWITCH_2);
        sens_sw2.sensorId = SENID_SW2;
		sens_sw2.seq = 2; // init to 2 because it is called 2 times before first xmit
		getSw2();
		if (config.sw2_pc) {
			GIMSK |= (1<<SWITCH_2_GMSK);
			PCMSK0 = (1<<SWITCH_2_MSK);
		}
    } else {
		// if we leave pin as input, it will draw more current if it oscillates
		// but if we pgm pin as output and there REALLY is a switch connected
		// we are would  do damage. That is why there is a series resistor on
		// switch pin.
		DDRA |= (1<<SWITCH_2);
		PORTA &= ~(1<<SWITCH_2);
    }
#endif

	// initialize pins and apply power to NRF
	NRF_VCC_INIT(&config);
	NRF_VCC_ASSERT(&config);
	NRF_VCC_DLY_MS(&config, 25000);

	// switch to a lower clock rate while reading/writing NRF. For some
	// reason I can't get anywhere near 10MHz SPI CLK rate.
	CORE_CLK_SET(CORE_SLOW);

	// Initialize radio channel and payload length
	nrf24_config(&config, NRF24_PAYLOAD_LEN, speed);
	nrf24_flush_tx();

	// Power off radio as we go into our first sleep
	nrf24_powerDown();            
	NRF_VCC_DEASSERT(&config);

    // initialize watchdog and associated variables
    swCnts = config.swCntsMax - 1; //0;
	ctrCnts = config.ctrCntsMax - 1;
	vccCnts = config.vccCntsMax - 1;
	tempCnts = config.tempCntsMax - 1;

#if EN_TPL5111
	config.en_wd = 0;
	watchdog_disable();
#else
	if (config.en_wd) {
		setup_watchdog(config.wd_timeout + 5);
	}
#endif

	// set sleep mode one time here
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here

	// Enable interrupts
    sei();

	// must be called after global interrupts are enabled
	if (config.en_txDbg) {
		printConfig();
		_delay_loop_2(25000); // ~100ms at 1MHz F_CPU
	}

	//
	// Start of main loop
	//
	while (1) {

#if EN_TPL5111
		if (PINA & (1<<SWITCH_2)) {
			FLAGS |= wdFlag;
			// assert DONE for TPL5111
//			GIMSK &= ~(1<<SWITCH_2_GMSK);
//			PCMSK0 &= ~(1<<SWITCH_2_MSK);
//			PCMSK0 = 0;
			PORTA |= (1<<3);
//			_delay_loop_1(15);
			PORTA &= ~(1<<3);
//			_delay_loop_1(15);
//			GIMSK |= (1<<SWITCH_2_GMSK);
//			PCMSK0 = (1<<SWITCH_2_MSK);
		}
#endif


		if (FLAGS & wdFlag)	{	// can only enter here if en_wd is true
			FLAGS &= ~wdFlag;
			if (++swCnts >= config.swCntsMax) {
				swCnts = 0;
				FLAGS |= swFlag;
			}
			if (config.en_ctr) {
				if (++ctrCnts >= config.ctrCntsMax) {
					ctrCnts = 0;
					FLAGS |= ctrFlag;
				}
			}
			if (config.en_vcc) {
				if (++vccCnts >= config.vccCntsMax) {
					vccCnts = 0;
					FLAGS |= vccFlag;
				}
			}
			if (config.en_temp) {
				if (++tempCnts >= config.tempCntsMax) {
					tempCnts = 0;
					FLAGS |= tempFlag;
				}
			}
		}

		jj = 0;

		if (config.en_sw1) {
			if (FLAGS & (swFlag | sw1Flag)) {
				FLAGS &= ~sw1Flag;
				txBuf[txBufWr][jj++] = getSw1();
			}
		}

		if (config.en_sw2) {
			if (FLAGS & (swFlag | sw2Flag)) {
				FLAGS &= ~sw2Flag;
				txBuf[txBufWr][jj++] = getSw2();
			}
		}

		FLAGS &= ~swFlag;

		if (jj) {
			if (jj == 1) {
				txBuf[txBufWr][jj] = 0;
			}
			txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);
		}


		if (FLAGS & ctrFlag) {
			FLAGS &= ~ctrFlag;
            memcpy(&txBuf[txBufWr][0], &sens_ctr, sizeof(sens_ctr));
			sens_ctr.seq++;
            if (++sens_ctr.ctr_lo == 0)
                sens_ctr.ctr_hi++;
			txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);
        }

	   	if (FLAGS & vccFlag) {
            uint16_t vcc = readVccTemp(VCC_MUX);
			vcc += config.vccFudge;
			FLAGS &= ~vccFlag;
            sens_vcc.sensorId = SENID_VCC;
            sens_vcc.vcc_lo = vcc & 0xFF;
            sens_vcc.vcc_hi = (vcc>>8) & 0x3;
            memcpy(&txBuf[txBufWr][0], &sens_vcc, sizeof(sens_vcc));
			txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);

		}

		if (FLAGS & tempFlag) {
            uint16_t temp = readVccTemp(TEMP_MUX);
			temp += config.tempFudge;
			FLAGS &= ~tempFlag;
            sens_temp.sensorId = SENID_TEMP;
            sens_temp.temp_lo = temp & 0xFF;
            sens_temp.temp_hi = (temp>>8) & 0x3;
            memcpy(&txBuf[txBufWr][0], &sens_temp, sizeof(sens_temp));
			txBufWr = (txBufWr + 1) & (TXBUF_SIZE - 1);
		}

//		while (txBufRd != txBufWr) {	// enable to send all pkts
		if (txBufRd != txBufWr) {		// enable to send 1 pkt per WDT
			int i;

			// Set Divide by 8 for 8MHz RC oscillator 
			cli();
			CORE_CLK_SET(CORE_SLOW);
			sei();

			NRF_VCC_ASSERT(&config);
			NRF_VCC_DLY_MS(&config, 1250); // running at 500kHz
			nrf24_powerUpTx();
			dlyMS(4);
			nrf24_clearStatus();

			if (config.en_nrfVcc) {
				// Initialize radio channel and payload length
				nrf24_config(&config, NRF24_PAYLOAD_LEN, speed);
			}

		    /* Automatically goes to TX mode */
			nrf24_send(&config, &txBuf[txBufRd][0], NRF24_PAYLOAD_LEN-1);        

			/* Start the transmission */
			nrf24_pulseCE();
	//ASSERT_CE();
//	nrf24_isSending();
//	_delay_loop_2(250);

			i = 0;
			while (nrf24_isSending() && i < 100) {
				i++;
			}
			
	//DEASSERT_CE();

		    nrf24_powerDown();            

			NRF_VCC_DEASSERT(&config);

			cli();
			CORE_CLK_SET(CORE_FAST);
			sei();

//			if (config.en_aa) {
				if (gstatus & (1 << MAX_RT)) {        
					LED_ASSERT(LED_RED);
				} else {
					LED_ASSERT(LED_GRN);
					// Bump the read index
					txBufRd = (txBufRd + 1) & (TXBUF_SIZE - 1);
				}

				_delay_loop_1(33); // ~100us at 1MHz F_CPU
				LED_DEASSERT(LED_RED | LED_GRN);
//			}

        } //endof: while (txBufRd != TxBufWr) {

		// go to sleep and wait for interrupt (watchdog or pin change)
	    sleep_mode();                // System sleeps here

    } //endof: while (1) {

    return 0;
}

uint8_t  getSw1(void)
{
	uint8_t pinState;

	pinState = (PINB>>SWITCH_1) & 1;
	sens_sw1.closed = config.sw1_rev ^ pinState;
	sens_sw1.lastState = pinState;
	sens_sw1.seq++;
	return (*(uint8_t *)&sens_sw1);
}

uint8_t getSw2(void)
{
	uint8_t pinState;

	pinState = (PINA>>SWITCH_2) & 1;
	sens_sw2.closed = config.sw2_rev ^ pinState;
	sens_sw2.lastState = pinState;
	sens_sw2.seq++;
	return (*(uint8_t *)&sens_sw2);
}

void printConfig(void)
{
	uint8_t  ta[8];

    xprintf("\nnrf config:\n");
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

