/*
 * NoPartsBatteryGuageAVR.c
 *
 * This is a simplified demonstration of how to detect power supply voltage by using the on chip 
 * analog-to-digital converter to measure the voltage of the internal band-gap reference voltage. 
 *
 * The code will read the current power supply voltage, and then blink an LED attached to pin 6 (PA7). 
 *
 * 1 blink  = 1 volts <= Vcc < 2 volts	(only applicable on low voltage parts like ATTINY84AV)
 * 2 blinks = 2 volts <= Vcc < 3 volts 
 * 2 blinks = 2 volts <= Vcc < 3 volts
 * 3 blinks = 3 volts <= Vcc < 4 volts
 * 4 blinks = 4 volts <= Vcc < 5 volts
 * 5 blinks = 5 volts <= Vcc  
 *
 * 0 blinks = power supply turned off :) 
 *
 * This code was tested on an ATTINY84A with default fuse settings, but should work unchanged on 
 * any ATTINYx4 or ATTINYx4A, and should be easily ported to any 8-bit AVR with ADC and internal 1.1 reference voltage.
 *
 * More info at...
 *
 * http://wp.josh.com/2014/11/06/battery-fuel-guage-with-zero-parts-and-zero-pins-on-avr/
 *
 */ 


//Comments in slash/asterisk form are quoted from the datasheet

/*
The device is shipped with CKSEL = “0010”, SUT = “10”, and CKDIV8 programmed. The default
clock source setting is therefore the Internal Oscillator running at 8.0 MHz with longest start-up
time and an initial system clock prescaling of 8, resulting in 1.0 MHz system clock.
*/

#define F_CPU 1000000

#include <avr/io.h>
#include <util/delay.h>


// Returns the current Vcc voltage as a fixed point number with 1 implied decimal places, i.e.
// 50 = 5 volts, 25 = 2.5 volts,  19 = 1.9 volts
//
// On each reading we: enable the ADC, take the measurement, and then disable the ADC for power savings.
// This takes >1ms becuase the internal reference voltage must stabilize each time the ADC is enabled.
// For faster readings, you could initialize once, and then take multiple fast readings, just make sure to
// disable the ADC before going to sleep so you don't waste power. 

uint8_t readVccVoltage(void) {
	
	// Select ADC inputs
	// bit    76543210 
	// REFS = 00       = Vcc used as Vref
	// MUX  =   100001 = Single ended, 1.1V (Internal Ref) as Vin
	
	ADMUX = 0b00100001;
	
	/*
	By default, the successive approximation circuitry requires an input clock frequency between 50
	kHz and 200 kHz to get maximum resolution.
	*/	
				
	// Enable ADC, set prescaller to /8 which will give a ADC clock of 1mHz/8 = 125kHz
	
	ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);
	
	/*
		After switching to internal voltage reference the ADC requires a settling time of 1ms before
		measurements are stable. Conversions starting before this may not be reliable. The ADC must
		be enabled during the settling time.
	*/
		
	_delay_ms(1);
				
	/*
		The first conversion after switching voltage source may be inaccurate, and the user is advised to discard this result.
	*/
	
		
	ADCSRA |= _BV(ADSC);				// Start a conversion


	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...
										//..and ignore the result
						
		
	/*
		After the conversion is complete (ADIF is high), the conversion result can be found in the ADC
		Result Registers (ADCL, ADCH).		
		
		When an ADC conversion is complete, the result is found in these two registers.
		When ADCL is read, the ADC Data Register is not updated until ADCH is read.		
	*/
	
	// Note we could have used ADLAR left adjust mode and then only needed to read a single byte here
		
	uint8_t low  = ADCL;
	uint8_t high = ADCH;

	uint16_t adc = (high << 8) | low;		// 0<= result <=1023
			
	// Compute a fixed point with 1 decimal place (i.e. 5v= 50)
	//
	// Vcc   =  (1.1v * 1024) / ADC
	// Vcc10 = ((1.1v * 1024) / ADC ) * 10				->convert to 1 decimal fixed point
	// Vcc10 = ((11   * 1024) / ADC )				->simplify to all 16-bit integer math
				
	uint8_t vccx10 = (uint8_t) ( (11 * 1024) / adc); 
	
	/*	
		Note that the ADC will not automatically be turned off when entering other sleep modes than Idle
		mode and ADC Noise Reduction mode. The user is advised to write zero to ADEN before entering such
		sleep modes to avoid excessive power consumption.
	*/
	
	ADCSRA &= ~_BV( ADEN );			// Disable ADC to save power
	
	return( vccx10 );
	
}


int main(void)
{
	
	// Enable output for LED pin
	
	DDRA |= _BV(PORTA7);
	

    while(1)
    {
		
		// Read the current Vcc voltage as a 2 decimal fixed point value
		
		uint8_t vccx10 = readVccVoltage();
		
		// Convert to whole integer value (rounds down)
		
		uint8_t vcc = vccx10 / 10;
													
		// Indicate the Vcc voltage by blinking the LED Vcc times...
		
		for(int i=0;i<vcc;i++) {	
			
			PORTA |= _BV(PORTA7);		// Turn on LED
							
			_delay_ms(250);
				
			PORTA &= ~_BV(PORTA7);		// LED off
			
			_delay_ms(250);			
		
		}
		
		_delay_ms(1000);			// Pause before next round
						
	}
}
