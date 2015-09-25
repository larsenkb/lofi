#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
//#include <inavr.h>

#define F_CPU 1000000L

#include <util/delay.h>

#if 1
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#endif

#define LED_RED   0x01
#define LED_GREEN 0x02

#if 0
#define TRUE                      1
#define FALSE                     0
#define OK                        1
#define FAIL                      0

/* Variable in non-volatile memory for counting Watchdog System Resets */
unsigned char __eeprom wdr_count = 0;   // Initialize to 0 on first programming.
#define WDR_LIMIT                 3     // Watchdog System Reset count cannot exceed this limit.


/* Initialization routine */
unsigned char WDT_Initialization_as_Reset_Source( void )
{
  unsigned char reset_flags;
    
  /* Read and clear reset flags */
  reset_flags = MCUSR;                // Save reset flags.
  MCUSR       = 0;                    // Clear all flags previously set.

  /* Setup Watchdog */
  WDTCR   =   (1<<WDTIF)|(0<<WDTIE)|(1<<WDCE)|(1<<WDE )| // Set Change Enable bit and Enable Watchdog System Reset Mode.
              (1<<WDP3 )|(0<<WDP2 )|(0<<WDP1)|(0<<WDP0); // Set Watchdog timeout period to 4.0 sec.
    
  /* If no reset flags are set, runaway code wrapped back to address 0 */
  if( reset_flags == 0 ) {
    __disable_interrupt();
    for(;;);                        // Let the Watchdog time out and cause a Watchdog System Reset.
  }

  /* Check for Watchdog System Reset */
  if( reset_flags & (1<<WDRF) ) {
    wdr_count++;                    // Increase Watchdog System Reset counter.
        
    /* Has the number of subsequent Watchdog System Resets exceeded its max limits? */
    if( wdr_count >= WDR_LIMIT ) {
      return FAIL;               // Report back an error message.
    }
  }
    
  /* Clear the Watchdog System Reset Counter on power-up or external reset */
  if( reset_flags & (1<<PORF) || reset_flags & (1<<EXTRF) ) {
    wdr_count = 0;
  }
    
  return OK;                        
}
#endif

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
//  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb |= (1<<5);
  bb |= (1<<WDCE);
//  ww=bb;
//  Serial.println(ww);


  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}

//This runs each time the watch dog wakes us up from sleep
ISR(WDT_vect) {
//  watchdog_counter++;
}

int main(void)
{
  DDRB |= (LED_RED | LED_GREEN);

  // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)

  cbi( MCUCR,SE );      // sleep enable, power down mode
  cbi( MCUCR,SM0 );     // power down mode
  sbi( MCUCR,SM1 );     // power down mode
//  cbi( SMCR,SM2 );     // power down mode

  setup_watchdog(9);

  /* Perform system initialization */
//  WDT_Initialization_as_Wakeup_Source();

  /* Enable interrupts */
  sei();
//  __enable_interrupt();

  while (1) {
//    PORTB &= ~(LED_RED);
    PORTB |= (LED_GREEN);
    _delay_ms(10);
    PORTB &= ~(LED_GREEN);
    _delay_ms(480);
    
    PORTB |= (LED_RED);
    _delay_ms(10);
    PORTB &= ~(LED_RED);
    _delay_ms(480);

    system_sleep();

//setup_watchdog(6); //Setup watchdog to go off after 1sec
//sleep_mode(); //Go to sleep! Wake up 1sec later and check water

#if 0
    /* Sleep for one Watchdog Timer period */
    __watchdog_reset();     // Reset Wathdog Timer to ensure sleep for one whole Watchdog Timer period
    WDTCR |= (1<<WDTIE);    // Enable Watchdog Interrupt Mode
    __sleep();
#endif
  }
  return 0;
}

#if 0
disable BOD to save power prior to sleep
disable ADC prior to sleep
first conversion after disable/enable will be an extended conversion


#endif
