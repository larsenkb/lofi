//-------------------------------------------------------------
// uartbb.c
//-------------------------------------------------------------
// This is bit bang uart software for ATtiny84A
//
//
//
//
//
//---------------------------------------------------------

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>

#include "uartbb.h"

// declare the tx buffer (whose size is always a power of two)
static char                 uartbb_tx[1<<UARTBB_TX_PWR];
// keep track of current/next state of bit bang state machine
static uint8_t              uartbb_state;
volatile uint8_t            uartbb_next_state;
// keep track of which bit of the byte is being transmitted
static uint8_t		        uartbb_cnt;
// current byte that is being serialzed
static uint8_t		        uartbb_val;
// read/write indices into the tx buffer
static volatile uint8_t     uartbb_tx_ridx;
static volatile uint8_t     uartbb_tx_widx;


//****************************************************************
// uartbb_init - initialize bit bang uart capability
//
// We will be using timer 0.
//
void uartbb_init(void)
{

	// TX-Pin as output
	UARTBB_TXDDR |=  (1<<UARTBB_TXBIT);
	uartbb_next_state = UARTBB_STATE_IDLE;
	uartbb_state = UARTBB_STATE_IDLE;
	uartbb_tx_ridx = uartbb_tx_widx = 0;
    SET_TX();

	// initialize timer
	PRR &= ~(1<<PRTIM0);
    OCR0A = UARTBB_TIMERTOP;
    TCNT0 = 0;

    TCCR0A = 0x02;
    TCCR0B = 0x00;

    TIFR0 |= (1<<OCF0A);
    TIMSK0 |= (1<<OCIE0A);
}

//****************************************************************
// timer0_isr - interrupts at the bit rate 
//
ISR( TIM0_COMPA_vect )
{
	// move to next state
    uartbb_state = uartbb_next_state;

	// execute code for current state
	switch (uartbb_state) {
	case UARTBB_STATE_START:
		CLR_TX();
		uartbb_next_state = UARTBB_STATE_CHAR;
		uartbb_cnt = 8;
		uartbb_val = uartbb_tx[uartbb_tx_ridx];
		uartbb_tx_ridx = (uartbb_tx_ridx + 1) & ((1<<UARTBB_TX_PWR) - 1);
		break;
	case UARTBB_STATE_CHAR:
		if (uartbb_val & 0x01) { SET_TX(); }
	   	else                   { CLR_TX(); }
		uartbb_val >>= 1;
		if (--uartbb_cnt == 0) {
			uartbb_next_state = UARTBB_STATE_STOP;
		}
		break;
	case UARTBB_STATE_STOP:
		SET_TX();
		if (uartbb_tx_ridx != uartbb_tx_widx) {
			uartbb_next_state = UARTBB_STATE_START;
		} else {
            uartbb_next_state = UARTBB_STATE_IDLE;
		}
        break;
    case UARTBB_STATE_IDLE:
		// disable interrupt
//        TIMSK0 &= ~(1<<OCIE0A);
//	    UARTBB_T_INTCTL_REG &= ~UARTBB_CMPINT_EN_MASK;
		/* stop timer */
        TCCR0B = 0x00;
        break;
	default:
		break;
	}
}

//****************************************************************
// uartbb_putchar - transmit one byte 
// 
// Put the character to transmit into the transmit buffer and start
// the bit bang tx state machine, if not already running
//
void uartbb_putchar(char val)
{
	uint8_t tidx;

	// calculate where the write index will be after putting byte into buffer
	// if calculated write index equals read index - buffer is full
	// (buffer is full with one empty slot - this way we don't need a count
	// variable and buffer wrap is simpler)
	tidx = (uartbb_tx_widx + 1) & ((1<<UARTBB_TX_PWR)-1);
#if 1
	// delay until there is room in buffer
    while (tidx == uartbb_tx_ridx) {
        _delay_loop_1(255);
    }
#else
	// drop character and return
    if (tidx == uartbb_tx_ridx)
		return;
#endif

	// put character in buffer and update write index
	uartbb_tx[uartbb_tx_widx] = val;
	uartbb_tx_widx = tidx;

    // get the xmit ball rolling if we are in idle state
	if (uartbb_next_state == UARTBB_STATE_IDLE) {
		uartbb_next_state = UARTBB_STATE_START;
		// clear timer
        TCNT0 = 0;
		// start timer
        TCCR0B = 0x01;
	}
}

#if 0
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
#endif

