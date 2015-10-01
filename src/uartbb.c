#include <avr/io.h>
#include <avr/interrupt.h>

#include "uartbb.h"


static char                 uartbb_tx[1<<UARTBB_TX_PWR];
uint8_t		                uartbb_state;
static uint8_t		        uartbb_cnt;
static uint8_t		        uartbb_val;
static volatile uint8_t		uartbb_tx_ridx;
static volatile uint8_t		uartbb_tx_widx;

void uartbb_init(void)
{

	// TX-Pin as output
	UARTBB_TXDDR |=  (1<<UARTBB_TXBIT);
	uartbb_state = UARTBB_STATE_IDLE;
	uartbb_tx_ridx = uartbb_tx_widx = 0;
    SET_TX();

	/* initialize timer */
    OCR0A = UARTBB_TIMERTOP;
    TCNT0 = 0;

    TCCR0A = 0x02;
    TCCR0B = 0x00;

    TIFR0 |= (1<<OCF0A);
    TIMSK0 |= (1<<OCIE0A);
}

ISR( TIM0_COMPA_vect )
{
	switch (uartbb_state) {
	case UARTBB_STATE_START:
		CLR_TX();
		uartbb_state = UARTBB_STATE_CHAR;
		uartbb_cnt = 8;
		uartbb_val = uartbb_tx[uartbb_tx_ridx];
		uartbb_tx_ridx = (uartbb_tx_ridx + 1) & ((1<<UARTBB_TX_PWR) - 1);
		break;
	case UARTBB_STATE_CHAR:
		if (uartbb_val & 0x01) {
			SET_TX();
		} else {
			CLR_TX();
		}
		uartbb_val >>= 1;
		if (--uartbb_cnt == 0) {
			uartbb_state = UARTBB_STATE_STOP;
		}
		break;
	case UARTBB_STATE_STOP:
		SET_TX();
		if (uartbb_tx_ridx != uartbb_tx_widx) {
			uartbb_state = UARTBB_STATE_START;
		} else {
            uartbb_state = UARTBB_STATE_IDLE;
		}
        break;
    case UARTBB_STATE_IDLE:
		/* disable interrupt */
//        TIMSK0 &= ~(1<<OCIE0A);
//	    UARTBB_T_INTCTL_REG &= ~UARTBB_CMPINT_EN_MASK;
		/* stop timer */
        TCCR0B = 0x00;
        break;
	default:
		break;
	}
}

void uartbb_putchar(char val)
{
	uint8_t tidx;


	tidx = (uartbb_tx_widx + 1) & ((1<<UARTBB_TX_PWR)-1);
	if (tidx == uartbb_tx_ridx)
		return;

	uartbb_tx[uartbb_tx_widx] = val;
	uartbb_tx_widx = tidx;

    /* get the ball rolling if we are in idle state */
	if (uartbb_state == UARTBB_STATE_IDLE) {
		uartbb_state = UARTBB_STATE_START;
		/* clear timer */
        TCNT0 = 0;
		/* start timer */
        TCCR0B = 0x01;
	}
}


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
