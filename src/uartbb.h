#ifndef __UARTBB_H__
#define __UARTBB_H__

#include <avr/io.h>

#define UARTBB_BAUD_RATE		9600

#define UARTBB_TXPORT  PORTA
#define UARTBB_TXDDR   DDRA
#define UARTBB_TXBIT   7
#define UARTBB_PRESCALE (1)
#define UARTBB_TIMERTOP ( F_CPU/UARTBB_PRESCALE/UARTBB_BAUD_RATE - 1)

#define CLR_TX()  (UARTBB_TXPORT &= ~(1<<UARTBB_TXBIT))
#define SET_TX()  (UARTBB_TXPORT |= (1<<UARTBB_TXBIT))

/* the tx/rx buffer sizes are this power of two */
/* always need to be a power of two */
#define UARTBB_TX_PWR		4

enum { UARTBB_STATE_IDLE = 0,
	   UARTBB_STATE_START,
	   UARTBB_STATE_CHAR,
	   UARTBB_STATE_STOP,
	   UARTBB_STATE_POST
} uartbb_state_t;

extern volatile uint8_t      uartbb_next_state;

void uartbb_init(void);
//void uartbb_puthex( uint8_t data );
//void uartbb_puts( const char *s );
void uartbb_putchar(char val);

#endif  /* __UARTBB_H__ */
