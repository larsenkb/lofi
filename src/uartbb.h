#ifndef __UARTBB_H__
#define __UARTBB_H__


#define UARTBB_BAUD_RATE		9600

#if 0
#define UARTBB_TXPORT  PORTB
#define UARTBB_TXDDR   DDRB
#define UARTBB_TXBIT   PB2
#define UARTBB_T_COMP_REG        OCR0A
#define UARTBB_T_CONTR_REGA      TCCR0A
#define UARTBB_T_CONTR_REGB      TCCR0B
#define UARTBB_T_CNT_REG         TCNT0
#define UARTBB_T_INTCTL_REG      TIMSK0

#define UARTBB_CMPINT_EN_MASK    (1 << OCIE0A)

#define UARTBB_CTC_MASKA         (1 << WGM01)
#define UARTBB_CTC_MASKB         (0)

    /* "A timer interrupt must be set to interrupt at three times 
       the required baud rate." */
#define UARTBB_PRESCALE (1)

#if (UARTBB_PRESCALE == 8)
    #define UARTBB_PRESC_MASKA         (0)
    #define UARTBB_PRESC_MASKB         (1 << CS01)
#elif (UARTBB_PRESCALE==1)
    #define UARTBB_PRESC_MASKA         (0)
    #define UARTBB_PRESC_MASKB         (1 << CS00)
#else 
    #error "prescale unsupported"
#endif
#endif
#define UARTBB_TXPORT  PORTB
#define UARTBB_TXDDR   DDRB
#define UARTBB_TXBIT   PB2
#define UARTBB_PRESCALE (1)
#define UARTBB_TIMERTOP ( F_CPU/UARTBB_PRESCALE/UARTBB_BAUD_RATE - 1)

#define CLR_TX()  (UARTBB_TXPORT &= ~(1<<UARTBB_TXBIT))
#define SET_TX()  (UARTBB_TXPORT |= (1<<UARTBB_TXBIT))

#define UARTBB_TX_PWR		5

enum { UARTBB_STATE_IDLE = 0,
	   UARTBB_STATE_START,
	   UARTBB_STATE_CHAR,
	   UARTBB_STATE_STOP,
	   UARTBB_STATE_POST
} uartbb_state_t;

extern uint8_t      uartbb_state;

void uartbb_init(void);
void uartbb_puthex( uint8_t data );
void uartbb_puts( const char *s );
void uartbb_putchar(char val);

#endif  /* __UARTBB_H__ */
