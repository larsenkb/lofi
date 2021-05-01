//-------------------------------------------------------------
// lofi.h
//-------------------------------------------------------------
//
// Configuration is stored in the eeprom starting at addr 0.
// See config_t structure for details.
//
//---------------------------------------------------------

#ifndef __LOFI_H__
#define __LOFI_H__

#define LOFI_VER		3

#define EN_TPL5111				1
#define EEPROM_NODEID_ADR		((uint8_t *)0)
#define NRF24_PAYLOAD_LEN		3
#define TXBUF_SIZE				8	// must be a power of 2!!!

#if LOFI_VER==2
	// define which flag is set for which PC ISR
	#define PCINT0_FLAG			WD_FLAG
	#define PCINT1_FLAG			SW_FLAG

	// define SWITCH pin
	#define SWITCH_PIN			2	// PB2
	#define SWITCH_PORT_DDR		DDRB
	#define SWITCH_PORT_OUT		PORTB
	#define SWITCH_PORT_IN		PINB
	#define SWITCH_PORT_MSK		PCMSK1
	#define SWITCH_GMSK			5

	// define TPL_DRV pin
	#define TPL_DRV_PIN				2	// PA2
	#define TPL_DRV_PORT_DDR		DDRA
	#define TPL_DRV_PORT_OUT		PORTA
	#define TPL_DRV_PORT_MSK		PCMSK0
	#define TPL_DRV_GMSK			4

	// define UNUSED pins
	#define NC1_PIN				0		// PB0
	#define NC1_PORT_DDR		DDRB
	#define NC1_PORT_OUT		PORTB
	#define NC2_PIN				7		// PA7
	#define NC2_PORT_DDR		DDRA
	#define NC2_PORT_OUT		PORTA

#elif LOFI_VER==3
	// define which flag is set for which PC ISR
	#define PCINT0_FLAG			SW_FLAG
	#define PCINT1_FLAG			WD_FLAG

	// define SWITCH pin
	#define SWITCH_PIN			7	// PA7
	#define SWITCH_PORT_DDR		DDRA
	#define SWITCH_PORT_OUT		PORTA
	#define SWITCH_PORT_IN		PINA
	#define SWITCH_PORT_MSK		PCMSK0
	#define SWITCH_GMSK			4

	// define TPL_DRV pin
	#define TPL_DRV_PIN				0	// PB0
	#define TPL_DRV_PORT_DDR		DDRB
	#define TPL_DRV_PORT_OUT		PORTB
	#define TPL_DRV_PORT_MSK		PCMSK1
	#define TPL_DRV_GMSK			5

	// define UNUSED pins
	#define NC1_PIN				2		// PB2
	#define NC1_PORT_DDR		DDRB
	#define NC1_PORT_OUT		PORTB
	#define NC2_PIN				2		// PA2
	#define NC2_PORT_DDR		DDRA
	#define NC2_PORT_OUT		PORTA

#else
	#error "LOFI_VER not defined"
#endif

// define SWITCH pin functions
#define SWITCH_MSK			SWITCH_PIN
#define INIT_SWITCH()		(SWITCH_PORT_DDR &= ~(1<<SWITCH_PIN))
#define READ_SWITCH()		((SWITCH_PORT_IN>>SWITCH_PIN) & 1)
#define SAFE_SWITCH()		do {SWITCH_PORT_DDR |= (1<<SWITCH_PIN); \
								SWITCH_PORT_OUT &= ~(1<<SWITCH_PIN); \
						    } while(0)
#define INIT_SWITCH_PC()	do {GIMSK |= (1<<SWITCH_GMSK); \
								SWITCH_PORT_MSK |= (1<<SWITCH_MSK); \
							} while(0)

// define TPL_DRV pin functions
#define TPL_DRV_MSK_PIN			TPL_DRV_PIN
#define TPL_DRV_INIT()			do {TPL_DRV_PORT_DDR &= ~(1<<TPL_DRV_PIN); \
									GIMSK |= (1<<TPL_DRV_GMSK); \
									TPL_DRV_PORT_MSK |= (1<<TPL_DRV_MSK_PIN); \
								} while(0)

// define TPL_DONE pin and macro
#define TPL_DONE_PIN			1
#define TPL_DONE_PORT_DDR		DDRB
#define TPL_DONE_PORT_OUT		PORTB
#define TPL_DONE_INIT()			do {TPL_DONE_PORT_DDR |= (1<<TPL_DONE_PIN); \
									TPL_DONE_PORT_OUT &= ~(1<<TPL_DONE_PIN); \
								} while(0)
#define TPL_DONE_PULSE()		do {TPL_DONE_PORT_OUT |= (1<<TPL_DONE_PIN); \
									TPL_DONE_PORT_OUT &= ~(1<<TPL_DONE_PIN); \
								} while(0)


// define UNUSED pins INIT function
#define INIT_UNUSED_PINS()	do { NC1_PORT_DDR |= (1<<NC1_PIN); \
								NC1_PORT_OUT &= ~(1<<NC1_PIN); \
								NC2_PORT_DDR |= (1<<NC2_PIN); \
								NC2_PORT_OUT &= ~(1<<NC2_PIN); \
							} while(0)


// ---------  LED MACROS  ----------
//was PORTB bit 1
#define LED						(1<<3)  // PORTA bit3

#define LED_INIT(x)				do {DDRA |= (x); PORTA |= (x);} while(0)
#define LED_ASSERT(x)			do {PORTA &= ~(x);} while(0)
#define LED_DEASSERT(x)			do {PORTA |= (x);} while(0)

// --------- BITDBG ---------
#define BITDBG					(1<<7)	// PORTA bit7
#define BITDBG_INIT()			do {DDRA |= (BITDBG); PORTA &= ~(BITDBG);} while(0)
#define BITDBG_ASSERT()			do {PORTA |= BITDBG;} while(0)
#define BITDBG_DEASSERT()		do {PORTA &= ~BITDBG;} while(0)

	
// define macros to slow clock even more than fuse setting
#define CLK_DIV			3
#define CORE_FAST		CLK_DIV
#define CORE_SLOW		(CLK_DIV+2)	
#define CORE_CLK_SET(x)  do {	\
		CLKPR = (1<<CLKPCE);	\
		CLKPR = (x);			\
		clk_div = (x);			\
	} while(0)
#define CORE_CLK_SETi(x)  do {	\
		cli();					\
		CORE_CLK_SET(x);		\
		sei();					\
	} while(0)


// use one of the unused internal I/O registers as a 1-clock access register
#define FLAGS       GPIOR0
#define WD_FLAG      (1<<0)
#define SW_FLAG      (1<<1)
#define CTR_FLAG     (1<<2)
#define VCC_FLAG     (1<<3)
#define TEMP_FLAG    (1<<4)

#define VCC_MUX		0b00100001
#define TEMP_MUX	0b10100010

// define message IDs
typedef enum {
	SENID_NONE = 0,
	SENID_SW1,
	SENID_SW2,
	SENID_VCC,
	SENID_TEMP,
	SENID_CTR
} senId_t;


// define switch message format
typedef struct {
    uint8_t     lastState       :1;
    uint8_t     closed          :1;
    uint8_t     seq             :2;
    uint8_t     sensorId        :4;
} sensor_switch_t;

// define Vcc message format
typedef struct {
    uint8_t     vcc_hi          :2;
    uint8_t     seq             :2;
    uint8_t     sensorId        :4;
    uint8_t     vcc_lo;
} sensor_vcc_t;

// define temperature message format
typedef struct {
    uint8_t     temp_hi			:2;
    uint8_t     seq             :2;
    uint8_t     sensorId		:4;
    uint8_t     temp_lo;
} sensor_temp_t;

// define counter message format
typedef struct {
    uint8_t     ctr_hi          :2;
    uint8_t     seq             :2;
    uint8_t     sensorId        :4;
    uint8_t     ctr_lo;
} sensor_ctr_t;


// define eeprom configuration format
typedef struct {
	// byte 0
	uint8_t		nodeId;

	// byte 1
	uint8_t		sw1_rev			:1;
	uint8_t		sw1_pc			:1;
	uint8_t		en_sw1			:1;
	uint8_t		rsvd_1			:3;
	uint8_t		spd_1M			:1;
	uint8_t		spd_250K		:1;

	// byte 2
    uint8_t     en_ctr			:1;
    uint8_t     en_vcc			:1;
	uint8_t		en_temp			:1;
	uint8_t		en_txDbg		:1;
	uint8_t		en_led			:1;
	uint8_t		en_led_nack		:1;
	uint8_t		rsvd_2			:2;

	// byte 3
	uint8_t		rf_chan			:7;		// use only even chan #s at 2Mbps
	uint8_t		rsvd_3			:1;

	// byte 4
	uint8_t		rf_gain			:2;
	uint8_t		rsvd_4			:2;
	uint8_t		en_dyn_ack		:1;		// 0: tell receiver to NOT send an ACK
	uint8_t		en_aa			:1;
	uint8_t		rsvd_5			:2;

	// byte 5
	uint8_t		setup_retr;				// nrf SETUP_RETR register contents

	// bytes 6 & 7
	uint16_t	rsvd_6;

	// bytes 8 & 9
	uint16_t	swCntsMax;				// LE 0x01c2 for ~1 hr 
	
	// bytes 10 & 11
	uint16_t	ctrCntsMax;				// little-endian

	// bytes 12 & 13
	uint16_t	vccCntsMax;				// LE 0x2a30 for ~1 day

	// bytes 14 & 15
	uint16_t	tempCntsMax;			// LE 0x00e1 for ~0.5 hr

	// bytes 16 & 17
	int16_t		vccFudge;				// signed little-endian

	// bytes 18 & 19
	int16_t		tempFudge;				// signed little-endian

} config_t;

// FORWARD DECLARATIONS ----------------------------------------
uint16_t readVccTemp(uint8_t mux_select);
void printConfig(void);
uint8_t getSw1(void);
uint8_t getSw2(void);
void ctr_msg_init(void);
void temp_msg_init(void);
void vcc_msg_init(void);
void sw1_msg_init(void);
void sw2_msg_init(void);
void flags_update(void);
void msgs_build(void);
void dlyMS(uint16_t ms);


#endif  /* __LOFI_H__ */

