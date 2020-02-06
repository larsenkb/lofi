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


#define EN_TPL5111				1
#define EEPROM_NODEID_ADR		((uint8_t *)0)
#define NRF24_PAYLOAD_LEN		3
#define TXBUF_SIZE				8	// must be a power of 2!!!


// define masks to enable switch Pin Change
#define SWITCH_1        2       /* PORTB bit2 */
#define SWITCH_1_MSK    2
#define SWITCH_1_GMSK   5


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

// define DONE pin and macro
// was PORTB bit 3
#define DONE_PIN	1
#define DONE_INIT()	  do {			\
		DDRB |= (1<<DONE_PIN);		\
		PORTB &= ~(1<<DONE_PIN);	\
	} while(0)
#define DONE_PULSE()  do {			\
		PORTB |= (1<<DONE_PIN);		\
		PORTB &= ~(1<<DONE_PIN);	\
	} while(0)

	
// define macros to slow clock even more that fuse setting
#define CLK_DIV			3
#define CORE_FAST		CLK_DIV
#define CORE_SLOW		(CLK_DIV)	
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
#define wdFlag      (1<<0)
#define swFlag      (1<<1)
#define ctrFlag     (1<<2)
#define vccFlag     (1<<3)
#define tempFlag    (1<<4)

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

