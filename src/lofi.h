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

#define EEPROM_NODEID_ADR		((uint8_t *)0)
#define NRF24_PAYLOAD_LEN		(sizeof(sensor_t) + 1)
#define TXBUF_SIZE				8	// must be a power of 2!!!


// define macros to slow clock even more than fuse setting
#define CLK_DIV			3
#define CORE_FAST		CLK_DIV
#define CORE_SLOW		(CLK_DIV + 2)	

#define FAST_CLOCK() do {		\
		uint8_t jj = SREG;		\
		cli();					\
		CLKPR = (1<<CLKPCE);	\
		CLKPR = CORE_FAST;		\
		clk_div = CORE_FAST;	\
		PRR |= (1<<PRUSI);		\
		SREG = jj;				\
	} while (0);

#define SLOW_CLOCK() do {		\
		uint8_t jj = SREG;		\
		cli();					\
		CLKPR = (1<<CLKPCE);	\
		CLKPR = CORE_SLOW;		\
		clk_div = CORE_SLOW;	\
		PRR &= ~(1<<PRUSI);		\
		SREG = jj;				\
	} while (0);

#define PWB_REV       GPIOR1

// use one of the unused internal I/O registers as a 1-clock access register
#define FLAGS        GPIOR0
#define WD_FLAG      (1<<0)
#define SW_FLAG      (1<<1)
#define CTR_FLAG     (1<<2)
#define VCC_FLAG     (1<<3)
#define TEMP_FLAG    (1<<4)
#define ATEMP_FLAG   (1<<5)
#define AHUMD_FLAG   (1<<6)

#define VCC_MUX		0b00100001
#define TEMP_MUX	0b10100010

// define message IDs
typedef enum {
	SENID_NONE = 0,
	SENID_SW1,
	SENID_SW2,
	SENID_VCC,
	SENID_TEMP,
	SENID_CTR,
	SENID_REV,
	SENID_ATEMP,
	SENID_AHUMD
} senId_t;

// define message format; used by all sensor types
typedef struct {
    uint8_t     trig	        :1;
    uint8_t     closed          :1;
    uint8_t     seq             :2;
    uint8_t     sensorId        :4;
	uint8_t		hi				:4;
	uint8_t		rsvd			:4;
    uint8_t     mid;
    uint8_t     low;
} sensor_t;



// define eeprom configuration format
typedef struct {	// fills up bit fields LSB to MSB
	// byte 0
	uint8_t		nodeId;				// associated pipe nbr is (nodeId mod 6), i.e. 0-5

	// byte 1						// default: 0x86
	uint8_t		sw1_rev			:1;	// LSB
	uint8_t		sw1_pc			:1;
	uint8_t		en_sw1			:1;
	uint8_t		rsvd_1			:3;	// was wd_timeout - now hardcoded to max 8 sec.
	uint8_t		spd_1M			:1;
	uint8_t		spd_250K		:1;	// MSB

	// byte 2						// default: 0xe2
    uint8_t     en_ctr			:1;
    uint8_t     en_vcc			:1;
	uint8_t		en_temp			:1;
	uint8_t		en_txDbg		:1;
	uint8_t		en_led_ack		:1;
	uint8_t		en_led_nack		:1;
	uint8_t		en_atemp		:1;
	uint8_t		en_ahumd		:1;

	// byte 3						// default: 0x54
	uint8_t		rf_chan			:7;		// use only even chan #s at 2Mbps
	uint8_t		rsvd_3			:1;

	// byte 4						// default: 0x23
	uint8_t		rf_gain			:2;
	uint8_t		rsvd_4			:2;
	uint8_t		en_dyn_ack		:1;		// 0: tell receiver to NOT send an ACK
	uint8_t		en_aa			:1;
	uint8_t		en_aht10_cal	:1;		// enable AHT10 calibration
	uint8_t		rsvd_5			:1;

	// byte 5						// default: 0x33
	uint8_t		setup_retr;				// nrf SETUP_RETR register contents

	// byte 6
	uint8_t		pwbRev;					// 0=original PWB; 1=0.1 PWB; 2=0.2 PWB; 3 = 0.3/0.4 PWB
										// 5 = 0.5 PWB; 6 = I2C
	// byte 7
	uint8_t		rsvd_6;

	// using tpl5111 set at ~5 minute trigger; there are ~288 triggers per day
	// All these 2-byte values are little endian

	// bytes 8 & 9 	 LE 0x01c2(WD) for ~1 hr; TPL5111: set to 0x0001 for every ~5 minutes
	uint16_t	swCntsMax;
	
	// bytes 10 & 11  
	uint16_t	ctrCntsMax;

	// bytes 12 & 13  0x2a30(WD) for ~1 day; TPL5111 set to 0x0120 for once per day
	uint16_t	vccCntsMax;

	// bytes 14 & 15  0x00e1(WD) for ~0.5 hr; TPL5111 set to 0x0006 for ~0.5 hr
	uint16_t	tempCntsMax;

	// bytes 16 & 17
	int16_t		vccFudge;

	// bytes 18 & 19
	int16_t		tempFudge;

	// bytes 20 & 21			// default: 0x0001
	uint16_t	atempCntsMax;	// LE xmit atemp every n watchdog interrupt

	// bytes 22 & 23			// default: 0x0001
	uint16_t	ahumdCntsMax;	// LE xmit ahumd every m watchdog interrupt

	// bytes 24 & 25			// default: 0x0240
	int16_t		aht10CalibCntsMax;	// calibrate AHT10 every 'n' msg

	// bytes 26 & 27
	int16_t		atempFudge;

	// bytes 28 & 29
	int16_t		ahumdFudge;

	// bytes 30 & 31
	int16_t		rsvd_7;

	// bytes 32 to 36		// default: 0xe7 0xe7 0xe7 0xe7 0xe7
	uint8_t		p0Addr[5];

	// bytes 37 to 41		// default: 0xc2 0xc2 0xc2 0xc2 0xc2
	uint8_t		p15Addr[5];

	// bytes 42 to 45		// default: 0xc3 0xc4 0xc5 0xc6
	uint8_t		pxAddr[4];

} config_t;

// FORWARD DECLARATIONS ----------------------------------------
uint16_t readVccTemp(uint8_t mux_select);
void printConfig(void);
uint8_t getSw1(uint8_t pc_triggered);
uint8_t read_switch(void);
void safe_switch(void);
//uint8_t getSw2(void);
void msg_init(sensor_t *s, int id);
void ctr_msg_init(void);
void rev_msg_init(void);
void temp_msg_init(void);
void vcc_msg_init(void);
void sw1_msg_init(void);
void sw2_msg_init(void);
void atemp_msg_init(void);
void ahumd_msg_init(void);
void flags_update(void);
void msgs_build(int pc_triggered);
void dlyMS(uint16_t ms);
void tpl_done_init(void);
void tpl_drv_init(void);
void tpl_done_pulse(void);
void led_init(void);
void ledr_assert(void);
void ledr_deassert(void);
void ledg_assert(void);
void ledg_deassert(void);
void init_switch_PC(void);
void init_switch(void);
void init_unused_pins(void);
void blinkLed(uint8_t status);
void sw1_msg_build(void);

#endif  /* __LOFI_H__ */
