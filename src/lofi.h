#ifndef __LOFI_H__
#define __LOFI_H__

// use one of the unused internal I/O registers as a 1-clock access register
#define FLAGS       GPIOR0
#define wdFlag      (1<<GPIOR00)
#define sw1Flag     (1<<GPIOR01)
#define sw2Flag     (1<<GPIOR02)
#define swFlag      (1<<GPIOR03)
#define ctrFlag     (1<<GPIOR04)
#define vccFlag     (1<<GPIOR05)
#define tempFlag    (1<<GPIOR06)

#define VCC_MUX		0b00100001
#define TEMP_MUX	0b10100010

typedef enum {
	SENID_NONE = 0,
	SENID_SW1,
	SENID_SW2,
	SENID_VCC,
	SENID_TEMP,
	SENID_CTR
} senId_t;



typedef struct {
    uint8_t     lastState       :1;
    uint8_t     closed          :1;
    uint8_t     seq             :2;
    uint8_t     sensorId        :4;
} sensor_switch_t;


typedef struct {
	// byte 0
	uint8_t		nodeId;

	// byte 1
	uint8_t		sw1_rev			:1;
	uint8_t		sw1_pc			:1;
	uint8_t		en_sw1			:1;
	uint8_t		sw2_rev			:1;
	uint8_t		sw2_pc			:1;
	uint8_t		en_sw2			:1;
	uint8_t		en_wd			:1;
	uint8_t		rsvd_1			:1;

	// byte 2
    uint8_t     en_ctr			:1;
    uint8_t     en_vcc			:1;
	uint8_t		en_temp			:1;
	uint8_t		en_led			:1;
	uint8_t		en_nrfVcc		:1;
	uint8_t		en_txDbg		:1;
	uint8_t		spd_1M			:1;
	uint8_t		spd_250K		:1;

	// byte 3
	uint8_t		rf_chan			:7;		// use only even chan #s at 2Mbps
	uint8_t		rsvd_2			:1;

	// byte 4
	uint8_t		rf_gain			:2;
	uint8_t		wd_timeout		:3;		// 0-0.5,1-1,2-2,3-4,4-8,567-off
	uint8_t		en_dyn_ack		:1;		// 0: tell receiver to NOT send an ACK
	uint8_t		en_aa			:1;
	uint8_t		rsvd_3			:1;

	// byte 5
	uint8_t		setup_retr;				// nrf SETUP_RETR register contents

	// bytes 6 & 7
	uint16_t	rsvd_4;

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

typedef struct {
    uint8_t     vcc_hi          :2;
    uint8_t     seq             :2;
    uint8_t     sensorId        :4;
    uint8_t     vcc_lo;
} sensor_vcc_t;

typedef struct {
    uint8_t     temp_hi			:2;
    uint8_t     seq             :2;
    uint8_t     sensorId		:4;
    uint8_t     temp_lo;
} sensor_temp_t;

typedef struct {
    uint8_t     ctr_hi          :2;
    uint8_t     seq             :2;
    uint8_t     sensorId        :4;
    uint8_t     ctr_lo;
} sensor_ctr_t;

typedef enum {
	speed_1M = 0,
	speed_2M = 1,
	speed_250K = 2
} speed_t;

#endif  /* __LOFI_H__ */

