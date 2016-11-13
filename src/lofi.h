#ifndef __LOFI_H__
#define __LOFI_H__

#define EN_ENH_SWAVE	1

typedef enum {
	SENID_NONE = 0,
	SENID_CTR,
	SENID_SW1,
//	SENID_SW1_NC_PC,
	SENID_VCC,
	SENID_TEMP,
	SENID_SW2
//	SENID_SW1_NC_POLL,
//	SENID_SW1_NO_PC,
//	SENID_SW1_NO_POLL,
//	SENID_SW2_NC_PC,
//	SENID_SW2_NC_POLL,
//	SENID_SW2_NO_PC,
//	SENID_SW2_NO_POLL,
} senId_t;



typedef struct {
    uint8_t     changed         :1;
    uint8_t     closed          :1;
    uint8_t     lastState       :1;
    uint8_t     sensorId        :5;
} sensor_switch_t;

typedef struct {
    uint8_t     ctr				:1;
    uint8_t     sw1				:1;
    uint8_t     vcc				:1;
	uint8_t		temp			:1;
	uint8_t		sw2				:1;
} sensors_t;

typedef enum {
	SWITCH_NONE = 0,
	SWITCH_PC = 1,
	SWITCH_POLL = 2,
	SWITCH_RSVD = 3
} switch_t;

#define SWITCH_NC_IS_CLOSED		1
#define SWITCH_PIN_CHANGE		1
#define SWITCH_INSTALLED		1

typedef struct {
	// byte 0
	uint8_t		nodeId;

	// byte 1
	uint8_t		sw1_rev			:1;
	uint8_t		sw1_pc			:1;
	uint8_t		sw1_enb			:1;
	uint8_t		sw2_rev			:1;
	uint8_t		sw2_pc			:1;
	uint8_t		sw2_enb			:1;
	uint8_t		rsvd_1			:2;

	// byte 2
    uint8_t     ctr				:1;
    uint8_t     vcc				:1;
	uint8_t		temp			:1;
	uint8_t		enLed			:1;
	uint8_t		nrfVccCtrl		:1;
	uint8_t		txDbg			:1;
	uint8_t		spd_1M			:1;
	uint8_t		spd_250K		:1;

	// byte 3
	uint8_t		rf_chan			:7;		// use only even chan #s at 2Mbps
	uint8_t		rsvd_3			:1;

	// byte 4
	uint8_t		rf_gain			:2;
	uint8_t		wd_timeout		:3;		// 0-0.5,1-1,2-2,3-4,4-8,567-off
	uint8_t		rsvd_4			:3;

	// byte 5
	uint8_t		wdCnts;					// nbr of wd events before xmitting

	// byte 5
	uint8_t		rsvd_5;

} config_t;

typedef struct {
    uint8_t     vcc_hi          :3;
    uint8_t     sensorId        :5;
    uint8_t     vcc_lo;
} sensor_vcc_t;

typedef struct {
    uint8_t     temp_hi			:3;
    uint8_t     sensorId		:5;
    uint8_t     temp_lo;
} sensor_temp_t;

typedef struct {
    uint8_t     ctr_hi          :3;
    uint8_t     sensorId        :5;
    uint8_t     ctr_lo;
} sensor_ctr_t;

typedef enum {
	speed_1M = 0,
	speed_2M = 1,
	speed_250K = 2
} speed_t;

#endif  /* __LOFI_H__ */

