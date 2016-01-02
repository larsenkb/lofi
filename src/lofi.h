#ifndef __LOFI_H__
#define __LOFI_H__


typedef enum {
	SENID_NONE = 0,
	SENID_CTR,
	SENID_SW1_NC_PC,
	SENID_VCC,
	SENID_TEMP,
	SENID_SW1_NC_POLL,
	SENID_SW1_NO_PC,
	SENID_SW1_NO_POLL,
	SENID_SW2_NC_PC,
	SENID_SW2_NC_POLL,
	SENID_SW2_NO_PC,
	SENID_SW2_NO_POLL
} senId_t;

//#define SENS_ID_CTR             0x01
//#define SENS_ID_SW_NO_PC        0x02
//#define SENS_ID_VCC             0x03


typedef struct {
    uint8_t     swtich_changed  :1;
    uint8_t     switch_state    :1;
    uint8_t     rsvd            :1;
    uint8_t     sensorId        :5;
} sensor_switch_t;

typedef struct {
    uint8_t     ctr_hi          :3;
    uint8_t     sensorId        :5;
    uint8_t     ctr_lo;
} sensor_ctr_t;

typedef struct {
    uint8_t     ctr				:1;
    uint8_t     sw1				:1;
    uint8_t     vcc				:1;
	uint8_t		temp			:1;
	uint8_t		sw2				:1;
} sensors_t;

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


#endif  /* __LOFI_H__ */

