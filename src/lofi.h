#ifndef __LOFI_H__
#define __LOFI_H__

#if 0
// Sensor ID/Types
// 01   10-bit counter
// 02   NO switch on TR (PIN CHANGE)
// 03   NC switch on TR (PIN CHANGE)    // don't know if there needs to be a distinction between NC and NO
// 04   NO switch on TR (POLLED)
// 05   NC switch on TR (POLLED)
// 06   Light Sensor on AD
// 07   Temperature Sensor (Internal)
// 

#endif

#define SENS_ID_CTR             0x01
#define SENS_ID_SW_NO_PC        0x02
#define SENS_ID_VCC             0x03


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
    uint8_t     ctr             :1;
    uint8_t     sw01            :1;     /* switch NO on TR PIN CHANGE */
    uint8_t     vcc             :1;
} sensors_t;

typedef struct {
    uint8_t     vcc_hi          :3;
    uint8_t     sensorId        :5;
    uint8_t     vcc_lo;
} sensor_vcc_t;


#endif  /* __LOFI_H__ */

