#include "i2c.h"

#if 1

#define I2C_CLK 2	// PA2
#define I2C_DAT 3	// PA3

#define I2C_DDR		DDRA
#define I2C_PORT	PORTA
#define I2C_PIN		PINA

#define SDA_HI (I2C_DDR &= ~(1 << I2C_DAT))
#define SDA_LO (I2C_DDR |= (1 << I2C_DAT))

#define SCL_HI (I2C_DDR &= ~(1 << I2C_CLK))
#define SCL_LO (I2C_DDR |= (1 << I2C_CLK))

//#define SDA_READ ((I2C_PIN >> I2C_DAT) & 0x01)
//#define SCL_READ ((I2C_PIN >> I2C_CLK) & 0x01)
#define SDA_READ (I2C_PIN & (1 << I2C_DAT))
#define SCL_READ (I2C_PIN & (1 << I2C_CLK))

//inline void dly() { _NOP(); _NOP(); _NOP(); }

void idly(void) { _NOP(); }

void I2C_Init(void)
{
    I2C_PORT &= ~ ((1 << I2C_DAT) | (1 << I2C_CLK));
    SCL_HI;
    SDA_HI;
    idly();
}

void I2C_Start(void)
{
    // set both to high at the same time
//    I2C_DDR &= ~ ((1 << I2C_DAT) | (1 << I2C_CLK));
//    _delay_loop_1(200);

    SDA_LO;
//    idly();
    SCL_LO;
//    idly();
}

void I2C_Stop(void)
{
    SDA_LO;
//    idly();
    SCL_HI;
//    idly();
    SDA_HI;
//    idly();
}

#if 1
bool I2C_Write(uint8_t dat)
{
	(dat & 0x80) ? SDA_HI : SDA_LO;
	SCL_HI;
	dat <<= 1;
	SCL_LO;
	(dat & 0x80) ? SDA_HI : SDA_LO;
	SCL_HI;
	dat <<= 1;
	SCL_LO;
	(dat & 0x80) ? SDA_HI : SDA_LO;
	SCL_HI;
	dat <<= 1;
	SCL_LO;
	(dat & 0x80) ? SDA_HI : SDA_LO;
	SCL_HI;
	dat <<= 1;
	SCL_LO;
	(dat & 0x80) ? SDA_HI : SDA_LO;
	SCL_HI;
	dat <<= 1;
	SCL_LO;
	(dat & 0x80) ? SDA_HI : SDA_LO;
	SCL_HI;
	dat <<= 1;
	SCL_LO;
	(dat & 0x80) ? SDA_HI : SDA_LO;
	SCL_HI;
	dat <<= 1;
	SCL_LO;
	(dat & 0x80) ? SDA_HI : SDA_LO;
	SCL_HI;
	dat <<= 1;
	SCL_LO;
	SDA_HI;
	SCL_HI;
//	idly();
	bool ack = !SDA_READ;
	SCL_LO;
	return ack;
}
#else
bool I2C_Write(uint8_t dat)
{
    for (register uint8_t i = 8; i; i--) {
		(dat & 0x80) ? SDA_HI : SDA_LO;
//		idly();
		SCL_HI;
		dat <<= 1;
//		idly();
		SCL_LO;
//		idly();
    }
	SDA_HI;
	SCL_HI;
//	idly();
	bool ack = !SDA_READ;
	SCL_LO;
	return ack;
}
#endif

uint8_t I2C_Read(bool ack)
{
	register uint8_t dat = 0;

	SDA_HI;
	for (register uint8_t i = 8; i; i--) {
		dat <<= 1;
		SCL_HI;
//		while (SCL_READ == 0);
//		idly();
#if 0
		dat |= SDA_READ;
#else
		if (SDA_READ)
			dat |= 1;
#endif
//		idly();
		SCL_LO;
	}
	ack ? SDA_LO : SDA_HI;
//	idly();
	SCL_HI;
//	idly();
	SCL_LO;
//	idly();
	SDA_HI;
	return dat;
}

uint8_t raw[6];
uint32_t rawTemp;
uint32_t rawHumd;

void readAHT10(void)
{
	// initiate a sensor reading
	I2C_Start();
	I2C_Write( 0x70 );
	I2C_Write( 0xac );
	I2C_Write( 0x33 );
	I2C_Write( 0x00 );
	I2C_Stop();

	// loop/wait until reading done
	uint8_t stat;
	do {
		_delay_loop_2(1000);
		I2C_Start();
		I2C_Write( 0x71 );
		stat = I2C_Read( 0 );
		I2C_Stop();
	} while ((stat & 0x80) == 0x80);

	// read Temperature and Humidity
	I2C_Start();
	I2C_Write( 0x71 );
	raw[0] = I2C_Read( 1 );
	raw[1] = I2C_Read( 1 );
	raw[2] = I2C_Read( 1 );
	raw[3] = I2C_Read( 1 );
	raw[4] = I2C_Read( 1 );
	raw[5] = I2C_Read( 0 );
	I2C_Stop();
	
	// go into low power mode
	_delay_loop_2(5);
	I2C_Start();
	I2C_Write( 0x70 );
	I2C_Write( 0xe1 );
	I2C_Write( 0x08 );
	I2C_Write( 0x00 );
	I2C_Stop();

	rawHumd = raw[1];
	rawHumd = (rawHumd<<8) + raw[2];
	rawHumd = (rawHumd<<4) + ((raw[3]>>4) & 0xF);

	rawTemp = raw[3] & 0xF;
	rawTemp = (rawTemp<<8) + raw[4];
	rawTemp = (rawTemp<<8) + raw[5];
}

uint32_t readAHT10Temp(void)
{
	readAHT10();
	return rawTemp;
}

uint32_t readAHT10Humd(bool humdDone)
{
	if (!humdDone) {
		readAHT10();
	}
	return rawHumd;
}

#else

// Port for the I2C
#define I2C_DDR DDRA
#define I2C_PIN PINA
#define I2C_PORT PORTA

// Pins to be used in the bit banging
#define I2C_CLK 2	// PA2
#define I2C_DAT 3	// PA3

#define I2C_DATA_HI()  \
	I2C_DDR &= ~(1 << I2C_DAT);
//	I2C_PORT |= (1 << I2C_DAT);

#define I2C_DATA_LO()  \
	I2C_DDR |= (1 << I2C_DAT);  \
	I2C_PORT &= ~(1 << I2C_DAT);

#define I2C_CLOCK_HI()  \
	I2C_DDR &= ~(1 << I2C_CLK);
//	I2C_PORT |= (1 << I2C_CLK);

#define I2C_CLOCK_LO()  \
	I2C_DDR |= (1 << I2C_CLK);  \
	I2C_PORT &= ~(1 << I2C_CLK);

// Inits bitbanging port, must be called before using the functions below
//
void I2C_Init(void)
{
    //I2C_PORT &= ~ ((1 << I2C_DAT) | (1 << I2C_CLK));

    I2C_CLOCK_HI();
    I2C_DATA_HI();

    _delay_loop_1(200);
}

void I2C_WriteBit(unsigned char c)
{
    if (c) {
        I2C_DATA_HI();
    } else {
        I2C_DATA_LO();
    }
    _delay_loop_1(20);

    I2C_CLOCK_HI();
    _delay_loop_1(60);

    I2C_CLOCK_LO();
    _delay_loop_1(20);

	if (1) {
	    if (c) {
	        I2C_DATA_LO();
	    }
	} else {
		I2C_DATA_HI();
	}

    _delay_loop_1(100);
}

unsigned char I2C_ReadBit(void)
{
    I2C_DATA_HI();
    _delay_loop_1(20);

    I2C_CLOCK_HI();

    _delay_loop_1(30);
    unsigned char c = I2C_PIN;
    _delay_loop_1(30);

    I2C_CLOCK_LO();
    _delay_loop_1(20);

    _delay_loop_1(100);

    return (c >> I2C_DAT) & 0x01;
}

// Send a START Condition
//
void I2C_Start(void)
{
    // set both to high at the same time
//    I2C_DDR &= ~ ((1 << I2C_DAT) | (1 << I2C_CLK));
//    _delay_loop_1(200);

    I2C_DATA_LO();
    _delay_loop_1(50);

    I2C_CLOCK_LO();
    _delay_loop_1(50);
}

// Send a STOP Condition
//
void I2C_Stop(void)
{
    I2C_CLOCK_HI();
    _delay_loop_1(50);

    I2C_DATA_HI();
    _delay_loop_1(200);
}

// write a byte to the I2C slave device
//
unsigned char I2C_Write(unsigned char c)
{
    for (char i = 0; i < 8; i++) {
        I2C_WriteBit(c & 0x80);

        c <<= 1;
    }

	// read the ACKT
    I2C_ReadBit();

    return 0;
}


// read a byte from the I2C slave device
//
unsigned char I2C_Read(unsigned char ack)
{
    unsigned char res = 0;

    for (char i = 0; i < 8; i++) {
        res <<= 1;
        res |= I2C_ReadBit();
    }

    if (ack) {
        I2C_WriteBit(0);
    } else {
        I2C_WriteBit(1);
    }

//    _delay_loop_1(200);

    return res;
}
#endif
