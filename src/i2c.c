#include "i2c.h"


#define I2C_CLK 2	// PA2
#define I2C_DAT 3	// PA3

#define I2C_DDR		DDRA
#define I2C_PORT	PORTA
#define I2C_PIN		PINA

#define SDA_HI (I2C_DDR &= ~(1 << I2C_DAT))
#define SDA_LO (I2C_DDR |= (1 << I2C_DAT))

#define SCL_HI (I2C_DDR &= ~(1 << I2C_CLK))
#define SCL_LO (I2C_DDR |= (1 << I2C_CLK))

#define SDA_READ (I2C_PIN & (1 << I2C_DAT))
#define SCL_READ (I2C_PIN & (1 << I2C_CLK))


static void idly(void) { _NOP(); }

void I2C_Init(void)
{
    I2C_PORT &= ~((1 << I2C_DAT) | (1 << I2C_CLK));
    SCL_HI;
    SDA_HI;
    idly();
}

static void I2C_Start(void)
{
    // set both to high at the same time
//    I2C_DDR &= ~ ((1 << I2C_DAT) | (1 << I2C_CLK));
//    _delay_loop_1(200);

    SDA_LO;
    SCL_LO;
}

static void I2C_Stop(void)
{
    SDA_LO;
    SCL_HI;
    SDA_HI;
}

#if 1
static bool I2C_Write(uint8_t dat)
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
	bool ack = !SDA_READ;
	SCL_LO;
	return ack;
}
#else
static bool I2C_Write(uint8_t dat)
{
    for (register uint8_t i = 8; i; i--) {
		(dat & 0x80) ? SDA_HI : SDA_LO;
		SCL_HI;
		dat <<= 1;
		SCL_LO;
    }
	SDA_HI;
	SCL_HI;
	bool ack = !SDA_READ;
	SCL_LO;
	return ack;
}
#endif

static uint8_t I2C_Read(bool ack)
{
	register uint8_t dat = 0;

	SDA_HI;
	for (register uint8_t i = 8; i; i--) {
		dat <<= 1;
		SCL_HI;
//		while (SCL_READ == 0);
		if (SDA_READ)
			dat |= 1;
		SCL_LO;
	}
	ack ? SDA_LO : SDA_HI;
	SCL_HI;
	SCL_LO;
	SDA_HI;
	return dat;
}

static uint8_t raw[6];
static uint32_t rawTemp;
static uint32_t rawHumd;

static void readAHT10(void)
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
	
#if 0
	// go into low power mode
	_delay_loop_2(5);
	I2C_Start();
	I2C_Write( 0x70 );
	I2C_Write( 0xe1 );
	I2C_Write( 0x08 );
	I2C_Write( 0x00 );
	I2C_Stop();
#endif

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

void initAHT10(void)
{
	I2C_Start();
	I2C_Write( 0x70 );
	I2C_Write( 0xe1 );
	I2C_Write( 0x08 );
	I2C_Write( 0x00 );
	I2C_Stop();
}
