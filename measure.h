#ifndef EMONITOR_H
#define EMONITOR_H

#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#define ADC_BITS 16

#define ADC_COUNTS  (1 << ADC_BITS)

#define POWER_MEASURMENT_TEST 0
#define VOLTAGE_CALIBRATION 149.4 // Calibrated at home
#define VOLTAGE_CALIBRATION_SCHOOL_P1 145.4 // Calibrated at school
#define VOLTAGE_CALIBRATION_SCHOOL_P2 150.1 // Calibrated at school
#define CURRENT_CALIBRATION 1
#define ADC_VREF 4.07
#define BURDEN_RESISTOR_OHMS 39.0
#define CT_TURNS 3000.0

// ADS8688/ADS8684 Command Registers
#define ADS8688_CMD_REG(x)				(x << 8)
#define ADS8688_CMD_REG_NOOP			0x00
#define ADS8688_CMD_REG_RST				0x85
#define ADS8688_CMD_REG_MAN_CH(chan)	(0xC0 | (4 * chan))
#define ADS8688_AUTO_RST_CMD			0xA0
#define ADS8688_PWR_DWN_CMD				0x83
#define ADS8688_STDBY_CMD				0x82
#define ADS8688_CMD_DONT_CARE_BITS		16

// ADS8688/ADS8684 Program Register helper macros
#define ADS8688_PROG_REG(x)				(x << 9)
#define ADS8688_PROG_REG_RANGE_CH(chan)	(0x05 + chan)
#define ADS8688_PROG_WR_BIT				1 << 8
#define ADS8688_PROG_DONT_CARE_BITS		8

// ADS8688/ADS8684 Range Select Values
#define ADS8688_REG_PLUSMINUS25VREF		0 // +-2.5*Vref
#define ADS8688_REG_PLUSMINUS125VREF	1 // +-1.25*Vref
#define ADS8688_REG_PLUSMINUS0625VREF	2 // +-0.625*Vref
#define ADS8688_REG_PLUS25VREF			5 // +2.5*Vref
#define ADS8688_REG_PLUS125VREF			6 // +1.25*Vref

#define ADS8688_VREF_MV					4096
#define ADS8688_REALBITS				16

// power screen shot
typedef struct powersc {
	double realPower;
	double apparentPower;
	double powerFactor;
	double Vrms;
	double Irms;
	double Vraw;
	double Iraw;
	struct timespec *tv;
} powersc_t;

union {
	uint32_t d32;
	uint8_t d8[4];
} iobuf;

// ADS8688/ADS8684 Program Registers
enum uint8_t {
	AUTO_SEQ_EN_ADDR 		= 0x01,
	CHANNEL_PWR_DWN_ADDR	= 0x02,
	FEATURE_SELECT_ADDR		= 0x03,
	C0RANGE					= 0x05,
	C1RANGE					= 0x06,
	C2RANGE					= 0x07,
	C3RANGE					= 0x08,
	CMD_READ_BACK_ADDR		= 0x3F
};

//Calibration coefficients
//These need to be set in order to obtain accurate results
double VCAL;
double ICAL;
double PHASECAL;


void calcVI(unsigned int crossings, powersc_t *powerl_p, powersc_t *powerr_p, uint8_t inPinV, uint8_t inPinIl, uint8_t inPinIr);
double calcIrms(unsigned int number_of_samples, uint8_t inPinI);
#endif
