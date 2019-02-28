#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <stdint.h>
#include <endian.h>
#include <stdbool.h>
#include <math.h>

#include "spiComm.h"
#include "measure.h"

#define POWER_MEASURMENT_TEST 1
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

#define ADC_FREQ 500000
#define VIN_FREQ 60
//#define num_test_samples ((size_t)((1.0/VIN_FREQ)/(1.0/ADC_FREQ)) * 2)
#define num_test_samples 16667

static void parse_opts(int argc, char *argv[]);

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

#define PT_CHANNEL 0
#define CTL_CHANNEL 1
#define CTR_CHANNEL 2

// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static uint8_t channel = 0;
static uint8_t mode = 1;
static uint32_t speed = 500000;
static double PGA_GAIN_I = 0.625;
static double PGA_GAIN_V = 1.25;

// These buffer store a bunch of samples for each sensor.
// These were made to simplfy the testing process.
// We'll start with buffers that may store a minimum of 2 full wave cycles.
static int ct1_buf[num_test_samples];
static int ct2_buf[num_test_samples];
static int pt_buf[num_test_samples];
static int current_sample;
static int current_sample_pt;
static int current_sample_ct1;
static int current_sample_ct2;

static void print_buffer(uint8_t *buf, int length)
{
	//printf("Buffer: ");
	for (int ret = 0; ret < length; ret++) {
		//if (!(ret % 4))puts("");
		printf("%.2X ", buf[ret]);
	}
	puts("");
}

/*
 * Helper function used for testing sensor measurments. Returns value from
 * the buffer of the given channel as an int. This simplifies emonlib port.
 */
static inline int analogRead(uint8_t channel) {
	//int i = current_sample; current_sample++;
	int reading;
	switch(channel) {
		case CTR_CHANNEL:
			if (current_sample_ct1 >= num_test_samples) {
				printf("Trying to get too many sample\n");
				return -1;
			}
			reading = ct1_buf[current_sample_ct1];
			current_sample_ct1++;
			break;
		case CTL_CHANNEL:
			if (current_sample_ct2 >= num_test_samples) {
				printf("Trying to get too many sample\n");
				return -1;
			}
			reading = ct2_buf[current_sample_ct2];
			current_sample_ct2++;
			break;
		case PT_CHANNEL:
			if (current_sample_pt >= num_test_samples) {
				printf("Trying to get too many sample\n");
				return -1;
			}
			reading = pt_buf[current_sample_pt];
			current_sample_pt++;
			break;
		default:
			reading =  0;
			break;
	}
	return reading;
}

static inline double getVin(int adc_out, double pga_high, double pga_low) {
	return adc_out * ((pga_high - pga_low) / ADC_COUNTS) + pga_low;
}  

/*
 * Returns the theortiectial phase error due to ADC for a given multiplexed 
 * channel. (Refer to example application in ADS8688/ADS8684 datasheet
 */
static inline double getTheoreticalPhaseError(int channel) 
{
	return ((double)60 / 500000) * channel * 360;
}

static uint32_t ads8688_prog_write(int fd, unsigned int addr, unsigned int val)
{
	uint8_t rx[4] = {0};
	uint8_t tx[4] = {0};
	uint32_t tmp;

	tmp = ADS8688_PROG_REG(addr) | ADS8688_PROG_WR_BIT | val;
	tmp <<= ADS8688_PROG_DONT_CARE_BITS;
	tx[0] = (tmp >> 16)&0xff; tx[1] = (tmp >> 8)&0xff; tx[2] = tmp & 0xff;

//	printf("tx: "); print_buffer(tx, 4);
	spiDataRW(channel, tx, rx, 3);
//	printf("rx: "); print_buffer(rx, 4);
	/*tmp = ((uint32_t)rx[0] << 24) |
		((uint32_t)rx[1] << 16)	|
		((uint32_t)rx[2] << 8)	|
		((uint32_t)rx[3]);*/
	tmp = rx[2];

//	printf("rx word: %x\n", tmp);
	return tmp;
}
static inline void reset_program_regs()
{
	uint32_t cmd = ADS8688_CMD_REG(ADS8688_CMD_REG_RST);
	cmd <<= ADS8688_CMD_DONT_CARE_BITS;
	iobuf.d32 = htobe32(cmd);
	//printf("enter auto seq cmd: %x\n", cmd);
	//print_buffer(&iobuf.d8[0], 4);
	spiDataRW(channel, &iobuf.d8[0], &iobuf.d8[0], 4);
}
static inline void enter_auto_sequence_mode()
{
	uint32_t cmd = ADS8688_CMD_REG(ADS8688_AUTO_RST_CMD);
	cmd <<= ADS8688_CMD_DONT_CARE_BITS;
	iobuf.d32 = htobe32(cmd);
	//printf("enter auto seq cmd: %x\n", cmd);
	//print_buffer(&iobuf.d8[0], 4);
	spiDataRW(channel, &iobuf.d8[0], &iobuf.d8[0], 4);
}

// Assumes we are in AUTO_SEQ state so just sends NO_OP command
int32_t auto_channel_read()
{
	uint32_t tmp = ADS8688_CMD_REG(ADS8688_CMD_REG_NOOP);
	tmp <= ADS8688_CMD_DONT_CARE_BITS;
	iobuf.d32 = htobe32(tmp);
	spiDataRW(channel, &iobuf.d8[0], &iobuf.d8[0], 4);
//	print_buffer(&iobuf.d8[0], 4);
	return be32toh(iobuf.d32) & 0xffff;
}

int main(int argc, char *argv[])
{
	int fd;
	uint32_t prog_result;

	//int result;

	parse_opts(argc, argv);

	// Configure the interface.
	// channel indicates chip select,
	fd = spiSetup(channel, speed);
	if (fd < 0) return EXIT_FAILURE;

	/* Reset Program Regs */
	reset_program_regs();
	usleep(15000);

	/* Power down channel 4 */
	prog_result = ads8688_prog_write(fd, CHANNEL_PWR_DWN_ADDR, 0xf8);
	/* Enable auto scan for channels 0, 1 and 2 */
	prog_result = ads8688_prog_write(fd, AUTO_SEQ_EN_ADDR, 0x07);
	/* Set ranges for CT's on channel 1 and channel 2 */
	prog_result = ads8688_prog_write(fd, C1RANGE, ADS8688_REG_PLUSMINUS0625VREF);
	prog_result = ads8688_prog_write(fd, C2RANGE, ADS8688_REG_PLUSMINUS0625VREF);
	PGA_GAIN_I = 0.625;
	/* Set range for PT on channel 0 */
	prog_result = ads8688_prog_write(fd, C0RANGE, ADS8688_REG_PLUS125VREF);
	//printf("ADC program response: %x\n", prog_result);

	/* Enter auto sequence mode */
	enter_auto_sequence_mode();

	/* Automatically sequence through channels and read signal */
	/*int channel_signals[6];
	for (int i = 0; i < 6; i++) {
		channel_signals[i] = auto_channel_read();
	}

	for (int i = 0; i < 6; i += 3) {
		printf("pt: digital out = %d, analog in = %f\n", channel_signals[i], getVin(channel_signals[i], ADC_VREF*1.25, 0) );
		printf("ct left: digital = %d, analog in = %f\n", i%3, channel_signals[i+1], getVin(channel_signals[i+1], ADC_VREF*1.25, ADC_VREF*-1.25));
		printf("ct right: ditial = %d, analog in = %f\n", i%3, channel_signals[i+2], getVin(channel_signals[i+2], ADC_VREF*1.25, ADC_VREF*-1.25));
	}*/

// ----------------------------------------------------------------------
// Power Measurment Test Code
// ----------------------------------------------------------------------
#if (POWER_MEASURMENT_TEST)
	powersc_t psc = {0};
	powersc_t pscR = {0};
	// This call tries to estimate the phase error introduced into
	// the voltage reading due to the ADC multiplexer. This was taken from
	// the ADS8688/ADS8684 datasheet.
	double theoreticalPhaseError = getTheoreticalPhaseError(3);
	// We calculate the PHASECAL constant by finding the proportion
	// of our estimated error to a full period delay, which is 6.
	PHASECAL = 1.0 + (theoreticalPhaseError/6.0);
	printf("PHASECAL: %f\n", PHASECAL);
	//PHASECAL = 1.7; // from tutorial
	VCAL = VOLTAGE_CALIBRATION_SCHOOL_P2;
	ICAL = CURRENT_CALIBRATION;
	/* Reset auto sequence */
	//enter_auto_sequence_mode();


	int i = 0;
	for (current_sample = 0; current_sample < num_test_samples; current_sample++) {
		pt_buf[current_sample] = auto_channel_read();
		ct2_buf[current_sample] = auto_channel_read();
		ct1_buf[current_sample] = auto_channel_read();
		//printf("%d\n", auto_channel_read());
		i = current_sample;
		//printf("pt: digital out = %d, analog in = %f\n", pt_buf[i], getVin(pt_buf[i], ADC_VREF*1.25, 0) );
		//printf("ct left: digital = %d, analog in = %f\n", ct2_buf[i], getVin(ct2_buf[i], ADC_VREF*1.25, ADC_VREF*-1.25));
		//printf("ct right: ditial = %d, analog in = %f\n\n", ct1_buf[i], getVin(ct1_buf[i], ADC_VREF*1.25, ADC_VREF*-1.25));
	}

	current_sample = 0;
	calcVI(20, &psc, PT_CHANNEL, CTL_CHANNEL);
	current_sample_pt = 0;
	calcVI(20, &pscR, PT_CHANNEL, CTR_CHANNEL);
	printf("Vrms = %f, Irms Left = %f\n", psc.Vrms, psc.Irms);
	printf("Vrms = %f, Irms Right = %f\n", pscR.Vrms, pscR.Irms);
	/*
	double c1_current = calcIrms(num_test_samples, CTL_CHANNEL);
	current_sample = 0;
	double c2_current = calcIrms(num_test_samples, CTR_CHANNEL);
	current_sample = 0;
	printf("Irms left = %f\n", c1_current);
	printf("Irms right = %f\n", c2_current);
	printf("Vrms right = %f\n", calcIrms(num_test_samples, PT_CHANNEL));
	*/
#endif
	return EXIT_SUCCESS;
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-cs]\n", prog);
	puts(	"-c --chip-enable	chip enable pin (default CE0)\n"
			"-s --speed			max speed (Hz)\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "chip-enable",	1, 0, 'c' },
			{ "speed",			1, 0, 's' },
			{ NULL,				0, 0, 0 },
		};
		int c;
		c = getopt_long(argc, argv, "c:s:", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
			case 'c':
				channel = atoi(optarg);
				break;
			case 's':
				speed = atoi(optarg);
				break;
			default:
				print_usage(argv[0]);
				break;
		}
	}
}

//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kWh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
void calcVI(unsigned int crossings, powersc_t *power_p, uint8_t inPinV, uint8_t inPinI)
{
	double sqV = 0, sumV = 0, sqI = 0, sumI = 0, instP = 0, sumP = 0; //sq = squared, sum = Sum, inst = instantaneous
	int startV; // Instantaneous voltage at start of sample window.
	int sampleV; //sample_ holds the raw analog read value
	int sampleI;
	bool lastVCross = false, checkVCross = false; // Used to measure number of times threshold is crossed.
	double lastFilteredV, filteredV = 0; // Filtered is the raw analog value minus the DC offset
	double filteredI;
	double offsetV = ADC_COUNTS>>1;                          // Low-pass filter output
	double offsetI = (ADC_COUNTS>>1);
	double phaseShiftedV;                    // Holds the calibrated phase shifted voltage.

	double SupplyVoltage = ADC_VREF;//4096; // Vref
	unsigned int crossCount = 0; // Used to measure number of times threshold is crossed.
	unsigned int numberOfSamples = 0; // This is now incremented
	double V_RATIO = VCAL *((SupplyVoltage*PGA_GAIN_V) / (ADC_COUNTS));
	double I_RATIO = ICAL *((SupplyVoltage*PGA_GAIN_I) / (ADC_COUNTS));
	

	//---------------------------------------------------------------
	// 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
	//---------------------------------------------------------------
	//unsigned long start = millis(); //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

	while(1)                                   //the while loop...
	{
		startV = analogRead(inPinV); // using the voltage waveform
		if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;  //check its within range
		//if ((millis()-start)>timeout) break;
	}

	//-------------------------------------------------------------------------------------------------------------------------
	// 2) Main measurement loop
	//-------------------------------------------------------------------------------------------------------------------------
	//start = millis();

	while ((crossCount < crossings) && (current_sample_pt < num_test_samples))// && ((millis()-start)<timeout))
	{
		numberOfSamples++;                       //Count number of times looped.
		lastFilteredV = filteredV;               //Used for delay/phase compensation

		//-----------------------------------------------------------------------------
		// A) Read in raw voltage and current samples
		//-----------------------------------------------------------------------------
		sampleV = analogRead(inPinV); //Read in raw voltage signal
		sampleI = analogRead(inPinI); //Read in raw current signal

		//-----------------------------------------------------------------------------
		// B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
		//     then subtract this - signal is now centred on 0 counts.
		//-----------------------------------------------------------------------------
		offsetV = offsetV + ((sampleV-offsetV)/ADC_COUNTS);
		filteredV = sampleV - offsetV;
		power_p->Vraw = V_RATIO * filteredV;//(filteredV*SupplyVoltage)/ADC_COUNTS;
		//printf("Vraw: %f\n", power_p->Vraw);

		// Commenting out lines below this b/c our ADC is bipolar, so we 
		// don't need to filter current (or do we??) 
		offsetI = offsetI + ((sampleI-offsetI)/ADC_COUNTS);
		filteredI = sampleI - offsetI;
		//filteredI = sampleI;
		power_p->Iraw = (I_RATIO * filteredI) * (CT_TURNS / BURDEN_RESISTOR_OHMS);
		//printf("filteredI = %f\n", filteredI);
		//printf("Vraw = %f, Iraw =  %f\n", power_p->Vraw, power_p->Iraw);

		//-----------------------------------------------------------------------------
		// C) Root-mean-square method voltage
		//-----------------------------------------------------------------------------
		sqV= filteredV * filteredV;                 //1) square voltage values
		sumV += sqV;                                //2) sum

		//-----------------------------------------------------------------------------
		// D) Root-mean-square method current
		//-----------------------------------------------------------------------------
		sqI = filteredI * filteredI;                //1) square current values
		sumI += sqI;                                //2) sum

		//-----------------------------------------------------------------------------
		// E) Phase calibration
		//-----------------------------------------------------------------------------
		phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);

		//-----------------------------------------------------------------------------
		// F) Instantaneous power calc
		//-----------------------------------------------------------------------------
		instP = phaseShiftedV * filteredI;          //Instantaneous Power
		sumP +=instP;                               //Sum

		//-----------------------------------------------------------------------------
		// G) Find the number of times the voltage has crossed the initial voltage
		//    - every 2 crosses we will have sampled 1 wavelength
		//    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
		//-----------------------------------------------------------------------------
		lastVCross = checkVCross;
		if (sampleV > startV) checkVCross = true;
		else checkVCross = false;
		if (numberOfSamples==1) lastVCross = checkVCross;

		if (lastVCross != checkVCross) crossCount++;
	}

	//-------------------------------------------------------------------
	// 3) Post loop calculations
	//-------------------------------------------------------------------
	// Calculation of the root of the mean of the voltage and current squared (rms)
	// Calibration coefficients applied.

	power_p->Vrms = V_RATIO * sqrt(sumV / (double)numberOfSamples);

	power_p->Irms = I_RATIO * sqrt(sumI / (double)numberOfSamples) * (CT_TURNS / BURDEN_RESISTOR_OHMS);

	//Calculation power values
	power_p->realPower = V_RATIO * I_RATIO * sumP / (double)numberOfSamples;
	power_p->apparentPower = power_p->Vrms * power_p->Irms;
	power_p->powerFactor = power_p->realPower / power_p->apparentPower;

	//Reset accumulators
	//sumV = 0;
	//sumI = 0;
	//sumP = 0;
	//--------------------------------------------------------------------------------------
}

//--------------------------------------------------------------------------------------
double calcIrms(unsigned int number_of_samples, uint8_t inPinI)
{

	double supplyVoltage = ADC_VREF;
	double offsetI = ADC_COUNTS>>1;
	int sampleI;
	double filteredI;
	double sumI = 0;
	double sqI = 0;
	double I_RATIO = ICAL *((supplyVoltage*PGA_GAIN_I) / (ADC_COUNTS));

	for (unsigned int n = 0; n < number_of_samples; n++)
	{
		sampleI = analogRead(inPinI);
		offsetI = offsetI + ((sampleI-offsetI)/ADC_COUNTS);
		filteredI = sampleI - offsetI;
		// Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
		//  then subtract this - signal is now centered on 0 counts.
		//offsetI = (offsetI + (sampleI-offsetI)/ADC_COUNTS);
		//filteredI = (sampleI - offsetI);// (0.625*4.08);//ADC_VREF);
		//filteredI = (sampleI - offsetI);/// (1.25*4.08);//ADC_VREF);
		//filteredI = getVin(sampleI, ADC_VREF*1.25, ADC_VREF*-1.25);
		//filteredI = sampleI;

		// Root-mean-square method current
		// 1) square current values
		sqI = filteredI * filteredI;
		// 2) sum
		sumI += sqI;
	}

	double Irms = I_RATIO * sqrt(sumI / number_of_samples);
	//double Irms = sqrt(sumI / number_of_samples);

	//Reset accumulators
	//sumI = 0;
	//--------------------------------------------------------------------------------------

	return Irms;
}
