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
#include <pthread.h>
#include <semaphore.h>
#include <curl/curl.h>

// run helgrind to check deadlock conditions

#include "spiComm.h"
#include "measure.h"

#define POWER_MEASURMENT_TEST		0
#define CURL_VERBOSE_DEBUG_LEVEL	0L
#define POWER_DEBUG					0

#define ADC_FREQ 500000
#define VIN_FREQ 60
//#define max_samples ((size_t)((1.0/VIN_FREQ)/(1.0/ADC_FREQ)) * 2)
#define max_samples 20000 //16667

static void parse_opts(int argc, char *argv[]);
void powerMonitorTest();
void *power_monitor();

#define PT_CHANNEL 0
#define CTL_CHANNEL 1
#define CTR_CHANNEL 2

// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static uint8_t channel = 0;
//static uint8_t mode = 1;
static uint32_t speed = 500000;
static unsigned int default_crossings = 20;
static double PGA_GAIN_I = 0.625;
static double PGA_GAIN_V = 1.25;

// These buffer store a bunch of samples for each sensor.
// These were made to simplfy the testing process.
// We'll start with buffers that may store a minimum of 2 full wave cycles.
static int ct1_buf[max_samples];
static int ct2_buf[max_samples];
static int pt_buf[max_samples];
static uint32_t current_sample;
static uint32_t current_sample_pt;
static uint32_t current_sample_ct1;
static uint32_t current_sample_ct2;
static uint32_t oldest_sample_pt;
static uint32_t oldest_sample_ct1;
static uint32_t oldest_sample_ct2;

/* Synchronization */
sem_t buf_empty;
sem_t buf_full;

/* Debugging function */
#if POWER_MEASURMENT_TEST
static void print_buffer(uint8_t *buf, int length)
{
	//printf("Buffer: ");
	for (int ret = 0; ret < length; ret++) {
		//if (!(ret % 4))puts("");
		printf("%.2X ", buf[ret]);
	}
	puts("");
}
#endif

/* Integer timestamp to floating point */
static inline double get_time_sec(time_t tv_sec, long tv_nsec) {
	return (double)tv_sec + ((double)tv_nsec/1000000000);

}
/*
 * Helper function used for testing sensor measurments. Returns value from
 * the buffer of the given channel as an int. This simplifies emonlib port.
 */
static inline int analogRead(uint8_t channel) {
	//int i = current_sample; current_sample++;
	int reading;
	//sem_wait(&buf_empty);

	switch(channel) {
		case CTR_CHANNEL:
			/*if (current_sample_ct1 == oldest_sample_ct1) {
				printf("Trying to get too many samples");
				return -1;
			}*/
			reading = ct1_buf[oldest_sample_ct1];
			//oldest_sample_ct1++;
			//oldest_sample_ct1 = (oldest_sample_ct1 + 1) % max_samples;
			break;
		case CTL_CHANNEL:
			/*if (current_sample_ct2 == oldest_sample_ct2) {
				printf("Trying to get too many samples\n");
				return -1;
			}*/
			reading = ct2_buf[oldest_sample_ct2];
			//oldest_sample_ct2 = (oldest_sample_ct2 + 1) % max_samples;
			break;
		case PT_CHANNEL:
			/*if (current_sample_pt == oldest_sample_pt) {
				printf("Trying to get too many samples\n");
				return -1;
			}*/
			reading = pt_buf[oldest_sample_pt];
			//oldest_sample_pt = (oldest_sample_pt + 1) % max_samples;
			break;
		default:
			reading =  0;
			break;
	}
	oldest_sample_ct1 = (oldest_sample_ct1 + 1) % max_samples;
	oldest_sample_ct2 = (oldest_sample_ct2 + 1) % max_samples;
	oldest_sample_pt = (oldest_sample_pt + 1) % max_samples;
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

	spiDataRW(channel, tx, rx, 3);
	tmp = rx[2];

	return tmp;
}

static inline void reset_program_regs()
{
	uint32_t cmd = ADS8688_CMD_REG(ADS8688_CMD_REG_RST);
	cmd <<= ADS8688_CMD_DONT_CARE_BITS;
	iobuf.d32 = htobe32(cmd);
	spiDataRW(channel, &iobuf.d8[0], &iobuf.d8[0], 4);
}

static inline void enter_auto_sequence_mode()
{
	uint32_t cmd = ADS8688_CMD_REG(ADS8688_AUTO_RST_CMD);
	cmd <<= ADS8688_CMD_DONT_CARE_BITS;
	iobuf.d32 = htobe32(cmd);
	spiDataRW(channel, &iobuf.d8[0], &iobuf.d8[0], 4);
}

// Assumes we are in AUTO_SEQ state so just sends NO_OP command
int32_t auto_channel_read()
{
	uint32_t tmp = ADS8688_CMD_REG(ADS8688_CMD_REG_NOOP);
	//tmp <= ADS8688_CMD_DONT_CARE_BITS;
	iobuf.d32 = htobe32(tmp);
	spiDataRW(channel, &iobuf.d8[0], &iobuf.d8[0], 4);
	return be32toh(iobuf.d32) & 0xffff;
}

int main(int argc, char *argv[])
{
	int fd;
	uint32_t prog_result;

	parse_opts(argc, argv);

	printf("default crossings: %d\n", default_crossings);

	// Configure the interface.
	// channel indicates chip select,
	fd = spiSetup(channel, speed);
	if (fd < 0) return EXIT_FAILURE;

	/* Reset Program Regs */
	reset_program_regs();
	usleep(15000);

	/* Power down channel 4 */
	prog_result = ads8688_prog_write(fd, CHANNEL_PWR_DWN_ADDR, 0xf8);
	printf("power down channel response: %x\n", prog_result);
	/* Enable auto scan for channels 0, 1 and 2 */
	prog_result = ads8688_prog_write(fd, AUTO_SEQ_EN_ADDR, 0x07);
	printf("enable channels response: %x\n", prog_result);
	/* Set ranges for CT's on channel 1 and channel 2 */
	prog_result = ads8688_prog_write(fd, C1RANGE, ADS8688_REG_PLUSMINUS0625VREF);
	printf("set CT_L range response: %x\n", prog_result);
	prog_result = ads8688_prog_write(fd, C2RANGE, ADS8688_REG_PLUSMINUS0625VREF);
	printf("set CT_R range response: %x\n", prog_result);
	PGA_GAIN_I = 0.625;
	/* Set range for PT on channel 0 */
	prog_result = ads8688_prog_write(fd, C0RANGE, ADS8688_REG_PLUS125VREF);
	printf("set PT range response: %x\n", prog_result);

	// This call tries to estimate the phase error introduced into
	// the voltage reading due to the ADC multiplexer. This was taken from
	// the ADS8688/ADS8684 datasheet.
	double theoreticalPhaseError = getTheoreticalPhaseError(3);

	// We calculate the PHASECAL constant by finding the proportion
	// of our estimated error to a full period delay, which is 6.
	PHASECAL = 1.0 + (theoreticalPhaseError/6.0);
	//PHASECAL = 1.7; // from tutorial
	PHASECAL = 1;

	VCAL = VOLTAGE_CALIBRATION_SCHOOL_P2;
	ICAL_L = CURRENT_CALIBRATION_L;
	ICAL_R = CURRENT_CALIBRATION_R;
	printf("PHASECAL: %f, VCAL: %f, ICAL_L: %f, ICAL_R: %f\n", PHASECAL, VCAL, ICAL_L, ICAL_R);

	current_sample_pt = 0;
	current_sample_ct1 = 0;
	current_sample_ct2 = 0;

// ----------------------------------------------------------------------
// Power Measurment Test Code
// ----------------------------------------------------------------------
#if (POWER_MEASURMENT_TEST)
	powerMonitorTest();	
	return EXIT_FAILURE;
#endif

	sem_init(&buf_empty, 0, 0);	
	sem_init(&buf_full, 0, max_samples);	

	pthread_t power_thread;
	if (pthread_create(&power_thread, NULL, power_monitor, NULL) < 0) {
		perror("pthread_create failed");
		return EXIT_FAILURE;
	}

	/* Enter auto sequence mode */
	enter_auto_sequence_mode();

	/* Automatically sequence through channels and read signal */
	while (true) {
		sem_wait(&buf_full);
		pt_buf[current_sample_pt] = auto_channel_read();
		ct2_buf[current_sample_ct2] = auto_channel_read();
		ct1_buf[current_sample_ct1] = auto_channel_read(); 
		//printf("right ADC: %d, left ADC:  %d\n", ct1_buf[current_sample_ct1], ct2_buf[current_sample_ct2]);
		current_sample_pt = (current_sample_pt + 1) % max_samples;
		current_sample_ct2 = (current_sample_ct2 + 1) % max_samples;
		current_sample_ct1 = (current_sample_ct1 + 1) % max_samples;
		sem_post(&buf_empty);
	}

	pthread_join(power_thread, NULL);
	return EXIT_SUCCESS;
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-cs]\n", prog);
	puts(	"-c --chip-enable	chip enable pin (default CE0)\n"
			"-s --speed			max speed (Hz)\n"
			"-n --crossings		default number of crossings\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "chip-enable",	1, 0, 'c' },
			{ "speed",			1, 0, 's' },
			{ "crossings",		1, 0, 'n' },
			{ NULL,				0, 0, 0 },
		};
		int c;
		c = getopt_long(argc, argv, "c:s:n:", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
			case 'c':
				channel = atoi(optarg);
				break;
			case 's':
				speed = atoi(optarg);
				break;
			case 'n':
				default_crossings = atoi(optarg);
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
void calcVI(unsigned int crossings, powersc_t *powerl_p, powersc_t *powerr_p, uint8_t inPinV, uint8_t inPinIl, uint8_t inPinIr)
{
	double sqV = 0, sumV = 0, sqI = 0, sumI = 0, instP = 0, sumP = 0; //sq = squared, sum = Sum, inst = instantaneous
	double sqIr = 0, sumIr = 0, instPr = 0, sumPr = 0;
	int startV = 0; // Instantaneous voltage at start of sample window.
	int sampleV = 0; //sample_ holds the raw analog read value
	int sampleI = 0;
	int sampleIr = 0;
	bool lastVCross = false, checkVCross = false; // Used to measure number of times threshold is crossed.
	double lastFilteredV, filteredV = 0; // Filtered is the raw analog value minus the DC offset
	double filteredI = 0;
	double filteredIr = 0;
	double offsetV = ADC_COUNTS>>1; // Low-pass filter output
	double offsetI = (ADC_COUNTS>>1);
	double offsetIr = (ADC_COUNTS>>1);
	double phaseShiftedV = 0; // Holds the calibrated phase shifted voltage.

	double SupplyVoltage = ADC_VREF;//4096; // Vref
	unsigned int crossCount = 0; // Used to measure number of times threshold is crossed.
	unsigned int numberOfSamples = 0; // This is now incremented
	double V_RATIO = VCAL *((SupplyVoltage*PGA_GAIN_V) / (ADC_COUNTS));
	double I_RATIO_L = ICAL_L *((SupplyVoltage*PGA_GAIN_I) / (ADC_COUNTS)) * (CT_TURNS / BURDEN_RESISTOR_OHMS);
	double I_RATIO_R = ICAL_R *((SupplyVoltage*PGA_GAIN_I) / (ADC_COUNTS)) * (CT_TURNS / BURDEN_RESISTOR_OHMS);
	

	//---------------------------------------------------------------
	// 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
	//---------------------------------------------------------------
	//unsigned long start = millis(); //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

	while(1)                                   //the while loop...
	{
		sem_wait(&buf_empty);
		startV = analogRead(inPinV); // using the voltage waveform
		sem_post(&buf_full);
		if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;  //check its within range
		//if ((millis()-start)>timeout) break;
	}

	//-------------------------------------------------------------------------------------------------------------------------
	// 2) Main measurement loop
	//-------------------------------------------------------------------------------------------------------------------------
	//start = millis();

	while ((crossCount < crossings)) //&& (current_sample_pt < max_samples))// && ((millis()-start)<timeout))
	{
		numberOfSamples++;                       //Count number of times looped.
		lastFilteredV = filteredV;               //Used for delay/phase compensation

		//-----------------------------------------------------------------------------
		// A) Read in raw voltage and current samples
		//-----------------------------------------------------------------------------
		sem_wait(&buf_empty);
		sampleV = analogRead(inPinV); //Read in raw voltage signal
		sampleI = analogRead(inPinIl); // Read in left raw current signal
		sampleIr = analogRead(inPinIr); // Read in right raw current signal
		sem_post(&buf_full);

		//-----------------------------------------------------------------------------
		// B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
		//     then subtract this - signal is now centred on 0 counts.
		//-----------------------------------------------------------------------------
		offsetV = offsetV + ((sampleV-offsetV)/ADC_COUNTS);
		filteredV = sampleV - offsetV;
		powerl_p->Vraw = V_RATIO * filteredV;
		powerr_p->Vraw = V_RATIO * filteredV;

		offsetI = offsetI + ((sampleI-offsetI)/ADC_COUNTS);
		filteredI = sampleI - offsetI;
		powerl_p->Iraw = filteredI;

		offsetIr = offsetIr + ((sampleIr-offsetIr)/ADC_COUNTS);
		filteredIr = sampleIr - offsetIr;
		powerr_p->Iraw = filteredIr;
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

		sqIr = filteredIr * filteredIr;                //1) square current values
		sumIr += sqIr;

		//-----------------------------------------------------------------------------
		// E) Phase calibration
		//-----------------------------------------------------------------------------
		phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);

		//-----------------------------------------------------------------------------
		// F) Instantaneous power calc
		//-----------------------------------------------------------------------------
		instP = phaseShiftedV * filteredI;          //Instantaneous Power
		sumP +=instP;                               //Sum

		instPr = phaseShiftedV * filteredIr;          //Instantaneous Power
		sumPr += instPr;                               //Sum
		//printf("P = %f, Pr = %f\n", sumP, sumPr);
	//	printf("Iraw right = %f, Iraw left = %f, V = %f\n", filteredIr, filteredI, phaseShiftedV); 

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

	clock_gettime(CLOCK_REALTIME, powerl_p->tv);
	//-------------------------------------------------------------------
	// 3) Post loop calculations
	//-------------------------------------------------------------------
	// Calculation of the root of the mean of the voltage and current squared (rms)
	// Calibration coefficients applied.

	powerl_p->Vrms = V_RATIO * sqrt(sumV / (double)numberOfSamples);
	powerl_p->Irms = I_RATIO_L * sqrt(sumI / (double)numberOfSamples);

	powerr_p->Vrms = V_RATIO * sqrt(sumV / (double)numberOfSamples);
	powerr_p->Irms = I_RATIO_R * sqrt(sumIr / (double)numberOfSamples);

	//Calculation power values
	powerl_p->realPower = V_RATIO * I_RATIO_L * (double)sumP / (double)numberOfSamples;
	powerl_p->apparentPower = powerl_p->Vrms * powerl_p->Irms;
	powerl_p->powerFactor = powerl_p->realPower / powerl_p->apparentPower;

	powerr_p->realPower = V_RATIO * I_RATIO_R * (double)sumPr / (double)numberOfSamples;
	powerr_p->apparentPower = powerr_p->Vrms * powerr_p->Irms;
	powerr_p->powerFactor = powerr_p->realPower / powerr_p->apparentPower;
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
	double I_RATIO = ICAL_L * ((supplyVoltage*PGA_GAIN_I) / (ADC_COUNTS)) * (CT_TURNS / BURDEN_RESISTOR_OHMS);

	for (unsigned int n = 0; n < number_of_samples; n++)
	{
		// Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
		//  then subtract this - signal is now centered on 0 counts.
		sampleI = analogRead(inPinI);
		offsetI = offsetI + ((sampleI-offsetI)/ADC_COUNTS);
		filteredI = sampleI - offsetI;
		// Root-mean-square method current
		// 1) square current values
		sqI = filteredI * filteredI;
		// 2) sum
		sumI += sqI;
	}

	double Irms = I_RATIO * sqrt(sumI / number_of_samples);

	//--------------------------------------------------------------------------------------

	return Irms;
}

void powerMonitorTest() {
	powersc_t pscL = {0};
	powersc_t pscR = {0};

	/* Reset auto sequence */
	enter_auto_sequence_mode();

	for (current_sample = 0; current_sample < max_samples; current_sample++) {
		pt_buf[current_sample] = auto_channel_read();
		ct2_buf[current_sample] = auto_channel_read();
		ct1_buf[current_sample] = auto_channel_read();
		// posting to semaphore in single thread cause I'm lazy
		sem_post(&buf_empty);
	}

	calcVI(default_crossings, &pscL, &pscR, PT_CHANNEL, CTL_CHANNEL, CTR_CHANNEL);
	//current_sample = 0;
	//calcVI(20, &pscL, PT_CHANNEL, CTL_CHANNEL);
	//current_sample_pt = 0;
	//calcVI(20, &pscR, PT_CHANNEL, CTR_CHANNEL);

	printf("Vrms = %f, Irms Left = %f\n", pscL.Vrms, pscL.Irms);
	printf("Vrms = %f, Irms Right = %f\n", pscR.Vrms, pscR.Irms);
}

/****************************************************************************/
/* routine used by curl to read from the network */

/* curl calls this to transfer data from the network into RAM */
static size_t curl_write_cb(char * ptr, size_t size, size_t nmemb, void *userdata) {
#if CURL_VERBOSE_DEBUG_LEVEL
	//fprintf(stdout, "post response: %s\n", ptr);
#endif
	return size * nmemb;
}

void *power_monitor() {
	powersc_t pscl = {0};
	powersc_t pscr = {0};

	struct timespec tv;
	CURL *curl;
	CURLcode res;

	/* In windows, this will init the winsock stuff */ 
	if (curl_global_init(CURL_GLOBAL_ALL)) {
		fprintf(stderr, "curl_global_init failed... exiting.\n");
		pthread_exit(NULL);
	}

	/* get a curl handle */ 
	curl = curl_easy_init();
	if (!curl) {
		printf("[main] could not initialize curl");
		abort();
	}

	const char *format_str = "time=%f&current1=%f&voltage=%f&realP1=%f&current2=%f&realP2=%f&user=%u";
	char * post_str = (char*)malloc(sizeof(double)*6 + strlen(format_str));
	char * post_response = (char*)malloc(sizeof(double)*6 + strlen(format_str));

	/* First set the URL that is about to receive our POST. This URL can
	 *        just as well be a https:// URL if that is what should receive the
	 *               data. */ 
	curl_easy_setopt(curl, CURLOPT_URL, "https://flask-petal.herokuapp.com/");
	curl_easy_setopt(curl, CURLOPT_POST, 1L);	

	/* some servers don't like requests that are made without a user-agent
	 *        field, so we provide one */
	curl_easy_setopt(curl, CURLOPT_USERAGENT, "libcurl-agent/1.0");
	curl_easy_setopt(curl, CURLOPT_VERBOSE, CURL_VERBOSE_DEBUG_LEVEL);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, post_response);

	pscl.tv = &tv;
	pscr.tv = &tv;
	unsigned int userId = 0;
	while (true) {
		//memset(&pscl, 0, sizeof(powersc_t));
		//memset(&pscr, 0, sizeof(powersc_t));
		pscl.realPower = 0; pscr.realPower = 0;
		pscl.Vrms = 0; pscr.Vrms = 0;
		pscl.Irms = 0; pscr.Irms = 0;
		calcVI(default_crossings, &pscl, &pscr, PT_CHANNEL, CTL_CHANNEL, CTR_CHANNEL);
#if POWER_DEBUG
		printf("Vrms = %f, Irms Left = %f, Pl = %f, ", pscl.Vrms, pscl.Irms, pscl.realPower);
		printf("Vrms = %f, Irms Right = %f, Pr = %f\n", pscr.Vrms, pscr.Irms, pscr.realPower);
#endif
		sprintf(post_str,
				format_str,
				get_time_sec(tv.tv_sec, tv.tv_nsec),
				pscl.Irms,
				pscl.Vrms,
				pscl.realPower,
				pscr.Irms,
				pscr.realPower,
				userId);

		curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)strlen(post_str));
		/* Now specify the POST data */
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_str);
		/* Perform the request, res will get the return code */ 
		res = curl_easy_perform(curl);
		/* Check for errors */ 
		if(res != CURLE_OK) {
			fprintf(stderr, "curl_easy_perform failed: %s\n", curl_easy_strerror(res));
		}
	}

	curl_easy_cleanup(curl);
	curl_global_cleanup();
}
