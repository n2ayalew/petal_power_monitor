#ifndef EMONITOR_H
#define EMONITOR_H

#include <stdlib.h>
#include <stdint.h>

#define ADC_BITS 16

#define ADC_COUNTS  (1 << ADC_BITS)
// power screen shot
typedef struct powersc {
	double realPower;
	double apparentPower;
	double powerFactor;
	double Vrms;
	double Irms;
	double Vraw;
	double Iraw;
} powersc_t;

//Calibration coefficients
//These need to be set in order to obtain accurate results
double VCAL;
double ICAL;
double PHASECAL;


//-------------------------------------------------------------------------
// Variable declaration for emon_calc procedure
//-------------------------------------------------------------------------
//int sampleV;                        //sample_ holds the raw analog read value
//int sampleI;

//double lastFilteredV,filteredV;          //Filtered_ is the raw analog value minus the DC offset
//double filteredI;
//double offsetV;                          //Low-pass filter output
//double offsetI;                          //Low-pass filter output

//double phaseShiftedV;                             //Holds the calibrated phase shifted voltage.

//double sqV,sumV,sqI,sumI,instP,sumP;              //sq = squared, sum = Sum, inst = instantaneous

//int startV;                                       //Instantaneous voltage at start of sample window.

//bool lastVCross, checkVCross;                  //Used to measure number of times threshold is crossed.


void calcVI(unsigned int crossings, powersc_t *power_p, uint8_t inPinV, uint8_t inPinI);
double calcIrms(unsigned int number_of_samples, uint8_t inPinI);
#endif
