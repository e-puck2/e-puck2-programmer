
#include "main.h"
#include "battery_measurement.h"

#define ADC_NUM_CHANNELS		2
#define ADC_BATT_NUM_SAMPLES	32

#define ADC_CHANNEL_BATT		8

static adcsample_t batt_samples[ADC_NUM_CHANNELS * ADC_BATT_NUM_SAMPLES];
static float battery_measurement = 0;
static float vref_measurement = 0;

static const ADCConversionGroup adcgrpcfg =  {
	.circular = false, 
 	.num_channels = ADC_NUM_CHANNELS,
 	.end_cb = NULL,
	.error_cb = NULL,
	.cr1 = 0,
	.cr2 = ADC_CR2_SWSTART,
	.smpr2 = ADC_SMPR2_SMP_AN8(ADC_SAMPLE_480),
	.smpr1 = ADC_SMPR1_SMP_VREF(ADC_SAMPLE_480),
	.sqr3 = 0,
	.sqr2 = 0,
	.sqr1 = ADC_SQR3_SQ1_N(ADC_CHANNEL_BATT) | ADC_SQR3_SQ2_N(ADC_CHANNEL_VREFINT)
};

static THD_WORKING_AREA(batt_thd_wa, 256);
static THD_FUNCTION(batt_thd, arg)
{
	(void) arg;

	chRegSetThreadName("Battery measurement");

	while(1){

		adcConvert(&ADC_BATT, &adcgrpcfg, batt_samples, ADC_BATT_NUM_SAMPLES);
		battery_measurement = 0;
		vref_measurement = 0;
		for(uint16_t i = 0 ; i < (ADC_NUM_CHANNELS * ADC_BATT_NUM_SAMPLES) ; i += ADC_NUM_CHANNELS){
			battery_measurement += batt_samples[i];
			vref_measurement += batt_samples[i+1];
		}

		battery_measurement /= ADC_BATT_NUM_SAMPLES;
		vref_measurement /= ADC_BATT_NUM_SAMPLES;

		chprintf((BaseSequentialStream *) &SDU2,"batt =  %f , vref = %f\n",battery_measurement, vref_measurement);

		chThdSleepMilliseconds(500);
	}
}


void batteryMesurementStart(void){
	/*
	* Activates the ADC driver and the Vref sensor.
	*/
	adcStart(&ADC_BATT, NULL);
	adcSTM32EnableTSVREFE();
	/**
	 * Starts the battery measurement thread
	 */
	chThdCreateStatic(batt_thd_wa, sizeof(batt_thd_wa), NORMALPRIO, batt_thd, NULL);
}