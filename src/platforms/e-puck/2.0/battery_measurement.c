/**
 * @file	battery_measurement.c
 * @brief  	Functions to measure the battery voltage and change the battery state accordingly
 * 			Uses the event system to signal a new state
 * 
 * @written by  	Eliot Ferragni
 * @creation date	26.06.2018
 */

#include "main.h"
#include "battery_measurement.h"
#include "power_button.h"

#define ADC_NUM_CHANNELS		2	//Batt and VrefInt
#define ADC_BATT_NUM_SAMPLES	64	//32 samples by channel

#define ADC_ONE_CHANNEL			1
#define ADC_ONE_SAMPLE			1

#define ADC_CHANNEL_BATT		8

#define VOLTAGE_DROP			0.11f	//because of the diodes
#define VOLTAGE_DIVIDER         (1.0f * RESISTOR_R2 / (RESISTOR_R1 + RESISTOR_R2))

#define VREF                    3.0f //volt corresponds to the voltage on the VREF+ pin
#define ADC_RESOLUTION          4095

#define COEFF_ADC_TO_VOLT       ((1.0f * ADC_RESOLUTION * VOLTAGE_DIVIDER) / VREF) //conversion from adc value to voltage

#define LOW_PASS_COEFF_A		0.8f
#define LOW_PASS_COEFF_B		0.2f

#define ADC_VALUE_VREFINT		1651.5f	//value of the Vrefint (1.2V) measure when the uC is powered with 3V

//buffer filled by the ADC
static adcsample_t batt_samples[ADC_NUM_CHANNELS * ADC_BATT_NUM_SAMPLES];

//battery voltage variable
static float battery_voltage = 0;

//Event source used to send events to other threads
event_source_t battery_info_event;

static BSEMAPHORE_DECL(measurement_ready, true);

static void adcCb(ADCDriver *adcp, adcsample_t *buffer, size_t n);

//groupe conversion config for the ADC
static const ADCConversionGroup adcGroupConfig =  {
	.circular = true, 	//we need it to be circular to be able to relauche it simply by setting the SWSTART bit in CR2
 	.num_channels = ADC_ONE_CHANNEL,	//we need to do one conversion at a time -> num_channels need to be one
 	.end_cb = adcCb,
	.error_cb = NULL,
	.cr1 = 0,
	.cr2 = 0,
	.smpr2 = ADC_SMPR2_SMP_AN8(ADC_SAMPLE_480),
	.smpr1 = ADC_SMPR1_SMP_VREF(ADC_SAMPLE_480),
	.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_BATT),// The first channel to be sampled is the battery
	.sqr2 = 0,
	.sqr1 = 0,
};

/////////////////////////////////////////PRIVATE FUNCTIONS/////////////////////////////////////////

/**
 * @brief 	Launches one conversion of the given ADC
 * 			The ADC must already be configured in order to work properly
 * @param adcp The adc to start
 */
void ADCdoOneConversion(ADCDriver *adcp){
	adcp->adc->CR2 |= ADC_CR2_SWSTART;
}

/**
 * @brief 	Sets the chanel to sample for the given ADC
 * 			Only used to set a sequence of one conversion
 * 
 * @param adcp 			The adc to configure
 * @param adc_channel 	The channel to set
 */
void ADCsetChannel(ADCDriver *adcp, uint8_t adc_channel){
	adcp->adc->SQR3  = ADC_SQR3_SQ1_N(adc_channel);
}

//called after each single conversion
//relauches a convesion and alternates the channel to sample.
//we emulate a continuous conversion of 2 channels of ADC_BATT_NUM_SAMPLES samples
//becasuse the adc has a random offset at the start of the uC when the CONT bit is set in CR2
static void adcCb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	(void)adcp;
	(void)n;
	static uint8_t count = 0;
	batt_samples[count] = buffer[0];
	count++;
	if(count < (ADC_NUM_CHANNELS * ADC_BATT_NUM_SAMPLES)){
		if(count % 2){
			//odd numbers => next sampling is for ADC_CHANNEL_VREFINT
			ADCsetChannel(adcp, ADC_CHANNEL_VREFINT);
		}else{
			//even numbers => next sampling is for ADC_CHANNEL_BATT
			ADCsetChannel(adcp, ADC_CHANNEL_BATT);
		}
		ADCdoOneConversion(adcp);
	}else{
		//we have finished. Reset the adc and counter for the first sample => ADC_CHANNEL_BATT
		//and signal the end of conversion to the thread
		count = 0;
		ADCsetChannel(adcp, ADC_CHANNEL_BATT);
		chSysLockFromISR();
		chBSemSignalI(&measurement_ready);
		chSysUnlockFromISR();
	}
}

void batteryStateMachine(void){
	//actual state
	static uint8_t actual_state = MAX_VOLTAGE_FLAG;
	//state in which the battery voltage tells us to be.
	//-> we use it in order to change the actual state only after a fixed amount
	//of time. It prevents quick changes in the state if the measurements oscillate.
	static uint8_t new_state = MAX_VOLTAGE_FLAG;

	static systime_t time_state = 0;
	static systime_t time_battery_low = 0;

	if(battery_voltage <= MIN_VOLTAGE){
		new_state = MIN_VOLTAGE_FLAG;
		
	}else if(battery_voltage <= VERY_LOW_VOLTAGE){
		new_state = VERY_LOW_VOLTAGE_FLAG;
		
	}else if(battery_voltage <= LOW_VOLTAGE){
		new_state = LOW_VOLTAGE_FLAG;
		
	}else if(battery_voltage <= GOOD_VOLTAGE){
		new_state = GOOD_VOLTAGE_FLAG;
		
	}else{
		new_state = MAX_VOLTAGE_FLAG;
	}

	//checks if we are in the new state for at least CHANGE_STATE_TIME_MS time
	if(actual_state != new_state){
		if((time_state + CHANGE_STATE_TIME_MS) < chVTGetSystemTime()){
			//we change the state
			actual_state = new_state;
			//we signal the new state
			chEvtBroadcastFlags(&battery_info_event, actual_state);

			time_state = chVTGetSystemTime();
		}
	}else{
		time_state = chVTGetSystemTime();
	}

	//if the battery voltage is too low for TICK_BATTERY_LOW time
	//we turn OFF the robot
	if(actual_state == MIN_VOLTAGE_FLAG){
		if((time_battery_low + BATTERY_LOW_TIME_MS) < chVTGetSystemTime()){
			//shutdown robot
			powerButtonTurnOnOff(POWER_OFF);
		}
	}else{
		time_battery_low = chVTGetSystemTime();
	}

}

static THD_WORKING_AREA(batt_thd_wa, 256);
static THD_FUNCTION(batt_thd, arg)
{
	(void) arg;

	chRegSetThreadName("Battery measurement");

	adcsample_t adc_measure[ADC_ONE_SAMPLE];	//we do one sample at a time

	float battery_raw = 0;
	float vrefint_raw = 0;

	float battery_value = ADC_RESOLUTION;
	float vref_gain_correction_value = 1;	//100% -> no correction at first

	systime_t time = 0;

	//configures the ADC for the conversion but doesn't start it because the SWSTART is not set
	adcStartConversion(&ADC_BATT, &adcGroupConfig, adc_measure, ADC_ONE_SAMPLE);

	while(1){
		time = chVTGetSystemTime();
		//we compute the battery state only if the robot is turned on
		if(powerButtonGetPowerState() == POWER_ON){

			//starts the conversion process.
			//the adc callback will automatically relaunch the conversions until we have ADC_BATT_NUM_SAMPLES
			//for ADC_CHANNEL_VREFINT and ADC_CHANNEL_BATT
			ADCdoOneConversion(&ADC_BATT);
			//the semaphore is released when all the conversions are finished
			chBSemWait(&measurement_ready);

			//we do the computation stuff
			battery_raw = 0;
			vrefint_raw = 0;
			for(uint16_t i = 0 ; i < (ADC_NUM_CHANNELS * ADC_BATT_NUM_SAMPLES) ; i += ADC_NUM_CHANNELS){
				battery_raw += batt_samples[i];
				vrefint_raw += batt_samples[i+1];
			}

			battery_raw /= ADC_BATT_NUM_SAMPLES;
			vrefint_raw /= ADC_BATT_NUM_SAMPLES;

			//low-pass filter on vref_correction_value
			vref_gain_correction_value = LOW_PASS_COEFF_A * vref_gain_correction_value
									+ LOW_PASS_COEFF_B * (ADC_VALUE_VREFINT/vrefint_raw);

			//low-pass filter on battery_value
			battery_value = LOW_PASS_COEFF_A * battery_value 
							+ LOW_PASS_COEFF_B * battery_raw * vref_gain_correction_value;

			battery_voltage = (battery_value / COEFF_ADC_TO_VOLT) + VOLTAGE_DROP;

			// chprintf((BaseSequentialStream *) &SDU2,"batt_raw =  %f , vref_raw = %f\n",battery_raw, vrefint_raw);
			// chprintf((BaseSequentialStream *) &SDU2,"batt_filt =  %f , bat_volt_filt = %f\n\n",battery_value, battery_voltage);

			batteryStateMachine();
		}

		chThdSleepUntilWindowed(time, time + TIME_MS2I(500));
	}
}

//////////////////////////////////////////PUBLIC FUNCTIONS/////////////////////////////////////////

void batteryMesurementStart(void){
	/*
	* Activates the ADC driver and the Vref sensor.
	*/
	adcStart(&ADC_BATT, NULL);
	adcSTM32DisableVBATE();
	adcSTM32EnableTSVREFE();

	/**
	 * Starts the battery measurement thread
	 */
	chThdCreateStatic(batt_thd_wa, sizeof(batt_thd_wa), NORMALPRIO, batt_thd, NULL);
}