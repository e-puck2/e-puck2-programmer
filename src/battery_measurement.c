/**
 * @file	battery_measurement.c
 * @brief  	Functions to measure the battery voltage and change the battery state accordingly
 * 			Uses the event system to signal a new state
 * 
 * @written by  	Eliot Ferragni, Stefano Morgani
 * @creation date	26.06.2018
 */

#include "main.h"
#include "battery_measurement.h"
#include "power_button.h"

#define ADC_NUM_CHANNELS		2	//Batt and VrefInt
#define ADC_BATT_NUM_SAMPLES	64	//64 samples per channel

#define ADC_ONE_CHANNEL			1
#define ADC_ONE_SAMPLE			1

#define ADC_CHANNEL_BATT		ADC_CHANNEL_IN15
#define ADC_CHANNEL_VREFINT     ADC_CHANNEL_IN0 // The internal reference voltage (VREFINT) is connected to ADC1_INP0/INN0.

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
float battery_raw = 0;
float vrefint_raw = 0;

//Event source used to send events to other threads
event_source_t battery_info_event;

static BSEMAPHORE_DECL(measurement_ready, true);

static void adcCb(ADCDriver *adcp);

//groupe conversion config for the ADC
static const ADCConversionGroup adcGroupConfig =  {
	.circular = false, 	// linear mode
 	.num_channels = 2,	// reference and battery channels
 	.end_cb = adcCb,
	.error_cb = NULL,
	.cfgr = ADC_CFGR_CONT, // Continuous because we want to convert ADC_BATT_NUM_SAMPLES sequences at once
	// Each sampling total time takes about 26 us:
	// - ADC clock div = 8 (see mcuconf.h) => sysclock/8 = 80/8 = 10 MHz
	// - [sampling (260 cycles)] x 1'000'000/10'000'000 + conversion (slow channel) = 26 + 0.238 = 26.238 us
	.smpr =	{
				ADC_SMPR1_SMP_AN0(ADC_SMPR_SMP_247P5), //0,
				ADC_SMPR2_SMP_AN15(ADC_SMPR_SMP_247P5)
			},
	.sqr = {
				ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(ADC_CHANNEL_BATT) | ADC_SQR1_SQ2_N(ADC_CHANNEL_VREFINT),
				0,
				0,
				0
			}
};

/////////////////////////////////////////PRIVATE FUNCTIONS/////////////////////////////////////////

//The callback will be called when ADC_BATT_NUM_SAMPLES sequences are converted
static void adcCb(ADCDriver *adcp) {

	battery_raw = 0;
	vrefint_raw = 0;
	for(uint16_t i = 0 ; i < (ADC_NUM_CHANNELS * ADC_BATT_NUM_SAMPLES) ; i += ADC_NUM_CHANNELS){
		battery_raw += batt_samples[i];
		vrefint_raw += batt_samples[i+1];
	}

	battery_raw /= ADC_BATT_NUM_SAMPLES;
	vrefint_raw /= ADC_BATT_NUM_SAMPLES;

	chSysLockFromISR();
	chBSemSignalI(&measurement_ready);
	chSysUnlockFromISR();
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

	float battery_value = ADC_RESOLUTION;
	float vref_gain_correction_value = 1;	//100% -> no correction at first

	systime_t time = 0;

	adcStartConversion(&ADC_BATT, &adcGroupConfig, batt_samples, ADC_BATT_NUM_SAMPLES);

	while(1){
		time = chVTGetSystemTime();
		//we compute the battery state only if the robot is turned on
		if(powerButtonGetPowerState() == POWER_ON){

			//the semaphore is released when all the conversions are finished
			chBSemWait(&measurement_ready);

			//low-pass filter on vref_correction_value
			vref_gain_correction_value = LOW_PASS_COEFF_A * vref_gain_correction_value
									+ LOW_PASS_COEFF_B * (ADC_VALUE_VREFINT/vrefint_raw);

			//low-pass filter on battery_value
			battery_value = LOW_PASS_COEFF_A * battery_value 
							+ LOW_PASS_COEFF_B * battery_raw * vref_gain_correction_value;

			battery_voltage = (battery_value / COEFF_ADC_TO_VOLT) + VOLTAGE_DROP;

			//chprintf((BaseSequentialStream *) &SDU2,"batt_raw =  %f , vref_raw = %f\r\n",battery_raw, vrefint_raw);
			//chprintf((BaseSequentialStream *) &SDU2,"batt_filt =  %f , bat_volt_filt = %f\r\n\n",battery_value, battery_voltage);

			batteryStateMachine();

			// Restart sampling, callback will be called after ADC_BATT_NUM_SAMPLES sequences are converted
			adcStartConversion(&ADC_BATT, &adcGroupConfig, batt_samples, ADC_BATT_NUM_SAMPLES);
		}

		chThdSleepUntilWindowed(time, time + TIME_MS2I(500));
	}
}

//////////////////////////////////////////PUBLIC FUNCTIONS/////////////////////////////////////////

void batteryMesurementStart(void){
	/*
	* Activates the ADC driver and the Vref sensor.
	*/

	//palSetGroupMode(GPIOB, PAL_PORT_BIT(0), 0, PAL_MODE_INPUT_ANALOG);
	adcStart(&ADC_BATT, NULL);
	adcSTM32EnableVREF(&ADC_BATT); // Internal reference voltage (about 1.2 V) is used to adjust the value of the sampled battery value.
	adcSTM32DisableVBAT(&ADC_BATT);

	/**
	 * Starts the battery measurement thread
	 */
	chThdCreateStatic(batt_thd_wa, sizeof(batt_thd_wa), NORMALPRIO, batt_thd, NULL);
}
