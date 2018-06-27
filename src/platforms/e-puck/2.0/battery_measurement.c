
#include "main.h"
#include "battery_measurement.h"

#define ADC_NUM_CHANNELS		2	//Batt and VrefInt
#define ADC_BATT_NUM_SAMPLES	32	//32 samples by channel

#define ADC_CHANNEL_BATT		8

#define VOLTAGE_DIVIDER         (1.0f * RESISTOR_R2 / (RESISTOR_R1 + RESISTOR_R2))

#define VREF                    3.0f //volt corresponds to the voltage on the VREF+ pin
#define ADC_RESOLUTION          4096

#define COEFF_ADC_TO_VOLT       ((1.0f * ADC_RESOLUTION * VOLTAGE_DIVIDER) / VREF) //conversion from adc value to voltage

#define LOW_PASS_COEFF_A		0.8f
#define LOW_PASS_COEFF_B		0.2f

#define ADC_VALUE_VREFINT		1652	//value of the Vrefint (1.2V) measure when the uC is powered with 3V

//buffer filled by the ADC
static adcsample_t batt_samples[ADC_NUM_CHANNELS * ADC_BATT_NUM_SAMPLES];

//battery voltage variable
static float battery_voltage = 0;

//groupe conversion config for the ADC
static const ADCConversionGroup adcGroupConfig =  {
	.circular = false, 
 	.num_channels = ADC_NUM_CHANNELS,
 	.end_cb = NULL,
	.error_cb = NULL,
	.cr1 = 0,
	.cr2 = ADC_CR2_SWSTART,
	.smpr2 = ADC_SMPR2_SMP_AN8(ADC_SAMPLE_480),
	.smpr1 = ADC_SMPR1_SMP_VREF(ADC_SAMPLE_480),
	.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_BATT) | ADC_SQR3_SQ2_N(ADC_CHANNEL_VREFINT),
	.sqr2 = 0,
	.sqr1 = 0,
};

void batteryStateMachine(void){
	//actual state
	static uint8_t actual_state = MIN_VOLTAGE;
	//state in which the battery voltage tells us to be.
	//-> we use it in order to change the actual state only after a fixed amount
	//of time. It prevents quick changes in the state if the measurements oscillate.
	static uint8_t new_state = MIN_VOLTAGE;

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

			//send event here
			chprintf((BaseSequentialStream *) &SDU2,"new state = %d\n\n", actual_state);
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
			chprintf((BaseSequentialStream *) &SDU2,"robot shutdown !\n\n");
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

	float battery_raw = 0;
	float vrefint_raw = 0;

	float battery_value = 0;
	float vref_gain_correction = 0;

	while(1){
		//do one conversion and wait for it to finish -> no need for a callback
		adcConvert(&ADC_BATT, &adcGroupConfig, batt_samples, ADC_BATT_NUM_SAMPLES);
		
		battery_raw = 0;
		vrefint_raw = 0;
		for(uint16_t i = 0 ; i < (ADC_NUM_CHANNELS * ADC_BATT_NUM_SAMPLES) ; i += ADC_NUM_CHANNELS){
			battery_raw += batt_samples[i];
			vrefint_raw += batt_samples[i+1];
		}

		battery_raw /= ADC_BATT_NUM_SAMPLES;
		vrefint_raw /= ADC_BATT_NUM_SAMPLES;

		vref_gain_correction = ADC_VALUE_VREFINT/vrefint_raw;

		//low-pass filter
		battery_value = LOW_PASS_COEFF_A * battery_value 
						+ LOW_PASS_COEFF_B * battery_raw * vref_gain_correction;

		if(battery_value){
			battery_voltage = battery_value / COEFF_ADC_TO_VOLT;
		}


		chprintf((BaseSequentialStream *) &SDU2,"batt_raw =  %f , vref_raw = %f\n",battery_raw, vrefint_raw);
		chprintf((BaseSequentialStream *) &SDU2,"batt_filt =  %f , bat_volt_filt = %f\n\n",battery_value, battery_voltage);

		batteryStateMachine();

		chThdSleepMilliseconds(500);
	}
}


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