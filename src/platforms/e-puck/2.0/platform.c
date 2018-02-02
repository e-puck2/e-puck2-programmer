/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"
#include "morse.h"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/otg_fs.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include "gdb_packet.h"

#include <../USB251XB/USB251XB.h>
#include <../USB251XB/SMBus.h>

const struct rcc_clock_scale hse_24mhz_to_96mhz_413_epuck = {
	 /* 96MHz */
	.pllm = 12,
	.plln = 96,
	.pllp = 2,
	.pllq = 4,
	.pllr = 0,
	.hpre = RCC_CFGR_HPRE_DIV_NONE,
	.ppre1 = RCC_CFGR_PPRE_DIV_2,
	.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
	//power_save = 1 if ahb_frequency <= 84MHz
	//cf. 5.4.1 PWR power control register (PWR_CR) of the
	//stm32f413 reference manual
	.power_save = 0,
	.flash_config = FLASH_ACR_ICE_COPY | FLASH_ACR_DCE_COPY |
			FLASH_ACR_LATENCY_3WS_COPY,
	.ahb_frequency = 96000000,
	.apb1_frequency = 48000000,
	.apb2_frequency = 96000000,
};

//#pragma message "Platform options : " PLATFORM_OPTIONS

jmp_buf fatal_error_jmpbuf;
extern uint32_t _ebss;
//from .ld file
extern uint32_t _config_start; //sector 15
extern uint32_t _config_end;

//flash variables
static uint32_t config_start = (uint32_t)&_config_start;
static uint32_t config_end = (uint32_t)&_config_end;
static uint32_t config_addr;

//second serial over USB port variables
static uint8_t monitor_mode;
static uint8_t default_mode = 2;
//CAN has priority over uart. So if canUsed is set to true, the usbuart is disabled
uint32_t uartUsed = USBUSART_407;
bool canUsed = false; 

//Button variables
static uint16_t pwrBtnCounter = 0;
static uint8_t pwrBtnState = ROBOT_OFF;

//USB Hub variable
static uint8_t hub_state = NOT_CONFIGURED;

//Battery variables
static float vref_gain_correction = 1;
static uint16_t battery_value = MAX_VOLTAGE * COEFF_ADC_TO_VOLT;
static float battery_voltage = MAX_VOLTAGE;
static uint16_t adc_values[DMA_SIZE_ADC];
static uint16_t nb_adc_values = 0;
static uint16_t battery_low = 0;

void adc_battery_level_stop(void);
void adc_battery_level_init(void);

void PWR_ON_BTN_TIM_ISR(void) {
	/* need to clear timer update event */
	timer_clear_flag(PWR_ON_BTN_TIM, TIM_SR_UIF);
	pwrBtnCounter++;
	if((pwrBtnState==ROBOT_OFF) && (pwrBtnCounter>=TURN_ON_TIME)) {
		timer_disable_counter(PWR_ON_BTN_TIM);
		platform_pwr_on(true);
		gpio_clear(LED_PORT, LED_IDLE_RUN);
	}
	if((pwrBtnState==ROBOT_ON) && (pwrBtnCounter>=TURN_OFF_TIME)) {
		timer_disable_counter(PWR_ON_BTN_TIM);
		platform_pwr_on(false);
		gpio_set(LED_PORT, LED_IDLE_RUN);
	}
//	gpio_toggle(LED_PORT_ERROR, LED_ERROR);
}

void PWR_ON_BTN_EXTI_ISR(void) {
	if(exti_get_flag_status(PWR_ON_BTN_EXTI)) {
		exti_reset_request(PWR_ON_BTN_EXTI);
		if(!platform_pwr_on_btn_pressed()) { // Button released.
			timer_disable_counter(PWR_ON_BTN_TIM);
//			gpio_set(LED_PORT_UART, LED_UART);
		} else { // Button pressed.
			pwrBtnCounter = 0;
			timer_enable_counter(PWR_ON_BTN_TIM);
//			gpio_clear(LED_PORT_UART, LED_UART);
		}
	}
}

void VBUS_EXTI_ISR(void) {
	if(exti_get_flag_status(VBUS_EXTI)) {
		//wait a few moments to be sure the interruption was not triggered
		//by a glitch and then test the GPIO
		//also wait for the USB HUB to be running
		platform_delay(100);
		if(platform_get_vbus()){
			if(hub_state == NOT_CONFIGURED){
				// //reinit SMBus because of BUSY flag being kept high
				// //when certain glitches appear on the I2C bus.
				SMBus_init();
				USB251XB_init(USB2512B);
				hub_state = CONFIGURED;
			}
		}else{
			hub_state = NOT_CONFIGURED;
		}
		exti_reset_request(VBUS_EXTI);
	 }
}

//reads the battery voltage and manages the LED to indicates the battery level
//when the dma buffer is full
void DMA_ADC_ISR(void){

	bool teif = dma_get_interrupt_flag(DMA_ADC, DMA_ADC_STREAM, DMA_TEIF);
	bool dmeif = dma_get_interrupt_flag(DMA_ADC, DMA_ADC_STREAM, DMA_DMEIF);
	dma_clear_interrupt_flags(DMA_ADC, DMA_ADC_STREAM, DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_DMEIF);
	//handles error
	if( teif || dmeif){
		adc_battery_level_stop();
		adc_battery_level_init();
		return;
	}

	static uint32_t temp_adc = 0;
	static uint16_t i = 0;
	static uint8_t future_state = MAX_VOLTAGE_STATE;
	static uint8_t actual_state = MAX_VOLTAGE_STATE;
	static uint16_t change_state = 0;

	//we only update the battery values if the robot is turned ON
	//otherwise we would measure 0V...
	if(pwrBtnState == ROBOT_ON){

		//average of the Vrefint measurements
		//index 0,2,4,...
		temp_adc = 0;
	    for (i = 0 ; i < DMA_SIZE_ADC ; i += NB_ADC_CHANNEL) {
	    	temp_adc += adc_values[i];
	    }
	    if(temp_adc){
	    	temp_adc /= (DMA_SIZE_ADC/NB_ADC_CHANNEL);
	    }

	    vref_gain_correction = (float)ADC_VALUE_VREFINT/temp_adc;

		//average of the battery measurements
		//index 1,3,5,...
		temp_adc = 0;
	    for (i = 1 ; i < DMA_SIZE_ADC ; i += NB_ADC_CHANNEL) {
	    	temp_adc += adc_values[i];
	    }
	    if(temp_adc){
	    	temp_adc /= (DMA_SIZE_ADC/NB_ADC_CHANNEL);
	    }

	    //low-pass filter
		battery_value = LOW_PASS_COEFF_A * battery_value 
						+ LOW_PASS_COEFF_B * temp_adc * vref_gain_correction;
		if(battery_value){
			battery_voltage = battery_value / COEFF_ADC_TO_VOLT;
		}

		/*DEBUG
		* print the adc value and the calculated voltage to Bluetooth and USB
		* the programmer mode must be 1 to be able to receive the debug info
		*/
		// char msg[20] = {0};
		// uint16_t val = (uint16_t)(battery_voltage);
		// uint16_t dec = (uint16_t)((battery_voltage - val)*100);
		// //sprintf(msg,"batt = %d, %d.%d\n",battery_value, val, dec);
		// sprintf(msg,"%d.%d;\n",val, dec);
		// //to USB UART port
		// usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT, msg, 20);

		// //to bluetooth GDB port
		// for(int i = 0; i < 20; i++)
		// 	usart_send_blocking(USBUSART_ESP, msg[i]);

		/*END DEBUG*/

		//check if we are in the new state for at least TICK_CHANGE_STATE time
		if(actual_state != future_state){
			change_state++;
			if(change_state >= TICK_CHANGE_STATE){
				actual_state = future_state;
				change_state = 0;
			}
		}else{
			change_state = 0;
		}

		//RED led toggle + shutdown after TICK_BATTERY_LOW time
		if(battery_voltage <= MIN_VOLTAGE){
			future_state = MIN_VOLTAGE_STATE;

			//we count the battery_low only when we are sure 
			//that we are not in a oscillation state between 
			//VERY_LOW_VOLTAGE and LOW_VOLTAGE
			if(actual_state == MIN_VOLTAGE_STATE){
				battery_low++;

				gpio_toggle(LED_PORT_ERROR,LED_ERROR);
				gpio_set(LED_PORT,LED_IDLE_RUN);
			}
		//RED led toggle
		}else if(battery_voltage <= VERY_LOW_VOLTAGE){
			future_state = VERY_LOW_VOLTAGE_STATE;
			if(actual_state == VERY_LOW_VOLTAGE_STATE){
				battery_low = 0;
				gpio_toggle(LED_PORT_ERROR,LED_ERROR);
				gpio_set(LED_PORT,LED_IDLE_RUN);
			}
		//RED turned ON
		}else if(battery_voltage <= LOW_VOLTAGE){
			future_state = LOW_VOLTAGE_STATE;
			if(actual_state == LOW_VOLTAGE_STATE){
				battery_low = 0;
				gpio_clear(LED_PORT_ERROR,LED_ERROR);
				gpio_set(LED_PORT,LED_IDLE_RUN);
			}
		//RED and GREEN turned ON
		}else if(battery_voltage <= GOOD_VOLTAGE){
			future_state = GOOD_VOLTAGE_STATE;
			if(actual_state == GOOD_VOLTAGE_STATE){
				battery_low = 0;
				gpio_clear(LED_PORT_ERROR,LED_ERROR);
				gpio_clear(LED_PORT,LED_IDLE_RUN);
			}
		//GREEN turned ON
		}else{
			future_state = MAX_VOLTAGE_STATE;
			if(actual_state == MAX_VOLTAGE_STATE){
				battery_low = 0;
				gpio_set(LED_PORT_ERROR,LED_ERROR);
				gpio_clear(LED_PORT,LED_IDLE_RUN);
			}
		}

		//if the battery voltage is too low for TICK_BATTERY_LOW time
		//we turn OFF the robot
		if(battery_low >= TICK_BATTERY_LOW){
			platform_pwr_on(false);
		}
	}
}

//restart the ADC aquisition processus at a certain interval
void TIM_ADC_ISR(void){
	timer_clear_flag(TIM_ADC, TIM_SR_UIF);

	nb_adc_values = 0;

	// //start the ADC sampling
	nb_adc_values += NB_ADC_CHANNEL;
	adc_start_conversion_regular(ADC_USED);
}

//after each conversion of the ADC, redo one aquisition
//if the dma buffer isn't full
void ADC_USED_ISR(void){

	//apparently taking several single shots is more accurate 
	//than the continuous mode with the ADC
	if(nb_adc_values < DMA_SIZE_ADC){
		nb_adc_values += NB_ADC_CHANNEL;
		adc_start_conversion_regular(ADC_USED);
	}
}

void setup_vbus_detection(void){

	gpio_mode_setup(VBUS_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, VBUS_PIN);

	// Configure the EXTI subsystem.
	exti_select_source(VBUS_EXTI, VBUS_PORT);
	exti_set_trigger(VBUS_EXTI, EXTI_TRIGGER_BOTH);
	exti_enable_request(VBUS_EXTI);

	exti_reset_request(VBUS_EXTI);
	nvic_set_priority(VBUS_EXTI_IRQ, VBUS_EXTI_ISR_PRI);
	// Enable EXTI interrupt.
	nvic_enable_irq(VBUS_EXTI_IRQ);

	//test if vbus is present on power on to configure the hub
	//because the interruption can not be triggered on power on
	if(platform_get_vbus()){
		if(hub_state == NOT_CONFIGURED){
			//wait for the USB HUB to be running
			platform_delay(100);
			// //reinit SMBus because of BUSY flag being kept high
			// //when certain glitches appear on the I2C bus.
			SMBus_init();
			USB251XB_init(USB2512B);
			hub_state = CONFIGURED;
		}
	}
}

void setup_pwr_button() {
	// Enable EXTI9_5 interrupt (power button connected to PA7).
	nvic_enable_irq(PWR_ON_BTN_EXTI_IRQ);

	gpio_mode_setup(PWR_ON_BTN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PWR_ON_BTN_PIN);

	// Configure the EXTI subsystem.
	exti_select_source(PWR_ON_BTN_EXTI, PWR_ON_BTN_PORT);
	exti_set_trigger(PWR_ON_BTN_EXTI, EXTI_TRIGGER_BOTH);
	exti_enable_request(PWR_ON_BTN_EXTI);

	// Configure the timer to count how much time the button is pressed.
	// Enable TIM2 clock.
	rcc_periph_clock_enable(RCC_TIM2);

	// Reset TIM2 peripheral to defaults.
	timer_reset(PWR_ON_BTN_TIM);

	// Timer mode: internal clock source, no divider, alignment edge, direction up.
	timer_set_mode(PWR_ON_BTN_TIM, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	// Timer2 clock source is APB1 x 2. Set the prescaler to have the timer run at 1 KHz.
	timer_set_prescaler(PWR_ON_BTN_TIM, ((rcc_apb1_frequency * 2) / 1000)-1);

	// An interrupt every 10 ms.
	timer_set_period(PWR_ON_BTN_TIM, 10);

	// Enable TIM2 interrupt.
	nvic_enable_irq(NVIC_TIM2_IRQ);
	timer_enable_irq(PWR_ON_BTN_TIM, TIM_DIER_UIE);

	// Needed if no USB connexion : Test if PowerOn button is just pressed then power up the system
  if(platform_pwr_on_btn_pressed()) {
		pwrBtnCounter = 0;
		timer_enable_counter(PWR_ON_BTN_TIM);
	}

}

void adc_battery_level_init(void){

/////////////////////////////DMA/////////////////////////////////
	 /* Allocate DMA stream. */
	rcc_periph_clock_enable(RCC_DMA_ADC);
	rcc_periph_reset_pulse(RST_DMA_ADC);

	dma_stream_reset(DMA_ADC, DMA_ADC_STREAM);
	dma_disable_stream(DMA_ADC, DMA_ADC_STREAM);
	dma_set_priority(DMA_ADC, DMA_ADC_STREAM, DMA_SxCR_PL_VERY_HIGH);

	/* Configure DMA stream. */
	dma_set_peripheral_address(DMA_ADC, DMA_ADC_STREAM, (uint32_t)&ADC_DR(ADC_USED)); 
	dma_set_memory_address(DMA_ADC, DMA_ADC_STREAM, (uint32_t)adc_values);
	dma_set_number_of_data(DMA_ADC, DMA_ADC_STREAM, DMA_SIZE_ADC);

	/* set mode */
	dma_set_transfer_mode(DMA_ADC, DMA_ADC_STREAM, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	dma_set_memory_size(DMA_ADC, DMA_ADC_STREAM, DMA_SxCR_MSIZE_16BIT);
	dma_set_peripheral_size(DMA_ADC, DMA_ADC_STREAM, 
	                        DMA_SxCR_PSIZE_16BIT);
	dma_enable_memory_increment_mode(DMA_ADC, DMA_ADC_STREAM);
	dma_enable_circular_mode(DMA_ADC, DMA_ADC_STREAM);
	dma_clear_interrupt_flags(DMA_ADC, DMA_ADC_STREAM, DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_DMEIF);
	dma_enable_transfer_complete_interrupt(DMA_ADC, DMA_ADC_STREAM);
	dma_enable_direct_mode_error_interrupt(DMA_ADC, DMA_ADC_STREAM);
	dma_enable_transfer_error_interrupt(DMA_ADC, DMA_ADC_STREAM);
	dma_channel_select(DMA_ADC, DMA_ADC_STREAM, DMA_ADC_CHANNEL);


	nvic_set_priority(NVIC_DMA_ADC_IRQ, IRQ_DMA_ADC_PRI);
	nvic_enable_irq(NVIC_DMA_ADC_IRQ);
	dma_enable_stream(DMA_ADC, DMA_ADC_STREAM);

///////////////////////////////ADC///////////////////////////////
	/* Because of an error on the design of the alimentation
	 * of the uC, there is a voltage drop of about 0.4V between
	 * the battery and the alimentation of the uC.
	 * => Below 3.4V (battery), the Vref of the ADC goes below 3V
	 * => We can't know the real battery tension
	 * 
	 *  But we can measure the Vrefint (1.2V).
	 *  => If this measure changes, it means the Vref has drifted and
	 *  knowing the Vrefint stays at 1.2V, we can correct the measure of the battery :-)
	 *  
	 *  The sequence for the ADC measurements is Vrefint, then the battery.
	 *  Everything is stocked in the same DMA buffer
	 */ 
	rcc_periph_clock_enable(RCC_ADC_USED);
	rcc_periph_reset_pulse(RST_ADC_USED);
    adc_power_off(ADC_USED);

    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);
    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
    adc_enable_scan_mode(ADC_USED);
    adc_enable_temperature_sensor(); //enables the temperature sensor channel AND the Vrefint channel
    adc_set_sample_time(ADC_USED, ADC_CHANNEL_BATT, ADC_SMPR_SMP_480CYC);
    adc_set_sample_time(ADC_USED, ADC_CHANNEL_VREFINT, ADC_SMPR_SMP_480CYC);
    adc_disable_external_trigger_regular(ADC_USED);
    adc_set_right_aligned(ADC_USED);
    adc_set_resolution(ADC_USED, ADC_CR1_RES_12BIT);

    adc_enable_eoc_interrupt(ADC_USED);
    adc_clear_overrun_flag(ADC_USED);

    adc_set_single_conversion_mode(ADC_USED);

    //First we measure Vrefint, then the battery
   	uint8_t channel[NB_ADC_CHANNEL] = {ADC_CHANNEL_VREFINT, ADC_CHANNEL_BATT};
   	adc_set_regular_sequence(ADC_USED, NB_ADC_CHANNEL, channel);

   	adc_enable_dma(ADC_USED);
   	adc_set_dma_continue(ADC_USED);

   	nvic_set_priority(NVIC_ADC_USED_IRQ, IRQ_ADC_USED_PRI);
	nvic_enable_irq(NVIC_ADC_USED_IRQ);

    adc_power_on(ADC_USED);


////////////////////////////TIMER////////////////////////////////
    // Configure the timer to count how much time the button is pressed.
	// Enable TIM_ADC clock.
	rcc_periph_clock_enable(RCC_TIM_ADC);

	// Reset TIM_ADC peripheral to defaults.
	timer_reset(TIM_ADC);

	// Timer mode: internal clock source, no divider, alignment edge, direction up.
	timer_set_mode(TIM_ADC, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	//the timer run continuously
	timer_continuous_mode(TIM_ADC);

	// Timer5 clock source is APB1 x 2. Set the prescaler to have the timer run at 2 KHz.
	timer_set_prescaler(TIM_ADC, ((rcc_apb1_frequency * 2) / 2000)-1);

	// An interrupt every 500 ms.
	timer_set_period(TIM_ADC, 1000);

	// Enable TIM_ADC interrupt.
	timer_enable_irq(TIM_ADC, TIM_DIER_UIE);

	nvic_set_priority(NVIC_TIM_ADC_IRQ, IRQ_TIM_ADC_PRI);
	nvic_enable_irq(NVIC_TIM_ADC_IRQ);
	
	//start the timer
	timer_enable_counter(TIM_ADC);

}

void adc_battery_level_stop(void){
	timer_disable_counter(TIM_ADC);
	adc_power_off(ADC_USED);
	dma_disable_stream(DMA_ADC, DMA_ADC_STREAM);
}

uint8_t find_last_monitor_choice_flash(void){
	uint32_t* block = (uint32_t*)config_start;
	uint32_t* last = NULL;
	uint8_t choice = default_mode; //if nothing found on the flash, then the default choice is used

	uint32_t i = 0;

	//checks the the flash to find variables with the pattern.
	//continues until it founds the last pattern written.
	while(((uint32_t)(block + i) < config_end) && ((*(block + i) & 0xFFFFFFFC) == PATTERN_FLASH)){
		last = block + i;
		//we take only the 2 last bits
		choice = (*(block + i) & 0x03);
		i++; //32bits increment
	}

	if(last == NULL){
		config_addr = 0;
	}else{
		//sets the config_addr to the next writtable address
		config_addr = (uint32_t)last + sizeof(uint32_t);
	}

	return choice;
}

void write_monitor_choice_to_flash(uint8_t choice){
	flash_unlock();

	//erases the flash if we are at the end or if we found nothing on it
	if((config_addr == 0) || (config_addr >= config_end)){
		flash_erase_sector(CONFIG_SECTOR, FLASH_CR_PROGRAM_X32);
		config_addr = config_start;
	}

	//writes the choice and the pattern on the flash 
	flash_program_word(config_addr, PATTERN_FLASH | (uint32_t)choice);

	//increment for the next write
	config_addr += sizeof(uint32_t);

	flash_lock();
}

void platform_init(void)
{
	rcc_osc_bypass_enable(RCC_HSE);
	//rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_48MHZ]);
	rcc_clock_setup_hse_3v3(&hse_24mhz_to_96mhz_413_epuck);

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_CRC);
	rcc_periph_clock_enable(RCC_SYSCFG);

	gpio_set(LED_PORT_ERROR, LED_ERROR);
	gpio_mode_setup(LED_PORT_ERROR, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_ERROR);

	gpio_set(LED_PORT_UART, LED_UART);
	gpio_mode_setup(LED_PORT_UART, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_UART);

	gpio_set(LED_PORT, LED_IDLE_RUN);
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_IDLE_RUN);

	gpio_clear(PWR_ON_PORT, PWR_ON_PIN);
	gpio_set_output_options(PWR_ON_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_2MHZ,PWR_ON_PIN);
	gpio_mode_setup(PWR_ON_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_ON_PIN);
	setup_pwr_button();

	/* Set up USB Pins and alternate function*/
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	// Configure the programming pins to "fast speed".
	GPIOA_OSPEEDR &= ~0x00030C00; // Reset PA5 (SWCLK), PA8 (SWDIO).
	GPIOA_OSPEEDR |= 0x00020800; // PA5 (SWCLK), PA8 (SWDIO) to fast speed.
	gpio_mode_setup(JTAG_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,  SWDIO_PIN | SWCLK_PIN);

/* Can be needed for TRACESWO but not yet. */
	//gpio_mode_setup(TDO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TDO_PIN);

	gpio_set(SRST_PORT, SRST_PIN);
	gpio_set_output_options(SRST_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_50MHZ,SRST_PIN);
	gpio_mode_setup(SRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SRST_PIN);

	gpio_set(USB_CHARGE_PORT, USB_CHARGE_PIN);
	gpio_set_output_options(USB_CHARGE_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_2MHZ,USB_CHARGE_PIN);
	gpio_mode_setup(USB_CHARGE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, USB_CHARGE_PIN);

	gpio_set(USB_500_PORT, USB_500_PIN);
	gpio_set_output_options(USB_500_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_2MHZ,USB_500_PIN);
	gpio_mode_setup(USB_500_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, USB_500_PIN);

	gpio_set(EN_ESP32_PORT, EN_ESP32_PIN);
	gpio_set_output_options(EN_ESP32_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_2MHZ,EN_ESP32_PIN);
	gpio_mode_setup(EN_ESP32_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EN_ESP32_PIN);

	gpio_set(GPIO0_ESP32_PORT, GPIO0_ESP32_PIN);
	gpio_set_output_options(GPIO0_ESP32_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_2MHZ,GPIO0_ESP32_PIN);
	gpio_mode_setup(GPIO0_ESP32_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO0_ESP32_PIN);

	//ADC
	gpio_mode_setup(ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, ADC_PIN);

	platform_timing_init();
#ifndef PLATFORM_HAS_NO_SERIAL
	usbuart_init();
#endif
	usbcan_init();
	cdcacm_init();
	
	setup_vbus_detection();

	//load the selected mode for the second serial over USB port
	monitor_mode = find_last_monitor_choice_flash();
	platform_switch_monitor_to(monitor_mode);

}

void platform_set_idle_state(uint8_t state){
	gpio_set_val(LED_PORT, LED_IDLE_RUN, !(state==1 && (pwrBtnState==ROBOT_ON)));
}

void platform_switch_monitor_to(uint8_t choice){
	if(choice == 1){//mode 1 : serial monitor with 407 and gdb over bluetooth
		//disable CAN ASEBA
		can_disable_irq(CAN_USED, CAN_IER_FMPIE0);
	
		canUsed = false;
		uartUsed = USBUSART_407;
		monitor_mode = choice;

		write_monitor_choice_to_flash(choice);

		//enable UART 407
		usart_enable(USBUSART_407);
		nvic_enable_irq(USBUSART_407_IRQ);

	}else if(choice == 2){//mode 2 : serial monitor with ESP
		//disable UART 407
		usart_disable(USBUSART_407);
		nvic_disable_irq(USBUSART_407_IRQ);

		//disable CAN ASEBA
		can_disable_irq(CAN_USED, CAN_IER_FMPIE0);

		canUsed = false;
		uartUsed = USBUSART_ESP;
		monitor_mode = choice;

		write_monitor_choice_to_flash(choice);

	}else if(choice == 3){//mode 3 : ASEBA USB-CAN translator and gdb over bluetooth
		//disable UART 407
		usart_disable(USBUSART_407);
		nvic_disable_irq(USBUSART_407_IRQ);

		canUsed = true;
		uartUsed = USBUSART_407;
		monitor_mode = choice;
		write_monitor_choice_to_flash(choice);

		//enable CAN ASEBA
		can_enable_irq(CAN_USED, CAN_IER_FMPIE0);
	}
}

uint8_t platform_get_monitor_mode(void){
	return monitor_mode;
}

void platform_srst_set_val(bool assert)
{
	volatile int i;
	if (assert) {
		gpio_clear(SRST_PORT, SRST_PIN);
		for(i = 0; i < 10000; i++) asm("nop");
	} else {
		gpio_set(SRST_PORT, SRST_PIN);
	}
}

bool platform_srst_get_val(void)
{
	return gpio_get(SRST_PORT, SRST_PIN) == 0;
}

void platform_set_en_esp32(bool assert)
{
	if(uartUsed == USBUSART_ESP){
		if (assert)
			gpio_set(EN_ESP32_PORT, EN_ESP32_PIN);
		else
			gpio_clear(EN_ESP32_PORT, EN_ESP32_PIN);
	}else{
		gdb_out("Must be in mode 2 to perform this action\n");
	}
}

bool platform_get_en_esp32(void)
{
	return gpio_get(EN_ESP32_PORT, EN_ESP32_PIN) != 0;
}

void platform_set_gpio0_esp32(bool assert)
{
	if(uartUsed == USBUSART_ESP){
		if (assert)
			gpio_set(GPIO0_ESP32_PORT, GPIO0_ESP32_PIN);
		else
			gpio_clear(GPIO0_ESP32_PORT, GPIO0_ESP32_PIN);
	}else{
		gdb_out("Must be in mode 2 to perform this action\n");
	}
	
}

bool platform_get_gpio0_esp32(void)
{
	return gpio_get(GPIO0_ESP32_PORT, GPIO0_ESP32_PIN) != 0;
}

void platform_pwr_on(bool on_state)
{
	if (on_state){

		pwrBtnState = ROBOT_ON;
		adc_battery_level_init();
		gpio_set(PWR_ON_PORT, PWR_ON_PIN);

	}else{

		pwrBtnState = ROBOT_OFF;
		adc_battery_level_stop();

		battery_low = 0;

		//assign the default values for the battery levels
		battery_value = MAX_VOLTAGE * COEFF_ADC_TO_VOLT;
		battery_voltage = MAX_VOLTAGE;
		gpio_clear(PWR_ON_PORT, PWR_ON_PIN);
	}
}

bool platform_pwr_on_btn_pressed(void)
{
	/* Return true if button pressed else false. */
	return gpio_get(PWR_ON_PORT, PWR_ON_BTN_PIN) == 0;
}

bool platform_vbus_hub(void)
{
	return gpio_get(VBUS_HUB_PORT, VBUS_HUB_PIN) != 0;
}

bool platform_get_vbus(void)
{
	return gpio_get(VBUS_PORT, VBUS_PIN) != 0;
}

void platform_set_usb_charge(bool assert)
{
	if (assert)
		gpio_set(USB_CHARGE_PORT, USB_CHARGE_PIN);
	else
		gpio_clear(USB_CHARGE_PORT, USB_CHARGE_PIN);
}

bool platform_get_usb_charge(void)
{
	return gpio_get(USB_CHARGE_PORT, USB_CHARGE_PIN) != 0;
}

void platform_set_usb_500(bool assert)
{
	if (assert)
		gpio_set(USB_500_PORT, USB_500_PIN);
	else
		gpio_clear(USB_500_PORT, USB_500_PIN);
}

bool platform_get_usb_500(void)
{
	return gpio_get(USB_500_PORT, USB_500_PIN) != 0;
}

const char *platform_target_voltage(void)
{
	return "ABSENT!";
}
