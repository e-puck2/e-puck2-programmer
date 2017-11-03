/**
 * @file    SMBus.h
 * @brief   Functions to configure SMBus host over the right I2C
 *
 * @author  Eliot Ferragni
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <../USB251XB/SMBus.h>
#include <../2.0/platform.h>
#include "timing.h"

////////////// PARAMETERS ////////////////
#define I2C_SMBUS_HOST_MODE     0x000A

#define I2C_CR2_FREQ_48MHZ      0x30

#define SMBUS_TIMOUT_MS         10
//////////////////// PROTOTYPES PRIVATE FUNCTIONS /////////////////////

void i2c_smbus_host_mode(uint32_t i2c);
void SMBus_error(void);

//////////////////// PRIVATE FUNCTIONS /////////////////////////

/**
 * @brief [Set the SMBus host mode]
 *
 * @param i2c [I2C interface to configure]
 */
void i2c_smbus_host_mode(uint32_t i2c)
{
    I2C_CR1(i2c) |= I2C_SMBUS_HOST_MODE;
}

void SMBus_error(void){

    SET_ERROR_STATE(true);
}


//////////////////// PUBLIC FUNCTIONS /////////////////////////
void SMBus_init()
{
    /* Enable clocks for rigth I2C */
#if defined(EPUCK2)
#define I2C_USB_HUB I2C_HUB
    rcc_periph_clock_enable(RCC_I2C2);
#elif defined(DEV_ELIOT)
#define I2C_USB_HUB I2C1
    rcc_periph_clock_enable(RCC_I2C1);
#else
#error Call from not implemented platform !!
#endif

    /* Enable GPIOB clock. */
    rcc_periph_clock_enable(RCC_GPIOB);

    /* Set alternate functions for the SCL and SDA pins of the right I2C. */
#if defined(EPUCK2)
    gpio_set_af(I2C_PORT, I2C_SDA_AF, I2C_SDA_PIN);
    gpio_set_af(I2C_PORT, I2C_SCL_AF, I2C_SCL_PIN);
    gpio_mode_setup(I2C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C_SDA_PIN | I2C_SCL_PIN);
    gpio_set_output_options(I2C_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_50MHZ, I2C_SDA_PIN | I2C_SCL_PIN);
#elif defined(DEV_ELIOT)
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
              GPIO_I2C1_SCL | GPIO_I2C1_SDA);
#else
#error Call from not implemented platform !!
#endif

    /* Disable the I2C before changing any configuration. */
    i2c_peripheral_disable(I2C_USB_HUB);

    //reset of I2C periph to avoid glitch on BUSY flag
    I2C_CR1(I2C_USB_HUB) |= I2C_CR1_SWRST;
    I2C_CR1(I2C_USB_HUB) &= ~I2C_CR1_SWRST;

#if defined(EPUCK2)
    /* APB1 is running at 48MHz. */
    i2c_set_speed(I2C_USB_HUB, i2c_speed_sm_100k, I2C_CR2_FREQ_48MHZ);
#elif defined(DEV_ELIOT)
    /* APB1 is running at 36MHz. */
    i2c_set_clock_frequency(I2C_USB_HUB, I2C_CR2_FREQ_36MHZ);
    /* 100KHz - I2C Standard Mode */
    i2c_set_standard_mode(I2C_USB_HUB);
    /*
     * we want 100kHz for SMBus
     * fclock for I2C is 36MHz APB1 and ccr = fclock/(i2c_speed * 2) = 180 =>0xB4
     */
    i2c_set_ccr(I2C_USB_HUB, 0xB4);
    /*
     * fclock for I2C is 36MHz -> Trise = (48MHz/1000000) + 1 = 37 -> 0x25
     */
    i2c_set_trise(I2C_USB_HUB, 0x25);
#else
#error Call from not implemented platform !!
#endif

    /*SMBus Host mode*/
    i2c_smbus_host_mode(I2C_USB_HUB);

    /* If everything is configured -> enable the peripheral. */
    i2c_peripheral_enable(I2C_USB_HUB);
}

void SMBus_write(uint8_t i2c_addr, uint8_t reg, uint8_t nbDatas, uint8_t* datas)
{
    if(nbDatas > 0){
        uint32_t reg32 __attribute__((unused));

        platform_timeout timeout;

        platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
        while ((I2C_SR2(I2C_USB_HUB) & I2C_SR2_BUSY)){
            if(platform_timeout_is_expired(&timeout)){
                SMBus_error();
                return;
            }
        }

        /* Send START condition. */
        i2c_send_start(I2C_USB_HUB);

        /* Waiting for START is sent and switched to master mode. */
        platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
        while (!((I2C_SR1(I2C_USB_HUB) & I2C_SR1_SB)
            & (I2C_SR2(I2C_USB_HUB) & (I2C_SR2_MSL | I2C_SR2_BUSY)))){
            if(platform_timeout_is_expired(&timeout)){
                SMBus_error();
                return;
            }
        }

        /* Send destination address. */
        i2c_send_7bit_address(I2C_USB_HUB, i2c_addr>>1, I2C_WRITE);

        /* Waiting for address is transferred. */
        platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
        while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_ADDR)){
            if(platform_timeout_is_expired(&timeout)){
                SMBus_error();
                return;
            }
        }

        /* Cleaning ADDR condition sequence. */
        reg32 = I2C_SR2(I2C_USB_HUB);

        i2c_send_data(I2C_USB_HUB, reg); /* register */

        platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
        while (!(I2C_SR1(I2C_USB_HUB) & (I2C_SR1_BTF))){
            if(platform_timeout_is_expired(&timeout)){
                SMBus_error();
                return;
            }
        }

        i2c_send_data(I2C_USB_HUB, nbDatas); /* data count (SMBus specific) */

        platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
        while (!(I2C_SR1(I2C_USB_HUB) & (I2C_SR1_BTF))){
            if(platform_timeout_is_expired(&timeout)){
                SMBus_error();
                return;
            }
        }

        /* Sending the data. */
        do{
            i2c_send_data(I2C_USB_HUB, *datas++);
            platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
            while (!(I2C_SR1(I2C_USB_HUB) & (I2C_SR1_BTF))){
                if(platform_timeout_is_expired(&timeout)){
                    SMBus_error();
                    return;
                }
            }
            nbDatas--;
        }while(nbDatas > 0);

        /* Send STOP condition. */
        i2c_send_stop(I2C_USB_HUB);
    }
}

void SMBus_read(uint8_t i2c_addr, uint8_t reg, uint8_t nbDatas, uint8_t* datas)
{
    if(nbDatas > 0){
        //SMBus specific. the first byte received is the data count
        nbDatas++;

        uint32_t reg32 __attribute__((unused));

        platform_timeout timeout;

        /* Send START condition. */
        i2c_send_start(I2C_USB_HUB);

        /* Waiting for START is send and switched to master mode. */
        platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
        while (!((I2C_SR1(I2C_USB_HUB) & I2C_SR1_SB)
            & (I2C_SR2(I2C_USB_HUB) & (I2C_SR2_MSL | I2C_SR2_BUSY)))){
            if(platform_timeout_is_expired(&timeout)){
                SMBus_error();
                return;
            }
        }

        /* Say to which address we want to talk to. */
        i2c_send_7bit_address(I2C_USB_HUB, i2c_addr>>1, I2C_WRITE);

        /* Waiting for address is transferred. */
        platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
        while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_ADDR)){
            if(platform_timeout_is_expired(&timeout)){
                SMBus_error();
                return;
            }
        }

        /* Cleaning ADDR condition sequence. */
        reg32 = I2C_SR2(I2C_USB_HUB);

        i2c_send_data(I2C_USB_HUB, reg); /* register */

        platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
        while (!(I2C_SR1(I2C_USB_HUB) & (I2C_SR1_BTF))){
            if(platform_timeout_is_expired(&timeout)){
                SMBus_error();
                return;
            }
        }

        /* Send START condition. */
        i2c_send_start(I2C_USB_HUB);

        i2c_enable_ack(I2C_USB_HUB);

        /* Waiting for START is send and switched to master mode. */
        platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
        while (!((I2C_SR1(I2C_USB_HUB) & I2C_SR1_SB)
            & (I2C_SR2(I2C_USB_HUB) & (I2C_SR2_MSL | I2C_SR2_BUSY)))){
            if(platform_timeout_is_expired(&timeout)){
                SMBus_error();
                return;
            }
        }

        /* Say to what address we want to talk to. */
        i2c_send_7bit_address(I2C_USB_HUB, i2c_addr>>1, I2C_READ);

         /* Waiting for address is transferred. */
        platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
        while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_ADDR)){
            if(platform_timeout_is_expired(&timeout)){
                SMBus_error();
                return;
            }
        }
        /* Cleaning ADDR condition sequence. */
        reg32 = I2C_SR2(I2C_USB_HUB);

        uint8_t i = 0;
        for (i = 0; i < nbDatas; ++i) {
            if (i == nbDatas - 1) {
                i2c_disable_ack(I2C_USB_HUB);
            }
            platform_timeout_set(&timeout, SMBUS_TIMOUT_MS);
            while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_RxNE)){
                if(platform_timeout_is_expired(&timeout)){
                    SMBus_error();
                    return;
                }
            }
            if(i == 0){
                reg32 = i2c_get_data(I2C_USB_HUB);
            }else{
                datas[i-1] = i2c_get_data(I2C_USB_HUB);
            }
        }
        i2c_send_stop(I2C_USB_HUB);

    }
}
