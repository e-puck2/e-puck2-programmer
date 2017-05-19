/*
Exemples 
https://github.com/libopencm3/libopencm3-examples/tree/master/examples/stm32/f1/other/i2c_stts75_sensor
http://www.longlandclan.yi.org/~stuartl/stm32f10x_stdperiph_lib_um/stm32f10x__i2c_8c_source.html
https://github.com/Selat/stm32-smbus
*/

/**
 * @file    SMBus.h
 * @brief   Functions to configure SMBus host over I2C1
 * 
 * @author  Eliot Ferragni
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <USB2513B/SMBus.h>

////////////// PARAMETERS ////////////////
#define I2C_SMBUS_HOST_MODE     0x000A

//////////////////// PROTOTYPES PRIVATE FUNCTIONS /////////////////////

void i2c_smbus_host_mode(uint32_t i2c);

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


//////////////////// PUBLIC FUNCTIONS /////////////////////////
void SMBus_init()
{
    /* Enable clocks for I2C1 */
    rcc_periph_clock_enable(RCC_I2C1);

    /* Enable GPIOB clock. */
    rcc_periph_clock_enable(RCC_GPIOB);


    /* Set alternate functions for the SCL and SDA pins of I2C1. */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
              GPIO_I2C1_SCL | GPIO_I2C1_SDA);

    /* Disable the I2C before changing any configuration. */
    i2c_peripheral_disable(I2C1);

    /* APB1 is running at 36MHz. */
    i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);

    /* 100KHz - I2C Standard Mode */
    i2c_set_standard_mode(I2C1);

    /*SMBus Host mode*/
    i2c_smbus_host_mode(I2C1);

    /*
     * fclock for I2C is 36MHz APB1 and ccr = fclock/(i2c_speed * 2) = 180 =>0xB4
     */
    i2c_set_ccr(I2C2, 0xB4);

    /*
     * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
     * 400kHz => 300ns and 100kHz => 1000ns; 1000ns/28ns = 27.7=>27;
     * Incremented by 1 -> 28 ->.
     */
    i2c_set_trise(I2C2, 0x1C);

    /* If everything is configured -> enable the peripheral. */
    i2c_peripheral_enable(I2C1);
}

void SMBus_write(uint8_t i2c_addr, uint8_t reg, uint8_t nbDatas, uint8_t* datas)
{
    if(nbDatas > 0){
        uint32_t reg32 __attribute__((unused));

        /* Send START condition. */
        i2c_send_start(I2C1);

        /* Waiting for START is send and switched to master mode. */
        while (!((I2C_SR1(I2C1) & I2C_SR1_SB)
            & (I2C_SR2(I2C1) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

        /* Send destination address. */
        i2c_send_7bit_address(I2C1, i2c_addr>>1, I2C_WRITE);

        /* Waiting for address is transferred. */
        while (!(I2C_SR1(I2C1) & I2C_SR1_ADDR));

        /* Cleaning ADDR condition sequence. */
        reg32 = I2C_SR2(I2C1);

        i2c_send_data(I2C1, reg); /* register */
        while (!(I2C_SR1(I2C1) & (I2C_SR1_BTF | I2C_SR1_TxE)));

        /* Sending the data. */
        do{
            i2c_send_data(I2C1, *datas++); 
            while (!(I2C_SR1(I2C1) & I2C_SR1_BTF)); /* Await ByteTransferedFlag. */
            nbDatas--;
        }while(nbDatas > 0);
        
        /* Send STOP condition. */
        i2c_send_stop(I2C1);
    }
}

void SMBus_read(uint8_t i2c_addr, uint8_t reg, uint8_t nbDatas, uint8_t* datas)
{
    if(nbDatas > 0){
        uint32_t reg32 __attribute__((unused));

        /* Send START condition. */
        i2c_send_start(I2C1);

        /* Waiting for START is send and switched to master mode. */
        while (!((I2C_SR1(I2C1) & I2C_SR1_SB)
            & (I2C_SR2(I2C1) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

        /* Say to which address we want to talk to. */
        i2c_send_7bit_address(I2C1, i2c_addr>>1, I2C_WRITE);

        /* Waiting for address is transferred. */
        while (!(I2C_SR1(I2C1) & I2C_SR1_ADDR));

        /* Cleaning ADDR condition sequence. */
        reg32 = I2C_SR2(I2C1);

        i2c_send_data(I2C1, reg); /* register */
        while (!(I2C_SR1(I2C1) & (I2C_SR1_BTF | I2C_SR1_TxE)));

        /* Send START condition. */
        i2c_send_start(I2C1);

        /* Waiting for START is send and switched to master mode. */
        while (!((I2C_SR1(I2C1) & I2C_SR1_SB)
            & (I2C_SR2(I2C1) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

        /* Say to what address we want to talk to. */
        i2c_send_7bit_address(I2C1, i2c_addr>>1, I2C_READ); 

        /* Waiting for address is transferred. */
        while (!(I2C_SR1(I2C1) & I2C_SR1_ADDR));

        /* Cleaning ADDR condition sequence. */
        reg32 = I2C_SR2(I2C1);

        //activate ack
        i2c_enable_ack(I2C1);

        //read until the N-1 byte
        while(nbDatas > 1)
        {
             /* Now the slave should begin to send us the first byte. Await BTF. */
            while (!(I2C_SR1(I2C1) & I2C_SR1_BTF));
            *datas++ = i2c_get_data(I2C1);
            nbDatas--;
        }

        //Deactivate ack
        i2c_disable_ack(I2C1);
        /* Send STOP condition. */
        i2c_send_stop(I2C1);
        //Read the last Byte
        *datas = i2c_get_data(I2C1);
    }
}
