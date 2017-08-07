/**
 * @file    SMBus.h
 * @brief   Functions to configure SMBus host over the right I2C
 *
 * @author  Eliot Ferragni
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <../USB251XB/SMBus.h>

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
    /* Enable clocks for rigth I2C */
#if defined(EPUCK2)
#define I2C_USB_HUB I2C2
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
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3 | GPIO10);
    gpio_set_af(GPIOB, GPIO_AF9, GPIO3);
    gpio_set_af(GPIOB, GPIO_AF4, GPIO10);
#elif defined(DEV_ELIOT)
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
              GPIO_I2C1_SCL | GPIO_I2C1_SDA);
#else
#error Call from not implemented platform !!
#endif

    /* Disable the I2C before changing any configuration. */
    i2c_peripheral_disable(I2C_USB_HUB);

    /* APB1 is running at 36MHz. */
    i2c_set_clock_frequency(I2C_USB_HUB, I2C_CR2_FREQ_36MHZ);

    /* 100KHz - I2C Standard Mode */
    i2c_set_standard_mode(I2C_USB_HUB);

    /*SMBus Host mode*/
    i2c_smbus_host_mode(I2C_USB_HUB);

    /*
     * we want 100kHz for SMBus
     * fclock for I2C is 36MHz APB1 and ccr = fclock/(i2c_speed * 2) = 180 =>0xB4
     */
    i2c_set_ccr(I2C_USB_HUB, 0xB4);

    /*
     * fclock for I2C is 36MHz -> Trise = (36MHz/1000000) + 1 = 37 -> 0x25
     */
    i2c_set_trise(I2C_USB_HUB, 0x25);

    /* If everything is configured -> enable the peripheral. */
    i2c_peripheral_enable(I2C_USB_HUB);
}

void SMBus_write(uint8_t i2c_addr, uint8_t reg, uint8_t nbDatas, uint8_t* datas)
{
    if(nbDatas > 0){
        uint32_t reg32 __attribute__((unused));

        /* Send START condition. */
        i2c_send_start(I2C_USB_HUB);

        /* Waiting for START is sent and switched to master mode. */
        while (!((I2C_SR1(I2C_USB_HUB) & I2C_SR1_SB)
            & (I2C_SR2(I2C_USB_HUB) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

        /* Send destination address. */
        i2c_send_7bit_address(I2C_USB_HUB, i2c_addr>>1, I2C_WRITE);

        /* Waiting for address is transferred. */
        while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_ADDR));

        /* Cleaning ADDR condition sequence. */
        reg32 = I2C_SR2(I2C_USB_HUB);

        i2c_send_data(I2C_USB_HUB, reg); /* register */
        while (!(I2C_SR1(I2C_USB_HUB) & (I2C_SR1_BTF | I2C_SR1_TxE)));

        i2c_send_data(I2C1, nbDatas); /* data count (SMBus specific) */
        while (!(I2C_SR1(I2C_USB_HUB) & (I2C_SR1_BTF | I2C_SR1_TxE)));

        /* Sending the data. */
        do{
            i2c_send_data(I2C_USB_HUB, *datas++);
            while (!(I2C_SR1(I2C_USB_HUB) & (I2C_SR1_BTF | I2C_SR1_TxE))); /* Await ByteTransferedFlag. */
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

        /* Send START condition. */
        i2c_send_start(I2C_USB_HUB);

        /* Waiting for START is send and switched to master mode. */
        while (!((I2C_SR1(I2C_USB_HUB) & I2C_SR1_SB)
            & (I2C_SR2(I2C_USB_HUB) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

        /* Say to which address we want to talk to. */
        i2c_send_7bit_address(I2C_USB_HUB, i2c_addr>>1, I2C_WRITE);

        /* Waiting for address is transferred. */
        while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_ADDR));

        /* Cleaning ADDR condition sequence. */
        reg32 = I2C_SR2(I2C_USB_HUB);

        i2c_send_data(I2C_USB_HUB, reg); /* register */
        while (!(I2C_SR1(I2C_USB_HUB) & (I2C_SR1_BTF | I2C_SR1_TxE)));

        /* Send START condition. */
        i2c_send_start(I2C_USB_HUB);

        /* Waiting for START is send and switched to master mode. */
        while (!((I2C_SR1(I2C_USB_HUB) & I2C_SR1_SB)
            & (I2C_SR2(I2C_USB_HUB) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

        /* Say to what address we want to talk to. */
        i2c_send_7bit_address(I2C_USB_HUB, i2c_addr>>1, I2C_READ);

        //never used in SMBus mode but it works for I2C :-)
        if(nbDatas == 1){

            /* Waiting for address is transferred. */
            while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_ADDR));
            /* Cleaning ADDR condition sequence. */
            reg32 = I2C_SR2(I2C_USB_HUB);
            //Deactivate ack
            i2c_disable_ack(I2C_USB_HUB);
            /* Send STOP condition. */
            i2c_send_stop(I2C_USB_HUB);

            //Read the Byte
            while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_RxNE));
            *datas++ = i2c_get_data(I2C_USB_HUB);

        }else if(nbDatas == 2){
             //activate ack
            i2c_enable_ack(I2C_USB_HUB);
            i2c_nack_next(I2C_USB_HUB);

            /* Waiting for address is transferred. */
            while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_ADDR));
            /* Cleaning ADDR condition sequence. */
            reg32 = I2C_SR2(I2C_USB_HUB);

            //Deactivate ack
            i2c_disable_ack(I2C_USB_HUB);

            //Read the N-1 Byte
            while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_BTF));

            /* Send STOP condition. */
            i2c_send_stop(I2C_USB_HUB);

            //Read the N-1 Byte and trash it
            reg32 = i2c_get_data(I2C_USB_HUB);
            //Read the last Byte
            *datas++ = i2c_get_data(I2C_USB_HUB);
        }else{

            //activate ack
            i2c_enable_ack(I2C_USB_HUB);
            i2c_nack_current(I2C_USB_HUB);

             /* Waiting for address is transferred. */
            while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_ADDR));
            /* Cleaning ADDR condition sequence. */
            reg32 = I2C_SR2(I2C_USB_HUB);

            //read the first byte and trash it
            while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_BTF));
            reg32 = i2c_get_data(I2C_USB_HUB);
            nbDatas--;

            //read until the N-2 byte
            while(nbDatas > 2)
            {
                 /* Now the slave should begin to send us the first byte. Await BTF. */
                while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_BTF));
                *datas++ = i2c_get_data(I2C_USB_HUB);
                nbDatas--;
            }

            //Deactivate ack
            i2c_disable_ack(I2C_USB_HUB);
            /* Send STOP condition. */
            i2c_send_stop(I2C_USB_HUB);

            //Read the N-1 Byte
            while (!(I2C_SR1(I2C_USB_HUB) & I2C_SR1_BTF));
            *datas++ = i2c_get_data(I2C_USB_HUB);

            //Read the last Byte
            //while (!(I2C_SR1(I2C1) & I2C_SR1_BTF));
            *datas++ = i2c_get_data(I2C_USB_HUB);
        }
    }
}
