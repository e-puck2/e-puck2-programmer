#include "general.h"
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/dma.h>
#include "cdcacm.h"
#include "dfsdm.h"
#include "gdb_packet.h" 


static DFSDM_config_t* mic_used_cfg;
static bool dfsdm_used;
static bool dfsdm_continuous;

/** Function called on DFSDM interrupt. */
void dma2_stream0_isr(void)
{
    bool teif = dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_TEIF);
    bool htif = dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_HTIF);
    bool tcif = dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_TCIF);
    bool dmeif = dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_DMEIF);
    /* DMA errors handling.*/
    if ((teif || dmeif) != 0) {
        if (mic_used_cfg->error_cb != NULL) {
            mic_used_cfg->error_cb(mic_used_cfg->cb_arg);
        }
    } else if (tcif != 0) {
        DMA_LIFCR(DMA2) |= DMA_LISR_TCIF0;
        int32_t* buf;
        size_t length;
        if(!dfsdm_continuous){
            dma_disable_stream(DMA2, DMA_STREAM0);
            DFSDM1_Filter0->FLTCR1 &= ~DFSDM_FLTCR1_RCONT;
            dfsdm_used = false;
            buf = mic_used_cfg->samples;
            length = mic_used_cfg->samples_len;
        }else{
            length = mic_used_cfg->samples_len / 2;
            buf = &mic_used_cfg->samples[length];
        }
        /* End of the second half of the circular buffer. */
        if (mic_used_cfg->end_cb != NULL) {
            mic_used_cfg->end_cb(mic_used_cfg->cb_arg,
                             buf, length);
        }
    } else if (htif != 0) {
        DMA_LIFCR(DMA2) |= DMA_LISR_HTIF0;
        /* End of the first half of the circular buffer. */
        if (mic_used_cfg->end_cb != NULL && dfsdm_continuous) {
            size_t half = mic_used_cfg->samples_len / 2;
            mic_used_cfg->end_cb(mic_used_cfg->cb_arg,
                             mic_used_cfg->samples,
                             half);
        }
    }
}

/** Function called on DFSDM interrupt. */
void dma2_stream1_isr(void)
{
    bool teif = dma_get_interrupt_flag(DMA2, DMA_STREAM1, DMA_TEIF);
    bool htif = dma_get_interrupt_flag(DMA2, DMA_STREAM1, DMA_HTIF);
    bool tcif = dma_get_interrupt_flag(DMA2, DMA_STREAM1, DMA_TCIF);
    bool dmeif = dma_get_interrupt_flag(DMA2, DMA_STREAM1, DMA_DMEIF);
    /* DMA errors handling.*/
    if ((teif || dmeif) != 0) {
        if (mic_used_cfg->error_cb != NULL) {
            mic_used_cfg->error_cb(mic_used_cfg->cb_arg);
        }
    } else if (tcif != 0) {
        DMA_LIFCR(DMA2) |= DMA_LISR_TCIF1;
        int32_t* buf;
        size_t length;
        if(!dfsdm_continuous){
            dma_disable_stream(DMA2, DMA_STREAM1);
            DFSDM1_Filter1->FLTCR1 &= ~DFSDM_FLTCR1_RCONT;
            dfsdm_used = false;
            buf = mic_used_cfg->samples;
            length = mic_used_cfg->samples_len;
        }else{
            length = mic_used_cfg->samples_len / 2;
            buf = &mic_used_cfg->samples[length];
        }
        /* End of the second half of the circular buffer. */
        if (mic_used_cfg->end_cb != NULL) {
            mic_used_cfg->end_cb(mic_used_cfg->cb_arg,
                             buf, length);
        }
    } else if (htif != 0) {
        DMA_LIFCR(DMA2) |= DMA_LISR_HTIF1;
        /* End of the first half of the circular buffer. */
        if (mic_used_cfg->end_cb != NULL && dfsdm_continuous) {
            size_t half = mic_used_cfg->samples_len / 2;
            mic_used_cfg->end_cb(mic_used_cfg->cb_arg,
                             mic_used_cfg->samples,
                             half);
        }
    }
}

void dfsdm_start(void)
{

    /* Send clock to peripheral. */
    rcc_periph_clock_enable(RCC_DFSDM1EN);

    /* Configure DFSDM clock output (must be before enabling interface).
     *
     * The clock output is used by the microphones to send their data out.
     * DFSDM is on APB2 @ 24 Mhz. The MP45DT02 MEMS microphone runs @ 2.4 Mhz,
     * requiring a prescaler of 10 -> 9 for the register config.
     */
    const unsigned clkout_div = 9;
    DFSDM1_Channel0->CHCFGR1 |= (clkout_div & 0xff) << DFSDM_CHCFGR1_CKOUTDIV_Pos;

    /* Enable DFSDM interface */
    DFSDM1_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_DFSDMEN;

    /* Configure clock to internal clock (CKOUT) */
    DFSDM1_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_SPICKSEL_0;
    DFSDM1_Channel1->CHCFGR1 |= DFSDM_CHCFGR1_SPICKSEL_0;
    DFSDM1_Channel2->CHCFGR1 |= DFSDM_CHCFGR1_SPICKSEL_0;


    /* Filter units configuration:
     * - Fast mode enabled
     * - Corresponding channel must be selected
     * - Continuous mode
     * - For channel 1: start synchronously with channel 0
     * - Sinc3 filter (from ST application example)
     * - Oversampling factor (OF) = 55, integrator oversampling (IF) = 1
     *   -> acquisition rate = APB2 / (clkout_div * OF * IF)
     *                       = 24 Mhz / (10 * 55 * 1) = 43.6 Khz.
     *   -> resolution = +/- (OF)^3
     *                 = +/- 166375
     *                 = ~ 19 bits (including sign bit)
     *    For details on filter configuration see section 15.4.8 of the
     *    reference manual (Digital Filter Configuration).
     *
     * TODO: Get to a precise 44.1 Khz clock using audio PLL
     */
    DFSDM1_Filter0->FLTCR1 = DFSDM_FLTCR1_FAST \
                             | (1 << DFSDM_FLTCR1_RDMAEN_Pos)
                             | (0 << DFSDM_FLTCR1_RCH_Pos);     /* channel */
    DFSDM1_Filter0->FLTFCR = (3 << DFSDM_FLTFCR_FORD_Pos)       /* filter order */ \
                             | (55 << DFSDM_FLTFCR_FOSR_Pos)    /* filter oversampling */ \
                             | (0 << DFSDM_FLTFCR_IOSR_Pos);   /* integrator oversampling */

    /* Filter 1 is identical, except that RSYNC is enabled. */
    DFSDM1_Filter1->FLTCR1 = DFSDM_FLTCR1_FAST \
                             | DFSDM_FLTCR1_RSYNC \
                             | (1 << DFSDM_FLTCR1_RDMAEN_Pos)
                             | (1 << DFSDM_FLTCR1_RCH_Pos);     /* channel */
    DFSDM1_Filter1->FLTFCR = (3 << DFSDM_FLTFCR_FORD_Pos)       /* filter order */ \
                             | (55 << DFSDM_FLTFCR_FOSR_Pos)    /* filter oversampling */ \
                             | (0 << DFSDM_FLTFCR_IOSR_Pos);   /* integrator oversampling */


    /* Enable the DMA interrupts */
    nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
    nvic_enable_irq(NVIC_DMA2_STREAM1_IRQ);

}

void dfsdm_start_conversion(DFSDM_config_t *mic_config, bool continuous)
{
    if(dfsdm_used == true){
        return;
    }

    dfsdm_continuous = continuous;

    mic_used_cfg = mic_config;
    uint8_t channel = 0;

    /*filter configuration*/
    //stops the filters
    DFSDM1_Filter0->FLTCR1 &= ~DFSDM_FLTCR1_RCONT;
    DFSDM1_Filter1->FLTCR1 &= ~DFSDM_FLTCR1_RCONT;

    /* disable the filters */
    DFSDM1_Filter0->FLTCR1 &= ~DFSDM_FLTCR1_DFEN;
    DFSDM1_Filter1->FLTCR1 &= ~DFSDM_FLTCR1_DFEN;

    if(mic_used_cfg->dfsdm_channel == DFSDM1_Channel0){
        channel = 0;
    }else if(mic_used_cfg->dfsdm_channel == DFSDM1_Channel1){
        channel = 1;
    }else if (mic_used_cfg->dfsdm_channel == DFSDM1_Channel2){
        channel = 2;
    }else if (mic_used_cfg->dfsdm_channel == DFSDM1_Channel3){
        channel = 3;
    }
    //assign the channel to the filter
    uint32_t temp_cr1 = mic_used_cfg->dfsdm_filter->FLTCR1;
    temp_cr1 &= ~DFSDM_FLTCR1_RCH; //clear the RCH flag
    temp_cr1 |= channel << DFSDM_FLTCR1_RCH_Pos;
    mic_used_cfg->dfsdm_filter->FLTCR1 = temp_cr1;

    /*channel configuration*/
    //disable channel
    mic_used_cfg->dfsdm_channel->CHCFGR1 &= ~DFSDM_CHCFGR1_CHEN;

    //config the rising or falling edge of the channel and the input to use
    uint32_t temp_cfgr1 = mic_used_cfg->dfsdm_channel->CHCFGR1;
    temp_cfgr1 &= ~DFSDM_CHCFGR1_SITP; //clean the SITP flag
    temp_cfgr1 &= ~DFSDM_CHCFGR1_CHINSEL; //clean the CHINSEL flag
    temp_cfgr1 |= mic_used_cfg->dfsdm_edge << DFSDM_CHCFGR1_SITP_Pos ;
    temp_cfgr1 |= mic_used_cfg->dfsdm_input << DFSDM_CHCFGR1_CHINSEL_Pos;
    mic_used_cfg->dfsdm_channel->CHCFGR1 = temp_cfgr1;
    //enable channel
    mic_used_cfg->dfsdm_channel->CHCFGR1 |= DFSDM_CHCFGR1_CHEN;


    DFSDM1_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_CHINSEL;

    /* Allocate DMA stream. */
    dma_stream_reset(mic_used_cfg->dma, mic_used_cfg->dma_stream);
    dma_disable_stream(mic_used_cfg->dma, mic_used_cfg->dma_stream);
    dma_set_priority(   mic_used_cfg->dma, mic_used_cfg->dma_stream, 
                        mic_used_cfg->dma_priority);

    /* Configure DMA stream. */
    dma_set_peripheral_address( mic_used_cfg->dma, mic_used_cfg->dma_stream, 
                                (uint32_t) &mic_used_cfg->dfsdm_filter->FLTRDATAR);
    dma_set_memory_address( mic_used_cfg->dma, mic_used_cfg->dma_stream, 
                            (uint32_t) mic_used_cfg->samples);
    dma_set_number_of_data( mic_used_cfg->dma, mic_used_cfg->dma_stream, 
                            mic_used_cfg->samples_len);

    /* set mode */
    dma_set_transfer_mode(  mic_used_cfg->dma, mic_used_cfg->dma_stream, 
                            DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_memory_size(mic_used_cfg->dma, mic_used_cfg->dma_stream, 
                        DMA_SxCR_MSIZE_32BIT);
    dma_set_peripheral_size(mic_used_cfg->dma, mic_used_cfg->dma_stream, 
                            DMA_SxCR_PSIZE_32BIT);
    dma_enable_memory_increment_mode(mic_used_cfg->dma, mic_used_cfg->dma_stream);
    dma_enable_circular_mode(mic_used_cfg->dma, mic_used_cfg->dma_stream);
    dma_enable_half_transfer_interrupt(mic_used_cfg->dma, mic_used_cfg->dma_stream);
    dma_enable_transfer_complete_interrupt(mic_used_cfg->dma, mic_used_cfg->dma_stream);
    dma_enable_direct_mode_error_interrupt(mic_used_cfg->dma, mic_used_cfg->dma_stream);
    dma_enable_transfer_error_interrupt(mic_used_cfg->dma, mic_used_cfg->dma_stream);
    dma_channel_select(mic_used_cfg->dma, mic_used_cfg->dma_stream, mic_used_cfg->dma_channel);

    dma_enable_stream(mic_used_cfg->dma, mic_used_cfg->dma_stream);

    /* Enable the filters */
    DFSDM1_Filter0->FLTCR1 |= DFSDM_FLTCR1_DFEN;
    DFSDM1_Filter1->FLTCR1 |= DFSDM_FLTCR1_DFEN;
    /* Enable continuous conversion. */
    DFSDM1_Filter0->FLTCR1 |= DFSDM_FLTCR1_RCONT;
    DFSDM1_Filter1->FLTCR1 |= DFSDM_FLTCR1_RCONT;

    /* Start acquisition */
    DFSDM1_Filter0->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;

    dfsdm_used = true;
}

void dfsdm_stop_conversion(void)
{
    /* Halting the DFSDM conversions is done by clearing the continous
     * conversion bit. */
    DFSDM1_Filter0->FLTCR1 &= ~DFSDM_FLTCR1_RCONT;
    DFSDM1_Filter1->FLTCR1 &= ~DFSDM_FLTCR1_RCONT;

    dma_disable_stream(DMA2, DMA_STREAM0);
    dma_disable_stream(DMA2, DMA_STREAM1);

}

void dfsdm_data_callback(void *p, int32_t *buffer, size_t n)
{
    (void) n;
    /* Check if it is the microphone we are using. */
    if ((int) p) {
        samples = buffer;
        dfsdm_data_ready = true;
    }
}

void dfsdm_err_cb(void *p)
{
    (void) p;
    gdb_out("DFSDM DMA error\n");
    while(1);
}
