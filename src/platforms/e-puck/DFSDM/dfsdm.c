#include "general.h"
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/dma.h>
#include "cdcacm.h"
#include "dfsdm.h"
#include "register_complement.h"


/* Those defines are missing from the STM32F769 include, copied them from the L4 one. */
#define DFSDM_CHCFGR1_CKOUTDIV_Pos           (16U)
#define DFSDM_CHCFGR1_CKOUTDIV_Msk           (0xFFU << DFSDM_CHCFGR1_CKOUTDIV_Pos) /*!< 0x00FF0000 */
#define DFSDM_CHCFGR2_DTRBS_Pos              (3U)
#define DFSDM_FLTCR1_RCH_Pos                 (24U)
#define DFSDM_FLTCR1_RDMAEN_Pos              (21U)
#define DFSDM_FLTFCR_IOSR_Pos                (0U)
#define DFSDM_FLTFCR_FOSR_Pos                (16U)
#define DFSDM_FLTFCR_FORD_Pos                (29U)

/* Both DFSDM units are wired to the same DMA channel, but is changed depending
 * on the DMA stream.
 *
 * See Table 28 (DMA2 request mapping) in the STM32F413 reference manual for
 * complete list.
 * */
#define DFSDM_FLT0_DMA_CHN (7<<25)
#define DFSDM_FLT1_DMA_CHN (3<<25)

typedef struct {
    //const stm32_dma_stream_t *dma_stream;
    DFSDM_config_t *cfg;
} DFSDM_driver_t;

DFSDM_driver_t left_drv, right_drv;

/** Function called on DFSDM interrupt. */
void dma2_stream0_isr(void)
{
    DFSDM_driver_t *drv = (DFSDM_driver_t *) &left_drv;
    bool teif = dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_TEIF);
    bool htif = dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_HTIF);
    bool tcif = dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_TCIF);
    bool dmeif = dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_DMEIF);
    /* DMA errors handling.*/
    if ((teif || dmeif) != 0) {
        if (drv->cfg->error_cb != NULL) {
            drv->cfg->error_cb(drv->cfg->cb_arg);
        }
    } else if (tcif != 0) {
        /* End of the second halt of the circular buffer. */
        if (drv->cfg->end_cb != NULL) {
            nvic_disable_irq(NVIC_DMA2_STREAM0_IRQ);
            size_t half = drv->cfg->samples_len / 2;
            drv->cfg->end_cb(drv->cfg->cb_arg,
                             &drv->cfg->samples[half],
                             half);
        }
    } else if (htif != 0) {
        /* End of the first half of the circular buffer. */
        if (drv->cfg->end_cb != NULL) {
            nvic_disable_irq(NVIC_DMA2_STREAM0_IRQ);
            size_t half = drv->cfg->samples_len / 2;
            drv->cfg->end_cb(drv->cfg->cb_arg,
                             drv->cfg->samples,
                             half);
        }
    }
}

/** Function called on DFSDM interrupt. */
void dma2_stream1_isr(void)
{
    DFSDM_driver_t *drv = (DFSDM_driver_t *) &right_drv;
    bool teif = dma_get_interrupt_flag(DMA2, DMA_STREAM1, DMA_TEIF);
    bool htif = dma_get_interrupt_flag(DMA2, DMA_STREAM1, DMA_HTIF);
    bool tcif = dma_get_interrupt_flag(DMA2, DMA_STREAM1, DMA_TCIF);
    bool dmeif = dma_get_interrupt_flag(DMA2, DMA_STREAM1, DMA_DMEIF);
    /* DMA errors handling.*/
    if ((teif || dmeif) != 0) {
        if (drv->cfg->error_cb != NULL) {
            drv->cfg->error_cb(drv->cfg->cb_arg);
        }
    } else if (tcif != 0) {
        /* End of the second halt of the circular buffer. */
        if (drv->cfg->end_cb != NULL) {
            nvic_disable_irq(NVIC_DMA2_STREAM1_IRQ);
            size_t half = drv->cfg->samples_len / 2;
            drv->cfg->end_cb(drv->cfg->cb_arg,
                             &drv->cfg->samples[half],
                             half);
        }
    } else if (htif != 0) {
        /* End of the first half of the circular buffer. */
        if (drv->cfg->end_cb != NULL) {
            nvic_disable_irq(NVIC_DMA2_STREAM1_IRQ);
            size_t half = drv->cfg->samples_len / 2;
            drv->cfg->end_cb(drv->cfg->cb_arg,
                             drv->cfg->samples,
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
     * DFSDM is on APB2 @ 108 Mhz. The MP34DT01 MEMS microphone runs @ 2.4 Mhz,
     * requiring a prescaler of 46.
     */
    const unsigned clkout_div = 9;
    DFSDM1_Channel0->CHCFGR1 |= (clkout_div & 0xff) << DFSDM_CHCFGR1_CKOUTDIV_Pos;

    /* Enable DFSDM interface */
    DFSDM1_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_DFSDMEN;

    /* Serial input configuration.
     *
     * The two microphones (left and right) are connected to the same input pin.
     * As the microphone dont have a clock output, we reuse the internal clock.
     *
     * Channel 0 is connected on the input from channel 1 (CHINSEL=1)
     * Channel 0 data are on rising edge (SITP=0), while channel 1 are on falling edge(SITP=1).
     */
    DFSDM1_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_CHINSEL;
    DFSDM1_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_SPICKSEL_0;

    DFSDM1_Channel1->CHCFGR1 |= DFSDM_CHCFGR1_SPICKSEL_0;
    DFSDM1_Channel1->CHCFGR1 |= DFSDM_CHCFGR1_SITP_0;

    /* Enable channel 0 and 1. */
    DFSDM1_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_CHEN;
    DFSDM1_Channel1->CHCFGR1 |= DFSDM_CHCFGR1_CHEN;

    /* Filter units configuration:
     * - Fast mode enabled
     * - Corresponding channel must be selected
     * - Continuous mode
     * - For channel 1: start synchronously with channel 0
     * - Sinc3 filter (from ST application example)
     * - Oversampling factor (OF) = 55, integrator oversampling (IF) = 1
     *   -> acquisition rate = APB2 / (clkout_div * OF * IF)
     *                       = 108 Mhz / (45 * 55) = 43.6 Khz.
     *   -> resolution = +/- (OF)^3
     *                 = +/- 166375
     *                 = ~ 19 bits (including sign bit)
     *    For details on filter configuration see section 17.3.8 of the
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


    /* Enable the filters */
    DFSDM1_Filter0->FLTCR1 |= DFSDM_FLTCR1_DFEN;
    DFSDM1_Filter1->FLTCR1 |= DFSDM_FLTCR1_DFEN;

    /* Allocate DMA streams. */
    dma_stream_reset(DMA2, DMA_STREAM0);
    dma_disable_stream(DMA2, DMA_STREAM0);
    dma_set_priority(DMA2, DMA_STREAM0, STM32_DFSDM_MICROPHONE_LEFT_DMA_PRIORITY);

    dma_stream_reset(DMA2, DMA_STREAM1);
    dma_disable_stream(DMA2, DMA_STREAM1);
    dma_set_priority(DMA2, DMA_STREAM1, STM32_DFSDM_MICROPHONE_RIGHT_DMA_PRIORITY);

}

void dfsdm_start_conversion(DFSDM_config_t *left_config, DFSDM_config_t *right_config)
{

    left_drv.cfg = left_config;
    right_drv.cfg = right_config;

    ///////LEFT/////////
    /* Configure left DMA stream. */
    dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t) &DFSDM1_Filter0->FLTRDATAR);
    dma_set_memory_address(DMA2, DMA_STREAM0, (uint32_t) left_drv.cfg->samples);
    dma_set_number_of_data(DMA2, DMA_STREAM0, left_drv.cfg->samples_len);

    /*set mode*/
    dma_set_transfer_mode(DMA2, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_32BIT);
    dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_32BIT);
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);
    dma_enable_circular_mode(DMA2, DMA_STREAM0);
    dma_enable_half_transfer_interrupt(DMA2, DMA_STREAM0);
    dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM0);
    dma_enable_direct_mode_error_interrupt(DMA2, DMA_STREAM0);
    dma_enable_transfer_error_interrupt(DMA2, DMA_STREAM0);
    dma_channel_select(DMA2, DMA_STREAM0, DFSDM_FLT0_DMA_CHN);

    nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
    dma_enable_stream(DMA2, DMA_STREAM0);

    ///////RIGHT/////////
    /* Configure right DMA stream. */
    dma_set_peripheral_address(DMA2, DMA_STREAM1, (uint32_t) &DFSDM1_Filter1->FLTRDATAR);
    dma_set_memory_address(DMA2, DMA_STREAM1, (uint32_t) right_drv.cfg->samples);
    dma_set_number_of_data(DMA2, DMA_STREAM1, left_drv.cfg->samples_len);
    /*set mode*/
    dma_set_transfer_mode(DMA2, DMA_STREAM1, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_memory_size(DMA2, DMA_STREAM1, DMA_SxCR_MSIZE_32BIT);
    dma_set_peripheral_size(DMA2, DMA_STREAM1, DMA_SxCR_PSIZE_32BIT);
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM1);
    dma_enable_circular_mode(DMA2, DMA_STREAM1);
    dma_enable_half_transfer_interrupt(DMA2, DMA_STREAM1);
    dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM1);
    dma_enable_direct_mode_error_interrupt(DMA2, DMA_STREAM1);
    dma_enable_transfer_error_interrupt(DMA2, DMA_STREAM1);
    dma_channel_select(DMA2, DMA_STREAM1, DFSDM_FLT1_DMA_CHN);

    nvic_enable_irq(NVIC_DMA2_STREAM1_IRQ);
    dma_enable_stream(DMA2, DMA_STREAM1);

    /* Enable continuous conversion. */
    DFSDM1_Filter0->FLTCR1 |= DFSDM_FLTCR1_RCONT;
    DFSDM1_Filter1->FLTCR1 |= DFSDM_FLTCR1_RCONT;

    /* Start acquisition */
    DFSDM1_Filter0->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;
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
    (void) buffer;

    /* Only a half buffer is used at a time. This means that while we are
     * processing one half of the buffer, the other half already captures the
     * new data. */
    if(n != AUDIO_BUFFER_SIZE / 2){
        while(usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT,"Buffer size is invalid.", 23) <= 0);
        while(1);
    }

    /* Check if it is the microphone we are using. */
    if ((int) p) {
        samples = buffer;
        dfsdm_data_ready = true;
    }
}

void dfsdm_err_cb(void *p)
{
    (void) p;
    while(usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT,"DFSDM DMA error", 15) <= 0);
    while(1);
}
