#ifndef REG_COMPLEMENT_H
#define REG_COMPLEMENT_H

#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencmsis/core_cm3.h>

//////////////////////////////////////////TAKEN FROM CHIBIOS////////////////////////////////////
/**
  * @brief DFSDM channel configuration registers
  */
typedef struct
{
  __IO uint32_t CHCFGR1;     /*!< DFSDM channel configuration register1,            Address offset: 0x00 */
  __IO uint32_t CHCFGR2;     /*!< DFSDM channel configuration register2,            Address offset: 0x04 */
  __IO uint32_t CHAWSCDR;    /*!< DFSDM channel analog watchdog and
                                  short circuit detector register,                  Address offset: 0x08 */
  __IO uint32_t CHWDATAR;    /*!< DFSDM channel watchdog filter data register,      Address offset: 0x0C */
  __IO uint32_t CHDATINR;    /*!< DFSDM channel data input register,                Address offset: 0x10 */
} DFSDM_Channel_TypeDef;

/**
  * @brief DFSDM module registers
  */
typedef struct
{
  __IO uint32_t FLTCR1;         /*!< DFSDM control register1,                          Address offset: 0x100 */
  __IO uint32_t FLTCR2;         /*!< DFSDM control register2,                          Address offset: 0x104 */
  __IO uint32_t FLTISR;         /*!< DFSDM interrupt and status register,              Address offset: 0x108 */
  __IO uint32_t FLTICR;         /*!< DFSDM interrupt flag clear register,              Address offset: 0x10C */
  __IO uint32_t FLTJCHGR;       /*!< DFSDM injected channel group selection register,  Address offset: 0x110 */
  __IO uint32_t FLTFCR;         /*!< DFSDM filter control register,                    Address offset: 0x114 */
  __IO uint32_t FLTJDATAR;      /*!< DFSDM data register for injected group,           Address offset: 0x118 */
  __IO uint32_t FLTRDATAR;      /*!< DFSDM data register for regular group,            Address offset: 0x11C */
  __IO uint32_t FLTAWHTR;       /*!< DFSDM analog watchdog high threshold register,    Address offset: 0x120 */
  __IO uint32_t FLTAWLTR;       /*!< DFSDM analog watchdog low threshold register,     Address offset: 0x124 */
  __IO uint32_t FLTAWSR;        /*!< DFSDM analog watchdog status register             Address offset: 0x128 */
  __IO uint32_t FLTAWCFR;       /*!< DFSDM analog watchdog clear flag register         Address offset: 0x12C */
  __IO uint32_t FLTEXMAX;       /*!< DFSDM extreme detector maximum register,          Address offset: 0x130 */
  __IO uint32_t FLTEXMIN;       /*!< DFSDM extreme detector minimum register           Address offset: 0x134 */
  __IO uint32_t FLTCNVTIMR;     /*!< DFSDM conversion timer,                           Address offset: 0x138 */
} DFSDM_Filter_TypeDef;

/*!< APB2 peripherals */
#define DFSDM1_BASE           (PERIPH_BASE_APB2 + 0x6000U)
#define DFSDM1_Channel0_BASE  (DFSDM1_BASE + 0x00U)
#define DFSDM1_Channel1_BASE  (DFSDM1_BASE + 0x20U)
#define DFSDM1_Channel2_BASE  (DFSDM1_BASE + 0x40U)
#define DFSDM1_Channel3_BASE  (DFSDM1_BASE + 0x60U)
#define DFSDM1_Filter0_BASE   (DFSDM1_BASE + 0x100U)
#define DFSDM1_Filter1_BASE   (DFSDM1_BASE + 0x180U)

/** @addtogroup Peripheral_declaration
  * @{
  */  
#define DFSDM1_Channel0     ((DFSDM_Channel_TypeDef *) DFSDM1_Channel0_BASE)
#define DFSDM1_Channel1     ((DFSDM_Channel_TypeDef *) DFSDM1_Channel1_BASE)
#define DFSDM1_Channel2     ((DFSDM_Channel_TypeDef *) DFSDM1_Channel2_BASE)
#define DFSDM1_Channel3     ((DFSDM_Channel_TypeDef *) DFSDM1_Channel3_BASE)
#define DFSDM1_Filter0      ((DFSDM_Filter_TypeDef *) DFSDM1_Filter0_BASE)
#define DFSDM1_Filter1      ((DFSDM_Filter_TypeDef *) DFSDM1_Filter1_BASE)

/****************   DFSDM channel configuration registers  ********************/

/***************  Bit definition for DFSDM_CHCFGR1 register  ******************/
#define  DFSDM_CHCFGR1_DFSDMEN                0x80000000U            /*!< Global enable for DFSDM interface */
#define  DFSDM_CHCFGR1_CKOUTSRC               0x40000000U            /*!< Output serial clock source selection */
#define  DFSDM_CHCFGR1_CKOUTDIV               0x00FF0000U            /*!< CKOUTDIV[7:0] output serial clock divider */
#define  DFSDM_CHCFGR1_DATPACK                0x0000C000U            /*!< DATPACK[1:0] Data packing mode */
#define  DFSDM_CHCFGR1_DATPACK_1              0x00008000U            /*!< Data packing mode, Bit 1 */
#define  DFSDM_CHCFGR1_DATPACK_0              0x00004000U            /*!< Data packing mode, Bit 0 */
#define  DFSDM_CHCFGR1_DATMPX                 0x00003000U            /*!< DATMPX[1:0] Input data multiplexer for channel y */
#define  DFSDM_CHCFGR1_DATMPX_1               0x00002000U            /*!< Input data multiplexer for channel y, Bit 1 */
#define  DFSDM_CHCFGR1_DATMPX_0               0x00001000U            /*!< Input data multiplexer for channel y, Bit 0 */
#define  DFSDM_CHCFGR1_CHINSEL                0x00000100U            /*!< Serial inputs selection for channel y */
#define  DFSDM_CHCFGR1_CHEN                   0x00000080U            /*!< Channel y enable */
#define  DFSDM_CHCFGR1_CKABEN                 0x00000040U            /*!< Clock absence detector enable on channel y */
#define  DFSDM_CHCFGR1_SCDEN                  0x00000020U            /*!< Short circuit detector enable on channel y */
#define  DFSDM_CHCFGR1_SPICKSEL               0x0000000CU            /*!< SPICKSEL[1:0] SPI clock select for channel y */
#define  DFSDM_CHCFGR1_SPICKSEL_1             0x00000008U            /*!< SPI clock select for channel y, Bit 1 */
#define  DFSDM_CHCFGR1_SPICKSEL_0             0x00000004U            /*!< SPI clock select for channel y, Bit 0 */
#define  DFSDM_CHCFGR1_SITP                   0x00000003U            /*!< SITP[1:0] Serial interface type for channel y */
#define  DFSDM_CHCFGR1_SITP_1                 0x00000002U            /*!< Serial interface type for channel y, Bit 1 */
#define  DFSDM_CHCFGR1_SITP_0                 0x00000001U            /*!< Serial interface type for channel y, Bit 0 */

/***************  Bit definition for DFSDM_CHCFGR2 register  ******************/
#define  DFSDM_CHCFGR2_OFFSET                 0xFFFFFF00U            /*!< OFFSET[23:0] 24-bit calibration offset for channel y */
#define  DFSDM_CHCFGR2_DTRBS                  0x000000F8U            /*!< DTRBS[4:0] Data right bit-shift for channel y */

/******************  Bit definition for DFSDM_CHAWSCDR register *****************/
#define  DFSDM_CHAWSCDR_AWFORD                0x00C00000U            /*!< AWFORD[1:0] Analog watchdog Sinc filter order on channel y */
#define  DFSDM_CHAWSCDR_AWFORD_1              0x00800000U            /*!< Analog watchdog Sinc filter order on channel y, Bit 1 */
#define  DFSDM_CHAWSCDR_AWFORD_0              0x00400000U            /*!< Analog watchdog Sinc filter order on channel y, Bit 0 */
#define  DFSDM_CHAWSCDR_AWFOSR                0x001F0000U            /*!< AWFOSR[4:0] Analog watchdog filter oversampling ratio on channel y */
#define  DFSDM_CHAWSCDR_BKSCD                 0x0000F000U            /*!< BKSCD[3:0] Break signal assignment for short circuit detector on channel y */
#define  DFSDM_CHAWSCDR_SCDT                  0x000000FFU            /*!< SCDT[7:0] Short circuit detector threshold for channel y */

/****************  Bit definition for DFSDM_CHWDATR register *******************/
#define  DFSDM_CHWDATR_WDATA                  0x0000FFFFU            /*!< WDATA[15:0] Input channel y watchdog data */

/****************  Bit definition for DFSDM_CHDATINR register *****************/
#define  DFSDM_CHDATINR_INDAT0                0x0000FFFFU            /*!< INDAT0[31:16] Input data for channel y or channel (y+1) */
#define  DFSDM_CHDATINR_INDAT1                0xFFFF0000U            /*!< INDAT0[15:0] Input data for channel y */

/************************   DFSDM module registers  ****************************/

/********************  Bit definition for DFSDM_FLTCR1 register *******************/
#define  DFSDM_FLTCR1_AWFSEL                  0x40000000U            /*!< Analog watchdog fast mode select */
#define  DFSDM_FLTCR1_FAST                    0x20000000U            /*!< Fast conversion mode selection */
#define  DFSDM_FLTCR1_RCH                     0x07000000U            /*!< RCH[2:0] Regular channel selection */
#define  DFSDM_FLTCR1_RDMAEN                  0x00200000U            /*!< DMA channel enabled to read data for the regular conversion */
#define  DFSDM_FLTCR1_RSYNC                   0x00080000U            /*!< Launch regular conversion synchronously with DFSDMx */
#define  DFSDM_FLTCR1_RCONT                   0x00040000U            /*!< Continuous mode selection for regular conversions */
#define  DFSDM_FLTCR1_RSWSTART                0x00020000U            /*!< Software start of a conversion on the regular channel */
#define  DFSDM_FLTCR1_JEXTEN                  0x00006000U            /*!< JEXTEN[1:0] Trigger enable and trigger edge selection for injected conversions */
#define  DFSDM_FLTCR1_JEXTEN_1                0x00004000U            /*!< Trigger enable and trigger edge selection for injected conversions, Bit 1 */
#define  DFSDM_FLTCR1_JEXTEN_0                0x00002000U            /*!< Trigger enable and trigger edge selection for injected conversions, Bit 0 */
#define  DFSDM_FLTCR1_JEXTSEL                 0x00001F00U            /*!< JEXTSEL[4:0]Trigger signal selection for launching injected conversions */
#define  DFSDM_FLTCR1_JEXTSEL_0               0x00000100U            /*!< Trigger signal selection for launching injected conversions, Bit 0 */
#define  DFSDM_FLTCR1_JEXTSEL_1               0x00000200U            /*!< Trigger signal selection for launching injected conversions, Bit 1 */
#define  DFSDM_FLTCR1_JEXTSEL_2               0x00000400U            /*!< Trigger signal selection for launching injected conversions, Bit 2 */
#define  DFSDM_FLTCR1_JEXTSEL_3               0x00000800U            /*!< Trigger signal selection for launching injected conversions, Bit 3 */
#define  DFSDM_FLTCR1_JEXTSEL_4               0x00001000U            /*!< Trigger signal selection for launching injected conversions, Bit 4 */
#define  DFSDM_FLTCR1_JDMAEN                  0x00000020U            /*!< DMA channel enabled to read data for the injected channel group */
#define  DFSDM_FLTCR1_JSCAN                   0x00000010U            /*!< Scanning conversion in continuous mode selection for injected conversions */
#define  DFSDM_FLTCR1_JSYNC                   0x00000008U            /*!< Launch an injected conversion synchronously with DFSDMx JSWSTART trigger  */
#define  DFSDM_FLTCR1_JSWSTART                0x00000002U            /*!< Start the conversion of the injected group of channels */
#define  DFSDM_FLTCR1_DFEN                    0x00000001U            /*!< DFSDM enable */

/********************  Bit definition for DFSDM_FLTCR2 register *******************/
#define  DFSDM_FLTCR2_AWDCH                   0x00FF0000U            /*!< AWDCH[7:0] Analog watchdog channel selection */
#define  DFSDM_FLTCR2_EXCH                    0x0000FF00U            /*!< EXCH[7:0] Extreme detector channel selection */
#define  DFSDM_FLTCR2_CKABIE                  0x00000040U            /*!< Clock absence interrupt enable */
#define  DFSDM_FLTCR2_SCDIE                   0x00000020U            /*!< Short circuit detector interrupt enable */
#define  DFSDM_FLTCR2_AWDIE                   0x00000010U            /*!< Analog watchdog interrupt enable */
#define  DFSDM_FLTCR2_ROVRIE                  0x00000008U            /*!< Regular data overrun interrupt enable */
#define  DFSDM_FLTCR2_JOVRIE                  0x00000004U            /*!< Injected data overrun interrupt enable */
#define  DFSDM_FLTCR2_REOCIE                  0x00000002U            /*!< Regular end of conversion interrupt enable */
#define  DFSDM_FLTCR2_JEOCIE                  0x00000001U            /*!< Injected end of conversion interrupt enable */

/********************  Bit definition for DFSDM_FLTISR register *******************/
#define  DFSDM_FLTISR_SCDF                    0xFF000000U            /*!< SCDF[7:0] Short circuit detector flag */
#define  DFSDM_FLTISR_CKABF                   0x00FF0000U            /*!< CKABF[7:0] Clock absence flag */
#define  DFSDM_FLTISR_RCIP                    0x00004000U            /*!< Regular conversion in progress status */
#define  DFSDM_FLTISR_JCIP                    0x00002000U            /*!< Injected conversion in progress status */
#define  DFSDM_FLTISR_AWDF                    0x00000010U            /*!< Analog watchdog */
#define  DFSDM_FLTISR_ROVRF                   0x00000008U            /*!< Regular conversion overrun flag */
#define  DFSDM_FLTISR_JOVRF                   0x00000004U            /*!< Injected conversion overrun flag */
#define  DFSDM_FLTISR_REOCF                   0x00000002U            /*!< End of regular conversion flag */
#define  DFSDM_FLTISR_JEOCF                   0x00000001U            /*!< End of injected conversion flag */

/********************  Bit definition for DFSDM_FLTICR register *******************/
#define  DFSDM_FLTICR_CLRSCSDF                0xFF000000U            /*!< CLRSCSDF[7:0] Clear the short circuit detector flag */
#define  DFSDM_FLTICR_CLRCKABF                0x00FF0000U            /*!< CLRCKABF[7:0] Clear the clock absence flag */
#define  DFSDM_FLTICR_CLRROVRF                0x00000008U            /*!< Clear the regular conversion overrun flag */
#define  DFSDM_FLTICR_CLRJOVRF                0x00000004U            /*!< Clear the injected conversion overrun flag */

/*******************  Bit definition for DFSDM_FLTJCHGR register ******************/
#define  DFSDM_FLTJCHGR_JCHG                  0x000000FFU            /*!< JCHG[7:0] Injected channel group selection */

/********************  Bit definition for DFSDM_FLTFCR register *******************/
#define  DFSDM_FLTFCR_FORD                    0xE0000000U            /*!< FORD[2:0] Sinc filter order */
#define  DFSDM_FLTFCR_FORD_2                  0x80000000U            /*!< Sinc filter order, Bit 2 */
#define  DFSDM_FLTFCR_FORD_1                  0x40000000U            /*!< Sinc filter order, Bit 1 */
#define  DFSDM_FLTFCR_FORD_0                  0x20000000U            /*!< Sinc filter order, Bit 0 */
#define  DFSDM_FLTFCR_FOSR                    0x03FF0000U            /*!< FOSR[9:0] Sinc filter oversampling ratio (decimation rate) */
#define  DFSDM_FLTFCR_IOSR                    0x000000FFU            /*!< IOSR[7:0] Integrator oversampling ratio (averaging length) */

/******************  Bit definition for DFSDM_FLTJDATAR register *****************/
#define  DFSDM_FLTJDATAR_JDATA                0xFFFFFF00U            /*!< JDATA[23:0] Injected group conversion data */
#define  DFSDM_FLTJDATAR_JDATACH              0x00000007U            /*!< JDATACH[2:0] Injected channel most recently converted */

/******************  Bit definition for DFSDM_FLTRDATAR register *****************/
#define  DFSDM_FLTRDATAR_RDATA                0xFFFFFF00U            /*!< RDATA[23:0] Regular channel conversion data */
#define  DFSDM_FLTRDATAR_RPEND                0x00000010U            /*!< RPEND Regular channel pending data */
#define  DFSDM_FLTRDATAR_RDATACH              0x00000007U            /*!< RDATACH[2:0] Regular channel most recently converted */

/******************  Bit definition for DFSDM_FLTAWHTR register ******************/
#define  DFSDM_FLTAWHTR_AWHT                  0xFFFFFF00U             /*!< AWHT[23:0] Analog watchdog high threshold */
#define  DFSDM_FLTAWHTR_BKAWH                 0x0000000FU             /*!< BKAWH[3:0] Break signal assignment to analog watchdog high threshold event */

/******************  Bit definition for DFSDM_FLTAWLTR register ******************/
#define  DFSDM_FLTAWLTR_AWLT                  0xFFFFFF00U             /*!< AWLT[23:0] Analog watchdog low threshold */
#define  DFSDM_FLTAWLTR_BKAWL                 0x0000000FU             /*!< BKAWL[3:0] Break signal assignment to analog watchdog low threshold event */

/******************  Bit definition for DFSDM_FLTAWSR register ******************/
#define  DFSDM_FLTAWSR_AWHTF                  0x0000FF00U             /*!< AWHTF[15:8] Analog watchdog high threshold error on given channels */
#define  DFSDM_FLTAWSR_AWLTF                  0x000000FFU             /*!< AWLTF[7:0] Analog watchdog low threshold error on given channels */

/******************  Bit definition for DFSDM_FLTAWCFR register *****************/
#define  DFSDM_FLTAWCFR_CLRAWHTF              0x0000FF00U             /*!< CLRAWHTF[15:8] Clear the Analog watchdog high threshold flag */
#define  DFSDM_FLTAWCFR_CLRAWLTF              0x000000FFU             /*!< CLRAWLTF[7:0] Clear the Analog watchdog low threshold flag */

/******************  Bit definition for DFSDM_FLTEXMAX register ******************/
#define  DFSDM_FLTEXMAX_EXMAX                 0xFFFFFF00U             /*!< EXMAX[23:0] Extreme detector maximum value */
#define  DFSDM_FLTEXMAX_EXMAXCH               0x00000007U             /*!< EXMAXCH[2:0] Extreme detector maximum data channel */

/******************  Bit definition for DFSDM_FLTEXMIN register ******************/
#define  DFSDM_FLTEXMIN_EXMIN                 0xFFFFFF00U             /*!< EXMIN[23:0] Extreme detector minimum value */
#define  DFSDM_FLTEXMIN_EXMINCH               0x00000007U             /*!< EXMINCH[2:0] Extreme detector minimum data channel */

/******************  Bit definition for DFSDM_FLTCNVTIMR register ******************/
#define  DFSDM_FLTCNVTIMR_CNVCNT              0xFFFFFFF0U             /*!< CNVCNT[27:0]: 28-bit timer counting conversion time */

//////////////////////////////////////////TAKEN FROM CHIBIOS////////////////////////////////////

//////////////////////////////////////COMPLEMENT FOR LIBOPENCM3/////////////////////////////////

/*RCC*/
//#define RCC_DFSDM1EN	_REG_BIT(0x44, 21)
#define RCC_DFSDM1EN	(((0x44) << 5) + (21))

/*DMA*/
#define STM32_DFSDM_MICROPHONE_LEFT_DMA_STREAM DMA2_STREAM0
#define STM32_DFSDM_MICROPHONE_LEFT_DMA_PRIORITY DMA_SxCR_PL_HIGH
#define STM32_DFSDM_MICROPHONE_LEFT_DMA_IRQ_PRIORITY 6
#define STM32_DFSDM_MICROPHONE_RIGHT_DMA_STREAM DMA2_STREAM1
#define STM32_DFSDM_MICROPHONE_RIGHT_DMA_PRIORITY DMA_SxCR_PL_HIGH
#define STM32_DFSDM_MICROPHONE_RIGHT_DMA_IRQ_PRIORITY 6


#endif/*REG_COMPLEMENT_H*/
