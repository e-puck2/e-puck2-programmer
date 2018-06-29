/** @addtogroup flash_defines
 *
 * @author @htmlonly &copy; @endhtmlonly 2010
 * Thomas Otto <tommi@viadmin.org>
 * @author @htmlonly &copy; @endhtmlonly 2010
 * Mark Butler <mbutler@physics.otago.ac.nz>
 *
 */
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
 * Copyright (C) 2010 Mark Butler <mbutler@physics.otago.ac.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * For details see:
 * PM0081 Programming manual: STM32F40xxx and STM32F41xxx Flash programming
 * September 2011, Doc ID 018520 Rev 1
 * https://github.com/libopencm3/libopencm3-archive/blob/master/st_micro/DM00023388.pdf
 */

/** @endcond */
#ifndef LIBOPENCM3_FLASH_COMMON_F24_H
#define LIBOPENCM3_FLASH_COMMON_F24_H
/**@{*/

#include "hal.h"
#include "flash_common_f234.h"

/* --- FLASH registers ----------------------------------------------------- */

#define FLASH_OPTCR				MMIO32(FLASH_R_BASE + 0x14)
#define FLASH_OPTCR1			MMIO32(FLASH_R_BASE + 0x18)

/* --- FLASH_SR values ----------------------------------------------------- */

#define FLASH_SR_OPERR			(1 << 1)

/* --- FLASH_CR values ----------------------------------------------------- */

#define FLASH_CR_ERRIE			(1 << 25)
#define FLASH_CR_SNB_SHIFT		3
#define FLASH_CR_SNB_MASK		0x1f
#define FLASH_CR_PROGRAM_MASK		0x3
#define FLASH_CR_PROGRAM_SHIFT		8
/** @defgroup flash_cr_program_width Flash programming width
@ingroup flash_group

@{*/
#define FLASH_CR_PROGRAM_X8		0
#define FLASH_CR_PROGRAM_X16		1
#define FLASH_CR_PROGRAM_X32		2
#define FLASH_CR_PROGRAM_X64		3
/**@}*/

/* --- FLASH_OPTCR values -------------------------------------------------- */

/* FLASH_OPTCR[27:16]: nWRP */
/* FLASH_OBR[15:8]: RDP */
#define FLASH_OPTCR_NRST_STDBY		(1 << 7)
#define FLASH_OPTCR_NRST_STOP		(1 << 6)
#define FLASH_OPTCR_BOR_LEVEL_3		(0x00 << 2)
#define FLASH_OPTCR_BOR_LEVEL_2		(0x01 << 2)
#define FLASH_OPTCR_BOR_LEVEL_1		(0x02 << 2)
#define FLASH_OPTCR_BOR_OFF		(0x03 << 2)

/* --- FLASH_OPTCR1 values ------------------------------------------------- */
/* Only on some devices */
/* FLASH_OPTCR1[27:16]: nWRP bank 2 */

/* --- FLASH Keys -----------------------------------------------------------*/

#define FLASH_OPTKEYR_KEY1		((uint32_t)0x08192a3b)
#define FLASH_OPTKEYR_KEY2		((uint32_t)0x4c5d6e7f)

/* --- Function prototypes ------------------------------------------------- */

void flash_unlock_option_bytes(void);
void flash_lock_option_bytes(void);
void flash_clear_pgserr_flag(void);
void flash_clear_wrperr_flag(void);
void flash_clear_pgaerr_flag(void);
void flash_dcache_enable(void);
void flash_dcache_disable(void);
void flash_icache_enable(void);
void flash_icache_disable(void);
void flash_prefetch_enable(void);
void flash_prefetch_disable(void);
void flash_dcache_reset(void);
void flash_icache_reset(void);
void flash_erase_all_sectors(uint32_t program_size);
void flash_erase_sector(uint8_t sector, uint32_t program_size);
void flash_program_double_word(uint32_t address, uint64_t data);
void flash_program_word(uint32_t address, uint32_t data);
void flash_program_half_word(uint32_t address, uint16_t data);
void flash_program_byte(uint32_t address, uint8_t data);
void flash_program(uint32_t address, uint8_t *data, uint32_t len);
void flash_program_option_bytes(uint32_t data);

/**@}*/

#endif
