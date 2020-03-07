/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2013 by Roman Dmitrienko                                *
 *   me@iamroman.org                                                       *
 *                                                                         *
 *   Copyright (C) 2014 Nemui Trinomius                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/cortex_m.h>

#define EFM_FAMILY_ID_GIANT_GECKO       72
#define EFM_FAMILY_ID_LEOPARD_GECKO     74

#define EFM32_FLASH_ERASE_TMO           100
#define EFM32_FLASH_WDATAREADY_TMO      100
#define EFM32_FLASH_WRITE_TMO           100

/* size in bytes, not words; must fit all Gecko devices */
#define LOCKBITS_PAGE_SZ                512

#define EFM32_MSC_INFO_BASE             0x0fe00000

#define EFM32_MSC_USER_DATA             EFM32_MSC_INFO_BASE
#define EFM32_MSC_LOCK_BITS             (EFM32_MSC_INFO_BASE+0x4000)
#define EFM32_MSC_DEV_INFO              (EFM32_MSC_INFO_BASE+0x8000)

/* PAGE_SIZE is not present in Zero, Happy and the original Gecko MCU */
#define EFM32_MSC_DI_PAGE_SIZE          (EFM32_MSC_DEV_INFO+0x08)
#define EFM32_MSC_DI_FLASH_SZ           (EFM32_MSC_DEV_INFO+0x0c)
#define EFM32_MSC_DI_RAM_SZ             (EFM32_MSC_DEV_INFO+0x0e)
#define EFM32_MSC_DI_PART_NUM           (EFM32_MSC_DEV_INFO+0x06)
#define EFM32_MSC_DI_PART_FAMILY        (EFM32_MSC_DEV_INFO+0x1fe)
#define EFM32_MSC_DI_PROD_REV           (EFM32_MSC_DEV_INFO+0x02)

#define EFM32_MSC_REGBASE               0x400c0000
#define EFM32_MSC_REGBASE_SERIES1       0x400e0000
#define EFM32_MSC_REGBASE_SERIES2       0x50030000
#define EFM32_MSC_REG_WRITECTRL         0x00c
#define EFM32_MSC_REG_WRITECTRL_SERIES2 0x00c
#define EFM32_MSC_WRITECTRL_WREN_MASK   0x1
#define EFM32_MSC_REG_WRITECMD          0x010
#define EFM32_MSC_REG_WRITECMD_SERIES2  0x010
#define EFM32_MSC_WRITECMD_LADDRIM_MASK 0x1
#define EFM32_MSC_WRITECMD_ERASEPAGE_MASK 0x2
#define EFM32_MSC_WRITECMD_WRITEONCE_MASK 0x8
#define EFM32_MSC_REG_ADDRB             0x014
#define EFM32_MSC_REG_ADDRB_SERIES2     0x014
#define EFM32_MSC_REG_WDATA             0x018
#define EFM32_MSC_REG_STATUS            0x01c
#define EFM32_MSC_STATUS_BUSY_MASK      0x1
#define EFM32_MSC_STATUS_LOCKED_MASK    0x2
#define EFM32_MSC_STATUS_INVADDR_MASK   0x4
#define EFM32_MSC_STATUS_WDATAREADY_MASK 0x8
#define EFM32_MSC_STATUS_WORDTIMEOUT_MASK 0x40
#define EFM32_MSC_STATUS_WORDTIMEOUT_MASK_SERIES2 0x40
#define EFM32_MSC_STATUS_ERASEABORTED_MASK 0x10
#define EFM32_MSC_STATUS_ERASEABORTED_MASK_SERIES2 0x10
#define EFM32_MSC_REG_LOCK              0x03c
#define EFM32_MSC_REG_LOCK_SERIES1      0x040
#define EFM32_MSC_LOCK_LOCKKEY          0x1b71

struct efm32_family_data {
	int family_id;
	const char *name;

	/* EFM32 series (EFM32LG995F is the "old" series 0, while EFR32MG12P132
	   is the "new" series 1). Determines location of MSC registers. */
	int series;

	/* Page size in bytes, or 0 to read from EFM32_MSC_DI_PAGE_SIZE */
	int page_size;

	/* MSC register base address, or 0 to use default */
	uint32_t msc_regbase;
};

struct efm32x_flash_bank {
	int probed;
	uint32_t lb_page[LOCKBITS_PAGE_SZ/4];
	uint32_t reg_base;
	uint32_t reg_lock;
};

struct efm32_info {
	const struct efm32_family_data *family_data;
	uint16_t flash_sz_kib;
	uint16_t ram_sz_kib;
	uint16_t part_num;
	uint8_t part_family;
	uint8_t prod_rev;
	uint16_t page_size;
};

static const struct efm32_family_data efm32_families[] = {
		{ 16, "EFR32MG1P Mighty", .series = 1 },
		{ 17, "EFR32MG1B Mighty", .series = 1 },
		{ 18, "EFR32MG1V Mighty", .series = 1 },
		{ 19, "EFR32MG1P Blue", .series = 1 },
		{ 20, "EFR32MG1B Blue", .series = 1 },
		{ 21, "EFR32MG1V Blue", .series = 1 },
		{ 25, "EFR32FG1P Flex", .series = 1 },
		{ 26, "EFR32FG1B Flex", .series = 1 },
		{ 27, "EFR32FG1V Flex", .series = 1 },
		{ 28, "EFR32MG2P Mighty", .series = 1 },
		{ 29, "EFR32MG2B Mighty", .series = 1 },
		{ 30, "EFR32MG2V Mighty", .series = 1 },
		{ 31, "EFR32BG12P Blue", .series = 1 },
		{ 32, "EFR32BG12B Blue", .series = 1 },
		{ 33, "EFR32BG12V Blue", .series = 1 },
		{ 37, "EFR32FG12P Flex", .series = 1 },
		{ 38, "EFR32FG12B Flex", .series = 1 },
		{ 39, "EFR32FG12V Flex", .series = 1 },
		{ 40, "EFR32MG13P Mighty", .series = 1 },
		{ 41, "EFR32MG13B Mighty", .series = 1 },
		{ 42, "EFR32MG13V Mighty", .series = 1 },
		{ 43, "EFR32BG13P Blue", .series = 1 },
		{ 44, "EFR32BG13B Blue", .series = 1 },
		{ 45, "EFR32BG13V Blue", .series = 1 },
		{ 46, "EFR32ZG13P Zen", .series = 1 },
		{ 49, "EFR32FG13P Flex", .series = 1 },
		{ 50, "EFR32FG13B Flex", .series = 1 },
		{ 51, "EFR32FG13V Flex", .series = 1 },
		{ 52, "EFR32MG14P Mighty", .series = 1 },
		{ 53, "EFR32MG14B Mighty", .series = 1 },
		{ 54, "EFR32MG14V Mighty", .series = 1 },
		{ 55, "EFR32BG14P Blue", .series = 1 },
		{ 56, "EFR32BG14B Blue", .series = 1 },
		{ 57, "EFR32BG14V Blue", .series = 1 },
		{ 58, "EFR32ZG14P Zen", .series = 1 },
		{ 61, "EFR32FG14P Flex", .series = 1 },
		{ 62, "EFR32FG14B Flex", .series = 1 },
		{ 63, "EFR32FG14V Flex", .series = 1 },
		{ 71, "EFM32G", .series = 0, .page_size = 512 },
		{ 72, "EFM32GG Giant", .series = 0 },
		{ 73, "EFM32TG Tiny", .series = 0, .page_size = 512 },
		{ 74, "EFM32LG Leopard", .series = 0 },
		{ 75, "EFM32WG Wonder", .series = 0 },
		{ 76, "EFM32ZG Zero", .series = 0, .page_size = 1024 },
		{ 77, "EFM32HG Happy", .series = 0, .page_size = 1024 },
		{ 81, "EFM32PG1B Pearl", .series = 1 },
		{ 83, "EFM32JG1B Jade", .series = 1 },
		{ 85, "EFM32PG12B Pearl", .series = 1 },
		{ 87, "EFM32JG12B Jade", .series = 1 },
		{ 89, "EFM32PG13B Pearl", .series = 1 },
		{ 91, "EFM32JG13B Jade", .series = 1 },
		{ 100, "EFM32GG11B Giant", .series = 1, .msc_regbase = 0x40000000 },
		{ 103, "EFM32TG11B Tiny", .series = 1, .msc_regbase = 0x40000000 },
		{ 120, "EZR32WG Wonder", .series = 0 },
		{ 121, "EZR32LG Leopard", .series = 0 },
		{ 122, "EZR32HG Happy", .series = 0, .page_size = 1024 },
        { 128, "EFR32MG Series W", .series = 2, .page_size = 8192 },
};


static int efm32x_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count);

static int efm32x_get_flash_size(struct flash_bank *bank, uint16_t *flash_sz)
{
	return target_read_u16(bank->target, EFM32_MSC_DI_FLASH_SZ, flash_sz);
}

static int efm32x_get_ram_size(struct flash_bank *bank, uint16_t *ram_sz)
{
	return target_read_u16(bank->target, EFM32_MSC_DI_RAM_SZ, ram_sz);
}

static int efm32x_get_part_num(struct flash_bank *bank, uint16_t *pnum)
{
	return target_read_u16(bank->target, EFM32_MSC_DI_PART_NUM, pnum);
}

static int efm32x_get_part_family(struct flash_bank *bank, uint8_t *pfamily)
{
	return target_read_u8(bank->target, EFM32_MSC_DI_PART_FAMILY, pfamily);
}

static int efm32x_get_prod_rev(struct flash_bank *bank, uint8_t *prev)
{
	return target_read_u8(bank->target, EFM32_MSC_DI_PROD_REV, prev);
}

static int efm32x_read_reg_u32(struct flash_bank *bank, target_addr_t offset,
			       uint32_t *value)
{
	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;
	uint32_t base = efm32x_info->reg_base;

	return target_read_u32(bank->target, base + offset, value);
}

static int efm32x_write_reg_u32(struct flash_bank *bank, target_addr_t offset,
			       uint32_t value)
{
	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;
	uint32_t base = efm32x_info->reg_base;

	return target_write_u32(bank->target, base + offset, value);
}

static int efm32x_read_info(struct flash_bank *bank,
	struct efm32_info *efm32_info)
{
	int ret;
	uint32_t cpuid = 0;
	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;

	memset(efm32_info, 0, sizeof(struct efm32_info));

	ret = target_read_u32(bank->target, CPUID, &cpuid);
	if (ERROR_OK != ret)
		return ret;

	if (((cpuid >> 4) & 0xfff) == 0xc23) {
		/* Cortex-M3 device */
	} else if (((cpuid >> 4) & 0xfff) == 0xc24) {
		/* Cortex-M4 device (WONDER GECKO) */
	} else if (((cpuid >> 4) & 0xfff) == 0xc60) {
		/* Cortex-M0+ device */
	} else if (((cpuid >> 4) & 0xfff) == 0xd21) {
		/* Cortex-M33 device */
	} else {
		LOG_ERROR("Target is not Cortex-Mx Device");
		return ERROR_FAIL;
	}

	ret = efm32x_get_flash_size(bank, &(efm32_info->flash_sz_kib));
	if (ERROR_OK != ret)
		return ret;

	ret = efm32x_get_ram_size(bank, &(efm32_info->ram_sz_kib));
	if (ERROR_OK != ret)
		return ret;

	ret = efm32x_get_part_num(bank, &(efm32_info->part_num));
	if (ERROR_OK != ret)
		return ret;

	ret = efm32x_get_part_family(bank, &(efm32_info->part_family));
	if (ERROR_OK != ret)
		return ret;

	ret = efm32x_get_prod_rev(bank, &(efm32_info->prod_rev));
	if (ERROR_OK != ret)
		return ret;

	for (size_t i = 0; i < ARRAY_SIZE(efm32_families); i++) {
		if (efm32_families[i].family_id == efm32_info->part_family)
			efm32_info->family_data = &efm32_families[i];
	}

	if (efm32_info->family_data == NULL) {
		LOG_ERROR("Unknown MCU family %d", efm32_info->part_family);
		return ERROR_FAIL;
	}

	switch (efm32_info->family_data->series) {
		case 0:
			efm32x_info->reg_base = EFM32_MSC_REGBASE;
			efm32x_info->reg_lock = EFM32_MSC_REG_LOCK;
			break;
		case 1:
			efm32x_info->reg_base = EFM32_MSC_REGBASE_SERIES1;
			efm32x_info->reg_lock = EFM32_MSC_REG_LOCK_SERIES1;
			break;
        case 2:
			efm32x_info->reg_base = EFM32_MSC_REGBASE_SERIES2;
			efm32x_info->reg_lock = EFM32_MSC_REG_LOCK;
	}

	if (efm32_info->family_data->msc_regbase != 0)
		efm32x_info->reg_base = efm32_info->family_data->msc_regbase;

	if (efm32_info->family_data->page_size != 0) {
		efm32_info->page_size = efm32_info->family_data->page_size;
	} else {
		uint8_t pg_size = 0;
		ret = target_read_u8(bank->target, EFM32_MSC_DI_PAGE_SIZE,
			&pg_size);
		if (ERROR_OK != ret)
			return ret;

		efm32_info->page_size = (1 << ((pg_size+10) & 0xff));

		if (efm32_info->part_family == EFM_FAMILY_ID_GIANT_GECKO ||
				efm32_info->part_family == EFM_FAMILY_ID_LEOPARD_GECKO) {
			/* Giant or Leopard Gecko */
			if (efm32_info->prod_rev < 18) {
				/* EFM32 GG/LG errata: MEM_INFO_PAGE_SIZE is invalid
				   for MCUs with PROD_REV < 18 */
				if (efm32_info->flash_sz_kib < 512)
					efm32_info->page_size = 2048;
				else
					efm32_info->page_size = 4096;
			}
		}

		if ((efm32_info->page_size != 2048) &&
				(efm32_info->page_size != 4096)) {
			LOG_ERROR("Invalid page size %u", efm32_info->page_size);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/*
 * Helper to create a human friendly string describing a part
 */
static int efm32x_decode_info(struct efm32_info *info, char *buf, int buf_size)
{
	int printed = 0;
	printed = snprintf(buf, buf_size, "%s Gecko, rev %d",
			info->family_data->name, info->prod_rev);

	if (printed >= buf_size)
		return ERROR_BUF_TOO_SMALL;

	return ERROR_OK;
}

/* flash bank efm32 <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(efm32x_flash_bank_command)
{
	struct efm32x_flash_bank *efm32x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	efm32x_info = malloc(sizeof(struct efm32x_flash_bank));

	bank->driver_priv = efm32x_info;
	efm32x_info->probed = 0;
	memset(efm32x_info->lb_page, 0xff, LOCKBITS_PAGE_SZ);

	return ERROR_OK;
}

/* set or reset given bits in a register */
static int efm32x_set_reg_bits(struct flash_bank *bank, uint32_t reg,
	uint32_t bitmask, int set)
{
	int ret = 0;
	uint32_t reg_val = 0;

	ret = efm32x_read_reg_u32(bank, reg, &reg_val);
	if (ERROR_OK != ret)
		return ret;

	if (set)
		reg_val |= bitmask;
	else
		reg_val &= ~bitmask;

	return efm32x_write_reg_u32(bank, reg, reg_val);
}

static int efm32x_set_wren(struct flash_bank *bank, int write_enable)
{
	return efm32x_set_reg_bits(bank, EFM32_MSC_REG_WRITECTRL,
		EFM32_MSC_WRITECTRL_WREN_MASK, write_enable);
}

static int efm32x_msc_lock(struct flash_bank *bank, int lock)
{
	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;
	return efm32x_write_reg_u32(bank, efm32x_info->reg_lock,
		(lock ? 0 : EFM32_MSC_LOCK_LOCKKEY));
}

static int efm32x_wait_status(struct flash_bank *bank, int timeout,
	uint32_t wait_mask, int wait_for_set)
{
	int ret = 0;
	uint32_t status = 0;

	while (1) {
		ret = efm32x_read_reg_u32(bank, EFM32_MSC_REG_STATUS, &status);
		if (ERROR_OK != ret)
			break;

		LOG_DEBUG("status: 0x%" PRIx32 "", status);

		if (((status & wait_mask) == 0) && (0 == wait_for_set))
			break;
		else if (((status & wait_mask) != 0) && wait_for_set)
			break;

		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for MSC status");
			return ERROR_FAIL;
		}

		alive_sleep(1);
	}

	if (status & EFM32_MSC_STATUS_ERASEABORTED_MASK)
		LOG_WARNING("page erase was aborted");

	return ret;
}

static int efm32x_erase_page(struct flash_bank *bank, uint32_t addr)
{
	/* this function DOES NOT set WREN; must be set already */
	/* 1. write address to ADDRB
	   2. write LADDRIM
	   3. check status (INVADDR, LOCKED)
	   4. write ERASEPAGE
	   5. wait until !STATUS_BUSY
	 */
	int ret = 0;
	uint32_t status = 0;
	addr += bank->base;
	LOG_DEBUG("erasing flash page at 0x%08" PRIx32, addr);

	ret = efm32x_write_reg_u32(bank, EFM32_MSC_REG_ADDRB, addr);
	if (ERROR_OK != ret)
		return ret;

	ret = efm32x_set_reg_bits(bank, EFM32_MSC_REG_WRITECMD,
		EFM32_MSC_WRITECMD_LADDRIM_MASK, 1);
	if (ERROR_OK != ret)
		return ret;

	ret = efm32x_read_reg_u32(bank, EFM32_MSC_REG_STATUS, &status);
	if (ERROR_OK != ret)
		return ret;

	LOG_DEBUG("status 0x%" PRIx32, status);

	if (status & EFM32_MSC_STATUS_LOCKED_MASK) {
		LOG_ERROR("Page is locked");
		return ERROR_FAIL;
	} else if (status & EFM32_MSC_STATUS_INVADDR_MASK) {
		LOG_ERROR("Invalid address 0x%" PRIx32, addr);
		return ERROR_FAIL;
	}

	ret = efm32x_set_reg_bits(bank, EFM32_MSC_REG_WRITECMD,
		EFM32_MSC_WRITECMD_ERASEPAGE_MASK, 1);
	if (ERROR_OK != ret)
		return ret;

	return efm32x_wait_status(bank, EFM32_FLASH_ERASE_TMO,
		EFM32_MSC_STATUS_BUSY_MASK, 0);
}

static int efm32x_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int i = 0;
	int ret = 0;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	efm32x_msc_lock(bank, 0);
	ret = efm32x_set_wren(bank, 1);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to enable MSC write");
		return ret;
	}

	for (i = first; i <= last; i++) {
		ret = efm32x_erase_page(bank, bank->sectors[i].offset);
		if (ERROR_OK != ret)
			LOG_ERROR("Failed to erase page %d", i);
	}

	ret = efm32x_set_wren(bank, 0);
	efm32x_msc_lock(bank, 1);

	return ret;
}

static int efm32x_read_lock_data(struct flash_bank *bank)
{
	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;
	struct target *target = bank->target;
	int i = 0;
	int data_size = 0;
	uint32_t *ptr = NULL;
	int ret = 0;

	assert(bank->num_sectors > 0);

	/* calculate the number of 32-bit words to read (one lock bit per sector) */
	data_size = (bank->num_sectors + 31) / 32;

	ptr = efm32x_info->lb_page;

	for (i = 0; i < data_size; i++, ptr++) {
		ret = target_read_u32(target, EFM32_MSC_LOCK_BITS+i*4, ptr);
		if (ERROR_OK != ret) {
			LOG_ERROR("Failed to read PLW %d", i);
			return ret;
		}
	}

	/* also, read ULW, DLW, MLW, ALW and CLW words */

	/* ULW, word 126 */
	ptr = efm32x_info->lb_page + 126;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS+126*4, ptr);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to read ULW");
		return ret;
	}

	/* DLW, word 127 */
	ptr = efm32x_info->lb_page + 127;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS+127*4, ptr);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to read DLW");
		return ret;
	}

	/* MLW, word 125, present in GG, LG, PG, JG, EFR32 */
	ptr = efm32x_info->lb_page + 125;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS+125*4, ptr);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to read MLW");
		return ret;
	}

	/* ALW, word 124, present in GG, LG, PG, JG, EFR32 */
	ptr = efm32x_info->lb_page + 124;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS+124*4, ptr);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to read ALW");
		return ret;
	}

	/* CLW1, word 123, present in EFR32 */
	ptr = efm32x_info->lb_page + 123;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS+123*4, ptr);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to read CLW1");
		return ret;
	}

	/* CLW0, word 122, present in GG, LG, PG, JG, EFR32 */
	ptr = efm32x_info->lb_page + 122;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS+122*4, ptr);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to read CLW0");
		return ret;
	}

	return ERROR_OK;
}

static int efm32x_write_lock_data(struct flash_bank *bank)
{
	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;
	int ret = 0;

	ret = efm32x_erase_page(bank, EFM32_MSC_LOCK_BITS);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to erase LB page");
		return ret;
	}

	return efm32x_write(bank, (uint8_t *)efm32x_info->lb_page, EFM32_MSC_LOCK_BITS,
		LOCKBITS_PAGE_SZ);
}

static int efm32x_get_page_lock(struct flash_bank *bank, size_t page)
{
	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;
	uint32_t dw = efm32x_info->lb_page[page >> 5];
	uint32_t mask = 0;

	mask = 1 << (page & 0x1f);

	return (dw & mask) ? 0 : 1;
}

static int efm32x_set_page_lock(struct flash_bank *bank, size_t page, int set)
{
	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;
	uint32_t *dw = &efm32x_info->lb_page[page >> 5];
	uint32_t mask = 0;

	mask = 1 << (page & 0x1f);

	if (!set)
		*dw |= mask;
	else
		*dw &= ~mask;

	return ERROR_OK;
}

static int efm32x_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	int i = 0;
	int ret = 0;

	if (!set) {
		LOG_ERROR("Erase device data to reset page locks");
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (i = first; i <= last; i++) {
		ret = efm32x_set_page_lock(bank, i, set);
		if (ERROR_OK != ret) {
			LOG_ERROR("Failed to set lock on page %d", i);
			return ret;
		}
	}

	ret = efm32x_write_lock_data(bank);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to write LB page");
		return ret;
	}

	return ERROR_OK;
}

static int efm32x_write_block(struct flash_bank *bank, const uint8_t *buf,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;
	int ret = ERROR_OK;

	/* see contrib/loaders/flash/efm32/efm32.S for src */
	static const uint8_t efm32x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/efm32/efr32_s2.inc"
	};


	/* flash write code */
	if (target_alloc_working_area(target, sizeof(efm32x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

    LOG_INFO("Writing flash buffer to address 0x%08lx", write_algorithm->address);
	ret = target_write_buffer(target, write_algorithm->address,
			sizeof(efm32x_flash_write_code), efm32x_flash_write_code);
	if (ret != ERROR_OK) {
        LOG_ERROR("Error writing flash code: %d", ret);
		return ret;
    }

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}
    LOG_INFO("Preparing target registers: r0=0x%08X, r1=0x%08X, r2=0x%lX, r3=0x%lX, r4=0x%08X", efm32x_info->reg_base, count, source->address, source->address + source->size, address);

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (word-32bit) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, efm32x_info->reg_base);
	buf_set_u32(reg_params[1].value, 0, 32, count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;
    LOG_INFO("Running flash algo");

	ret = target_run_flash_async_algorithm(target, buf, count, 4,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);
    LOG_INFO("Flash result: %d", ret);
	if (ret == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("flash write failed at address 0x%"PRIx32,
				buf_get_u32(reg_params[4].value, 0, 32));

		if (buf_get_u32(reg_params[0].value, 0, 32) &
				EFM32_MSC_STATUS_LOCKED_MASK) {
			LOG_ERROR("flash memory write protected");
		}

		if (buf_get_u32(reg_params[0].value, 0, 32) &
				EFM32_MSC_STATUS_INVADDR_MASK) {
			LOG_ERROR("invalid flash memory write address");
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return ret;
}

static int efm32x_write_word(struct flash_bank *bank, uint32_t addr,
	uint32_t val)
{
	/* this function DOES NOT set WREN; must be set already */
	/* 1. write address to ADDRB
	   2. write LADDRIM
	   3. check status (INVADDR, LOCKED)
	   4. wait for WDATAREADY
	   5. write data to WDATA
	   6. write WRITECMD_WRITEONCE to WRITECMD
	   7. wait until !STATUS_BUSY
	 */

	/* FIXME: EFM32G ref states (7.3.2) that writes should be
	 * performed twice per dword */

	int ret = 0;
	uint32_t status = 0;

	/* if not called, GDB errors will be reported during large writes */
	keep_alive();

	ret = efm32x_write_reg_u32(bank, EFM32_MSC_REG_ADDRB, addr);
	if (ERROR_OK != ret)
		return ret;

/*	ret = efm32x_set_reg_bits(bank, EFM32_MSC_REG_WRITECMD,
		EFM32_MSC_WRITECMD_LADDRIM_MASK, 1);
	if (ERROR_OK != ret)
		return ret;*/

	ret = efm32x_read_reg_u32(bank, EFM32_MSC_REG_STATUS, &status);
	if (ERROR_OK != ret)
		return ret;

	LOG_DEBUG("status 0x%" PRIx32, status);

	if (status & EFM32_MSC_STATUS_LOCKED_MASK) {
		LOG_ERROR("Page is locked");
		return ERROR_FAIL;
	} else if (status & EFM32_MSC_STATUS_INVADDR_MASK) {
		LOG_ERROR("Invalid address 0x%" PRIx32, addr);
		return ERROR_FAIL;
	}

	ret = efm32x_wait_status(bank, EFM32_FLASH_WDATAREADY_TMO,
		EFM32_MSC_STATUS_WDATAREADY_MASK, 1);
	if (ERROR_OK != ret) {
		LOG_ERROR("Wait for WDATAREADY failed");
		return ret;
	}

	ret = efm32x_write_reg_u32(bank, EFM32_MSC_REG_WDATA, val);
	if (ERROR_OK != ret) {
		LOG_ERROR("WDATA write failed");
		return ret;
	}

	ret = efm32x_write_reg_u32(bank, EFM32_MSC_REG_WRITECMD,
		EFM32_MSC_WRITECMD_WRITEONCE_MASK);
	if (ERROR_OK != ret) {
		LOG_ERROR("WRITECMD write failed");
		return ret;
	}

	ret = efm32x_wait_status(bank, EFM32_FLASH_WRITE_TMO,
		EFM32_MSC_STATUS_BUSY_MASK, 0);
	if (ERROR_OK != ret) {
		LOG_ERROR("Wait for BUSY failed");
		return ret;
	}

	return ERROR_OK;
}

static int efm32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint8_t *new_buffer = NULL;
    LOG_INFO("writing %" PRIu32 " bytes to %" PRIu32, count, offset);
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x3) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-byte "
			"alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (count & 0x3) {
		uint32_t old_count = count;
		count = (old_count | 3) + 1;
		new_buffer = malloc(count);
		if (new_buffer == NULL) {
			LOG_ERROR("odd number of bytes to write and no memory "
				"for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("odd number of bytes to write (%" PRIu32 "), extending to %" PRIu32 " "
			"and padding with 0xff", old_count, count);
		memset(new_buffer, 0xff, count);
		buffer = memcpy(new_buffer, buffer, old_count);
	}

	uint32_t words_remaining = count / 4;
	int retval, retval2;
    LOG_INFO("Unlocking flash and setting WREN");
	/* unlock flash registers */
	efm32x_msc_lock(bank, 0);
	retval = efm32x_set_wren(bank, 1);
	if (retval != ERROR_OK) {
        LOG_ERROR("Unable to set WREN");
		goto cleanup;
    }

	/* try using a block write */
    LOG_INFO("Calling write_block");
  	retval = efm32x_write_block(bank, buffer, offset, words_remaining);
    LOG_INFO("write_block retval: %d", retval);

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single word accesses */
		LOG_WARNING("couldn't use block writes, falling back to single "
			"memory accesses");

		while (words_remaining > 0) {
			uint32_t value;
			memcpy(&value, buffer, sizeof(uint32_t));

			retval = efm32x_write_word(bank, offset, value);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			words_remaining--;
			buffer += 4;
			offset += 4;
		}
	}

reset_pg_and_lock:
	retval2 = efm32x_set_wren(bank, 0);
	efm32x_msc_lock(bank, 1);
	if (retval == ERROR_OK)
		retval = retval2;

cleanup:
	if (new_buffer)
		free(new_buffer);

	return retval;
}

static int efm32x_probe(struct flash_bank *bank)
{
	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;
	struct efm32_info efm32_mcu_info;
	int ret;
	int i;
	uint32_t base_address = 0x00000000;
	char buf[256];

	efm32x_info->probed = 0;
	memset(efm32x_info->lb_page, 0xff, LOCKBITS_PAGE_SZ);

	ret = efm32x_read_info(bank, &efm32_mcu_info);
	if (ERROR_OK != ret)
		return ret;

	ret = efm32x_decode_info(&efm32_mcu_info, buf, sizeof(buf));
	if (ERROR_OK != ret)
		return ret;

	LOG_INFO("detected part: %s", buf);
	LOG_INFO("flash size = %dkbytes", efm32_mcu_info.flash_sz_kib);
	LOG_INFO("flash page size = %dbytes", efm32_mcu_info.page_size);

	assert(0 != efm32_mcu_info.page_size);

	int num_pages = efm32_mcu_info.flash_sz_kib * 1024 /
		efm32_mcu_info.page_size;

	assert(num_pages > 0);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->size = (num_pages * efm32_mcu_info.page_size);
	bank->num_sectors = num_pages;

	ret = efm32x_read_lock_data(bank);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to read LB data");
		return ret;
	}

	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].offset = i * efm32_mcu_info.page_size;
		bank->sectors[i].size = efm32_mcu_info.page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	efm32x_info->probed = 1;

	return ERROR_OK;
}

static int efm32x_auto_probe(struct flash_bank *bank)
{
	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;
	if (efm32x_info->probed)
		return ERROR_OK;
	return efm32x_probe(bank);
}

static int efm32x_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int ret = 0;
	int i = 0;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	ret = efm32x_read_lock_data(bank);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to read LB data");
		return ret;
	}

	assert(NULL != bank->sectors);

	for (i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = efm32x_get_page_lock(bank, i);

	return ERROR_OK;
}

static int get_efm32x_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct efm32_info info;
	int ret = 0;

	ret = efm32x_read_info(bank, &info);
	if (ERROR_OK != ret) {
		LOG_ERROR("Failed to read EFM32 info");
		return ret;
	}

	return efm32x_decode_info(&info, buf, buf_size);
}

COMMAND_HANDLER(efm32x_handle_debuglock_command)
{
	struct target *target = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct efm32x_flash_bank *efm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t *ptr;
	ptr = efm32x_info->lb_page + 127;
	*ptr = 0;

	retval = efm32x_write_lock_data(bank);
	if (ERROR_OK != retval) {
		LOG_ERROR("Failed to write LB page");
		return retval;
	}

	command_print(CMD, "efm32x debug interface locked, reset the device to apply");

	return ERROR_OK;
}

static const struct command_registration efm32x_exec_command_handlers[] = {
	{
		.name = "debuglock",
		.handler = efm32x_handle_debuglock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock the debug interface of the device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration efm32x_command_handlers[] = {
	{
		.name = "efm32",
		.mode = COMMAND_ANY,
		.help = "efm32 flash command group",
		.usage = "",
		.chain = efm32x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver efm32_flash = {
	.name = "efm32",
	.commands = efm32x_command_handlers,
	.flash_bank_command = efm32x_flash_bank_command,
	.erase = efm32x_erase,
	.protect = efm32x_protect,
	.write = efm32x_write,
	.read = default_flash_read,
	.probe = efm32x_probe,
	.auto_probe = efm32x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = efm32x_protect_check,
	.info = get_efm32x_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
