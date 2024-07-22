/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
 *                                                                         *
 *                                                                         *
 *   Cortex-M3(tm) TRM, ARM DDI 0337E (r1p1) and 0337G (r2p0)              *
 *                                                                         *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif


#include "target.h"
#include "jtag/jtag.h"
#include "algorithm.h"

#include "jtag/interface.h"
#include "jtag/hla/hla_transport.h"
#include "jtag/hla/hla_interface.h"
#include "jtag/hla/hla_layout.h"
#include "breakpoints.h"
#include "numicro8051.h"
#include "target_request.h"
#include "target_type.h"
#include "arm_disassembler.h"
#include "register.h"
#include "arm_opcodes.h"
#include "arm_semihosting.h"
#include <helper/time_support.h>

/* NOTE:  most of this should work fine for the Cortex-M1 and
 * Cortex-M0 cores too, although they're ARMv6-M not ARMv7-M.
 * Some differences:  M0/M1 doesn't have FBP remapping or the
 * DWT tracing/profiling support.  (So the cycle counter will
 * not be usable; the other stuff isn't currently used here.)
 *
 * Although there are some workarounds for errata seen only in r0p0
 * silicon, such old parts are hard to find and thus not much tested
 * any longer.
 */

/**
 * Returns the type of a break point required by address location
 */
 
static struct reg_cache *numicro8051_build_reg_cache(struct target *target);
static int numicro8051_read_core_reg(struct target *target, unsigned int num);
static int numicro8051_write_core_reg(struct target *target, unsigned int num);

extern int nulink_8051_read_core_regs_all(void *handle, uint32_t addr, uint32_t count, uint32_t *buf);
extern int nulink_set_breakpoint(void *handle, uint32_t addr, uint32_t index);
extern int nulink_clr_breakpoint(void *handle, uint32_t addr, uint32_t index);
extern uint32_t m_stop_pc;
uint32_t g_bp_count = 0;

static const struct {
	unsigned id;
	const char *name;
	const uint8_t bits;
	enum reg_type type;
	const char *group;
	const char *feature;
	int flag;
} numicro8051_regs[] = {
	{  0,  "psw", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  1,  "sp", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  2,  "b", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  3,  "acc", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  4,  "dpl", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  5,  "dph", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  6,  "r0", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  7,  "r1", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  8,  "r2", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  9,  "r3", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  10,  "r4", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  11,  "r5", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  12,  "r6", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  13,  "r7", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.nu8051.core", 0 },
	{  14,  "pc", 32, REG_TYPE_UINT32, "general", "org.gnu.gdb.nu8051.core", 0 },
};

//#define BKPT_TYPE_BY_ADDR(addr) ((addr) < 0x20000000 ? BKPT_HARD : BKPT_SOFT)
#define NUMICRO8051_NUM_REGS ARRAY_SIZE(numicro8051_regs)
#define NUMICRO8051_PSW 0
#define NUMICRO8051_SP 1
#define NUMICRO8051_B 2
#define NUMICRO8051_ACC 3
#define NUMICRO8051_DPL 4
#define NUMICRO8051_DPH 5
#define NUMICRO8051_R0 6
#define NUMICRO8051_R1 7
#define NUMICRO8051_R2 8
#define NUMICRO8051_R3 9
#define NUMICRO8051_R4 10
#define NUMICRO8051_R5 11
#define NUMICRO8051_R6 12
#define NUMICRO8051_R7 13
#define NUMICRO8051_PC 14

struct numicro8051_algorithm {
	int common_magic;
};

struct numicro8051_core_reg {
	uint32_t num;
	struct target *target;
	struct numicro8051_common *numicro8051_common;
};

enum hw_break_type {
	/* break on execute */
	HWBRK_EXEC,
	/* break on read */
	HWBRK_RD,
	/* break on write */
	HWBRK_WR,
	/* break on read, write and execute */
	HWBRK_ACC
};

struct numicro8051_comparator {
	bool used;
	uint32_t bp_value;
	uint32_t reg_address;
	enum hw_break_type type;
};

int numicro8051_poll(struct target *target)
{
	uint32_t i;
	enum target_state state;
	struct hl_interface_s *adapter = target->tap->priv;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	enum target_state prev_target_state = target->state;

	state = adapter->layout->api->state(adapter->handle);

	if (state == TARGET_UNKNOWN) {
		LOG_ERROR("jtag status contains invalid mode value - communication failure");
		return ERROR_TARGET_FAILURE;
	}

	numicro8051->core_regs[NUMICRO8051_PC] = m_stop_pc;

	for (i = 0; i < NUMICRO8051_NUM_REGS; i++) {
		//if (!numicro8051->core_cache->reg_list[i].valid)
		numicro8051->read_core_reg(target, i);
	}

	if (prev_target_state == state)
	{
		return ERROR_OK;
	}

	target->state = state;

	if (state == TARGET_HALTED) {
		nulink_8051_read_core_regs_all(adapter->handle, 0, NUMICRO8051_NUM_REGS-1, numicro8051->core_regs);
		for (i = 0; i < NUMICRO8051_NUM_REGS; i++) {
			//if (!numicro8051->core_cache->reg_list[i].valid)
			numicro8051->read_core_reg(target, i);
		}		
		LOG_DEBUG("entered debug state at PC 0x%" PRIx32 ", target->state: %s",
			buf_get_u32(numicro8051->core_cache->reg_list[NUMICRO8051_PC].value, 0, 32),
			target_state_name(target));

		if (prev_target_state == TARGET_DEBUG_RUNNING) {
			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		} else {
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
		LOG_DEBUG("halted: PC: 0x%08" PRIx32, buf_get_u32(numicro8051->core_cache->reg_list[NUMICRO8051_PC].value, 0, 32));
	}

	return ERROR_OK;
}

int numicro8051_halt(struct target *target)
{
	int retval;

	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
		LOG_WARNING("target was in unknown state when halt was requested");

	if (target->state == TARGET_RESET) {
		if ((jtag_get_reset_config() & RESET_SRST_PULLS_TRST) && jtag_get_srst()) {
			LOG_ERROR("can't request a halt while in reset if nSRST pulls nTRST");
			return ERROR_TARGET_FAILURE;
		} else {
			/* we came here in a reset_halt or reset_init sequence
			 * debug entry was already prepared in numicro80513_assert_reset()
			 */
			target->debug_reason = DBG_REASON_DBGRQ;

			return ERROR_OK;
		}
	}

	struct hl_interface_s *adapter = target->tap->priv;
	retval = adapter->layout->api->halt(adapter->handle);	
	if (retval != ERROR_OK)
		return retval;

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int numicro8051_restore_context(struct target *target)
{
	unsigned int i;

	/* get pointers to arch-specific information */
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;

	nulink_8051_read_core_regs_all(adapter->handle, 0, NUMICRO8051_NUM_REGS-1, numicro8051->core_regs);
	numicro8051->core_regs[NUMICRO8051_PC] = m_stop_pc;

	for (i = 0; i < NUMICRO8051_NUM_REGS; i++) {
		numicro8051->read_core_reg(target, i);
	}

	return ERROR_OK;
}

int numicro8051_resume(struct target *target, int current,
	uint32_t address, int handle_breakpoints, int debug_execution)
{
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;
	struct breakpoint *breakpoint = NULL;
	uint32_t resume_pc;
	uint32_t retval, i;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (i = 0; i < NUMICRO8051_NUM_REGS; i++) {
		numicro8051->read_core_reg(target, i);
	}

	if (!current) {
		buf_set_u32(numicro8051->core_cache->reg_list[NUMICRO8051_PC].value,
			0, 32, address);
		numicro8051->core_cache->reg_list[NUMICRO8051_PC].dirty = true;
		numicro8051->core_cache->reg_list[NUMICRO8051_PC].valid = true;
	}

	if (!current) {
		resume_pc = address;
	}
	else {
		resume_pc = buf_get_u32(
			numicro8051->core_cache->reg_list[NUMICRO8051_PC].value,
			0, 32);
	}

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			numicro8051_remove_breakpoint(target, breakpoint);
			retval = adapter->layout->api->step(adapter->handle);
			if (retval != ERROR_OK)
				return retval;
			numicro8051_add_breakpoint(target, breakpoint);
		}
	}

	retval = adapter->layout->api->run(adapter->handle);

	numicro8051_restore_context(target);

	if (retval != ERROR_OK)
		return retval;

	target->debug_reason = DBG_REASON_NOTHALTED;

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIx32 "", resume_pc);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIx32 "", resume_pc);
	}

	return ERROR_OK;
}

int numicro8051_step(struct target *target, int current,
	uint32_t address, int handle_breakpoints)
{
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;
	struct breakpoint *breakpoint = NULL;
	uint32_t retval, i;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (i = 0; i < NUMICRO8051_NUM_REGS; i++) {
		numicro8051->read_core_reg(target, i);
	}

	if (!current)
		buf_set_u32(numicro8051->core_cache->reg_list[NUMICRO8051_PC].value, 0, 32, address);

	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, buf_get_u32(numicro8051->core_cache->reg_list[NUMICRO8051_PC].value, 0, 32));
		if (breakpoint) {
			numicro8051_remove_breakpoint(target, breakpoint);
		}
	}

	target->debug_reason = DBG_REASON_SINGLESTEP;

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	retval = adapter->layout->api->step(adapter->handle);

	numicro8051_restore_context(target);

	if (retval != ERROR_OK)
		return retval;

	if (breakpoint)
		numicro8051_add_breakpoint(target, breakpoint);

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	return ERROR_OK;
}

int numicro8051_assert_reset(struct target *target)
{
	int retval;

	struct hl_interface_s *adapter = target->tap->priv;
	retval = adapter->layout->api->reset(adapter->handle);
	if (retval != ERROR_OK)
		return retval;

	if (target->reset_halt) {
		target->state = TARGET_RESET;
		target->debug_reason = DBG_REASON_DBGRQ;
	} else {
		target->state = TARGET_HALTED;
	}

	return ERROR_OK;
}

int numicro8051_deassert_reset(struct target *target)
{
	int retval;

	struct hl_interface_s *adapter = target->tap->priv;
	retval = adapter->layout->api->reset(adapter->handle);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

int numicro8051_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	int i;

	if (reg_class == REG_CLASS_ALL)
		*reg_list_size = numicro8051->core_cache->num_regs;
	else
		*reg_list_size = NUMICRO8051_NUM_CORE_REGS;

	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));
	if (*reg_list == NULL)
		return ERROR_FAIL;

	for (i = 0; i < *reg_list_size; i++)
		(*reg_list)[i] = &numicro8051->core_cache->reg_list[i];

	return ERROR_OK;
}

int numicro8051_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;
	int i, retval;

	for (i = 0; i < BP_MAX; i++)
	{
		if (numicro8051->bp_check[i] == false)
		{
			g_bp_count = g_bp_count + 1;
			retval = adapter->layout->api->halt(adapter->handle);
			if (retval != ERROR_OK)
				return retval;

			numicro8051->bp_check[i] = true;
			numicro8051->bp_addr[i] = breakpoint->address & 0xFFFF;

			retval = nulink_set_breakpoint(adapter->handle, (breakpoint->address & 0xFFFF), g_bp_count);
			if (retval != ERROR_OK)
				return retval;

			break;
		}
	}

	if (i >= BP_MAX)
	{
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return ERROR_OK;
}

int numicro8051_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;
	int i, retval;

	for (i = 0; i < BP_MAX; i++)
	{
		if (numicro8051->bp_check[i] && ((breakpoint->address & 0xFFFF) == numicro8051->bp_addr[i]))
		{
			numicro8051->bp_check[i] = false;
			numicro8051->bp_addr[i] = 0x0;

			retval = adapter->layout->api->halt(adapter->handle);
			if (retval != ERROR_OK)
				return retval;

			retval = nulink_clr_breakpoint(adapter->handle, 0, 0);
			g_bp_count = 0;
			if (retval != ERROR_OK)
				return retval;

			for (int j = 0; j < BP_MAX; j++)
			{
				if (numicro8051->bp_check[j])
				{
					g_bp_count = g_bp_count + 1;
					retval = nulink_set_breakpoint(adapter->handle, (numicro8051->bp_addr[j] & 0xFFFF), g_bp_count);
					if (retval != ERROR_OK)
						return retval;
				}
			}

			break;
		}
	}
	return ERROR_OK;
}

#define REG_SFRPAGE			0x91
#define N76E_SCODE_ADDR 0x4800

int ReadData(struct target *target, uint32_t nAdr, uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_OK;
	unsigned long uPage_r, uAddr_r, uLen_r;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	struct hl_interface_s *adapter = target->tap->priv;
	uint32_t nSize = size * count;

	while (nSize)
	{
		uPage_r	= (nAdr >> 8) & 0xFFFF;
		uAddr_r	= (nAdr & 0xFF);
		uLen_r	= nSize;

		if ((uAddr_r + uLen_r) > 0x100)
			uLen_r = 0x100 - uAddr_r;

		if (uPage_r == 0)
		{
			retval = adapter->layout->api->read_mem(adapter->handle, (uAddr_r | 0x20000000), 1, uLen_r, (uint8_t *)buffer);
			if(retval != ERROR_OK)
				return retval;
			numicro8051->uReadMemCount += uLen_r;
		}
		else if (numicro8051->uSFR_PageNum > 0)
		{
			uPage_r &= ~0x0100;

			if (uPage_r < numicro8051->uSFR_PageNum)
			{
				unsigned char ucPage_Store;
				uint32_t val;

				retval = adapter->layout->api->read_reg(adapter->handle, (REG_SFRPAGE | 0x20000000), &val);
				if(retval != ERROR_OK)
					return retval;
				ucPage_Store = (unsigned char)val;

				if (uPage_r != ucPage_Store) {
					retval = adapter->layout->api->write_reg(adapter->handle, (REG_SFRPAGE | 0x20000000), uPage_r);
					if(retval != ERROR_OK)
						return retval;					
				}

				retval = adapter->layout->api->read_mem(adapter->handle, (uAddr_r | 0x20000000), 1, uLen_r, (uint8_t *)buffer);
				if(retval != ERROR_OK)
					return retval;
				numicro8051->uReadMemCount += uLen_r;

				if ((uAddr_r <= REG_SFRPAGE) && ((uAddr_r + uLen_r) > REG_SFRPAGE))
				{
					buffer[REG_SFRPAGE - uAddr_r] = ucPage_Store;
				}

				if (uPage_r != ucPage_Store) {
					retval = adapter->layout->api->write_reg(adapter->handle, (REG_SFRPAGE | 0x20000000), ucPage_Store);
					if(retval != ERROR_OK)
						return retval;
				}
			}
			else
			{
				memset(buffer, 0x00, uLen_r);
			}
		}
		else
		{
			memset(buffer, 0x00, uLen_r);
		}

		buffer		+= uLen_r;
		nAdr	+= uLen_r;
		nSize	-= uLen_r;
	}

	//---TODO:
	//---if Ok, then return 0, else error-address
	//   adr |= (amDATA << 24);     // make address uVision2 conforming
	return retval;						// say Ok.	
}

int ReadIdata(struct target *target, uint32_t nAdr, uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_OK;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;
	uint8_t ucIData[256];
	uint32_t i, uAddr;
	uint32_t nSize = size * count;

	retval = adapter->layout->api->read_mem(adapter->handle, (0x00 | 0x20000000), 4, 0x20, (uint8_t *)&ucIData[0x00]);
	if(retval != ERROR_OK)
		return retval;
	retval = adapter->layout->api->read_mem(adapter->handle, (0x80 | 0x10000000), 4, 0x20, (uint8_t *)&ucIData[0x80]);
	if(retval != ERROR_OK)
		return retval;

	numicro8051->uReadMemCount += 0x100;

	for (i = 0, uAddr = nAdr; i < nSize; i++, uAddr++) {
		buffer[i] = ucIData[(uAddr & 0xFF)];
	}

    //---TODO:
    //---if Ok, then return 0, else error-address
    //   adr |= (amIDATA << 24);    // make address uVision2 conforming
	return retval;						// say Ok.
}

int ReadXdata(struct target *target, uint32_t nAdr, uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_OK;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;

	unsigned int uAddr_x = nAdr, uAddr_t = numicro8051->uSFR_TKAddr;
	unsigned int uLen_x = 0, uLen_ex = 0, uLen_t = 0, uLen_et = 0;
	uint32_t nSize = size * count;

	if (numicro8051->uSFR_TKSize > 0) {
		if (nAdr < numicro8051->uXRAMSize) {
			if ((nAdr + nSize) > (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize)) {
				uLen_x	= numicro8051->uXRAMSize - nAdr;
				uLen_ex	= numicro8051->uSFR_TKAddr - numicro8051->uXRAMSize;
				uLen_t	= numicro8051->uSFR_TKSize;
				uLen_et	= (nAdr + nSize) - (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize);
			}
			else if ((nAdr + nSize) > numicro8051->uSFR_TKAddr) {
				uLen_x	= numicro8051->uXRAMSize - nAdr;
				uLen_ex	= numicro8051->uSFR_TKAddr - numicro8051->uXRAMSize;
				uLen_t	= (nAdr + nSize) - numicro8051->uSFR_TKAddr;
				uLen_et	= 0;
			}
			else if ((nAdr + nSize) > numicro8051->uXRAMSize) {
				uLen_x	= numicro8051->uXRAMSize - nAdr;
				uLen_ex	= (nAdr + nSize) - numicro8051->uXRAMSize;
				uLen_t	= 0;
				uLen_et	= 0;
			}
			else {
				uLen_x	= nSize;
				uLen_ex	= 0;
				uLen_t	= 0;
				uLen_et	= 0;
			}
		}
		else if (nAdr < numicro8051->uSFR_TKAddr) {
			if ((nAdr + nSize) > (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize)) {
				uLen_x	= 0;
				uLen_ex	= numicro8051->uSFR_TKAddr - nAdr;
				uLen_t	= numicro8051->uSFR_TKSize;
				uLen_et	= (nAdr + nSize) - (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize);
			}
			else if ((nAdr + nSize) > numicro8051->uSFR_TKAddr) {
				uLen_x	= 0;
				uLen_ex	= numicro8051->uSFR_TKAddr - nAdr;
				uLen_t	= (nAdr + nSize) - numicro8051->uSFR_TKAddr;
				uLen_et	= 0;
			}
			else {
				uLen_x	= 0;
				uLen_ex	= nSize;
				uLen_t	= 0;
				uLen_et	= 0;
			}
		}
		else if (nAdr < (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize)) {
			uAddr_t = nAdr;

			if ((nAdr + nSize) > (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize)) {
				uLen_x	= 0;
				uLen_ex	= 0;
				uLen_t	= (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize) - nAdr;
				uLen_et	= (nAdr + nSize) - (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize);
			}
			else {
				uLen_x	= 0;
				uLen_ex	= 0;
				uLen_t	= nSize;
				uLen_et	= 0;
			}
		}
		else {
			uLen_x	= 0;
			uLen_ex	= 0;
			uLen_t	= 0;
			uLen_et	= nSize;
		}
	}
	else {
		if (nAdr < numicro8051->uXRAMSize) {
			if ((nAdr + nSize) > numicro8051->uXRAMSize) {
				uLen_x	= numicro8051->uXRAMSize - nAdr;
				uLen_ex	= (nAdr + nSize) - numicro8051->uXRAMSize;
			}
			else {
				uLen_x	= nSize;
				uLen_ex	= 0;
			}
		}
		else {
			uLen_x	= 0;
			uLen_ex	= nSize;
		}
	}

	if (uLen_x) {
		retval = adapter->layout->api->read_mem(adapter->handle, (uAddr_x | 0x40000000), 1, uLen_x, (uint8_t *)buffer);
		if(retval != ERROR_OK)
			return retval;
		buffer += uLen_x;
		numicro8051->uReadMemCount += uLen_x;
	}

	if (uLen_ex) {
		memset(buffer, 0x00, uLen_ex);
		buffer += uLen_ex;
	}

	if (uLen_t) {
		retval = adapter->layout->api->read_mem(adapter->handle, (uAddr_t | 0x40000000), 1, uLen_t, (uint8_t *)buffer);
		if(retval != ERROR_OK)
			return retval;
		buffer += uLen_t;
		numicro8051->uReadMemCount += uLen_t;
	}

	if (uLen_et) {
		memset(buffer, 0x00, uLen_et);
		buffer += uLen_et;
	}

	//---TODO:
	//---if Ok, then return 0, else error-address
	//   adr |= (amIDATA << 24);	// make address uVision2 conforming
	return retval;						// say Ok.
}

int ReadCode(struct target *target, uint32_t nAdr, uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_OK;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;
	uint32_t nSize = size * count;

	unsigned int uAddr_c = nAdr, uAddr_l = numicro8051->uLIBROMAddr, uAddr_s = numicro8051->uSCodeAddr;
	unsigned int uLen_c = 0, uLen_ec = 0, uLen_l = 0, uLen_el = 0, uLen_s = 0, uLen_es = 0;

	if (numicro8051->uSCodeAddr >= numicro8051->uProgramFlashSize)
	{
		if (numicro8051->uLIBROMSize > 0)
		{
			if (nAdr < numicro8051->uProgramFlashSize)
			{
				if ((nAdr + nSize) > (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
				{
					uLen_c	= numicro8051->uProgramFlashSize - nAdr;
					uLen_ec	= numicro8051->uLIBROMAddr - numicro8051->uProgramFlashSize;
					uLen_l	= numicro8051->uLIBROMSize;
					uLen_el	= numicro8051->uSCodeAddr - (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize);
					uLen_s	= numicro8051->uSPROMSize;
					uLen_es	= (nAdr + nSize) - (numicro8051->uSCodeAddr + numicro8051->uSPROMSize);
				}
				else if ((nAdr + nSize) > numicro8051->uSCodeAddr)
				{
					uLen_c	= numicro8051->uProgramFlashSize - nAdr;
					uLen_ec	= numicro8051->uLIBROMAddr - numicro8051->uProgramFlashSize;
					uLen_l	= numicro8051->uLIBROMSize;
					uLen_el	= numicro8051->uSCodeAddr - (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize);
					uLen_s	= (nAdr + nSize) - numicro8051->uSCodeAddr;
					uLen_es	= 0;
				}
				else if ((nAdr + nSize) > (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize))
				{
					uLen_c	= numicro8051->uProgramFlashSize - nAdr;
					uLen_ec	= numicro8051->uLIBROMAddr - numicro8051->uProgramFlashSize;
					uLen_l	= numicro8051->uLIBROMSize;
					uLen_el	= (nAdr + nSize) - (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize);
					uLen_s	= 0;
					uLen_es	= 0;
				}
				else if ((nAdr + nSize) > numicro8051->uLIBROMAddr)
				{
					uLen_c	= numicro8051->uProgramFlashSize - nAdr;
					uLen_ec	= numicro8051->uLIBROMAddr - numicro8051->uProgramFlashSize;
					uLen_l	= (nAdr + nSize) - numicro8051->uLIBROMAddr;
					uLen_el	= 0;
					uLen_s	= 0;
					uLen_es	= 0;
				}
				else if ((nAdr + nSize) > numicro8051->uProgramFlashSize)
				{
					uLen_c	= numicro8051->uProgramFlashSize - nAdr;
					uLen_ec	= (nAdr + nSize) - numicro8051->uProgramFlashSize;
					uLen_l	= 0;
					uLen_el	= 0;
					uLen_s	= 0;
					uLen_es	= 0;
				}
				else
				{
					uLen_c	= nSize;
					uLen_ec	= 0;
					uLen_l	= 0;
					uLen_el	= 0;
					uLen_s	= 0;
					uLen_es	= 0;
				}
			}
			else if (nAdr < numicro8051->uLIBROMAddr)
			{
				if ((nAdr + nSize) > (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
				{
					uLen_c	= 0;
					uLen_ec	= numicro8051->uLIBROMAddr - nAdr;
					uLen_l	= numicro8051->uLIBROMSize;
					uLen_el	= numicro8051->uSCodeAddr - (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize);
					uLen_s	= numicro8051->uSPROMSize;
					uLen_es	= (nAdr + nSize) - (numicro8051->uSCodeAddr + numicro8051->uSPROMSize);
				}
				else if ((nAdr + nSize) > numicro8051->uSCodeAddr)
				{
					uLen_c	= 0;
					uLen_ec	= numicro8051->uLIBROMAddr - nAdr;
					uLen_l	= numicro8051->uLIBROMSize;
					uLen_el	= numicro8051->uSCodeAddr - (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize);
					uLen_s	= (nAdr + nSize) - numicro8051->uSCodeAddr;
					uLen_es	= 0;
				}
				else if ((nAdr + nSize) > (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize))
				{
					uLen_c	= 0;
					uLen_ec	= numicro8051->uLIBROMAddr - nAdr;
					uLen_l	= numicro8051->uLIBROMSize;
					uLen_el	= (nAdr + nSize) - (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize);
					uLen_s	= 0;
					uLen_es	= 0;
				}
				else if ((nAdr + nSize) > numicro8051->uLIBROMAddr)
				{
					uLen_c	= 0;
					uLen_ec	= numicro8051->uLIBROMAddr - nAdr;
					uLen_l	= (nAdr + nSize) - numicro8051->uLIBROMAddr;
					uLen_el	= 0;
					uLen_s	= 0;
					uLen_es	= 0;
				}
				else
				{
					uLen_c	= 0;
					uLen_ec	= nSize;
					uLen_l	= 0;
					uLen_el	= 0;
					uLen_s	= 0;
					uLen_es	= 0;
				}
			}
			else if (nAdr < (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize))
			{
				uAddr_l = nAdr;

				if ((nAdr + nSize) > (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
				{
					uLen_c	= 0;
					uLen_ec	= 0;
					uLen_l	= (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize) - nAdr;
					uLen_el	= numicro8051->uSCodeAddr - (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize);
					uLen_s	= numicro8051->uSPROMSize;
					uLen_es	= (nAdr + nSize) - (numicro8051->uSCodeAddr + numicro8051->uSPROMSize);
				}
				else if ((nAdr + nSize) > numicro8051->uSCodeAddr)
				{
					uLen_c	= 0;
					uLen_ec	= 0;
					uLen_l	= (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize) - nAdr;
					uLen_el	= numicro8051->uSCodeAddr - (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize);
					uLen_s	= (nAdr + nSize) - numicro8051->uSCodeAddr;
					uLen_es	= 0;
				}
				else if ((nAdr + nSize) > (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize))
				{
					uLen_c	= 0;
					uLen_ec	= 0;
					uLen_l	= (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize) - nAdr;
					uLen_el	= (nAdr + nSize) - (numicro8051->uLIBROMAddr + numicro8051->uLIBROMSize);
					uLen_s	= 0;
					uLen_es	= 0;
				}
				else
				{
					uLen_c	= 0;
					uLen_ec	= 0;
					uLen_l	= nSize;
					uLen_el	= 0;
					uLen_s	= 0;
					uLen_es	= 0;
				}
			}
			else if (nAdr < numicro8051->uSCodeAddr)
			{
				if ((nAdr + nSize) > (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
				{
					uLen_c	= 0;
					uLen_ec	= 0;
					uLen_l	= 0;
					uLen_el	= numicro8051->uSCodeAddr - nAdr;
					uLen_s	= numicro8051->uSPROMSize;
					uLen_es	= (nAdr + nSize) - (numicro8051->uSCodeAddr + numicro8051->uSPROMSize);
				}
				else if ((nAdr + nSize) > numicro8051->uSCodeAddr)
				{
					uLen_c	= 0;
					uLen_ec	= 0;
					uLen_l	= 0;
					uLen_el	= numicro8051->uSCodeAddr - nAdr;
					uLen_s	= (nAdr + nSize) - numicro8051->uSCodeAddr;
					uLen_es	= 0;
				}
				else
				{
					uLen_c	= 0;
					uLen_ec	= 0;
					uLen_l	= 0;
					uLen_el	= nSize;
					uLen_s	= 0;
					uLen_es	= 0;
				}
			}
			else if (nAdr < (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
			{
				uAddr_s = nAdr;

				if ((nAdr + nSize) > (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
				{
					uLen_c	= 0;
					uLen_ec	= 0;
					uLen_l	= 0;
					uLen_el	= 0;
					uLen_s	= (numicro8051->uSCodeAddr + numicro8051->uSPROMSize) - nAdr;
					uLen_es	= (nAdr + nSize) - (numicro8051->uSCodeAddr + numicro8051->uSPROMSize);
				}
				else
				{
					uLen_c	= 0;
					uLen_ec	= 0;
					uLen_l	= 0;
					uLen_el	= 0;
					uLen_s	= nSize;
					uLen_es	= 0;
				}
			}
			else
			{
				uLen_c	= 0;
				uLen_ec	= 0;
				uLen_l	= 0;
				uLen_el	= 0;
				uLen_s	= 0;
				uLen_es	= nSize;
			}
		}
		else
		{
			if (nAdr < numicro8051->uProgramFlashSize)
			{
				if ((nAdr + nSize) > (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
				{
					uLen_c	= numicro8051->uProgramFlashSize - nAdr;
					uLen_ec	= numicro8051->uSCodeAddr - numicro8051->uProgramFlashSize;
					uLen_s	= numicro8051->uSPROMSize;
					uLen_es	= (nAdr + nSize) - (numicro8051->uSCodeAddr + numicro8051->uSPROMSize);
				}
				else if ((nAdr + nSize) > numicro8051->uSCodeAddr)
				{
					uLen_c	= numicro8051->uProgramFlashSize - nAdr;
					uLen_ec	= numicro8051->uSCodeAddr - numicro8051->uProgramFlashSize;
					uLen_s	= (nAdr + nSize) - numicro8051->uSCodeAddr;
					uLen_es	= 0;
				}
				else if ((nAdr + nSize) > numicro8051->uProgramFlashSize)
				{
					uLen_c	= numicro8051->uProgramFlashSize - nAdr;
					uLen_ec	= (nAdr + nSize) - numicro8051->uProgramFlashSize;
					uLen_s	= 0;
					uLen_es	= 0;
				}
				else
				{
					uLen_c	= nSize;
					uLen_ec	= 0;
					uLen_s	= 0;
					uLen_es	= 0;
				}
			}
			else if (nAdr < numicro8051->uSCodeAddr)
			{
				if ((nAdr + nSize) > (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
				{
					uLen_c	= 0;
					uLen_ec	= numicro8051->uSCodeAddr - nAdr;
					uLen_s	= numicro8051->uSPROMSize;
					uLen_es	= (nAdr + nSize) - (numicro8051->uSCodeAddr + numicro8051->uSPROMSize);
				}
				else if ((nAdr + nSize) > numicro8051->uSCodeAddr)
				{
					uLen_c	= 0;
					uLen_ec	= numicro8051->uSCodeAddr - nAdr;
					uLen_s	= (nAdr + nSize) - numicro8051->uSCodeAddr;
					uLen_es	= 0;
				}
				else
				{
					uLen_c	= 0;
					uLen_ec	= nSize;
					uLen_s	= 0;
					uLen_es	= 0;
				}
			}
			else if (nAdr < (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
			{
				uAddr_s = nAdr;

				if ((nAdr + nSize) > (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
				{
					uLen_c	= 0;
					uLen_ec	= 0;
					uLen_s	= (numicro8051->uSCodeAddr + numicro8051->uSPROMSize) - nAdr;
					uLen_es	= (nAdr + nSize) - (numicro8051->uSCodeAddr + numicro8051->uSPROMSize);
				}
				else
				{
					uLen_c	= 0;
					uLen_ec	= 0;
					uLen_s	= nSize;
					uLen_es	= 0;
				}
			}
			else
			{
				uLen_c	= 0;
				uLen_ec	= 0;
				uLen_s	= 0;
				uLen_es	= nSize;
			}
		}
	}
	else
	{
		if (nAdr < numicro8051->uProgramFlashSize)
		{
			if ((nAdr + nSize) > numicro8051->uProgramFlashSize)
			{
				uLen_c	= numicro8051->uProgramFlashSize - nAdr;
				uLen_ec	= (nAdr + nSize) - numicro8051->uProgramFlashSize;
				uLen_s	= 0;
				uLen_es	= 0;
			}
			else
			{
				uLen_c	= nSize;
				uLen_ec	= 0;
				uLen_s	= 0;
				uLen_es	= 0;
			}
		}
		else
		{
			uLen_c	= 0;
			uLen_ec	= nSize;
			uLen_s	= 0;
			uLen_es	= 0;
		}
	}

	if (uLen_c)
	{
		retval = adapter->layout->api->read_mem(adapter->handle, (uAddr_c | 0x80000000), 1, uLen_c, (uint8_t *)buffer);
		if(retval != ERROR_OK)
			return retval;

		buffer += uLen_c;
		numicro8051->uReadMemCount += uLen_c;
	}

	if (uLen_ec)
	{
		memset(buffer, 0xFF, uLen_ec);
		buffer += uLen_ec;
	}

	if (uLen_l)
	{
		retval = adapter->layout->api->read_mem(adapter->handle, (uAddr_l | 0x80000000), 1, uLen_l, (uint8_t *)buffer);
		if(retval != ERROR_OK)
			return retval;
		
		buffer += uLen_l;
		numicro8051->uReadMemCount += uLen_l;
	}

	if (uLen_el)
	{
		memset(buffer, 0xFF, uLen_el);
		buffer += uLen_el;
	}

	if (uLen_s)
	{
		if (numicro8051->bSupportSCode)
		{
			retval = adapter->layout->api->read_mem(adapter->handle, (uAddr_s | 0x80000000), 1, uLen_s, (uint8_t *)buffer);
			if(retval != ERROR_OK)
				return retval;

			numicro8051->uReadMemCount += uLen_s;
		}
		else
			memset(buffer, 0xFF, uLen_s);

		buffer += uLen_s;
	}

	if (uLen_es)
	{
		memset(buffer, 0xFF, uLen_es);
		buffer += uLen_es;
	}

	return retval;
}

int numicro8051_read_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval;
	unsigned long uZone, uAddr, address_align, offset = 0;
	
	address_align = (address + size * count) & 0xFF0000;

	if (address < address_align) {
		offset = address_align - address;
		count = count - (offset / size);
		address = address_align;
	}

	uAddr = address & 0x00FF0000;
	if ((uAddr == (amDATA << 16)) || (uAddr == (amDATA << 16) * 2) || (uAddr == (amDATA << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amDATA << 16);
	}
	else if ((uAddr == (amDATA1 << 16)) || (uAddr == (amDATA1 << 16) * 2) || (uAddr == (amDATA1 << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amDATA1 << 16);
	}
	else if ((uAddr == (amDATA2 << 16)) || (uAddr == (amDATA2 << 16) * 2) || (uAddr == (amDATA2 << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amDATA2 << 16);
	}
	else if ((uAddr == (amDATA3 << 16)) || (uAddr == (amDATA3 << 16) * 2) || (uAddr == (amDATA3 << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amDATA3 << 16);
	}
	else if ((uAddr == (amIDATA << 16)) || (uAddr == (amIDATA << 16) * 2) || (uAddr == (amIDATA << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amIDATA << 16);
	}
	else if ((uAddr == (amXDATA << 16)) || (uAddr == (amXDATA << 16) * 2) || (uAddr == (amXDATA << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amXDATA << 16);
	}

	uZone = (address >> 16) & 0xFF;
	
	switch (uZone)				// extract mSpace from address.
	{
		case amDATA:			// Data
		case amDATA1:			
		case amDATA2:			
		case amDATA3:			
		{
			LOG_DEBUG("amDATA1");
			uAddr = address & 0x0FFFFF;
			retval = ReadData(target, uAddr, size, count, buffer + offset);
			break;	
		}
		case amIDATA:			// Idata
		{
			LOG_DEBUG("amIDATA1");
			uAddr = address & 0x00FFFF;
			retval = ReadIdata(target, uAddr, size, count, buffer + offset);
			break;
		}
		case amXDATA:			// Xdata
		{
			LOG_DEBUG("amXDATA1");
			uAddr = address & 0x00FFFF;
			retval = ReadXdata(target, uAddr, size, count, buffer + offset);
			break;
		}
		case amCODE:			// Code
		default:
		{
			LOG_DEBUG("amCODE1");
			uAddr = address & 0x00FFFF;
			retval = ReadCode(target, uAddr, size, count, buffer + offset);
			break;
		}
	}

	return retval;
}

int WriteData(struct target *target, uint32_t nAdr, uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_OK;
	unsigned long uPage_w, uAddr_w, uLen_w;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;
	uint32_t nSize = size * count;

	while (nSize)
	{
		uPage_w	= (nAdr >> 8) & 0xFFFF;
		uAddr_w	= (nAdr & 0xFF);
		uLen_w	= nSize;

		if ((uAddr_w + uLen_w) > 0x100)
			uLen_w = 0x100 - uAddr_w;

		if (uPage_w == 0)
		{
			retval = adapter->layout->api->write_mem(adapter->handle, (uAddr_w | 0x20000000), 4, uLen_w / 4, (uint8_t *)buffer);
			if(retval != ERROR_OK)
				return retval;
		}
		else if (numicro8051->uSFR_PageNum > 0)
		{
			uPage_w &= ~0x0100;

			if (uPage_w < numicro8051->uSFR_PageNum)
			{
				unsigned char ucPage_Store;
				uint32_t val;

				retval = adapter->layout->api->read_reg(adapter->handle, (REG_SFRPAGE | 0x20000000), &val);
				if(retval != ERROR_OK)
					return retval;
				ucPage_Store = (unsigned char)val;

				if (uPage_w != ucPage_Store) {
					retval = adapter->layout->api->write_reg(adapter->handle, (REG_SFRPAGE | 0x20000000), uPage_w);
					if(retval != ERROR_OK)
						return retval;					
				}

				retval = adapter->layout->api->write_mem(adapter->handle, (uAddr_w | 0x20000000), 4, uLen_w / 4, (uint8_t *)buffer);
				if(retval != ERROR_OK)
					return retval;

				if ((uAddr_w <= REG_SFRPAGE) && ((uAddr_w + uLen_w) > REG_SFRPAGE))
				{
					ucPage_Store = buffer[REG_SFRPAGE - uAddr_w];
				}

				if (uPage_w != ucPage_Store) {
					retval = adapter->layout->api->write_reg(adapter->handle, (REG_SFRPAGE | 0x20000000), ucPage_Store);
					if(retval != ERROR_OK)
						return retval;
				}
			}
			else
			{
				memset(buffer, 0x00, uLen_w);
			}
		}
		else
		{
			memset(buffer, 0x00, uLen_w);
		}

		buffer	+= uLen_w;
		nAdr	+= uLen_w;
		nSize	-= uLen_w;
	}

	return retval;						// say Ok.	
}

int WriteIdata(struct target *target, uint32_t nAdr, uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_OK;
	struct hl_interface_s *adapter = target->tap->priv;
	uint32_t nSize = size * count;

	unsigned long uAddr_d, uAddr_i;
	unsigned long uLen_d = 0, uLen_i = 0, uLen_e = 0;

	if (nAdr < 0x80)
	{
		uAddr_d = nAdr;
		uAddr_i = 0x80;

		if ((nAdr + nSize) > 0x100)
		{
			uLen_d	= 0x80 - nAdr;
			uLen_i	= 0x80;
			uLen_e	= (nAdr + nSize) - 0x100;
		}
		else if ((nAdr + nSize) > 0x80)
		{
			uLen_d	= 0x80 - nAdr;
			uLen_i	= (nAdr + nSize) - 0x80;
			uLen_e	= 0;
		}
		else
		{
			uLen_d	= nSize;
			uLen_i	= 0;
			uLen_e	= 0;
		}
	}
	else if (nAdr < 0x100)
	{
		uAddr_d = 0;
		uAddr_i = nAdr;

		if ((nAdr + nSize) > 0x100)
		{
			uLen_d	= 0;
			uLen_i	= 0x100 - nAdr;
			uLen_e	= (nAdr + nSize) - 0x100;
		}
		else
		{
			uLen_d	= 0;
			uLen_i	= nSize;
			uLen_e	= 0;
		}
	}
	else
	{
		uLen_d	= 0;
		uLen_i	= 0;
		uLen_e	= nSize;
	}

	if (uLen_d)
	{
		retval = adapter->layout->api->write_mem(adapter->handle, (uAddr_d | 0x20000000), 4, uLen_d / 4, (uint8_t *)buffer);
		if(retval != ERROR_OK)
			return retval;
		buffer += uLen_d;
	}

	if (uLen_i)
	{
		retval = adapter->layout->api->write_mem(adapter->handle, (uAddr_i | 0x10000000), 4, uLen_i / 4, (uint8_t *)buffer);
		if(retval != ERROR_OK)
			return retval;
		buffer += uLen_i;
	}

	return retval;						// say Ok.
}

int WriteXdata(struct target *target, uint32_t nAdr, uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_OK;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;
	uint32_t nSize = size * count;

	unsigned int uAddr_x = nAdr, uAddr_t = numicro8051->uSFR_TKAddr;
	unsigned int uLen_x = 0, uLen_ex = 0, uLen_t = 0, uLen_et = 0;

	if (numicro8051->uSFR_TKSize > 0)
	{
		if (nAdr < numicro8051->uXRAMSize)
		{
			if ((nAdr + nSize) > (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize))
			{
				uLen_x	= numicro8051->uXRAMSize - nAdr;
				uLen_ex	= numicro8051->uSFR_TKAddr - numicro8051->uXRAMSize;
				uLen_t	= numicro8051->uSFR_TKSize;
				uLen_et	= (nAdr + nSize) - (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize);
			}
			else if ((nAdr + nSize) > numicro8051->uSFR_TKAddr)
			{
				uLen_x	= numicro8051->uXRAMSize - nAdr;
				uLen_ex	= numicro8051->uSFR_TKAddr - numicro8051->uXRAMSize;
				uLen_t	= (nAdr + nSize) - numicro8051->uSFR_TKAddr;
				uLen_et	= 0;
			}
			else if ((nAdr + nSize) > numicro8051->uXRAMSize)
			{
				uLen_x	= numicro8051->uXRAMSize - nAdr;
				uLen_ex	= (nAdr + nSize) - numicro8051->uXRAMSize;
				uLen_t	= 0;
				uLen_et	= 0;
			}
			else
			{
				uLen_x	= nSize;
				uLen_ex	= 0;
				uLen_t	= 0;
				uLen_et	= 0;
			}
		}
		else if (nAdr < numicro8051->uSFR_TKAddr)
		{
			if ((nAdr + nSize) > (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize))
			{
				uLen_x	= 0;
				uLen_ex	= numicro8051->uSFR_TKAddr - nAdr;
				uLen_t	= numicro8051->uSFR_TKSize;
				uLen_et	= (nAdr + nSize) - (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize);
			}
			else if ((nAdr + nSize) > numicro8051->uSFR_TKAddr)
			{
				uLen_x	= 0;
				uLen_ex	= numicro8051->uSFR_TKAddr - nAdr;
				uLen_t	= (nAdr + nSize) - numicro8051->uSFR_TKAddr;
				uLen_et	= 0;
			}
			else
			{
				uLen_x	= 0;
				uLen_ex	= nSize;
				uLen_t	= 0;
				uLen_et	= 0;
			}
		}
		else if (nAdr < (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize))
		{
			uAddr_t = nAdr;

			if ((nAdr + nSize) > (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize))
			{
				uLen_x	= 0;
				uLen_ex	= 0;
				uLen_t	= (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize) - nAdr;
				uLen_et	= (nAdr + nSize) - (numicro8051->uSFR_TKAddr + numicro8051->uSFR_TKSize);
			}
			else
			{
				uLen_x	= 0;
				uLen_ex	= 0;
				uLen_t	= nSize;
				uLen_et	= 0;
			}
		}
		else
		{
			uLen_x	= 0;
			uLen_ex	= 0;
			uLen_t	= 0;
			uLen_et	= nSize;
		}
	}
	else
	{
		if (nAdr < numicro8051->uXRAMSize)
		{
			if ((nAdr + nSize) > numicro8051->uXRAMSize)
			{
				uLen_x	= numicro8051->uXRAMSize - nAdr;
				uLen_ex	= (nAdr + nSize) - numicro8051->uXRAMSize;
			}
			else
			{
				uLen_x	= nSize;
				uLen_ex	= 0;
			}
		}
		else
		{
			uLen_x	= 0;
			uLen_ex	= nSize;
		}
	}

	if (uLen_x)
	{
		retval = adapter->layout->api->write_mem(adapter->handle, (uAddr_x | 0x40000000), 4, uLen_x / 4, (uint8_t *)buffer);
		if(retval != ERROR_OK)
			return retval;
		buffer += uLen_x;
	}

	if (uLen_ex)
	{
		buffer += uLen_ex;
	}

	if (uLen_t)
	{
		retval = adapter->layout->api->write_mem(adapter->handle, (uAddr_t | 0x40000000), 4, uLen_t / 4, (uint8_t *)buffer);
		if(retval != ERROR_OK)
			return retval;
		buffer += uLen_t;
	}

	if (uLen_et)
	{
		buffer += uLen_et;
	}

	return retval;						// say Ok.
}

int WriteCode(struct target *target, uint32_t nAdr, uint32_t size, uint32_t count, const uint8_t *buffer)
{
	return ERROR_OK;
}

int numicro8051_write_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval;
	unsigned long uZone, uAddr, address_align, offset = 0;

	address_align = (address + size * count) & 0xFF0000;

	if (address < address_align) {
		offset = address_align - address;
		count = count - (offset / size);
		address = address_align;
	}

	uAddr = address & 0x00FF0000;
	if ((uAddr == (amDATA << 16)) || (uAddr == (amDATA << 16) * 2) || (uAddr == (amDATA << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amDATA << 16);
	}
	else if ((uAddr == (amDATA1 << 16)) || (uAddr == (amDATA1 << 16) * 2) || (uAddr == (amDATA1 << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amDATA1 << 16);
	}
	else if ((uAddr == (amDATA2 << 16)) || (uAddr == (amDATA2 << 16) * 2) || (uAddr == (amDATA2 << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amDATA2 << 16);
	}
	else if ((uAddr == (amDATA3 << 16)) || (uAddr == (amDATA3 << 16) * 2) || (uAddr == (amDATA3 << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amDATA3 << 16);
	}
	else if ((uAddr == (amIDATA << 16)) || (uAddr == (amIDATA << 16) * 2) || (uAddr == (amIDATA << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amIDATA << 16);
	}
	else if ((uAddr == (amXDATA << 16)) || (uAddr == (amXDATA << 16) * 2) || (uAddr == (amXDATA << 16) * 4)) {
		address = (address & 0x0000FFFF) | (amXDATA << 16);
	}

	uZone = (address >> 16) & 0xFF;

	switch (uZone)				// extract mSpace from address.
	{
		case amDATA:			// Data
		case amDATA1:			
		case amDATA2:			
		case amDATA3:
		{
			LOG_DEBUG("amDATA1");
			uAddr = address & 0x0FFFFF;
			retval = WriteData(target, uAddr, size, count, buffer + offset);
			break;
		}
		case amIDATA:			// Idata
		{
			LOG_DEBUG("amIDATA1");
			uAddr = address & 0x00FFFF;
			retval = WriteIdata(target, uAddr, size, count, buffer + offset);
			break;
		}
		case amXDATA:			// Xdata
		{
			LOG_DEBUG("amXDATA1");
			uAddr = address & 0x00FFFF;
			retval = WriteXdata(target, uAddr, size, count, buffer + offset);
			break;
		}
		case amCODE:			// Code
		default:
		{
			LOG_DEBUG("amCODE1");
			uAddr = address & 0x00FFFF;
			retval = WriteCode(target, uAddr, size, count, buffer + offset);
			break;
		}
	}

	return retval;
}

static int numicro8051_get_core_reg(struct reg *reg)
{
	int retval = ERROR_OK;
	struct numicro8051_core_reg *numicro8051_reg = reg->arch_info;
	struct target *target = numicro8051_reg->target;
	struct numicro8051_common *numicro8051_target = target_to_numicro8051(target);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = numicro8051_target->read_core_reg(target, numicro8051_reg->num);

	return retval;
}

static int numicro8051_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct numicro8051_core_reg *numicro8051_reg = reg->arch_info;
	struct target *target = numicro8051_reg->target;
	struct numicro8051_common *numicro8051_target = target_to_numicro8051(target);
	int retval = ERROR_OK;
	uint32_t value = buf_get_u32(buf, 0, reg->size);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, 32, value);
	reg->dirty = true;
	reg->valid = true;

	numicro8051_target->core_regs[numicro8051_reg->num] = value;

	return retval;
}

static const struct reg_arch_type numicro8051_reg_type = {
	.get = numicro8051_get_core_reg,
	.set = numicro8051_set_core_reg,
};

static struct reg_cache *numicro8051_build_reg_cache(struct target *target)
{
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	int num_regs = NUMICRO8051_NUM_REGS;
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct numicro8051_core_reg *arch_info = malloc(
			sizeof(struct numicro8051_core_reg) * num_regs);
	struct reg_feature *feature;
	int i;

	/* Build the process context cache */
	cache->name = "numicro8051 registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;
	numicro8051->core_cache = cache;

	for (i = 0; i < num_regs; i++) {
		arch_info[i].num = numicro8051_regs[i].id;
		arch_info[i].target = target;
		arch_info[i].numicro8051_common = numicro8051;

		reg_list[i].name = numicro8051_regs[i].name;
		reg_list[i].size = numicro8051_regs[i].bits;

		reg_list[i].value = calloc(1, 4);
		reg_list[i].valid = false;
		reg_list[i].type = &numicro8051_reg_type;
		reg_list[i].arch_info = &arch_info[i];

		reg_list[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (reg_list[i].reg_data_type)
			reg_list[i].reg_data_type->type = numicro8051_regs[i].type;
		else {
			LOG_ERROR("unable to allocate reg type list");
			return NULL;
		}

		reg_list[i].dirty = false;
		reg_list[i].group = numicro8051_regs[i].group;
		reg_list[i].number = numicro8051_regs[i].id;
		reg_list[i].exist = true;
		reg_list[i].caller_save = true;	/* gdb defaults to true */

		feature = calloc(1, sizeof(struct reg_feature));
		if (feature) {
			feature->name = numicro8051_regs[i].feature;
			reg_list[i].feature = feature;
		} else
			LOG_ERROR("unable to allocate feature list");
	}

	return cache;
}

int numicro8051_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	numicro8051_build_reg_cache(target);
	return ERROR_OK;
}

int numicro8051_arch_state(struct target *target)
{
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	numicro8051_poll(target);

	LOG_USER("target halted due to %s, pc: 0x%8.8" PRIx32 "",
		debug_reason_name(target),
		buf_get_u32(numicro8051->core_cache->reg_list[NUMICRO8051_PC].value, 0, 32));

	return ERROR_OK;
}

//
void numicro8051_deinit_target(struct target *target)
{
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	free(numicro8051);
}

#define MVFR0 0xe000ef40
#define MVFR1 0xe000ef44

#define MVFR0_DEFAULT_M4 0x10110021
#define MVFR1_DEFAULT_M4 0x11000011

#define MVFR0_DEFAULT_M7_SP 0x10110021
#define MVFR0_DEFAULT_M7_DP 0x10110221
#define MVFR1_DEFAULT_M7_SP 0x11000011
#define MVFR1_DEFAULT_M7_DP 0x12000011

int numicro8051_examine(struct target *target)
{
	struct hl_interface_s *adapter = target->tap->priv;

	if (!target_was_examined(target)) {
		nulink_8051_reset(adapter->handle);
		target_set_examined(target);

		return ERROR_OK;
	}

	return ERROR_OK;
}

static int numicro8051_init_arch_info(struct target *target,
	struct numicro8051_common *numicro8051, struct jtag_tap *tap)
{
	target->endianness = TARGET_BIG_ENDIAN;
	target->arch_info = numicro8051;
	numicro8051->uLIBROMAddr = 0xFFFFFFFF;
	numicro8051->uSCodeAddr = N76E_SCODE_ADDR;
	numicro8051->uLIBROMSize = 0;
	numicro8051->uProgramFlashAddr = 0;
	numicro8051->uProgramFlashSize = 18 * 1024;
	numicro8051->uSPROMAddr = 0;
	numicro8051->uSPROMSize = 128;
	numicro8051->uSPROMMode = 0;
	numicro8051->bSupportSCode = 0;
	numicro8051->uReadMemCount = 0;
	numicro8051->uSFR_PageNum = 0;
	numicro8051->uSFR_TKAddr = 0;
	numicro8051->uSFR_TKSize = 0;
	numicro8051->uXRAMSize = 0x300;

	/* has breakpoint/watchpoint unit been scanned */
	numicro8051->bp_scanned = false;
	numicro8051->hw_break_list = NULL;

	numicro8051->read_core_reg = numicro8051_read_core_reg;
	numicro8051->write_core_reg = numicro8051_write_core_reg;

	return ERROR_OK;
}

static int numicro8051_read_core_reg(struct target *target, unsigned int num)
{
	uint32_t reg_value;

	/* get pointers to arch-specific information */
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	if (num >= NUMICRO8051_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = numicro8051->core_regs[num];
	buf_set_u32(numicro8051->core_cache->reg_list[num].value, 0, 32, reg_value);
	numicro8051->core_cache->reg_list[num].valid = true;
	numicro8051->core_cache->reg_list[num].dirty = false;

	return ERROR_OK;
}

static int numicro8051_write_core_reg(struct target *target, unsigned int num)
{
	uint32_t reg_value;

	/* get pointers to arch-specific information */
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	if (num >= NUMICRO8051_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = buf_get_u32(numicro8051->core_cache->reg_list[num].value, 0, 32);
	numicro8051->core_regs[num] = reg_value;

	numicro8051->core_cache->reg_list[num].valid = true;
	numicro8051->core_cache->reg_list[num].dirty = true;

	return ERROR_OK;
}

static int numicro8051_configure_break_unit(struct target *target)
{
	/* get pointers to arch-specific information */
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	if (numicro8051->bp_scanned)
		return ERROR_OK;

	numicro8051->num_hw_bpoints = BP_MAX;
	numicro8051->num_hw_bpoints_avail = numicro8051->num_hw_bpoints;

	numicro8051->hw_break_list = calloc(numicro8051->num_hw_bpoints,
		sizeof(struct numicro8051_comparator));

	numicro8051->hw_break_list[0].reg_address = 0;
	numicro8051->hw_break_list[1].reg_address = 1;

	numicro8051->bp_scanned = true;

	return ERROR_OK;
}

int numicro8051_target_create(struct target *target, Jim_Interp *interp)
{
	struct numicro8051_common *numicro8051 = calloc(1, sizeof(struct numicro8051_common));

	numicro8051_init_arch_info(target, numicro8051, target->tap);
	numicro8051_configure_break_unit(target);

	return ERROR_OK;
}


struct target_type numicro8051_target = {
	.name = "nu8051",

	.poll = numicro8051_poll,
	.arch_state = numicro8051_arch_state,
	.halt = numicro8051_halt,
	.resume = numicro8051_resume,
	.step = numicro8051_step,

	.assert_reset = numicro8051_assert_reset,
	.deassert_reset = numicro8051_deassert_reset,
	.get_gdb_reg_list = numicro8051_get_gdb_reg_list,

	.read_memory = numicro8051_read_memory,
	.write_memory = numicro8051_write_memory,
	.add_breakpoint = numicro8051_add_breakpoint,
	.remove_breakpoint = numicro8051_remove_breakpoint,
	.target_create = numicro8051_target_create,
	.init_target = numicro8051_init_target,
	.examine = numicro8051_examine,
	.deinit_target = numicro8051_deinit_target,
};
