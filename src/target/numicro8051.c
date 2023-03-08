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
//static int numicro8051_save_context(struct target *target);
//static void numicro8051_enable_breakpoints(struct target *target);
//static int numicro8051_unset_breakpoint(struct target *target,
//		struct breakpoint *breakpoint);
//static int numicro8051_set_breakpoint(struct target *target,
//		struct breakpoint *breakpoint);
//static void numicro8051_enable_watchpoints(struct target *target);
//static int numicro8051_unset_watchpoint(struct target *target,
//		struct watchpoint *watchpoint);
//static int (*adapter_speed)(int speed);

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

//extern struct hl_layout_api_s nulink_usb_layout_api;

///* forward declarations */
//static int numicro8051_store_core_reg_u32(struct target *target,
//		uint32_t num, uint32_t value);
//static void numicro8051_dwt_free(struct target *target);

//static int numicro8051_dap_read_coreregister_u32(struct target *target,
//	uint32_t *value, int regnum)
//{
//	struct armv7m_common *armv7m = target_to_armv7m(target);
//	int retval;
//	uint32_t dcrdr;
//
//	/* because the DCB_DCRDR is used for the emulated dcc channel
//	 * we have to save/restore the DCB_DCRDR when used */
//	if (target->dbg_msg_enabled) {
//		retval = mem_ap_read_u32(armv7m->debug_ap, DCB_DCRDR, &dcrdr);
//		if (retval != ERROR_OK)
//			return retval;
//	}
//
//	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DCRSR, regnum);
//	if (retval != ERROR_OK)
//		return retval;
//
//	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DCRDR, value);
//	if (retval != ERROR_OK)
//		return retval;
//
//	if (target->dbg_msg_enabled) {
//		/* restore DCB_DCRDR - this needs to be in a separate
//		 * transaction otherwise the emulated DCC channel breaks */
//		if (retval == ERROR_OK)
//			retval = mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DCRDR, dcrdr);
//	}
//
//	return retval;
//}
//
//static int numicro8051_dap_write_coreregister_u32(struct target *target,
//	uint32_t value, int regnum)
//{
//	struct armv7m_common *armv7m = target_to_armv7m(target);
//	int retval;
//	uint32_t dcrdr;
//
//	/* because the DCB_DCRDR is used for the emulated dcc channel
//	 * we have to save/restore the DCB_DCRDR when used */
//	if (target->dbg_msg_enabled) {
//		retval = mem_ap_read_u32(armv7m->debug_ap, DCB_DCRDR, &dcrdr);
//		if (retval != ERROR_OK)
//			return retval;
//	}
//
//	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DCRDR, value);
//	if (retval != ERROR_OK)
//		return retval;
//
//	retval = mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DCRSR, regnum | DCRSR_WnR);
//	if (retval != ERROR_OK)
//		return retval;
//
//	if (target->dbg_msg_enabled) {
//		/* restore DCB_DCRDR - this needs to be in a seperate
//		 * transaction otherwise the emulated DCC channel breaks */
//		if (retval == ERROR_OK)
//			retval = mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DCRDR, dcrdr);
//	}
//
//	return retval;
//}
//
//static int numicro8051_write_debug_halt_mask(struct target *target,
//	uint32_t mask_on, uint32_t mask_off)
//{
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//	struct armv7m_common *armv7m = &numicro8051->armv7m;
//
//	/* mask off status bits */
//	numicro8051->dcb_dhcsr &= ~((0xFFFF << 16) | mask_off);
//	/* create new register mask */
//	numicro8051->dcb_dhcsr |= DBGKEY | C_DEBUGEN | mask_on;
//
//	return mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DHCSR, numicro8051->dcb_dhcsr);
//}
//
//static int numicro8051_clear_halt(struct target *target)
//{
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//	struct armv7m_common *armv7m = &numicro8051->armv7m;
//	int retval;
//
//	/* clear step if any */
//	numicro8051_write_debug_halt_mask(target, C_HALT, C_STEP);
//
//	/* Read Debug Fault Status Register */
//	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, NVIC_DFSR, &numicro8051->nvic_dfsr);
//	if (retval != ERROR_OK)
//		return retval;
//
//	/* Clear Debug Fault Status */
//	retval = mem_ap_write_atomic_u32(armv7m->debug_ap, NVIC_DFSR, numicro8051->nvic_dfsr);
//	if (retval != ERROR_OK)
//		return retval;
//	LOG_DEBUG(" NVIC_DFSR 0x%" PRIx32 "", numicro8051->nvic_dfsr);
//
//	return ERROR_OK;
//}
//
//static int numicro8051_single_step_core(struct target *target)
//{
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//	struct armv7m_common *armv7m = &numicro8051->armv7m;
//	uint32_t dhcsr_save;
//	int retval;
//
//	/* backup dhcsr reg */
//	dhcsr_save = numicro8051->dcb_dhcsr;
//
//	/* Mask interrupts before clearing halt, if done already.  This avoids
//	 * Erratum 377497 (fixed in r1p0) where setting MASKINTS while clearing
//	 * HALT can put the core into an unknown state.
//	 */
//	if (!(numicro8051->dcb_dhcsr & C_MASKINTS)) {
//		retval = mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DHCSR,
//				DBGKEY | C_MASKINTS | C_HALT | C_DEBUGEN);
//		if (retval != ERROR_OK)
//			return retval;
//	}
//	retval = mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DHCSR,
//			DBGKEY | C_MASKINTS | C_STEP | C_DEBUGEN);
//	if (retval != ERROR_OK)
//		return retval;
//	LOG_DEBUG(" ");
//
//	/* restore dhcsr reg */
//	numicro8051->dcb_dhcsr = dhcsr_save;
//	numicro8051_clear_halt(target);
//
//	return ERROR_OK;
//}
//
//static int numicro8051_enable_fpb(struct target *target)
//{
//	int retval = target_write_u32(target, FP_CTRL, 3);
//	if (retval != ERROR_OK)
//		return retval;
//
//	/* check the fpb is actually enabled */
//	uint32_t fpctrl;
//	retval = target_read_u32(target, FP_CTRL, &fpctrl);
//	if (retval != ERROR_OK)
//		return retval;
//
//	if (fpctrl & 1)
//		return ERROR_OK;
//
//	return ERROR_FAIL;
//}
//
//static int numicro8051_endreset_event(struct target *target)
//{
//	int i;
//	int retval;
//	uint32_t dcb_demcr;
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//	struct armv7m_common *armv7m = &numicro8051->armv7m;
//	struct adiv5_dap *swjdp = numicro8051->armv7m.arm.dap;
//	struct numicro8051_fp_comparator *fp_list = numicro8051->fp_comparator_list;
//	struct numicro8051_dwt_comparator *dwt_list = numicro8051->dwt_comparator_list;
//
//	/* REVISIT The four debug monitor bits are currently ignored... */
//	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DEMCR, &dcb_demcr);
//	if (retval != ERROR_OK)
//		return retval;
//	LOG_DEBUG("DCB_DEMCR = 0x%8.8" PRIx32 "", dcb_demcr);
//
//	/* this register is used for emulated dcc channel */
//	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DCRDR, 0);
//	if (retval != ERROR_OK)
//		return retval;
//
//	/* Enable debug requests */
//	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DHCSR, &numicro8051->dcb_dhcsr);
//	if (retval != ERROR_OK)
//		return retval;
//	if (!(numicro8051->dcb_dhcsr & C_DEBUGEN)) {
//		retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DHCSR, DBGKEY | C_DEBUGEN);
//		if (retval != ERROR_OK)
//			return retval;
//	}
//
//	/* clear any interrupt masking */
//	numicro8051_write_debug_halt_mask(target, 0, C_MASKINTS);
//
//	/* Enable features controlled by ITM and DWT blocks, and catch only
//	 * the vectors we were told to pay attention to.
//	 *
//	 * Target firmware is responsible for all fault handling policy
//	 * choices *EXCEPT* explicitly scripted overrides like "vector_catch"
//	 * or manual updates to the NVIC SHCSR and CCR registers.
//	 */
//	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DEMCR, TRCENA | armv7m->demcr);
//	if (retval != ERROR_OK)
//		return retval;
//
//	/* Paranoia: evidently some (early?) chips don't preserve all the
//	 * debug state (including FBP, DWT, etc) across reset...
//	 */
//
//	/* Enable FPB */
//	retval = numicro8051_enable_fpb(target);
//	if (retval != ERROR_OK) {
//		LOG_ERROR("Failed to enable the FPB");
//		return retval;
//	}
//
//	numicro8051->fpb_enabled = 1;
//
//	/* Restore FPB registers */
//	for (i = 0; i < numicro8051->fp_num_code + numicro8051->fp_num_lit; i++) {
//		retval = target_write_u32(target, fp_list[i].fpcr_address, fp_list[i].fpcr_value);
//		if (retval != ERROR_OK)
//			return retval;
//	}
//
//	/* Restore DWT registers */
//	for (i = 0; i < numicro8051->dwt_num_comp; i++) {
//		retval = target_write_u32(target, dwt_list[i].dwt_comparator_address + 0,
//				dwt_list[i].comp);
//		if (retval != ERROR_OK)
//			return retval;
//		retval = target_write_u32(target, dwt_list[i].dwt_comparator_address + 4,
//				dwt_list[i].mask);
//		if (retval != ERROR_OK)
//			return retval;
//		retval = target_write_u32(target, dwt_list[i].dwt_comparator_address + 8,
//				dwt_list[i].function);
//		if (retval != ERROR_OK)
//			return retval;
//	}
//	retval = dap_run(swjdp);
//	if (retval != ERROR_OK)
//		return retval;
//
//	register_cache_invalidate(armv7m->arm.core_cache);
//
//	/* make sure we have latest dhcsr flags */
//	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DHCSR, &numicro8051->dcb_dhcsr);
//
//	return retval;
//}
//
//static int numicro8051_examine_debug_reason(struct target *target)
//{
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//
//	/* THIS IS NOT GOOD, TODO - better logic for detection of debug state reason
//	 * only check the debug reason if we don't know it already */
//
//	if ((target->debug_reason != DBG_REASON_DBGRQ)
//		&& (target->debug_reason != DBG_REASON_SINGLESTEP)) {
//		if (numicro8051->nvic_dfsr & DFSR_BKPT) {
//			target->debug_reason = DBG_REASON_BREAKPOINT;
//			if (numicro8051->nvic_dfsr & DFSR_DWTTRAP)
//				target->debug_reason = DBG_REASON_WPTANDBKPT;
//		} else if (numicro8051->nvic_dfsr & DFSR_DWTTRAP)
//			target->debug_reason = DBG_REASON_WATCHPOINT;
//		else if (numicro8051->nvic_dfsr & DFSR_VCATCH)
//			target->debug_reason = DBG_REASON_BREAKPOINT;
//		else	/* EXTERNAL, HALTED */
//			target->debug_reason = DBG_REASON_UNDEFINED;
//	}
//
//	return ERROR_OK;
//}
//
//static int numicro8051_examine_exception_reason(struct target *target)
//{
//	uint32_t shcsr = 0, except_sr = 0, cfsr = -1, except_ar = -1;
//	struct armv7m_common *armv7m = target_to_armv7m(target);
//	struct adiv5_dap *swjdp = armv7m->arm.dap;
//	int retval;
//
//	retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_SHCSR, &shcsr);
//	if (retval != ERROR_OK)
//		return retval;
//	switch (armv7m->exception_number) {
//		case 2:	/* NMI */
//			break;
//		case 3:	/* Hard Fault */
//			retval = mem_ap_read_atomic_u32(armv7m->debug_ap, NVIC_HFSR, &except_sr);
//			if (retval != ERROR_OK)
//				return retval;
//			if (except_sr & 0x40000000) {
//				retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_CFSR, &cfsr);
//				if (retval != ERROR_OK)
//					return retval;
//			}
//			break;
//		case 4:	/* Memory Management */
//			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_CFSR, &except_sr);
//			if (retval != ERROR_OK)
//				return retval;
//			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_MMFAR, &except_ar);
//			if (retval != ERROR_OK)
//				return retval;
//			break;
//		case 5:	/* Bus Fault */
//			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_CFSR, &except_sr);
//			if (retval != ERROR_OK)
//				return retval;
//			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_BFAR, &except_ar);
//			if (retval != ERROR_OK)
//				return retval;
//			break;
//		case 6:	/* Usage Fault */
//			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_CFSR, &except_sr);
//			if (retval != ERROR_OK)
//				return retval;
//			break;
//		case 11:	/* SVCall */
//			break;
//		case 12:	/* Debug Monitor */
//			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_DFSR, &except_sr);
//			if (retval != ERROR_OK)
//				return retval;
//			break;
//		case 14:	/* PendSV */
//			break;
//		case 15:	/* SysTick */
//			break;
//		default:
//			except_sr = 0;
//			break;
//	}
//	retval = dap_run(swjdp);
//	if (retval == ERROR_OK)
//		LOG_DEBUG("%s SHCSR 0x%" PRIx32 ", SR 0x%" PRIx32
//			", CFSR 0x%" PRIx32 ", AR 0x%" PRIx32,
//			armv7m_exception_string(armv7m->exception_number),
//			shcsr, except_sr, cfsr, except_ar);
//	return retval;
//}
//
//static int numicro8051_debug_entry(struct target *target)
//{
//	int i;
//	uint32_t xPSR;
//	int retval;
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//	struct armv7m_common *armv7m = &numicro8051->armv7m;
//	struct arm *arm = &armv7m->arm;
//	struct reg *r;
//
//	LOG_DEBUG(" ");
//
//	numicro8051_clear_halt(target);
//	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DHCSR, &numicro8051->dcb_dhcsr);
//	if (retval != ERROR_OK)
//		return retval;
//
//	retval = armv7m->examine_debug_reason(target);
//	if (retval != ERROR_OK)
//		return retval;
//
//	/* Examine target state and mode
//	 * First load register accessible through core debug port */
//	int num_regs = arm->core_cache->num_regs;
//
//	for (i = 0; i < num_regs; i++) {
//		r = &armv7m->arm.core_cache->reg_list[i];
//		if (!r->valid)
//			arm->read_core_reg(target, r, i, ARM_MODE_ANY);
//	}
//
//	r = arm->cpsr;
//	xPSR = buf_get_u32(r->value, 0, 32);
//
//	/* For IT instructions xPSR must be reloaded on resume and clear on debug exec */
//	if (xPSR & 0xf00) {
//		r->dirty = r->valid;
//		numicro8051_store_core_reg_u32(target, 16, xPSR & ~0xff);
//	}
//
//	/* Are we in an exception handler */
//	if (xPSR & 0x1FF) {
//		armv7m->exception_number = (xPSR & 0x1FF);
//
//		arm->core_mode = ARM_MODE_HANDLER;
//		arm->map = armv7m_msp_reg_map;
//	} else {
//		unsigned control = buf_get_u32(arm->core_cache
//				->reg_list[ARMV7M_CONTROL].value, 0, 2);
//
//		/* is this thread privileged? */
//		arm->core_mode = control & 1
//			? ARM_MODE_USER_THREAD
//			: ARM_MODE_THREAD;
//
//		/* which stack is it using? */
//		if (control & 2)
//			arm->map = armv7m_psp_reg_map;
//		else
//			arm->map = armv7m_msp_reg_map;
//
//		armv7m->exception_number = 0;
//	}
//
//	if (armv7m->exception_number)
//		numicro8051_examine_exception_reason(target);
//
//	LOG_DEBUG("entered debug state in core mode: %s at PC 0x%" PRIx32 ", target->state: %s",
//		arm_mode_name(arm->core_mode),
//		buf_get_u32(arm->pc->value, 0, 32),
//		target_state_name(target));
//
//	if (armv7m->post_debug_entry) {
//		retval = armv7m->post_debug_entry(target);
//		if (retval != ERROR_OK)
//			return retval;
//	}
//
//	return ERROR_OK;
//}
//

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
//		int retval = adapter_debug_entry(target);
//		if (retval != ERROR_OK)
//			return retval;
//
		if (prev_target_state == TARGET_DEBUG_RUNNING) {
			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		} else {
//			if (arm_semihosting(target, &retval) != 0)
//				return retval;
//
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
//
		LOG_DEBUG("halted: PC: 0x%08" PRIx32, buf_get_u32(numicro8051->core_cache->reg_list[NUMICRO8051_PC].value, 0, 32));
	}

	return ERROR_OK;
}
//	int detected_failure = ERROR_OK;
//	int retval = ERROR_OK;
//	enum target_state prev_target_state = target->state;
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//	struct armv7m_common *armv7m = &numicro8051->armv7m;
//
//	/* Read from Debug Halting Control and Status Register */
//	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DHCSR, &numicro8051->dcb_dhcsr);
//	if (retval != ERROR_OK) {
//		target->state = TARGET_UNKNOWN;
//		return retval;
//	}
//
//	/* Recover from lockup.  See ARMv7-M architecture spec,
//	 * section B1.5.15 "Unrecoverable exception cases".
//	 */
//	if (numicro8051->dcb_dhcsr & S_LOCKUP) {
//		LOG_ERROR("%s -- clearing lockup after double fault",
//			target_name(target));
//		numicro8051_write_debug_halt_mask(target, C_HALT, 0);
//		target->debug_reason = DBG_REASON_DBGRQ;
//
//		/* We have to execute the rest (the "finally" equivalent, but
//		 * still throw this exception again).
//		 */
//		detected_failure = ERROR_FAIL;
//
//		/* refresh status bits */
//		retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DHCSR, &numicro8051->dcb_dhcsr);
//		if (retval != ERROR_OK)
//			return retval;
//	}
//
//	if (numicro8051->dcb_dhcsr & S_RESET_ST) {
//		target->state = TARGET_RESET;
//		return ERROR_OK;
//	}
//
//	if (target->state == TARGET_RESET) {
//		/* Cannot switch context while running so endreset is
//		 * called with target->state == TARGET_RESET
//		 */
//		LOG_DEBUG("Exit from reset with dcb_dhcsr 0x%" PRIx32,
//			numicro8051->dcb_dhcsr);
//		retval = numicro8051_endreset_event(target);
//		if (retval != ERROR_OK) {
//			target->state = TARGET_UNKNOWN;
//			return retval;
//		}
//		target->state = TARGET_RUNNING;
//		prev_target_state = TARGET_RUNNING;
//	}
//
//	if (numicro8051->dcb_dhcsr & S_HALT) {
//		target->state = TARGET_HALTED;
//
//		if ((prev_target_state == TARGET_RUNNING) || (prev_target_state == TARGET_RESET)) {
//			retval = numicro8051_debug_entry(target);
//			if (retval != ERROR_OK)
//				return retval;
//
//			if (arm_semihosting(target, &retval) != 0)
//				return retval;
//
//			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
//		}
//		if (prev_target_state == TARGET_DEBUG_RUNNING) {
//			LOG_DEBUG(" ");
//			retval = numicro8051_debug_entry(target);
//			if (retval != ERROR_OK)
//				return retval;
//
//			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
//		}
//	}
//
//	/* REVISIT when S_SLEEP is set, it's in a Sleep or DeepSleep state.
//	 * How best to model low power modes?
//	 */
//
//	if (target->state == TARGET_UNKNOWN) {
//		/* check if processor is retiring instructions */
//		if (numicro8051->dcb_dhcsr & S_RETIRE_ST) {
//			target->state = TARGET_RUNNING;
//			retval = ERROR_OK;
//		}
//	}
//
//	/* Did we detect a failure condition that we cleared? */
//	if (detected_failure != ERROR_OK)
//		retval = detected_failure;
//	return retval;
//}
//
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

	/* Write to Debug Halting Control and Status Register */
//	numicro8051_write_debug_halt_mask(target, C_HALT, 0);

	struct hl_interface_s *adapter = target->tap->priv;
	retval = adapter->layout->api->halt(adapter->handle);	
	if (retval != ERROR_OK)
		return retval;

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}
//
//static int numicro8051_soft_reset_halt(struct target *target)
//{
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//	struct armv7m_common *armv7m = &numicro8051->armv7m;
//	uint32_t dcb_dhcsr = 0;
//	int retval, timeout = 0;
//
//	/* soft_reset_halt is deprecated on numicro8051 as the same functionality
//	 * can be obtained by using 'reset halt' and 'numicro8051 reset_config vectreset'
//	 * As this reset only used VC_CORERESET it would only ever reset the numicro8051
//	 * core, not the peripherals */
//	LOG_WARNING("soft_reset_halt is deprecated, please use 'reset halt' instead.");
//
//	/* Enter debug state on reset; restore DEMCR in endreset_event() */
//	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DEMCR,
//			TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
//	if (retval != ERROR_OK)
//		return retval;
//
//	/* Request a core-only reset */
//	retval = mem_ap_write_atomic_u32(armv7m->debug_ap, NVIC_AIRCR,
//			AIRCR_VECTKEY | AIRCR_VECTRESET);
//	if (retval != ERROR_OK)
//		return retval;
//	target->state = TARGET_RESET;
//
//	/* registers are now invalid */
//	register_cache_invalidate(numicro8051->armv7m.arm.core_cache);
//
//	while (timeout < 100) {
//		retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DHCSR, &dcb_dhcsr);
//		if (retval == ERROR_OK) {
//			retval = mem_ap_read_atomic_u32(armv7m->debug_ap, NVIC_DFSR,
//					&numicro8051->nvic_dfsr);
//			if (retval != ERROR_OK)
//				return retval;
//			if ((dcb_dhcsr & S_HALT)
//				&& (numicro8051->nvic_dfsr & DFSR_VCATCH)) {
//				LOG_DEBUG("system reset-halted, DHCSR 0x%08x, "
//					"DFSR 0x%08x",
//					(unsigned) dcb_dhcsr,
//					(unsigned) numicro8051->nvic_dfsr);
//				numicro8051_poll(target);
//				/* FIXME restore user's vector catch config */
//				return ERROR_OK;
//			} else
//				LOG_DEBUG("waiting for system reset-halt, "
//					"DHCSR 0x%08x, %d ms",
//					(unsigned) dcb_dhcsr, timeout);
//		}
//		timeout++;
//		alive_sleep(1);
//	}
//
//	return ERROR_OK;
//}
//
//void numicro8051_enable_breakpoints(struct target *target)
//{
//	struct breakpoint *breakpoint = target->breakpoints;
//
//	/* set any pending breakpoints */
//	while (breakpoint) {
//		if (!breakpoint->set)
//			numicro8051_set_breakpoint(target, breakpoint);
//		breakpoint = breakpoint->next;
//	}
//}
//
//static int numicro8051_write_regs(struct target *target, uint32_t regs[])
//{
//	struct hl_interface_s *adapter = target->tap->priv;
//	adapter->layout->api->write_reg(adapter->handle, (unsigned long)(0x20000000 | REG_SP ), regs[NUMICRO8051_SP]);
//	adapter->layout->api->write_reg(adapter->handle, (unsigned long)(0x20000000 | REG_PSW), regs[NUMICRO8051_PSW]);
//	adapter->layout->api->write_reg(adapter->handle, (unsigned long)(0x20000000 | REG_B  ), regs[NUMICRO8051_B]);
//	adapter->layout->api->write_reg(adapter->handle, (unsigned long)(0x20000000 | REG_ACC), regs[NUMICRO8051_ACC]);
//	adapter->layout->api->write_reg(adapter->handle, (unsigned long)(0x20000000 | REG_DPL), regs[NUMICRO8051_DPL]);
//	adapter->layout->api->write_reg(adapter->handle, (unsigned long)(0x20000000 | REG_DPH), regs[NUMICRO8051_DPH]);
//
//	unsigned char  CurrentBank = (regs[NUMICRO8051_PSW] >> 3) & 0x03;
//
//	adapter->layout->api->write_mem(adapter->handle, (unsigned long)(0x20000000 | (CurrentBank * 8)), 1, 8, (const char *)&regs[NUMICRO8051_R0]);	
//}

static int numicro8051_restore_context(struct target *target)
{
	unsigned int i;

	/* get pointers to arch-specific information */
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;

	nulink_8051_read_core_regs_all(adapter->handle, 0, NUMICRO8051_NUM_REGS-1, numicro8051->core_regs);
	numicro8051->core_regs[NUMICRO8051_PC] = m_stop_pc;

	for (i = 0; i < NUMICRO8051_NUM_REGS; i++) {
//		if (numicro8051->core_cache->reg_list[i].dirty)
			numicro8051->read_core_reg(target, i);
//			LOG_DEBUG("reg: 0x%x\n", numicro8051->core_regs[i]);
//			LOG_DEBUG("reg gdb: 0x%x\n", numicro8051->core_cache->reg_list[i].value);
	}
//	LOG_DEBUG("before write\n");
//	/* write core regs */
//	numicro8051_write_regs(target, numicro8051->core_regs);

	return ERROR_OK;
}

int numicro8051_resume(struct target *target, int current,
	uint32_t address, int handle_breakpoints, int debug_execution)
{
	//LOG_DEBUG("numicro8051_resume current: 0x%x, address: 0x%x, handle_breakpoints: 0x%x, debug_execution: 0x%x", current, address, handle_breakpoints, debug_execution);
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;
	struct breakpoint *breakpoint = NULL;
	uint32_t resume_pc;
//	struct reg *r;
	uint32_t retval, i;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

//	if (!debug_execution) {
//		target_free_all_working_areas(target);
//		numicro8051_enable_breakpoints(target);
//		numicro8051_enable_watchpoints(target);
//	}
//
//	if (debug_execution) {
//		r = numicro8051->core_cache->reg_list + ARMV7M_PRIMASK;
//
//		/* Disable interrupts */
//		/* We disable interrupts in the PRIMASK register instead of
//		 * masking with C_MASKINTS.  This is probably the same issue
//		 * as Cortex-M3 Erratum 377493 (fixed in r1p0):  C_MASKINTS
//		 * in parallel with disabled interrupts can cause local faults
//		 * to not be taken.
//		 *
//		 * REVISIT this clearly breaks non-debug execution, since the
//		 * PRIMASK register state isn't saved/restored...  workaround
//		 * by never resuming app code after debug execution.
//		 */
//		buf_set_u32(r->value, 0, 1, 1);
//		r->dirty = true;
//		r->valid = true;
//
//		/* Make sure we are in Thumb mode */
//		r = armv7m->arm.cpsr;
//		buf_set_u32(r->value, 24, 1, 1);
//		r->dirty = true;
//		r->valid = true;
//	}
//
	for (i = 0; i < NUMICRO8051_NUM_REGS; i++) {
		//if (!numicro8051->core_cache->reg_list[i].valid)
		numicro8051->read_core_reg(target, i);
	}

//	/* current = 1: continue on current pc, otherwise continue at <address> */
//	LOG_DEBUG("set pc before");
	if (!current) {
//		LOG_DEBUG("set pc");
		buf_set_u32(numicro8051->core_cache->reg_list[NUMICRO8051_PC].value,
			0, 32, address);
		numicro8051->core_cache->reg_list[NUMICRO8051_PC].dirty = true;
		numicro8051->core_cache->reg_list[NUMICRO8051_PC].valid = true;
	}
//	LOG_DEBUG("set pc after");

//	LOG_DEBUG("get pc before");
	if (!current) {
//		LOG_DEBUG("0x%x", address);
		resume_pc = address;
	}
	else {
//		LOG_DEBUG("get pc");
		resume_pc = buf_get_u32(
			numicro8051->core_cache->reg_list[NUMICRO8051_PC].value,
			0, 32);
//		LOG_DEBUG("0x%x", resume_pc);
	}
//	LOG_DEBUG("get pc after");

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			//LOG_DEBUG("unset breakpoint at 0x%x", breakpoint->address);
			numicro8051_remove_breakpoint(target, breakpoint);
			retval = adapter->layout->api->step(adapter->handle);
			if (retval != ERROR_OK)
				return retval;
			numicro8051_add_breakpoint(target, breakpoint);
		}
	}
//
//	/* Restart core */
//	numicro8051_write_debug_halt_mask(target, 0, C_HALT);
	retval = adapter->layout->api->run(adapter->handle);

	numicro8051_restore_context(target);

	if (retval != ERROR_OK)
		return retval;

	target->debug_reason = DBG_REASON_NOTHALTED;
//
//	/* registers are now invalid */
//	register_cache_invalidate(armv7m->arm.core_cache);
//
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
//
///* int irqstepcount = 0; */
int numicro8051_step(struct target *target, int current,
	uint32_t address, int handle_breakpoints)
{
//	LOG_DEBUG("numicro8051_step current: 0x%x, address: 0x%x, handle_breakpoints: 0x%x", current, address, handle_breakpoints);
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;
	struct breakpoint *breakpoint = NULL;
//	bool bkpt_inst_found = false;
	uint32_t retval, i;
//	bool isr_timed_out = false;
//
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (i = 0; i < NUMICRO8051_NUM_REGS; i++) {
		//if (!numicro8051->core_cache->reg_list[i].valid)
		numicro8051->read_core_reg(target, i);
	}
//	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(numicro8051->core_cache->reg_list[NUMICRO8051_PC].value, 0, 32, address);

	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, buf_get_u32(numicro8051->core_cache->reg_list[NUMICRO8051_PC].value, 0, 32));
		if (breakpoint) {
			//LOG_DEBUG("unset breakpoint at 0x%x", breakpoint->address);
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
//	/* if no bkpt instruction is found at pc then we can perform
//	 * a normal step, otherwise we have to manually step over the bkpt
//	 * instruction - as such simulate a step */
//	if (bkpt_inst_found == false) {
//		/* Automatic ISR masking mode off: Just step over the next instruction */
//		if ((numicro8051->isrmasking_mode != NUMICRO8051_ISRMASK_AUTO))
//			numicro8051_write_debug_halt_mask(target, C_STEP, C_HALT);
//		else {
//			/* Process interrupts during stepping in a way they don't interfere
//			 * debugging.
//			 *
//			 * Principle:
//			 *
//			 * Set a temporary break point at the current pc and let the core run
//			 * with interrupts enabled. Pending interrupts get served and we run
//			 * into the breakpoint again afterwards. Then we step over the next
//			 * instruction with interrupts disabled.
//			 *
//			 * If the pending interrupts don't complete within time, we leave the
//			 * core running. This may happen if the interrupts trigger faster
//			 * than the core can process them or the handler doesn't return.
//			 *
//			 * If no more breakpoints are available we simply do a step with
//			 * interrupts enabled.
//			 *
//			 */
//
//			/* 2012-09-29 ph
//			 *
//			 * If a break point is already set on the lower half word then a break point on
//			 * the upper half word will not break again when the core is restarted. So we
//			 * just step over the instruction with interrupts disabled.
//			 *
//			 * The documentation has no information about this, it was found by observation
//			 * on STM32F1 and STM32F2. Proper explanation welcome. STM32F0 dosen't seem to
//			 * suffer from this problem.
//			 *
//			 * To add some confusion: pc_value has bit 0 always set, while the breakpoint
//			 * address has it always cleared. The former is done to indicate thumb mode
//			 * to gdb.
//			 *
//			 */
//			if ((pc_value & 0x02) && breakpoint_find(target, pc_value & ~0x03)) {
//				LOG_DEBUG("Stepping over next instruction with interrupts disabled");
//				numicro8051_write_debug_halt_mask(target, C_HALT | C_MASKINTS, 0);
//				numicro8051_write_debug_halt_mask(target, C_STEP, C_HALT);
//				/* Re-enable interrupts */
//				numicro8051_write_debug_halt_mask(target, C_HALT, C_MASKINTS);
//			}
//			else {
//
//				/* Set a temporary break point */
//				if (breakpoint)
//					retval = numicro8051_set_breakpoint(target, breakpoint);
//				else
//					retval = breakpoint_add(target, pc_value, 2, BKPT_TYPE_BY_ADDR(pc_value));
//				bool tmp_bp_set = (retval == ERROR_OK);
//
//				/* No more breakpoints left, just do a step */
//				if (!tmp_bp_set)
//					numicro8051_write_debug_halt_mask(target, C_STEP, C_HALT);
//				else {
//					/* Start the core */
//					LOG_DEBUG("Starting core to serve pending interrupts");
//					int64_t t_start = timeval_ms();
//					numicro8051_write_debug_halt_mask(target, 0, C_HALT | C_STEP);
//
//					/* Wait for pending handlers to complete or timeout */
//					do {
//						retval = mem_ap_read_atomic_u32(armv7m->debug_ap,
//								DCB_DHCSR,
//								&numicro8051->dcb_dhcsr);
//						if (retval != ERROR_OK) {
//							target->state = TARGET_UNKNOWN;
//							return retval;
//						}
//						isr_timed_out = ((timeval_ms() - t_start) > 500);
//					} while (!((numicro8051->dcb_dhcsr & S_HALT) || isr_timed_out));
//
//					/* only remove breakpoint if we created it */
//					if (breakpoint)
//						numicro8051_unset_breakpoint(target, breakpoint);
//					else {
//						/* Remove the temporary breakpoint */
//						breakpoint_remove(target, pc_value);
//					}
//
//					if (isr_timed_out) {
//						LOG_DEBUG("Interrupt handlers didn't complete within time, "
//							"leaving target running");
//					} else {
//						/* Step over next instruction with interrupts disabled */
//						numicro8051_write_debug_halt_mask(target,
//							C_HALT | C_MASKINTS,
//							0);
//						numicro8051_write_debug_halt_mask(target, C_STEP, C_HALT);
//						/* Re-enable interrupts */
//						numicro8051_write_debug_halt_mask(target, C_HALT, C_MASKINTS);
//					}
//				}
//			}
//		}
//	}
//
//	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DHCSR, &numicro8051->dcb_dhcsr);
//	if (retval != ERROR_OK)
//		return retval;
//
//	/* registers are now invalid */
//	register_cache_invalidate(armv7m->arm.core_cache);
//
//	if (breakpoint)
//		numicro8051_set_breakpoint(target, breakpoint);
//
//	if (isr_timed_out) {
//		/* Leave the core running. The user has to stop execution manually. */
//		target->debug_reason = DBG_REASON_NOTHALTED;
//		target->state = TARGET_RUNNING;
//		return ERROR_OK;
//	}
//
//	LOG_DEBUG("target stepped dcb_dhcsr = 0x%" PRIx32
//		" nvic_icsr = 0x%" PRIx32,
//		numicro8051->dcb_dhcsr, numicro8051->nvic_icsr);
//
//	retval = numicro8051_debug_entry(target);

int numicro8051_assert_reset(struct target *target)
{
	int retval;
//	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
//	struct armv7m_common *armv7m = &numicro8051->armv7m;
//	enum numicro8051_soft_reset_config reset_config = numicro8051->soft_reset_config;
//

	struct hl_interface_s *adapter = target->tap->priv;
	retval = adapter->layout->api->reset(adapter->handle);
	if (retval != ERROR_OK)
		return retval;

//	enum reset_types jtag_reset_config = jtag_get_reset_config();
//
//	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT)) {
//		/* allow scripts to override the reset event */
//
//		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
//		register_cache_invalidate(numicro8051->armv7m.arm.core_cache);
//		target->state = TARGET_RESET;
//
//		return ERROR_OK;
//	}
//
//	/* some cores support connecting while srst is asserted
//	 * use that mode is it has been configured */
//
//	bool srst_asserted = false;
//
//	if ((jtag_reset_config & RESET_HAS_SRST) &&
//	    (jtag_reset_config & RESET_SRST_NO_GATING)) {
//		adapter_assert_reset();
//		srst_asserted = true;
//	}
//
//	/* Enable debug requests */
//	int retval;
//	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DHCSR, &numicro8051->dcb_dhcsr);
//	/* Store important errors instead of failing and proceed to reset assert */
//
//	if (retval != ERROR_OK || !(numicro8051->dcb_dhcsr & C_DEBUGEN))
//		retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DHCSR, DBGKEY | C_DEBUGEN);
//
//	/* If the processor is sleeping in a WFI or WFE instruction, the
//	 * C_HALT bit must be asserted to regain control */
//	if (retval == ERROR_OK && (numicro8051->dcb_dhcsr & S_SLEEP))
//		retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DHCSR, DBGKEY | C_HALT | C_DEBUGEN);
//
//	mem_ap_write_u32(armv7m->debug_ap, DCB_DCRDR, 0);
//	/* Ignore less important errors */
//
//	if (!target->reset_halt) {
//		/* Set/Clear C_MASKINTS in a separate operation */
//		if (numicro8051->dcb_dhcsr & C_MASKINTS)
//			mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DHCSR,
//					DBGKEY | C_DEBUGEN | C_HALT);
//
//		/* clear any debug flags before resuming */
//		numicro8051_clear_halt(target);
//
//		/* clear C_HALT in dhcsr reg */
//		numicro8051_write_debug_halt_mask(target, 0, C_HALT);
//	} else {
//		/* Halt in debug on reset; endreset_event() restores DEMCR.
//		 *
//		 * REVISIT catching BUSERR presumably helps to defend against
//		 * bad vector table entries.  Should this include MMERR or
//		 * other flags too?
//		 */
//		int retval2;
//		retval2 = mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DEMCR,
//				TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
//		if (retval != ERROR_OK || retval2 != ERROR_OK)
//			LOG_INFO("AP write error, reset will not halt");
//	}
//
//	if (jtag_reset_config & RESET_HAS_SRST) {
//		/* default to asserting srst */
//		if (!srst_asserted)
//			adapter_assert_reset();
//
//		/* srst is asserted, ignore AP access errors */
//		retval = ERROR_OK;
//	} else {
//		/* Use a standard Cortex-M3 software reset mechanism.
//		 * We default to using VECRESET as it is supported on all current cores.
//		 * This has the disadvantage of not resetting the peripherals, so a
//		 * reset-init event handler is needed to perform any peripheral resets.
//		 */
//		LOG_DEBUG("Using Cortex-M %s", (reset_config == NUMICRO8051_RESET_SYSRESETREQ)
//			? "SYSRESETREQ" : "VECTRESET");
//
//		if (reset_config == NUMICRO8051_RESET_VECTRESET) {
//			LOG_WARNING("Only resetting the Cortex-M core, use a reset-init event "
//				"handler to reset any peripherals or configure hardware srst support.");
//		}
//
//		int retval3;
//		retval3 = mem_ap_write_atomic_u32(armv7m->debug_ap, NVIC_AIRCR,
//				AIRCR_VECTKEY | ((reset_config == NUMICRO8051_RESET_SYSRESETREQ)
//				? AIRCR_SYSRESETREQ : AIRCR_VECTRESET));
//		if (retval3 != ERROR_OK)
//			LOG_DEBUG("Ignoring AP write error right after reset");
//
//		retval3 = dap_dp_init(armv7m->debug_ap->dap);
//		if (retval3 != ERROR_OK)
//			LOG_ERROR("DP initialisation failed");
//
//		else {
//			/* I do not know why this is necessary, but it
//			 * fixes strange effects (step/resume cause NMI
//			 * after reset) on LM3S6918 -- Michael Schwingen
//			 */
//			uint32_t tmp;
//			mem_ap_read_atomic_u32(armv7m->debug_ap, NVIC_AIRCR, &tmp);
//		}
//	}
//
//	target->state = TARGET_RESET;
//	jtag_add_sleep(50000);
//
//	register_cache_invalidate(numicro8051->armv7m.arm.core_cache);
//
//	/* now return stored error code if any */
//	if (retval != ERROR_OK)
//		return retval;
//
	if (target->reset_halt) {
		target->state = TARGET_RESET;
		target->debug_reason = DBG_REASON_DBGRQ;
	} else {
		target->state = TARGET_HALTED;
	}
//		retval = target_halt(target);
//		if (retval != ERROR_OK)
//			return retval;
//	}
//
	return ERROR_OK;
}
//
int numicro8051_deassert_reset(struct target *target)
{
	int retval;
//	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	struct hl_interface_s *adapter = target->tap->priv;
	retval = adapter->layout->api->reset(adapter->handle);
	if (retval != ERROR_OK)
		return retval;

//	struct armv7m_common *armv7m = &target_to_cm(target)->armv7m;
//
//	LOG_DEBUG("target->state: %s",
//		target_state_name(target));
//
//	/* deassert reset lines */
//	adapter_deassert_reset();
//
//	enum reset_types jtag_reset_config = jtag_get_reset_config();
//
//	if ((jtag_reset_config & RESET_HAS_SRST) &&
//	    !(jtag_reset_config & RESET_SRST_NO_GATING)) {
//		int retval = dap_dp_init(armv7m->debug_ap->dap);
//		if (retval != ERROR_OK) {
//			LOG_ERROR("DP initialisation failed");
//			return retval;
//		}
//	}
//
	return ERROR_OK;
}

int numicro8051_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
//	LOG_DEBUG("numicro8051_get_gdb_reg_list");
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

//	if (breakpoint->type == BKPT_HARD) {
	for (i = 0; i < BP_MAX; i++)
	{
		//LOG_DEBUG("find add bp index: %d", i);
		if (numicro8051->bp_check[i] == false)
		{
			g_bp_count = g_bp_count + 1;
			retval = adapter->layout->api->halt(adapter->handle);
			if (retval != ERROR_OK)
				return retval;

			numicro8051->bp_check[i] = true;
			numicro8051->bp_addr[i] = breakpoint->address & 0xFFFF;
			//LOG_DEBUG("add bp_check index found: %d", i);

			retval = nulink_set_breakpoint(adapter->handle, (breakpoint->address & 0xFFFF), g_bp_count);
			if (retval != ERROR_OK)
				return retval;

//			retval = adapter->layout->api->run(adapter->handle);
//			if (retval != ERROR_OK)
//				return retval;

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

//	for (i = 0; i < BP_MAX; i++)
//	{
//		LOG_DEBUG("bp check: %d, address: 0x%x", numicro8051->bp_check[i], numicro8051->bp_addr[i]);
//	}

	for (i = 0; i < BP_MAX; i++)
	{
		if (numicro8051->bp_check[i] && ((breakpoint->address & 0xFFFF) == numicro8051->bp_addr[i]))
		{
//			LOG_DEBUG("find remove bp index: %d", i);
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
//				LOG_DEBUG("recover bp check: %d, address: 0x%x", numicro8051->bp_check[j], numicro8051->bp_addr[j]);
				if (numicro8051->bp_check[j])
				{
					g_bp_count = g_bp_count + 1;
//					LOG_DEBUG("remove bp_check index found: %d", j);
					retval = nulink_set_breakpoint(adapter->handle, (numicro8051->bp_addr[j] & 0xFFFF), g_bp_count);
					if (retval != ERROR_OK)
						return retval;
				}
			}

//			retval = adapter->layout->api->run(adapter->handle);
//			if (retval != ERROR_OK)
//				return retval;

			break;
		}
	}
	return ERROR_OK;
}

//int numicro8051_set_watchpoint(struct target *target, struct watchpoint *watchpoint)
//{
//	int dwt_num = 0;
//	uint32_t mask, temp;
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//	struct armv7m_common *armv7m = target_to_armv7m(target);
//
//	/* watchpoint params were validated earlier */
//	mask = 0;
//	temp = watchpoint->length;
//	while (temp) {
//		temp >>= 1;
//		mask++;
//	}
//	mask--;
//
//	/* REVISIT Don't fully trust these "not used" records ... users
//	 * may set up breakpoints by hand, e.g. dual-address data value
//	 * watchpoint using comparator #1; comparator #0 matching cycle
//	 * count; send data trace info through ITM and TPIU; etc
//	 */
//	struct numicro8051_dwt_comparator *comparator;
//
//	for (comparator = numicro8051->dwt_comparator_list;
//		comparator->used && dwt_num < numicro8051->dwt_num_comp;
//		comparator++, dwt_num++)
//		continue;
//	if (dwt_num >= numicro8051->dwt_num_comp) {
//		LOG_ERROR("Can not find free DWT Comparator");
//		return ERROR_FAIL;
//	}
//	comparator->used = 1;
//	watchpoint->set = dwt_num + 1;
//
//	comparator->comp = watchpoint->address;
//	target_write_u32(target, comparator->dwt_comparator_address + 0,
//		comparator->comp);
//
//	if (!armv7m->arm.is_armv8m) {
//		comparator->mask = mask;
//		target_write_u32(target, comparator->dwt_comparator_address + 4,
//			comparator->mask);
//
//		switch (watchpoint->rw) {
//		case WPT_READ:
//			comparator->function = 5;
//			break;
//		case WPT_WRITE:
//			comparator->function = 6;
//			break;
//		case WPT_ACCESS:
//			comparator->function = 7;
//			break;
//		}
//		target_write_u32(target, comparator->dwt_comparator_address + 8,
//			comparator->function);
//
//		LOG_DEBUG("Watchpoint (ID %d) DWT%d 0x%08x 0x%x 0x%05x",
//			watchpoint->unique_id, dwt_num,
//			(unsigned)comparator->comp,
//			(unsigned)comparator->mask,
//			(unsigned)comparator->function);
//	}
//	else {
//		switch (watchpoint->length) {
//		case 1:
//			mask = 0;
//			break;
//		case 2:
//			mask = 1;
//			break;
//		case 4:
//			mask = 2;
//			break;
//		}
//		comparator->mask = mask;
//
//		switch (watchpoint->rw) {
//		case WPT_READ:
//			comparator->function = 5;
//			break;
//		case WPT_WRITE:
//			comparator->function = 6;
//			break;
//		case WPT_ACCESS:
//			comparator->function = 4;
//			break;
//		}
//
//		comparator->function = comparator->function + (mask << 10) + (1 << 4);
//		target_write_u32(target, comparator->dwt_comparator_address + 8,
//			comparator->function);
//
//		LOG_DEBUG("Watchpoint (ID %d) DWT%d 0x%08x 0x%08x",
//			watchpoint->unique_id, dwt_num,
//			(unsigned)comparator->comp,
//			(unsigned)comparator->function);
//	}
//
//	return ERROR_OK;
//}
//
//int numicro8051_unset_watchpoint(struct target *target, struct watchpoint *watchpoint)
//{
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//	struct numicro8051_dwt_comparator *comparator;
//	int dwt_num;
//
//	if (!watchpoint->set) {
//		LOG_WARNING("watchpoint (wpid: %d) not set",
//			watchpoint->unique_id);
//		return ERROR_OK;
//	}
//
//	dwt_num = watchpoint->set - 1;
//
//	LOG_DEBUG("Watchpoint (ID %d) DWT%d address: 0x%08x clear",
//		watchpoint->unique_id, dwt_num,
//		(unsigned) watchpoint->address);
//
//	if ((dwt_num < 0) || (dwt_num >= numicro8051->dwt_num_comp)) {
//		LOG_DEBUG("Invalid DWT Comparator number in watchpoint");
//		return ERROR_OK;
//	}
//
//	comparator = numicro8051->dwt_comparator_list + dwt_num;
//	comparator->used = 0;
//	comparator->function = 0;
//	target_write_u32(target, comparator->dwt_comparator_address + 8,
//		comparator->function);
//
//	watchpoint->set = false;
//
//	return ERROR_OK;
//}
//
//int numicro8051_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
//{
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//
//	if (numicro8051->dwt_comp_available < 1) {
//		LOG_DEBUG("no comparators?");
//		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
//	}
//
//	/* hardware doesn't support data value masking */
//	if (watchpoint->mask != ~(uint32_t)0) {
//		LOG_DEBUG("watchpoint value masks not supported");
//		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
//	}
//
//	/* hardware allows address masks of up to 32K */
//	unsigned mask;
//
//	for (mask = 0; mask < 16; mask++) {
//		if ((1u << mask) == watchpoint->length)
//			break;
//	}
//	if (mask == 16) {
//		LOG_DEBUG("unsupported watchpoint length");
//		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
//	}
//	if (watchpoint->address & ((1 << mask) - 1)) {
//		LOG_DEBUG("watchpoint address is unaligned");
//		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
//	}
//
//	/* Caller doesn't seem to be able to describe watching for data
//	 * values of zero; that flags "no value".
//	 *
//	 * REVISIT This DWT may well be able to watch for specific data
//	 * values.  Requires comparator #1 to set DATAVMATCH and match
//	 * the data, and another comparator (DATAVADDR0) matching addr.
//	 */
//	if (watchpoint->value) {
//		LOG_DEBUG("data value watchpoint not YET supported");
//		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
//	}
//
//	numicro8051->dwt_comp_available--;
//	LOG_DEBUG("dwt_comp_available: %d", numicro8051->dwt_comp_available);
//
//	return ERROR_OK;
//}
//
//int numicro8051_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
//{
//	struct numicro8051_common *numicro8051 = target_to_cm(target);
//
//	/* REVISIT why check? DWT can be updated with core running ... */
//	if (target->state != TARGET_HALTED) {
//		LOG_WARNING("target not halted");
//		return ERROR_TARGET_NOT_HALTED;
//	}
//
//	if (watchpoint->set)
//		numicro8051_unset_watchpoint(target, watchpoint);
//
//	numicro8051->dwt_comp_available++;
//	LOG_DEBUG("dwt_comp_available: %d", numicro8051->dwt_comp_available);
//
//	return ERROR_OK;
//}
//
//void numicro8051_enable_watchpoints(struct target *target)
//{
//	struct watchpoint *watchpoint = target->watchpoints;
//
//	/* set any pending watchpoints */
//	while (watchpoint) {
//		if (!watchpoint->set)
//			numicro8051_set_watchpoint(target, watchpoint);
//		watchpoint = watchpoint->next;
//	}
//}
//
//static int numicro8051_load_core_reg_u32(struct target *target,
//		uint32_t num, uint32_t *value)
//{
//	int retval;
//
//	/* NOTE:  we "know" here that the register identifiers used
//	 * in the v7m header match the Cortex-M3 Debug Core Register
//	 * Selector values for R0..R15, xPSR, MSP, and PSP.
//	 */
//	switch (num) {
//		case 0 ... 18:
//			/* read a normal core register */
//			retval = numicro8051_dap_read_coreregister_u32(target, value, num);
//
//			if (retval != ERROR_OK) {
//				LOG_ERROR("JTAG failure %i", retval);
//				return ERROR_JTAG_DEVICE_ERROR;
//			}
//			LOG_DEBUG("load from core reg %i  value 0x%" PRIx32 "", (int)num, *value);
//			break;
//
//		case ARMV7M_FPSCR:
//			/* Floating-point Status and Registers */
//			retval = target_write_u32(target, DCB_DCRSR, 0x21);
//			if (retval != ERROR_OK)
//				return retval;
//			retval = target_read_u32(target, DCB_DCRDR, value);
//			if (retval != ERROR_OK)
//				return retval;
//			LOG_DEBUG("load from FPSCR  value 0x%" PRIx32, *value);
//			break;
//
//		case ARMV7M_S0 ... ARMV7M_S31:
//			/* Floating-point Status and Registers */
//			retval = target_write_u32(target, DCB_DCRSR, num - ARMV7M_S0 + 0x40);
//			if (retval != ERROR_OK)
//				return retval;
//			retval = target_read_u32(target, DCB_DCRDR, value);
//			if (retval != ERROR_OK)
//				return retval;
//			LOG_DEBUG("load from FPU reg S%d  value 0x%" PRIx32,
//				  (int)(num - ARMV7M_S0), *value);
//			break;
//
//		case ARMV7M_PRIMASK:
//		case ARMV7M_BASEPRI:
//		case ARMV7M_FAULTMASK:
//		case ARMV7M_CONTROL:
//			/* Cortex-M3 packages these four registers as bitfields
//			 * in one Debug Core register.  So say r0 and r2 docs;
//			 * it was removed from r1 docs, but still works.
//			 */
//			numicro8051_dap_read_coreregister_u32(target, value, 20);
//
//			switch (num) {
//				case ARMV7M_PRIMASK:
//					*value = buf_get_u32((uint8_t *)value, 0, 1);
//					break;
//
//				case ARMV7M_BASEPRI:
//					*value = buf_get_u32((uint8_t *)value, 8, 8);
//					break;
//
//				case ARMV7M_FAULTMASK:
//					*value = buf_get_u32((uint8_t *)value, 16, 1);
//					break;
//
//				case ARMV7M_CONTROL:
//					*value = buf_get_u32((uint8_t *)value, 24, 2);
//					break;
//			}
//
//			LOG_DEBUG("load from special reg %i value 0x%" PRIx32 "", (int)num, *value);
//			break;
//
//		default:
//			return ERROR_COMMAND_SYNTAX_ERROR;
//	}
//
//	return ERROR_OK;
//}
//
//static int numicro8051_store_core_reg_u32(struct target *target,
//		uint32_t num, uint32_t value)
//{
//	int retval;
//	uint32_t reg;
//	struct armv7m_common *armv7m = target_to_armv7m(target);
//
//	/* NOTE:  we "know" here that the register identifiers used
//	 * in the v7m header match the Cortex-M3 Debug Core Register
//	 * Selector values for R0..R15, xPSR, MSP, and PSP.
//	 */
//	switch (num) {
//		case 0 ... 18:
//			retval = numicro8051_dap_write_coreregister_u32(target, value, num);
//			if (retval != ERROR_OK) {
//				struct reg *r;
//
//				LOG_ERROR("JTAG failure");
//				r = armv7m->arm.core_cache->reg_list + num;
//				r->dirty = r->valid;
//				return ERROR_JTAG_DEVICE_ERROR;
//			}
//			LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", (int)num, value);
//			break;
//
//		case ARMV7M_FPSCR:
//			/* Floating-point Status and Registers */
//			retval = target_write_u32(target, DCB_DCRDR, value);
//			if (retval != ERROR_OK)
//				return retval;
//			retval = target_write_u32(target, DCB_DCRSR, 0x21 | (1<<16));
//			if (retval != ERROR_OK)
//				return retval;
//			LOG_DEBUG("write FPSCR value 0x%" PRIx32, value);
//			break;
//
//		case ARMV7M_S0 ... ARMV7M_S31:
//			/* Floating-point Status and Registers */
//			retval = target_write_u32(target, DCB_DCRDR, value);
//			if (retval != ERROR_OK)
//				return retval;
//			retval = target_write_u32(target, DCB_DCRSR, (num - ARMV7M_S0 + 0x40) | (1<<16));
//			if (retval != ERROR_OK)
//				return retval;
//			LOG_DEBUG("write FPU reg S%d  value 0x%" PRIx32,
//				  (int)(num - ARMV7M_S0), value);
//			break;
//
//		case ARMV7M_PRIMASK:
//		case ARMV7M_BASEPRI:
//		case ARMV7M_FAULTMASK:
//		case ARMV7M_CONTROL:
//			/* Cortex-M3 packages these four registers as bitfields
//			 * in one Debug Core register.  So say r0 and r2 docs;
//			 * it was removed from r1 docs, but still works.
//			 */
//			numicro8051_dap_read_coreregister_u32(target, &reg, 20);
//
//			switch (num) {
//				case ARMV7M_PRIMASK:
//					buf_set_u32((uint8_t *)&reg, 0, 1, value);
//					break;
//
//				case ARMV7M_BASEPRI:
//					buf_set_u32((uint8_t *)&reg, 8, 8, value);
//					break;
//
//				case ARMV7M_FAULTMASK:
//					buf_set_u32((uint8_t *)&reg, 16, 1, value);
//					break;
//
//				case ARMV7M_CONTROL:
//					buf_set_u32((uint8_t *)&reg, 24, 2, value);
//					break;
//			}
//
//			numicro8051_dap_write_coreregister_u32(target, reg, 20);
//
//			LOG_DEBUG("write special reg %i value 0x%" PRIx32 " ", (int)num, value);
//			break;
//
//		default:
//			return ERROR_COMMAND_SYNTAX_ERROR;
//	}
//
//	return ERROR_OK;
//}
//

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
	//LOG_DEBUG("ReadCode nAdr: 0x%x, size: 0x%x, count: 0x%x", nAdr, size, count);
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
	else //if (numicro8051->uSCodeAddr < numicro8051->uProgramFlashSize)
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
		//LOG_DEBUG("uLen_c: 0x%x", uLen_c);
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
		//LOG_DEBUG("uLen_l: 0x%x", uLen_l);
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
			//LOG_DEBUG("uLen_s: 0x%x", uLen_s);
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

	//---TODO:
	//---if Ok, then return 0, else error-address
	//   adr |= (amIDATA << 24);	// make address uVision2 conforming
	return retval;						// say Ok.
}

int numicro8051_read_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	//LOG_DEBUG("numicro8051_read_memory address: 0x%x, size: 0x%x, count:0x%x", address, size, count);
	int retval;
	unsigned long uZone, uAddr, uAddr_valid, address_align, offset = 0;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	uAddr_valid = (address & 0xFFFFFF) >> 16;
	if (uAddr_valid == 0xFF) {
		address_align = (address + size * count) & 0xFF000000;
		offset = address_align - address;
		count = count - (offset / size);
		address = address_align;
	}

	uZone = address >> 24;
	uAddr = address & 0xFFFFFF;

	switch (uZone)				// extract mSpace from address.
	{
		case amDATA:			// Data
		{
			LOG_DEBUG("amDATA1");
			retval = ReadData(target, uAddr, size, count, buffer + offset);
			//LOG_DEBUG("amDATA2");
			break;	
		}
		case amIDATA:			// Idata
		{
			LOG_DEBUG("amIDATA1");
			retval = ReadIdata(target, uAddr, size, count, buffer + offset);
			//LOG_DEBUG("amIDATA2");
			break;
		}
		case amXDATA:			// Xdata
		{
			LOG_DEBUG("amXDATA1");
			retval = ReadXdata(target, uAddr, size, count, buffer + offset);
			//LOG_DEBUG("amXDATA2");
			break;
		}
		case amCODE:			// Code
		default:
		{
			LOG_DEBUG("amCODE1");
			retval = ReadCode(target, uAddr, size, count, buffer + offset);
			//LOG_DEBUG("amCODE2");
			break;
		}


		//case amPDATA:
//		default:				// anything else assumed to be c-address
//			retval = ERROR_FAIL;
	}

	return retval;
}

int WriteData(struct target *target, uint32_t nAdr, uint32_t size, uint32_t count, const uint8_t *buffer)
{
	//LOG_DEBUG("WriteData nAdr: 0x%x, size: 0x%x, count: 0x%x", nAdr, size, count);
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
	//LOG_DEBUG("WriteIdata nAdr: 0x%x, size: 0x%x, count: 0x%x", nAdr, size, count);
	int retval = ERROR_OK;
//	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
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
	//LOG_DEBUG("WriteXdata nAdr: 0x%x, size: 0x%x, count: 0x%x", nAdr, size, count);
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
	//LOG_DEBUG("WriteCode nAdr: 0x%x, size: 0x%x, count: 0x%x", nAdr, size, count);
	return ERROR_OK;
}

int numicro8051_write_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	//LOG_DEBUG("numicro8051_write_memory");
	int retval;
	unsigned long uZone, uAddr, uAddr_valid, address_align, offset = 0;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	uAddr_valid = (address & 0xFFFFFF) >> 16;
	if (uAddr_valid == 0xFF) {
		address_align = (address + size * count) & 0xFF000000;
		offset = address_align - address;
		count = count - (offset / size);
		address = address_align;
	}

	uZone = (address >> 24) & 0xFF;
	uAddr = address & 0xFFFFFF;

	switch (uZone)				// extract mSpace from address.
	{
		case amDATA:			// Data
		{
			LOG_DEBUG("amDATA1");
			retval = WriteData(target, uAddr, size, count, buffer + offset);
			//LOG_DEBUG("amDATA2");
			break;
		}
		case amIDATA:			// Idata
		{
			LOG_DEBUG("amIDATA1");
			retval = WriteIdata(target, uAddr, size, count, buffer + offset);
			//LOG_DEBUG("amIDATA2");
			break;
		}
		case amXDATA:			// Xdata
		{
			LOG_DEBUG("amXDATA1");
			retval = WriteXdata(target, uAddr, size, count, buffer + offset);
			//LOG_DEBUG("amXDATA2");
			break;
		}
		case amCODE:			// Code
		default:
		{
			LOG_DEBUG("amCODE1");
			retval = WriteCode(target, uAddr, size, count, buffer + offset);
			//LOG_DEBUG("amCODE2");
			break;
		}
		//case amPDATA:
//		default:				// anything else assumed to be c-address
//			retval = ERROR_FAIL;
	}

	return retval;

//	struct armv7m_common *armv7m = target_to_armv7m(target);
//
//	if (armv7m->arm.is_armv6m) {
//		/* armv6m does not handle unaligned memory access */
//		if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
//			return ERROR_TARGET_UNALIGNED_ACCESS;
//	}
//
}
//
static int numicro8051_get_core_reg(struct reg *reg)
{
	//LOG_DEBUG("numicro8051_get_core_reg");
	int retval = ERROR_OK;
	struct numicro8051_core_reg *numicro8051_reg = reg->arch_info;
	struct target *target = numicro8051_reg->target;
	struct numicro8051_common *numicro8051_target = target_to_numicro8051(target);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	//LOG_DEBUG("read reg value 0x%x", numicro8051_target->core_regs[numicro8051_reg->num]);
	retval = numicro8051_target->read_core_reg(target, numicro8051_reg->num);

	return retval;
}

static int numicro8051_set_core_reg(struct reg *reg, uint8_t *buf)
{
	//LOG_DEBUG("numicro8051_set_core_reg");
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
	//LOG_DEBUG("numicro8051_build_reg_cache");
	/* get pointers to arch-specific information */
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
	//LOG_DEBUG("numicro8051_init_target");
	numicro8051_build_reg_cache(target);
	return ERROR_OK;
}

int numicro8051_arch_state(struct target *target)
{
	//LOG_DEBUG("numicro8051_arch_state");
//	struct hl_interface_s *adapter = target->tap->priv;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	numicro8051_poll(target);

	//LOG_DEBUG("m_stop_pc: 0x%x", m_stop_pc);
//	numicro8051->core_regs[NUMICRO8051_PC] = m_stop_pc;
//	for (i = 0; i < NUMICRO8051_NUM_REGS; i++) {
//		//if (!numicro8051->core_cache->reg_list[i].valid)
//		numicro8051->read_core_reg(target, i);
//	}

	LOG_USER("target halted due to %s, pc: 0x%8.8" PRIx32 "",
		debug_reason_name(target),
		buf_get_u32(numicro8051->core_cache->reg_list[NUMICRO8051_PC].value, 0, 32));

	return ERROR_OK;
}

//
void numicro8051_deinit_target(struct target *target)
{
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
//
//	free(numicro8051->fp_comparator_list);
//
//	numicro8051_dwt_free(target);
//	armv7m_free_reg_cache(target);
//
	free(numicro8051);
}
//
///* REVISIT cache valid/dirty bits are unmaintained.  We could set "valid"
// * on r/w if the core is not running, and clear on resume or reset ... or
// * at least, in a post_restore_context() method.
// */
//
//struct dwt_reg_state {
//	struct target *target;
//	uint32_t addr;
//	uint8_t value[4];		/* scratch/cache */
//};
//
//static int numicro8051_dwt_get_reg(struct reg *reg)
//{
//	struct dwt_reg_state *state = reg->arch_info;
//
//	uint32_t tmp;
//	int retval = target_read_u32(state->target, state->addr, &tmp);
//	if (retval != ERROR_OK)
//		return retval;
//
//	buf_set_u32(state->value, 0, 32, tmp);
//	return ERROR_OK;
//}
//
//static int numicro8051_dwt_set_reg(struct reg *reg, uint8_t *buf)
//{
//	struct dwt_reg_state *state = reg->arch_info;
//
//	return target_write_u32(state->target, state->addr,
//			buf_get_u32(buf, 0, reg->size));
//}
//
//struct dwt_reg {
//	uint32_t addr;
//	char *name;
//	unsigned size;
//};
//
//static struct dwt_reg dwt_base_regs[] = {
//	{ DWT_CTRL, "dwt_ctrl", 32, },
//	/* NOTE that Erratum 532314 (fixed r2p0) affects CYCCNT:  it wrongly
//	 * increments while the core is asleep.
//	 */
//	{ DWT_CYCCNT, "dwt_cyccnt", 32, },
//	/* plus some 8 bit counters, useful for profiling with TPIU */
//};
//
//static struct dwt_reg dwt_comp[] = {
//#define DWT_COMPARATOR(i) \
//		{ DWT_COMP0 + 0x10 * (i), "dwt_" #i "_comp", 32, }, \
//		{ DWT_MASK0 + 0x10 * (i), "dwt_" #i "_mask", 4, }, \
//		{ DWT_FUNCTION0 + 0x10 * (i), "dwt_" #i "_function", 32, }
//	DWT_COMPARATOR(0),
//	DWT_COMPARATOR(1),
//	DWT_COMPARATOR(2),
//	DWT_COMPARATOR(3),
//#undef DWT_COMPARATOR
//};
//
//static const struct reg_arch_type dwt_reg_type = {
//	.get = numicro8051_dwt_get_reg,
//	.set = numicro8051_dwt_set_reg,
//};
//
//static void numicro8051_dwt_addreg(struct target *t, struct reg *r, struct dwt_reg *d)
//{
//	struct dwt_reg_state *state;
//
//	state = calloc(1, sizeof *state);
//	if (!state)
//		return;
//	state->addr = d->addr;
//	state->target = t;
//
//	r->name = d->name;
//	r->size = d->size;
//	r->value = state->value;
//	r->arch_info = state;
//	r->type = &dwt_reg_type;
//}
//
//void numicro8051_dwt_setup(struct numicro8051_common *cm, struct target *target)
//{
//	uint32_t dwtcr, dwtfun;
//	struct reg_cache *cache;
//	struct numicro8051_dwt_comparator *comparator;
//	int reg, i;
//	bool bDisplayDWT_FUNCTION = false;
//
//	target_read_u32(target, DWT_CTRL, &dwtcr);
//	if (!dwtcr) {
//		LOG_DEBUG("no DWT");
//		return;
//	}
//
//	cm->dwt_num_comp = (dwtcr >> 28) & 0xF;
//	cm->dwt_comp_available = cm->dwt_num_comp;
//	cm->dwt_comparator_list = calloc(cm->dwt_num_comp,
//			sizeof(struct numicro8051_dwt_comparator));
//	if (!cm->dwt_comparator_list) {
//fail0:
//		cm->dwt_num_comp = 0;
//		LOG_ERROR("out of mem");
//		return;
//	}
//
//	cache = calloc(1, sizeof *cache);
//	if (!cache) {
//fail1:
//		free(cm->dwt_comparator_list);
//		goto fail0;
//	}
//	cache->name = "Cortex-M DWT registers";
//	cache->num_regs = 2 + cm->dwt_num_comp * 3;
//	cache->reg_list = calloc(cache->num_regs, sizeof *cache->reg_list);
//	if (!cache->reg_list) {
//		free(cache);
//		goto fail1;
//	}
//
//	for (reg = 0; reg < 2; reg++)
//		numicro8051_dwt_addreg(target, cache->reg_list + reg,
//			dwt_base_regs + reg);
//
//	comparator = cm->dwt_comparator_list;
//	for (i = 0; i < cm->dwt_num_comp; i++, comparator++) {
//		int j;
//
//		comparator->dwt_comparator_address = DWT_COMP0 + 0x10 * i;
//		for (j = 0; j < 3; j++, reg++)
//			numicro8051_dwt_addreg(target, cache->reg_list + reg,
//				dwt_comp + 3 * i + j);
//
//		/* make sure we clear any watchpoints enabled on the target */
//		target_write_u32(target, comparator->dwt_comparator_address + 8, 0);
//
//		if (bDisplayDWT_FUNCTION) {
//			target_read_u32(target, comparator->dwt_comparator_address + 8, &dwtfun);
//			LOG_DEBUG("DWT_FUNCTION%d: 0x%" PRIx32 "",
//				i,
//				dwtfun);
//		}
//	}
//
//	*register_get_last_cache_p(&target->reg_cache) = cache;
//	cm->dwt_cache = cache;
//
//	LOG_DEBUG("DWT dwtcr 0x%" PRIx32 ", comp %d, watch%s",
//		dwtcr, cm->dwt_num_comp,
//		(dwtcr & (0xf << 24)) ? " only" : "/trigger");
//
//	/* REVISIT:  if num_comp > 1, check whether comparator #1 can
//	 * implement single-address data value watchpoints ... so we
//	 * won't need to check it later, when asked to set one up.
//	 */
//}
//
//static void numicro8051_dwt_free(struct target *target)
//{
//	struct numicro8051_common *cm = target_to_cm(target);
//	struct reg_cache *cache = cm->dwt_cache;
//
//	free(cm->dwt_comparator_list);
//	cm->dwt_comparator_list = NULL;
//	cm->dwt_num_comp = 0;
//
//	if (cache) {
//		register_unlink_cache(&target->reg_cache, cache);
//
//		if (cache->reg_list) {
//			for (size_t i = 0; i < cache->num_regs; i++)
//				free(cache->reg_list[i].arch_info);
//			free(cache->reg_list);
//		}
//		free(cache);
//	}
//	cm->dwt_cache = NULL;
//}

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

//static int numicro8051_dcc_read(struct target *target, uint8_t *value, uint8_t *ctrl)
//{
//	struct armv7m_common *armv7m = target_to_armv7m(target);
//	uint16_t dcrdr;
//	uint8_t buf[2];
//	int retval;
//
//	retval = mem_ap_read_buf_noincr(armv7m->debug_ap, buf, 2, 1, DCB_DCRDR);
//	if (retval != ERROR_OK)
//		return retval;
//
//	dcrdr = target_buffer_get_u16(target, buf);
//	*ctrl = (uint8_t)dcrdr;
//	*value = (uint8_t)(dcrdr >> 8);
//
//	LOG_DEBUG("data 0x%x ctrl 0x%x", *value, *ctrl);
//
//	/* write ack back to software dcc register
//	 * signify we have read data */
//	if (dcrdr & (1 << 0)) {
//		target_buffer_set_u16(target, buf, 0);
//		retval = mem_ap_write_buf_noincr(armv7m->debug_ap, buf, 2, 1, DCB_DCRDR);
//		if (retval != ERROR_OK)
//			return retval;
//	}
//
//	return ERROR_OK;
//}
//
//static int numicro8051_target_request_data(struct target *target,
//	uint32_t size, uint8_t *buffer)
//{
//	uint8_t data;
//	uint8_t ctrl;
//	uint32_t i;
//
//	for (i = 0; i < (size * 4); i++) {
//		int retval = numicro8051_dcc_read(target, &data, &ctrl);
//		if (retval != ERROR_OK)
//			return retval;
//		buffer[i] = data;
//	}
//
//	return ERROR_OK;
//}
//
//static int numicro8051_handle_target_request(void *priv)
//{
//	struct target *target = priv;
//	if (!target_was_examined(target))
//		return ERROR_OK;
//
//	if (!target->dbg_msg_enabled)
//		return ERROR_OK;
//
//	if (target->state == TARGET_RUNNING) {
//		uint8_t data;
//		uint8_t ctrl;
//		int retval;
//
//		retval = numicro8051_dcc_read(target, &data, &ctrl);
//		if (retval != ERROR_OK)
//			return retval;
//
//		/* check if we have data */
//		if (ctrl & (1 << 0)) {
//			uint32_t request;
//
//			/* we assume target is quick enough */
//			request = data;
//			for (int i = 1; i <= 3; i++) {
//				retval = numicro8051_dcc_read(target, &data, &ctrl);
//				if (retval != ERROR_OK)
//					return retval;
//				request |= ((uint32_t)data << (i * 8));
//			}
//			target_request(target, request);
//		}
//	}
//
//	return ERROR_OK;
//}
//
static int numicro8051_init_arch_info(struct target *target,
	struct numicro8051_common *numicro8051, struct jtag_tap *tap)
{
	//LOG_DEBUG("numicro8051_init_arch_info");
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

//	numicro8051_init_flash_regs(0, numicro8051);

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
	//LOG_DEBUG("numicro8051_write_core_reg");
	uint32_t reg_value;

	/* get pointers to arch-specific information */
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	if (num >= NUMICRO8051_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = buf_get_u32(numicro8051->core_cache->reg_list[num].value, 0, 32);
	numicro8051->core_regs[num] = reg_value;
	//LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", num, reg_value);
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

	//LOG_DEBUG("hw breakpoints: numinst %i numdata %i", numicro8051->num_hw_bpoints,
	//	numicro8051->num_hw_bpoints);

	numicro8051->bp_scanned = true;

	return ERROR_OK;
}

int numicro8051_target_create(struct target *target, Jim_Interp *interp)
{
	//LOG_DEBUG("numicro8051_target_create");
	struct numicro8051_common *numicro8051 = calloc(1, sizeof(struct numicro8051_common));

	numicro8051_init_arch_info(target, numicro8051, target->tap);
	numicro8051_configure_break_unit(target);

	return ERROR_OK;
}


struct target_type numicro8051_target = {
	.name = "nu8051",
//	.deprecated_name = "numicro80513",

	.poll = numicro8051_poll,
	.arch_state = numicro8051_arch_state,
//
//	.target_request_data = numicro8051_target_request_data,
//
	.halt = numicro8051_halt,
	.resume = numicro8051_resume,
	.step = numicro8051_step,
//
	.assert_reset = numicro8051_assert_reset,
	.deassert_reset = numicro8051_deassert_reset,
//	.soft_reset_halt = numicro8051_soft_reset_halt,
//
	.get_gdb_reg_list = numicro8051_get_gdb_reg_list,
//
	.read_memory = numicro8051_read_memory,
	.write_memory = numicro8051_write_memory,
//	.checksum_memory = armv7m_checksum_memory,
//	.blank_check_memory = armv7m_blank_check_memory,
//
//	.run_algorithm = armv7m_run_algorithm,
//	.start_algorithm = armv7m_start_algorithm,
//	.wait_algorithm = armv7m_wait_algorithm,
//
	.add_breakpoint = numicro8051_add_breakpoint,
	.remove_breakpoint = numicro8051_remove_breakpoint,
//	.add_watchpoint = numicro8051_add_watchpoint,
//	.remove_watchpoint = numicro8051_remove_watchpoint,
//
	.target_create = numicro8051_target_create,
	.init_target = numicro8051_init_target,
	.examine = numicro8051_examine,
	.deinit_target = numicro8051_deinit_target,
};
