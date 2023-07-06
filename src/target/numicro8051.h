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
 ***************************************************************************/

#ifndef OPENOCD_TARGET_NUMICRO8051_H
#define OPENOCD_TARGET_NUMICRO8051_H

struct target;

#define NUMICRO8051_COMMON_MAGIC 0x1A451A45
#define NUMICRO8051_NUM_CORE_REGS 15//6
#define BP_MAX 8
#define REG_SP				0x81
#define REG_DPL				0x82
#define REG_DPH				0x83
#define REG_SFRPAGE			0x91
#define REG_PSW				0xD0
#define REG_ACC				0xE0
#define REG_B				0xF0
#define amDATA   0x00F0           // DATA
#define amIDATA  0x00F1           // IDATA
#define amXDATA  0x00F2           // XDATA
#define amCODE   0x0000           // CODE 

struct numicro8051_common {
	uint32_t uLIBROMAddr;
	uint32_t uSCodeAddr;
	uint32_t uLIBROMSize;
	uint32_t uProgramFlashAddr;
	uint32_t uProgramFlashSize;
	uint32_t uSPROMAddr;
	uint32_t uSPROMSize;
	uint32_t uSPROMMode;
	uint32_t bSupportSCode;
	uint32_t uReadMemCount;
	uint32_t uSFR_PageNum;
	uint32_t uSFR_TKAddr;
	uint32_t uSFR_TKSize;
	uint32_t uXRAMSize;
	uint32_t uDeviceID;
	uint32_t uPartID;
	uint32_t uMemorySpace;
	void *arch_info;
	struct reg_cache *core_cache;
	uint32_t core_regs[NUMICRO8051_NUM_CORE_REGS];

	/* working area for fastdata access */
	struct working_area *fast_data_area;

	bool swim_configured;
	bool bp_scanned;
	uint8_t num_hw_bpoints;
	uint8_t num_hw_bpoints_avail;
	bool bp_check[BP_MAX];
	uint32_t bp_addr[BP_MAX];
	struct numicro8051_comparator *hw_break_list;

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, unsigned int num);
	int (*write_core_reg)(struct target *target, unsigned int num);
};

extern int nulink_erase_flash(uint32_t addr, uint32_t len);
extern int nulink_write_flash(uint32_t addr, uint32_t len, const uint8_t *buffer);
extern int nulink_read_flash(uint32_t addr, uint32_t len, uint8_t *buffer);
int nulink_set_flash_mode();
extern int nulink_mcu_disconnect();
extern int nulink_flash_init();
extern int nulink_flash_sprom_init();
extern int nulink_flash_uninit();
extern int nulink_flash_sprom_uninit(int uSPROM_Mode);
extern int nulink_8051_reset(void *handle);

static inline struct numicro8051_common *
target_to_numicro8051(struct target *target)
{
	return target->arch_info;
}

int numicro8051_examine(struct target *target);
int numicro8051_init_target(struct command_context *cmd_ctx, struct target *target);
int numicro8051_target_create(struct target *target, Jim_Interp *interp);
int numicro8051_arch_state(struct target *target);
int numicro8051_poll(struct target *target);
int numicro8051_halt(struct target *target);
int numicro8051_resume(struct target *target, int current,
	uint32_t address, int handle_breakpoints, int debug_execution);
int numicro8051_step(struct target *target, int current,
	uint32_t address, int handle_breakpoints);
int numicro8051_assert_reset(struct target *target);
int numicro8051_deassert_reset(struct target *target);
int numicro8051_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class);
int numicro8051_read_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, uint8_t *buffer);
int numicro8051_write_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer);
//int cortex_m_set_breakpoint(struct target *target, struct breakpoint *breakpoint);
//int cortex_m_unset_breakpoint(struct target *target, struct breakpoint *breakpoint);
int numicro8051_add_breakpoint(struct target *target, struct breakpoint *breakpoint);
int numicro8051_remove_breakpoint(struct target *target, struct breakpoint *breakpoint);
//int cortex_m_set_watchpoint(struct target *target, struct watchpoint *watchpoint);
//int cortex_m_unset_watchpoint(struct target *target, struct watchpoint *watchpoint);
//int cortex_m_add_watchpoint(struct target *target, struct watchpoint *watchpoint);
//int cortex_m_remove_watchpoint(struct target *target, struct watchpoint *watchpoint);
//void cortex_m_enable_breakpoints(struct target *target);
//void cortex_m_enable_watchpoints(struct target *target);
//void cortex_m_dwt_setup(struct cortex_m_common *cm, struct target *target);
void numicro8051_deinit_target(struct target *target);

//const struct command_registration numicro8051_command_handlers[];

#endif /* OPENOCD_TARGET_NUMICRO8051_H */
