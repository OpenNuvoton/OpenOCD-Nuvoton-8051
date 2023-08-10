/***************************************************************************
 *   Copyright (C) 2016-2017 by Nuvoton                                    *
 *   Zale Yu <cyyu@nuvoton.com>                                            *
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

/* project specific includes */
#include <helper/binarybuffer.h>
#include <jtag/interface.h>
#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>
#include <target/target.h>
#include <target/cortex_m.h>
#include "log.h"
#include "libusb_common.h"
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#define ENDPOINT_IN  0x80
#define ENDPOINT_OUT 0x00

#define NULINK_WRITE_TIMEOUT 1000
#define NULINK_READ_TIMEOUT  1000

#define NULINK_INTERFACE_NUM  0
#define NULINK2_INTERFACE_NUM 3

#define NULINK_RX_EP  (1|ENDPOINT_IN)
#define NULINK_TX_EP  (2|ENDPOINT_OUT)
#define NULINK2_RX_EP (6|ENDPOINT_IN)
#define NULINK2_TX_EP (7|ENDPOINT_OUT)

#define NULINK_HID_MAX_SIZE    (64)
#define NULINK2_HID_MAX_SIZE   (1024)
#define MAX_READ_BLOCK(N)	   ((N)/4*4)
#define MAX_WRITE_BLOCK(N)	   ((N)/4*4 - 12)
#define V6M_MAX_COMMAND_LENGTH (NULINK_HID_MAX_SIZE - 2)
#define V7M_MAX_COMMAND_LENGTH (NULINK2_HID_MAX_SIZE - 3)

#define USBCMD_TIMEOUT		5000
#define OT8051_FLASH_CONFIG_ADDR 0x00030000UL
#define OT8051_FLASH_CONFIG_SIZE 8
#define OT8051_CONFIG_OCDEN		0x10

struct nulink_usb_handle_s {
	struct jtag_libusb_device_handle *fd;
	struct libusb_transfer *trans;
	uint8_t interface_num;
	uint8_t rx_ep;
	uint8_t tx_ep;
	uint16_t max_packet_size;
	uint32_t usbcmdidx;
	uint8_t cmdidx;
	uint8_t cmdsize;
	uint8_t cmdbuf[NULINK2_HID_MAX_SIZE];
	uint8_t tempbuf[NULINK2_HID_MAX_SIZE];
	uint8_t databuf[NULINK2_HID_MAX_SIZE];
	uint32_t max_mem_packet;
	enum hl_transports transport;
	uint16_t hardwareConfig;
	uint32_t reset_command;
	uint32_t connect_command;
	uint32_t extMode_command;
	uint32_t io_voltage;
	uint32_t chip_type;
	uint32_t idcode_index;
} *m_nulink_usb_handle;

bool g_bOCDMode = false;
bool g_bICPMode = false;
uint32_t m_stop_pc;

struct nulink_usb_internal_api_s {
	int (*nulink_usb_xfer) (void *handle, uint8_t *buf, int size);
	void (*nulink_usb_init_buffer) (void *handle, uint32_t size);
} m_nulink_usb_api;

//ICE Command
#define CMD_READ_REG_ALL			0xB0UL
#define CMD_READ_BLOCK				0xB7UL
#define CMD_WRITE_RAM				0xB9UL
#define CMD_WRITE_BLOCK				0xBCUL
#define CMD_CHECK_ID				0xA3UL
#define CMD_MCU_RESET				0xE2UL
#define CMD_CHECK_MCU_STOP			0xD8UL
#define CMD_MCU_STEP_RUN			0xD1UL
#define CMD_MCU_STOP_RUN			0xD2UL
#define CMD_MCU_FREE_RUN			0xD3UL
#define CMD_SET_CONFIG				0xA2UL
#define CMD_ERASE_FLASHCHIP			0xA4UL
#define CMD_SET_BREAKPOINT			0xC0UL
#define CMD_CLR_BREAKPOINT			0xC1UL
#define CMD_ERASE_FLASH				0xAFUL
#define CMD_SET_FLASH_MODE			0xA5UL
#define CMD_ERASE_FLASHCHIP			0xA4UL
#define CMD_WRITE_FLASH				0xA0UL
#define CMD_WRITE_FLASH				0xA0UL
#define CMD_READ_FLASH				0xA1UL
#define ARM_SRAM_BASE				0x20000000UL

#define HARDWARECONFIG_NULINKPRO    1
#define HARDWARECONFIG_NULINK2      2

enum PROCESSOR_STATE_E {
	PROCESSOR_STOP,
	PROCESSOR_RUN,
	PROCESSOR_IDLE,
	PROCESSOR_POWER_DOWN
};

enum RESET_E
{
		RESET_AUTO			= 0,
		RESET_HW			= 1,
		RESET_SYSRESETREQ	= 2,
		RESET_VECTRESET		= 3,
		RESET_FAST_RESCUE	= 4,	/* Rescue and erase the chip, need very fast speed */
		RESET_NONE_NULINK	= 5,	/* Connect only */
		RESET_NONE2			= 6		/* For 8051 1T */
	};

enum CONNECT_E {
	CONNECT_NORMAL = 0,      /* Support all reset method */
	CONNECT_PRE_RESET = 1,   /* Support all reset method */
	CONNECT_UNDER_RESET = 2, /* Support all reset method */
	CONNECT_NONE = 3,        /* Support RESET_HW, (RESET_AUTO = RESET_HW) */
	CONNECT_DISCONNECT = 4,  /* Support RESET_NONE, (RESET_AUTO = RESET_NONE) */
	CONNECT_ICP_MODE = 5     /* Support NUC505 ICP mode*/
};

enum EXTMODE_E {
	EXTMODE_NORMAL = 0,        /* Support the most of Nuvoton chips */
	EXTMODE_8051OT_1 = 1,        /* 8051 1st mode */
	EXTMODE_8051OT_3 = 3,        /* 8051 2nd mode */
	EXTMODE_M0A21  = 0x100,    /* Support M0A21 */
	EXTMODE_M030G  = 0x10000 , /* Support M030G */
};

enum NUC_CHIP_TYPE_E {
	NUC_CHIP_TYPE_M460	= 0x49A,
	NUC_CHIP_TYPE_GENERAL_1T = 0x800
};

static void print64BytesBufferContent(char *bufferName, uint8_t *buf, int size)
{
	unsigned i, j;
	LOG_DEBUG("%s:", bufferName);

	for (i = 0; i < 4; i++) {
		j = i * 16;
		LOG_DEBUG("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x ",
		buf[j + 0],  buf[j + 1],  buf[j + 2],  buf[j + 3],
		buf[j + 4],  buf[j + 5],  buf[j + 6],  buf[j + 7],
		buf[j + 8],  buf[j + 9],  buf[j + 10], buf[j + 11],
		buf[j + 12], buf[j + 13], buf[j + 14], buf[j + 15]
		);
	}
}

#ifndef _WIN32
double GetTickCount(void)
{
	struct timespec now;
	if (clock_gettime(CLOCK_MONOTONIC, &now))
		return 0;
	return now.tv_sec * 1000.0 + now.tv_nsec / 1000000.0;
}
#endif

static void nulink_usb_init_buffer(void *handle, uint32_t size);

static int nulink_usb_xfer_rw(void *handle, int cmdsize, uint8_t *buf)
{
	struct nulink_usb_handle_s *h = handle;
	int res = ERROR_OK, startTime = GetTickCount(), cmdID;
	assert(handle != NULL);
#if defined(_WIN32) && (NUVOTON_CUSTOMIZED)
	jtag_libusb_nuvoton_mutex_lock();
#endif
	jtag_libusb_interrupt_write(h->fd, h->tx_ep, (char *)h->cmdbuf, h->max_packet_size,
		NULINK_WRITE_TIMEOUT);
	if (debug_level >= LOG_LVL_NULINK)
	{
		char bufName[20] = "cmd transferred";
		print64BytesBufferContent(bufName, h->cmdbuf, h->max_packet_size);
	}
	do {
		jtag_libusb_interrupt_read(h->fd, h->rx_ep, (char *)buf,
			h->max_packet_size, NULINK_READ_TIMEOUT);
		if (debug_level >= LOG_LVL_NULINK)
		{
			char bufName1[20] = "data received";
			print64BytesBufferContent(bufName1, buf, h->max_packet_size);
		}
		if(GetTickCount() - startTime > USBCMD_TIMEOUT)
		{
			res = ERROR_FAIL;
			break;
		}
		cmdID = h->cmdbuf[2];
	} while ((h->cmdbuf[0] != (buf[0] & 0x7F)) ||
			(cmdsize != buf[1]) ||
			(cmdID != 0xff && cmdID != CMD_WRITE_RAM &&
			 cmdID != CMD_CHECK_MCU_STOP  && cmdID != buf[2] &&
			 cmdID != CMD_READ_REG_ALL && cmdID != CMD_READ_BLOCK &&
			 cmdID != CMD_READ_FLASH));
#if defined(_WIN32) && (NUVOTON_CUSTOMIZED)
	jtag_libusb_nuvoton_mutex_unlock();
#endif
	return res;
}

static int nulink2_usb_xfer_rw(void *handle, int cmdsize, uint8_t *buf)
{
	struct nulink_usb_handle_s *h = handle;
	int res = ERROR_OK, startTime = GetTickCount(), cmdID;
	assert(handle != NULL);
#if defined(_WIN32) && (NUVOTON_CUSTOMIZED)
	jtag_libusb_nuvoton_mutex_lock();
#endif
	jtag_libusb_interrupt_write(h->fd, h->tx_ep, (char *)h->cmdbuf, h->max_packet_size,
		NULINK_WRITE_TIMEOUT);
	if (debug_level >= LOG_LVL_NULINK)
	{
		char bufName[20] = "cmd transferred";
		print64BytesBufferContent(bufName, h->cmdbuf, h->max_packet_size);
	}
	do {
		jtag_libusb_interrupt_read(h->fd, h->rx_ep, (char *)buf,
			h->max_packet_size, NULINK_READ_TIMEOUT);
		if (debug_level >= LOG_LVL_NULINK)
		{
			char bufName1[20] = "data received";
			print64BytesBufferContent(bufName1, buf, h->max_packet_size);
		}
		if(GetTickCount() - startTime > USBCMD_TIMEOUT)
		{
			res = ERROR_FAIL;
			break;
		}
		cmdID = h->cmdbuf[3];
	} while ((h->cmdbuf[0] != (buf[0] & 0x7F)) ||
			(cmdsize != (((int)buf[1]) << 8) + ((int)buf[2] & 0xFF)) ||
			(cmdID != 0xff && cmdID != CMD_WRITE_RAM &&
			 cmdID != CMD_CHECK_MCU_STOP && cmdID != buf[3] &&
			 cmdID != CMD_READ_REG_ALL && cmdID != CMD_READ_BLOCK &&
			 cmdID != CMD_READ_FLASH));
#if defined(_WIN32) && (NUVOTON_CUSTOMIZED)
	jtag_libusb_nuvoton_mutex_unlock();
#endif
	return res;
}

static int nulink_usb_xfer(void *handle, uint8_t *buf, int size)
{
	int err;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	err = nulink_usb_xfer_rw(h, size, h->tempbuf);
	memcpy(buf, h->tempbuf + 2, h->max_packet_size - 2);

	return err;
}

static int nulink2_usb_xfer(void *handle, uint8_t *buf, int size)
{
	int err;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	err = nulink2_usb_xfer_rw(h, size, h->tempbuf);
	memcpy(buf, h->tempbuf + 3, h->max_packet_size - 3);

	return err;
}

static void nulink_usb_init_buffer(void *handle, uint32_t size)
{
	struct nulink_usb_handle_s *h = handle;

	h->cmdidx = 0;

	memset(h->cmdbuf, 0, h->max_packet_size);
	memset(h->tempbuf, 0, h->max_packet_size);
	memset(h->databuf, 0, h->max_packet_size);

	h->cmdbuf[0] = (char)(++h->usbcmdidx & (unsigned char)0x7F);
	h->cmdbuf[1] = (char)size;
	h->cmdidx += 2;
}

static void nulink2_usb_init_buffer(void *handle, uint32_t size)
{
	struct nulink_usb_handle_s *h = handle;

	h->cmdidx = 0;

	memset(h->cmdbuf, 0, h->max_packet_size);
	memset(h->tempbuf, 0, h->max_packet_size);
	memset(h->databuf, 0, h->max_packet_size);

	h->cmdbuf[0] = (char)(++h->usbcmdidx & (unsigned char)0x7F);
	h->cmdbuf[1] = (char)((size >> 8) & 0xFF);
	h->cmdbuf[2] = (char)(size & 0xFF);
	h->cmdidx += 3;
}

static int nulink_usb_version(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, V6M_MAX_COMMAND_LENGTH);

	memset(h->cmdbuf + h->cmdidx, 0xFF, V6M_MAX_COMMAND_LENGTH);
	h->cmdbuf[h->cmdidx + 4] = (char)0xA1; /* host_rev_num: 6561 */;
	h->cmdbuf[h->cmdidx + 5] = (char)0x19;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, h->cmdsize);

	if (res != ERROR_OK)
		return res;

	LOG_INFO("NULINK firmware_version(%d), product_id(0x%08x)",
		le_to_h_u32(h->databuf),
		le_to_h_u32(h->databuf + 4 * 1));

	bool isNulinkPro = ((le_to_h_u32(h->databuf + 4 * 2) & 1) ? true : false);

	if (isNulinkPro)
	{
		LOG_INFO("NULINK is Nu-Link-Pro, target_voltage_mv(%d), usb_voltage_mv(%d)",
			(int)(unsigned short)(le_to_h_u32(h->databuf + 4 * 3)),
			(int)(unsigned short)(le_to_h_u32(h->databuf + 4 * 3) >> 16));

		h->hardwareConfig = (h->hardwareConfig & ~(HARDWARECONFIG_NULINKPRO)) | HARDWARECONFIG_NULINKPRO;
	}
	else {
		LOG_DEBUG("NULINK is Normal Nu-Link");
	}

	return ERROR_OK;
}

static int nulink_usb_assert_srst(void *handle, int srst);

static int nulink_usb_write_debug_reg(void *handle, uint32_t addr, uint32_t val)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
	h->cmdidx += 4;
	/* Count of registers */
	h->cmdbuf[h->cmdidx] = 1;
	h->cmdidx += 1;
	/* Array of bool value (u8ReadOld) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
	h->cmdidx += 1;
	/* Array of bool value (u8Verify) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
	h->cmdidx += 1;
	/* ignore */
	h->cmdbuf[h->cmdidx] = 0;
	h->cmdidx += 1;
	/* u32Addr */
	h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
	h->cmdidx += 4;
	/* u32Data */
	h_u32_to_le(h->cmdbuf + h->cmdidx, val);
	h->cmdidx += 4;
	/* u32Mask */
	h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x00000000);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	return res;
}

static int nulink_usb_trace_read(void *handle, uint8_t *buf, size_t *size)
{
	/* not supported*/
	LOG_DEBUG("nulink_usb_trace_read is not supported");

	return ERROR_OK;
}

static int nulink_usb_reset(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	switch (h->reset_command) {
		case RESET_AUTO:
			//LOG_DEBUG("nulink_usb_reset: RESET_AUTO");
			break;
		case RESET_HW:
			//LOG_DEBUG("nulink_usb_reset: RESET_HW");
			break;
		case RESET_SYSRESETREQ:
			//LOG_DEBUG("nulink_usb_reset: RESET_SYSRESETREQ");
			break;
		case RESET_VECTRESET:
			//LOG_DEBUG("nulink_usb_reset: RESET_VECTRESET");
			break;
		case RESET_FAST_RESCUE:
			//LOG_DEBUG("nulink_usb_reset: RESET_FAST_RESCUE");
			break;
		case RESET_NONE_NULINK:
			//LOG_DEBUG("nulink_usb_reset: RESET_NONE_NULINK");
			break;
		case RESET_NONE2:
			//LOG_DEBUG("nulink_usb_reset: RESET_NONE2");
			break;
		default:
			//LOG_DEBUG("nulink_usb_reset: reset_command not found");
			break;
	}

	switch (h->connect_command) {
		case CONNECT_NORMAL:
			//LOG_DEBUG("nulink_usb_connect: CONNECT_NORMAL");
			break;
		case CONNECT_PRE_RESET:
			//LOG_DEBUG("nulink_usb_reset: CONNECT_PRE_RESET");
			break;
		case CONNECT_UNDER_RESET:
			//LOG_DEBUG("nulink_usb_connect: CONNECT_UNDER_RESET");
			break;
		case CONNECT_NONE:
			//LOG_DEBUG("nulink_usb_connect: CONNECT_NONE");
			break;
		case CONNECT_DISCONNECT:
			//LOG_DEBUG("nulink_usb_connect: CONNECT_DISCONNECT");
			break;
		case CONNECT_ICP_MODE:
			//LOG_DEBUG("nulink_usb_connect: CONNECT_ICP_MODE");
			break;
		default:
			//LOG_DEBUG("nulink_usb_connect: connect_command not found");
			break;
	}

	switch (h->extMode_command) {
		case EXTMODE_NORMAL:
			//LOG_DEBUG("nulink_usb_reset: EXTMODE_NORMAL");
			break;
		case EXTMODE_8051OT_1:
			//LOG_DEBUG("nulink_usb_reset: EXTMODE_8051OT_1");
			break;
		case EXTMODE_8051OT_3:
			//LOG_DEBUG("nulink_usb_reset: EXTMODE_8051OT_3");
			break;
		case EXTMODE_M0A21:
			//LOG_DEBUG("nulink_usb_reset: EXTMODE_M0A21");
			break;
		case EXTMODE_M030G:
			//LOG_DEBUG("nulink_usb_reset: EXTMODE_M030G");
			break;
		default:
			//LOG_DEBUG("nulink_usb_reset: extMode_command not found");
			break;
	}

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 4);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_RESET);
	h->cmdidx += 4;
	/* set reset type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, h->reset_command);
	h->cmdidx += 4;
	/* set connect type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, h->connect_command);
	h->cmdidx += 4;
	/* set extMode */
	h_u32_to_le(h->cmdbuf + h->cmdidx, h->extMode_command);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 1);

	return res;
}

int nulink_mcu_reset(void *handle, uint32_t reset_command, uint32_t connect_command, uint32_t extMode_command)
{
	int res;
	struct nulink_usb_handle_s *h = handle;
	h->reset_command = reset_command;
	h->connect_command = connect_command;
	h->extMode_command = extMode_command;
	res = nulink_usb_reset(handle);

	return res;
}

static enum target_state nulink_usb_state(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	if(!g_bOCDMode) {
		nulink_mcu_reset(h, RESET_NONE2, CONNECT_ICP_MODE, EXTMODE_NORMAL);	// ICP Mode Exit
		g_bICPMode = false;
		nulink_mcu_reset(h, RESET_NONE_NULINK, CONNECT_NORMAL, EXTMODE_NORMAL);	//OCD Mode Entry
		g_bOCDMode = true;
	}

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_CHECK_MCU_STOP);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 3);

	if (res != ERROR_OK)
		return TARGET_UNKNOWN;

	if (le_to_h_u32(h->databuf + 4 * 2) == 0) {
		m_stop_pc = le_to_h_u32(h->databuf + 4);
		return TARGET_HALTED;
	}
	else
	{
		return TARGET_RUNNING;
	}

	return TARGET_UNKNOWN;
}

int nulink_flash_init(void)
{
	int res = ERROR_OK;

	if(g_bOCDMode) {
		res = nulink_mcu_reset(m_nulink_usb_handle, RESET_NONE2, CONNECT_NORMAL, EXTMODE_NORMAL);	//OCD Mode Exit
		g_bOCDMode = false;
		if(res != ERROR_OK)
			return res;
	}

	if(!g_bICPMode) {
		res = nulink_mcu_reset(m_nulink_usb_handle, RESET_NONE_NULINK, CONNECT_ICP_MODE, EXTMODE_NORMAL);	//ICP Mode Entry
		g_bICPMode = true;
	}

	return res;
}

int nulink_flash_sprom_init(void)
{
	int res = ERROR_OK;

	if(g_bICPMode) {
		res = nulink_mcu_reset(m_nulink_usb_handle, RESET_NONE2, CONNECT_ICP_MODE, EXTMODE_NORMAL);
		g_bICPMode = false;
		if(res != ERROR_OK)
			return res;
	}

	res = nulink_mcu_reset(m_nulink_usb_handle, RESET_NONE_NULINK, CONNECT_ICP_MODE, EXTMODE_8051OT_1);

	return res;
}

int nulink_flash_uninit(void)
{
	int res = ERROR_OK;

	if(g_bICPMode) {
		res = nulink_mcu_reset(m_nulink_usb_handle, RESET_NONE2, CONNECT_ICP_MODE, EXTMODE_NORMAL);	// ICP Mode Exit
		g_bICPMode = false;
	}

	return res;
}

int nulink_flash_sprom_uninit(int uSPROM_Mode)
{
	int res = ERROR_OK;

	res = nulink_mcu_reset(m_nulink_usb_handle, RESET_NONE2, CONNECT_ICP_MODE, uSPROM_Mode);
	if(res != ERROR_OK)
		return res;

	res = nulink_mcu_reset(m_nulink_usb_handle, RESET_NONE_NULINK, CONNECT_ICP_MODE, EXTMODE_NORMAL);
	g_bICPMode = true;

	return res;
}

int nulink_mcu_disconnect(void)
{
	int res;

	res = nulink_mcu_reset(m_nulink_usb_handle, RESET_NONE2, CONNECT_ICP_MODE, EXTMODE_NORMAL);
	if(res != ERROR_OK)
		return res;

	res = nulink_mcu_reset(m_nulink_usb_handle, RESET_AUTO, CONNECT_ICP_MODE, EXTMODE_8051OT_1);

	return res;
}

int nulink_8051_read_core_regs_all(void *handle, uint32_t addr, uint32_t count, uint32_t *buf)
{
	int res;
	uint32_t i;
	struct nulink_usb_handle_s *h = handle;
	assert(handle != NULL);

	res = nulink_usb_state(handle);
	if(res != TARGET_HALTED)
		return res;

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 3);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_READ_REG_ALL);
	h->cmdidx += 4;
	/* set address */
	h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
	h->cmdidx += 4;
	/* set count */
	h_u32_to_le(h->cmdbuf + h->cmdidx, count);
	h->cmdidx += 4;
	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * count);
	if (res == ERROR_OK) {
		for (i = 0; i < count; i++) {
			buf[i] = le_to_h_u32(h->databuf + 4 * i);
		}
	}

	return res;
}

int nulink_set_breakpoint(void *handle, uint32_t addr, uint32_t index)
{
	int res;
	struct nulink_usb_handle_s *h = handle;
	assert(handle != NULL);

	res = nulink_usb_state(handle);
	if(res != TARGET_HALTED)
		return res;

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 4);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_SET_BREAKPOINT);
	h->cmdidx += 4;
	/* set bp type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 1);
	h->cmdidx += 4;
	/* set address */
	h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
	h->cmdidx += 4;
	/* set index */
	h_u32_to_le(h->cmdbuf + h->cmdidx, index);
	h->cmdidx += 4;
	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	return res;
}

int nulink_clr_breakpoint(void *handle, uint32_t addr, uint32_t index)
{
	int res;
	struct nulink_usb_handle_s *h = handle;
	assert(handle != NULL);

	res = nulink_usb_state(handle);
	if(res != TARGET_HALTED)
		return res;

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 4);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_CLR_BREAKPOINT);
	h->cmdidx += 4;
	/* set bp type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 1);
	h->cmdidx += 4;
	/* set address */
	h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
	h->cmdidx += 4;
	/* set index */
	h_u32_to_le(h->cmdbuf + h->cmdidx, index);
	h->cmdidx += 4;
	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	return res;
}

int nulink_erase_flash(uint32_t addr, uint32_t len)
{
	int res = ERROR_OK;
	struct nulink_usb_handle_s *h = m_nulink_usb_handle;
	assert(m_nulink_usb_handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(m_nulink_usb_handle, 4 * 3);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_ERASE_FLASH);
	h->cmdidx += 4;
	/* set address */
	h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
	h->cmdidx += 4;
	/* set count */
	h_u32_to_le(h->cmdbuf + h->cmdidx, len);
	h->cmdidx += 4;
	res = m_nulink_usb_api.nulink_usb_xfer(m_nulink_usb_handle, h->databuf, 4 * 1);

	return res;
}

int nulink_write_flash(uint32_t addr, uint32_t len, const uint8_t *buffer)
{
	int res = ERROR_OK;
	struct nulink_usb_handle_s *h = m_nulink_usb_handle;
	assert(m_nulink_usb_handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(m_nulink_usb_handle, 4 * 3 + len);

	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_FLASH);
	h->cmdidx += 4;
	/* set address */
	h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
	h->cmdidx += 4;
	/* set count */
	h_u32_to_le(h->cmdbuf + h->cmdidx, len);
	h->cmdidx += 4;
	/* set write data */
	memcpy(h->cmdbuf + h->cmdidx, buffer, len);
	h->cmdidx += len;
	res = m_nulink_usb_api.nulink_usb_xfer(m_nulink_usb_handle, h->databuf, 4 * 1);

	return res;
}

int nulink_read_flash(uint32_t addr, uint32_t len, uint8_t *buffer)
{
	int res = ERROR_OK;
	struct nulink_usb_handle_s *h = m_nulink_usb_handle;
	assert(m_nulink_usb_handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(m_nulink_usb_handle, 4 * 3);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_READ_FLASH);
	h->cmdidx += 4;
	/* set address */
	h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
	h->cmdidx += 4;
	/* set count */
	h_u32_to_le(h->cmdbuf + h->cmdidx, len);
	h->cmdidx += 4;
	res = m_nulink_usb_api.nulink_usb_xfer(m_nulink_usb_handle, h->databuf, len);
	if (res == ERROR_OK) {
		memcpy(buffer, h->databuf, len);
	}

	return res;
}

int nulink_set_flash_mode(void)
{
	int res = ERROR_OK;
	struct nulink_usb_handle_s *h = m_nulink_usb_handle;

	assert(m_nulink_usb_handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(m_nulink_usb_handle, 4 * 16);

	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_SET_FLASH_MODE);
	h->cmdidx += 4;
	//FLASH_MODE_FLUSH
	h_u32_to_le(h->cmdbuf + h->cmdidx, 3);
	h->cmdidx += 4;
	/* set other command */
	for(int index = 0; index < 14; index++) {
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
		h->cmdidx += 4;
	}

	if (h->max_mem_packet == V7M_MAX_COMMAND_LENGTH) {
		res = m_nulink_usb_api.nulink_usb_xfer(m_nulink_usb_handle, h->databuf, 4 * 2);
	}
	else {
		res = m_nulink_usb_api.nulink_usb_xfer(m_nulink_usb_handle, h->databuf, 4);
	}

	return res;
}

int nulink_usb_chip_erase(void)
{
	int res = ERROR_FAIL;
	struct nulink_usb_handle_s *h = m_nulink_usb_handle;


	if (m_nulink_usb_handle != NULL) {
		// Erase whole chip
		m_nulink_usb_api.nulink_usb_init_buffer(m_nulink_usb_handle, 4 * 1);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_ERASE_FLASHCHIP);
		h->cmdidx += 4;

		res = m_nulink_usb_api.nulink_usb_xfer(m_nulink_usb_handle, h->databuf, 4 * 1);
	}
	else {
		LOG_DEBUG("m_nulink_usb_handle not found");
	}

	return res;	
}

int nulink_usb_assert_reset(void)
{
	int res;
	struct nulink_usb_handle_s *h = m_nulink_usb_handle;

	m_nulink_usb_api.nulink_usb_init_buffer(m_nulink_usb_handle, 4 * 4);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_RESET);
	h->cmdidx += 4;
	/* set reset type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, RESET_SYSRESETREQ);
	h->cmdidx += 4;
	/* set connect type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CONNECT_NORMAL);
	h->cmdidx += 4;
	/* set extMode */
	h_u32_to_le(h->cmdbuf + h->cmdidx, h->extMode_command);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(m_nulink_usb_handle, h->databuf, 4 * 1);

	return res;
}

static int nulink_usb_idcode(void *handle, uint32_t *idcode)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	if(g_bOCDMode) {
		nulink_mcu_reset(h, RESET_NONE2, CONNECT_NORMAL, EXTMODE_NORMAL);	//OCD Mode Exit
		g_bOCDMode = false;

		nulink_mcu_reset(h, RESET_NONE_NULINK, CONNECT_ICP_MODE, EXTMODE_NORMAL);	//ICP Mode Entry
	}

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 2);

	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_CHECK_ID);
	h->cmdidx += 4;
	/* set index */
	if (*idcode == 2) {
		h_u32_to_le(h->cmdbuf + h->cmdidx, 2);
	}
	else {
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
	}
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);
	
	if (res != ERROR_OK)
		return res;
	*idcode = le_to_h_u32(h->databuf + 4 * 1);

	LOG_INFO("IDCODE: 0x%08"PRIX32, *idcode);

	return ERROR_OK;
}

static int nulink_usb_run(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_FREE_RUN);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 1);

	return res;
}

static int nulink_usb_halt(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	if(!g_bOCDMode) {
		nulink_mcu_reset(h, RESET_NONE2, CONNECT_ICP_MODE, EXTMODE_NORMAL);	// ICP Mode Exit
		g_bICPMode = false;
		nulink_mcu_reset(h, RESET_NONE_NULINK, CONNECT_NORMAL, EXTMODE_NORMAL);	//OCD Mode Entry
		g_bOCDMode = true;
	}

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_STOP_RUN);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);
	m_stop_pc = le_to_h_u32(h->databuf + 4);
	LOG_DEBUG("nulink_usb_halt: m_stop_pc(0x%x)", m_stop_pc);

	return res;
}

int nulink_8051_reset(void *handle)
{
	int res;

	if(!g_bOCDMode) {
		nulink_mcu_reset(m_nulink_usb_handle, RESET_NONE2, CONNECT_ICP_MODE, EXTMODE_NORMAL);	// ICP Mode Exit
		g_bICPMode = false;
		nulink_mcu_reset(m_nulink_usb_handle, RESET_NONE_NULINK, CONNECT_NORMAL, EXTMODE_NORMAL);	//OCD Mode Entry
		g_bOCDMode = true;
	}

	res = nulink_mcu_reset(m_nulink_usb_handle, RESET_VECTRESET, CONNECT_NORMAL, EXTMODE_NORMAL);
	if (res != ERROR_OK) {
		return res;
	}
	res = nulink_mcu_reset(m_nulink_usb_handle, RESET_AUTO, CONNECT_NONE, EXTMODE_NORMAL);
	if (res != ERROR_OK) {
		return res;
	}

	nulink_mcu_reset(m_nulink_usb_handle, RESET_NONE_NULINK, CONNECT_NORMAL, EXTMODE_NORMAL);	//OCD Mode Entry
	g_bOCDMode = true;

	res = nulink_usb_halt(m_nulink_usb_handle);

	return res;
}

static int nulink_usb_assert_srst(void *handle, int srst)
{
	int res;
	res = nulink_8051_reset(handle);
	return res;
}

static int nulink_usb_step(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_STEP_RUN);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	m_stop_pc = le_to_h_u32(h->databuf + 4);
	LOG_DEBUG("nulink_usb_step: m_stop_pc(0x%x)", m_stop_pc);

	return res;
}

static int nulink_usb_read_regs(void *handle)
{
	return ERROR_OK;
}

static int nulink_usb_read_reg(void *handle, int num, uint32_t *val)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
	h->cmdidx += 4;
	/* Count of registers */
	h->cmdbuf[h->cmdidx] = 1;
	h->cmdidx += 1;
	/* Array of bool value (u8ReadOld) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0xFF;
	h->cmdidx += 1;
	/* Array of bool value (u8Verify) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
	h->cmdidx += 1;
	/* ignore */
	h->cmdbuf[h->cmdidx] = 0;
	h->cmdidx += 1;
	/* u32Addr */
	h_u32_to_le(h->cmdbuf + h->cmdidx, num);
	h->cmdidx += 4;
	/* u32Data */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
	h->cmdidx += 4;
	/* u32Mask */
	h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFFFF);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	*val = le_to_h_u32(h->databuf + 4 * 1);

	return res;
}

static int nulink_usb_write_reg(void *handle, int num, uint32_t val)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
	h->cmdidx += 4;
	/* Count of registers */
	h->cmdbuf[h->cmdidx] = 1;
	h->cmdidx += 1;
	/* Array of bool value (u8ReadOld) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
	h->cmdidx += 1;
	/* Array of bool value (u8Verify) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
	h->cmdidx += 1;
	/* ignore */
	h->cmdbuf[h->cmdidx] = 0;
	h->cmdidx += 1;
	/* u32Addr */
	h_u32_to_le(h->cmdbuf + h->cmdidx, num);
	h->cmdidx += 4;
	/* u32Data */
	h_u32_to_le(h->cmdbuf + h->cmdidx, val);
	h->cmdidx += 4;
	/* u32Mask */
	h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x00000000);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	return res;
}

static int nulink_usb_read_mem8(void *handle, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res = ERROR_OK;
	unsigned i, count;
	unsigned alignedAddr, offset = 0;
	uint32_t bytes_remaining = 4;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* check whether data is word aligned */
	if (addr % 4) {
		alignedAddr = addr / 4;
		alignedAddr = alignedAddr * 4;
		offset = addr - alignedAddr;

		addr = alignedAddr;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		if (len < 4)
			count = 1;
		else // len == 4
			count = 2;

		m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * count);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_READ_BLOCK);
		h->cmdidx += 4;
		/* Count of registers */
		h->cmdbuf[h->cmdidx] = count;
		h->cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0xFF;
		h->cmdidx += 1;
		/* Array of bool value (u8Verify) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
		h->cmdidx += 1;
		/* ignore */
		h->cmdbuf[h->cmdidx] = 0;
		h->cmdidx += 1;

		for (i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
			h->cmdidx += 4;
			/* u32Mask */
			h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFFFF);
			h->cmdidx += 4;
			/* proceed to the next one  */
			addr += 4;
		}

		res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

		/* fill in the output buffer */
		for (i = 0; i < count; i++) {
			if (i == 0)
				memcpy(buffer, h->databuf + 4 + offset, len);
			else
				memcpy(buffer + 2 * 1, h->databuf + 4 * (2 * 1 + 1), len - 2);
		}

		if (len >= bytes_remaining)
			len -= bytes_remaining;
		else
			len = 0;
	}

	return res;
}

static int nulink_usb_write_mem8(void *handle, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res = ERROR_OK;
	unsigned i, count;
	unsigned alignedAddr, offset = 0;
	uint32_t bytes_remaining = 12;
	uint32_t u32bufferData;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* check whether data is word aligned */
	if (addr % 4) {
		alignedAddr = addr / 4;
		alignedAddr = alignedAddr * 4;
		offset = addr - alignedAddr;

		addr = alignedAddr;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		if (len + offset <= 4)
			count = 1;
		else
			count = 2;

		m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * count);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_BLOCK);
		h->cmdidx += 4;
		/* Count of registers */
		h->cmdbuf[h->cmdidx] = count;
		h->cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
		h->cmdidx += 1;
		/* Array of bool value (u8Verify) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
		h->cmdidx += 1;
		/* ignore */
		h->cmdbuf[h->cmdidx] = 0;
		h->cmdidx += 1;

		for (i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			u32bufferData = buf_get_u32(buffer, 0, len * 8);
			u32bufferData = (u32bufferData << offset * 8);
			h_u32_to_le(h->cmdbuf + h->cmdidx, u32bufferData);
			h->cmdidx += 4;
			/* u32Mask */
			if (i == 0) {
				if (offset == 0) {
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFF00);
					}
					else if (len == 2) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFF0000);
					}
					else { // len == 3
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFF000000);
					}
				}
				else if (offset == 1) {
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFF00FF);
					}
					else if (len == 2) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFF0000FF);
					}
					else { // len == 3
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x000000FF);
					}
				}
				else if (offset == 2) {
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFF00FFFF);
					}
					else { // len == 2
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x0000FFFF);
					}
				}
				else { // offset == 3
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x00FFFFFF);
					}
				}
			}
			else { // i == 1
				if (offset == 1) {
					// len == 4
					h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFF00);
				}
				else if (offset == 2) {
					if (len == 3) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFF00);
					}
					else { // len == 4
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFF0000);
					}
				}
				else { // offset == 3
					if (len == 2) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFF00);
					}
					else if (len == 3) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFF0000);
					}
					else { // len == 4
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFF000000);
					}
				}
			}
			h->cmdidx += 4;

			/* proceed to the next one */
			addr += 4;
			buffer += 4;
		}

		res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

		if (len >= bytes_remaining)
			len -= bytes_remaining;
		else
			len = 0;
	}

	return res;
}

static int nulink_usb_read_mem32(void *handle, uint32_t addr, uint32_t len,
			  uint8_t *buffer)
{
	int res = ERROR_OK;
	struct nulink_usb_handle_s *h = handle;
	assert(handle != NULL);

	res = nulink_usb_state(handle);
	if(res != TARGET_HALTED)
		return res;

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 3);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_READ_BLOCK);
	h->cmdidx += 4;
	/* set address */
	h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
	h->cmdidx += 4;
	/* set count */
	h_u32_to_le(h->cmdbuf + h->cmdidx, len);
	h->cmdidx += 4;
	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, len);
	if (res == ERROR_OK) {
		memcpy(buffer, h->databuf, len);
	}

	return res;
}

static int nulink_usb_write_mem32(void *handle, uint32_t addr, uint16_t len,
	const uint8_t *buffer)
{
	int res = ERROR_OK;
	struct nulink_usb_handle_s *h = handle;
	assert(handle != NULL);

	res = nulink_usb_state(handle);
	if(res != TARGET_HALTED)
		return res;

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 3 + len);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_BLOCK);
	h->cmdidx += 4;
	/* set address */
	h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
	h->cmdidx += 4;
	/* set count */
	h_u32_to_le(h->cmdbuf + h->cmdidx, len);
	h->cmdidx += 4;
	/* set write data */
	memcpy(h->cmdbuf + h->cmdidx, buffer, len);
	h->cmdidx += len;
	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 1);

	return res;
}

static uint32_t nulink_max_block_size(uint32_t tar_autoincr_block, uint32_t address)
{
	uint32_t max_tar_block = (tar_autoincr_block - ((tar_autoincr_block - 1) & address));

	if (max_tar_block == 0)
		max_tar_block = 4;

	return max_tar_block;
}

static int nulink_usb_read_mem(void *handle, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	struct nulink_usb_handle_s *h = handle;

	/* calculate byte count */
	count *= size;

	while (count) {
		bytes_remaining = MAX_READ_BLOCK(h->max_mem_packet);

		if (count < bytes_remaining)
			bytes_remaining = count;

		if (bytes_remaining >= 4)
			size = 4;

		retval = nulink_usb_read_mem32(handle, addr, bytes_remaining, buffer);

		if (retval != ERROR_OK)
		{
			return retval;
		}

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}
	return retval;
}

static int nulink_usb_write_mem(void *handle, uint32_t addr, uint32_t size,
		uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	struct nulink_usb_handle_s *h = handle;

	/* calculate byte count */
	count *= size;

	while (count) {
		bytes_remaining = nulink_max_block_size(h->max_mem_packet, addr);

		if (count < bytes_remaining)
			bytes_remaining = count;

		if (bytes_remaining >= 4)
			size = 4;

		/* the nulink only supports 8/32bit memory read/writes
		 * honour 32bit, all others will be handled as 8bit access */
		retval = nulink_usb_write_mem32(handle, addr, bytes_remaining, buffer);

		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return retval;
}

static int nulink_usb_override_target(const char *targetname)
{
	return !strcmp(targetname, "cortex_m");
}

static int nulink_speed(void *handle, int khz, bool query)
{
	struct nulink_usb_handle_s *h = handle;
	unsigned long max_ice_clock = khz;

	LOG_DEBUG("nulink_speed: query(%d)", query);

	if (max_ice_clock >= 700)
	{
		max_ice_clock = 0;
	}
	else if ((max_ice_clock >= 400) || (max_ice_clock < 700))
	{
		max_ice_clock = 1;
	}
	else if ((max_ice_clock >= 16) || (max_ice_clock < 400))
	{
		max_ice_clock = 30;
	}
	else
	{
		max_ice_clock = 125;
	}

	LOG_DEBUG("NULINK nulink_speed: %lu",
			max_ice_clock);

	if (!query) {
		m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 6);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_SET_CONFIG);
		h->cmdidx += 4;
		/* set max SWD clock */
		h_u32_to_le(h->cmdbuf + h->cmdidx, max_ice_clock);
		h->cmdidx += 4;
		/* chip type */
		h_u32_to_le(h->cmdbuf + h->cmdidx, h->chip_type);
		h->cmdidx += 4;
		/* IO voltage */
		h_u32_to_le(h->cmdbuf + h->cmdidx, h->io_voltage);
		h->cmdidx += 4;
		/* If supply voltage to target or not */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
		h->cmdidx += 4;
		/* USB_FUNC_E: USB_FUNC_HID_BULK */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 2);
		h->cmdidx += 4;

		m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 3);

		LOG_DEBUG("nulink_speed: h->hardwareConfig(%d)", h->hardwareConfig);
		if (h->hardwareConfig & 1) {
			LOG_INFO("NULINK target_voltage_mv[0](%04x), target_voltage_mv[1](%04x), target_voltage_mv[2](%04x), if_target_power_supplied(%d)",
				le_to_h_u32(h->databuf + 4 * 1),
				le_to_h_u32(h->databuf + 4 * 1) >> 16,
				le_to_h_u32(h->databuf + 4 * 2),
				(le_to_h_u32(h->databuf + 4 * 2) >> 16) & 1
				);
		}
		/* wait for NUC505 IBR operations */
		busy_sleep(50);
	}

	return max_ice_clock;
}

static int nulink_usb_close(void *handle)
{
	if (handle != NULL) {
		nulink_mcu_reset(handle, RESET_NONE2, CONNECT_NORMAL, EXTMODE_NORMAL);
		nulink_mcu_reset(handle, RESET_AUTO, CONNECT_DISCONNECT, EXTMODE_NORMAL);
	}

	return ERROR_OK;
}

static int nulink_usb_open(struct hl_interface_param_s *param, void **fd)
{
	int err;
	uint32_t config[2];
	uint32_t nulink_idcode = 0;
	struct nulink_usb_handle_s *h;
	LOG_DEBUG("nulink_usb_open");

	m_nulink_usb_handle = NULL;
	g_bOCDMode = false;

	h = calloc(1, sizeof(struct nulink_usb_handle_s));

	if (h == 0) {
		LOG_ERROR("malloc failed");
		return ERROR_FAIL;
	}

	h->transport = param->transport;

	const uint16_t vids[] = { param->vid, 0 };
	const uint16_t pids[] = { param->pid, 0 };
	const uint16_t vid_nulink2[] = { 0x0416, 0x0416, 0x0416, 0x0416, 0x0416, 0x0416, 0x0416, 0x0416, 0x0416, 0x0416, 0x0416, 0 };
	const uint16_t pid_nulink2[] = { 0x5200, 0x5201, 0x5202, 0x5203, 0x5204, 0x5205, 0x2004, 0x2005, 0x2006, 0x2007, 0x2008, 0 };
	const char *serial = param->serial;

	if (param->vid != 0 && param->pid != 0) {
		LOG_DEBUG("transport: %d vid: 0x%04x pid: 0x%04x serial: %s",
			param->transport, param->vid, param->pid,
			param->serial ? param->serial : "");
	}

	/* get the Nu-Link version */
	if (jtag_libusb_open(vid_nulink2, pid_nulink2, serial, &h->fd) == ERROR_OK) {
		h->hardwareConfig = (h->hardwareConfig & ~(HARDWARECONFIG_NULINK2)) | HARDWARECONFIG_NULINK2;
		m_nulink_usb_api.nulink_usb_xfer = nulink2_usb_xfer;
		m_nulink_usb_api.nulink_usb_init_buffer = nulink2_usb_init_buffer;
		h->interface_num = NULINK2_INTERFACE_NUM;
		h->max_packet_size = jtag_libusb_get_maxPacketSize(h->fd, 0, h->interface_num, (unsigned int *)&h->rx_ep, (unsigned int *)&h->tx_ep);
		if (h->max_packet_size == (uint16_t)-1) {
			h->max_packet_size = NULINK2_HID_MAX_SIZE * 4 / 4;
		}
		h->max_mem_packet = V7M_MAX_COMMAND_LENGTH;
		LOG_INFO("NULINK is Nu-Link2");
	}
	else {
		if (jtag_libusb_open(param->vids, param->pids, serial, &h->fd) != ERROR_OK) {
			if (jtag_libusb_open(vids, pids, serial, &h->fd) != ERROR_OK) {
				LOG_ERROR("open failed");
				goto error_open;
			}
		}

		m_nulink_usb_api.nulink_usb_xfer = nulink_usb_xfer;
		m_nulink_usb_api.nulink_usb_init_buffer = nulink_usb_init_buffer;
		h->interface_num = NULINK_INTERFACE_NUM;
		h->max_packet_size = jtag_libusb_get_maxPacketSize(h->fd, 0, h->interface_num, (unsigned int *)&h->rx_ep, (unsigned int *)&h->tx_ep);
		if (h->max_packet_size == (uint16_t)-1) {
			h->max_packet_size = NULINK_HID_MAX_SIZE;
		}
		h->max_mem_packet = V6M_MAX_COMMAND_LENGTH;
		LOG_INFO("NULINK is Nu-Link1");
	}

	jtag_libusb_set_configuration(h->fd, 0);

	err = jtag_libusb_detach_kernel_driver(h->fd, h->interface_num);
	if (err != ERROR_OK) {
		LOG_DEBUG("detach kernel driver failed(%d)", err);
	}
	else {
		LOG_DEBUG("jtag_libusb_detach_kernel_driver succeeded");
	}

	err = jtag_libusb_claim_interface(h->fd, h->interface_num);
	if (err != ERROR_OK) {
		LOG_ERROR("claim interface failed(%d)", err);
		goto error_open;
	}
	else {
		LOG_DEBUG("jtag_libusb_claim_interface succeeded");
	}

	h->usbcmdidx = 0;
	h->hardwareConfig = 0;

	/* get the device version */
	h->cmdsize = 4 * 6;
	err = nulink_usb_version(h);
	if (err != ERROR_OK) {
		LOG_DEBUG("nulink_usb_version failed with cmdSize(4 * 6)");
		h->cmdsize = 4 * 5;
		err = nulink_usb_version(h);
		if (err != ERROR_OK) {
			LOG_DEBUG("nulink_usb_version failed with cmdSize(4 * 5)");
		}
	}

	if ((strcmp(param->device_desc, "Nu-Link-Pro output voltage 1800") == 0) ||
		(strcmp(param->device_desc, "Nu-Link2-Pro output voltage 1800") == 0)) {
		h->io_voltage = 1800;
	}
	else if ((strcmp(param->device_desc, "Nu-Link-Pro output voltage 2500") == 0) ||
			 (strcmp(param->device_desc, "Nu-Link2-Pro output voltage 2500") == 0)) {
		h->io_voltage = 2500;
	}
	else if ((strcmp(param->device_desc, "Nu-Link-Pro output voltage 5000") == 0) ||
			 (strcmp(param->device_desc, "Nu-Link2-Pro output voltage 5000") == 0)) {
		h->io_voltage = 5000;
	}
	else {
		h->io_voltage = 3300;
	}

	/* SWD clock rate : 1MHz */
	/* chip type: NUC_CHIP_TYPE_GENERAL_V6M */
	h->chip_type = NUC_CHIP_TYPE_GENERAL_1T;
	nulink_speed(h, param->initial_interface_speed, false);

	LOG_DEBUG("nulink_usb_open: we manually perform nulink_usb_reset");
	err = nulink_mcu_reset(h, RESET_AUTO, CONNECT_ICP_MODE, EXTMODE_NORMAL);	//Mode Entry
	if (err != ERROR_OK) {
		return err;
	}
	err = nulink_mcu_reset(h, RESET_NONE_NULINK, CONNECT_ICP_MODE, EXTMODE_NORMAL);	//ICP Mode Entry
	if (err != ERROR_OK) {
		return err;
	}
	err = nulink_usb_idcode(h, &nulink_idcode);
	if (err != ERROR_OK) {
		return err;
	}

	*fd = h;
	m_nulink_usb_handle = h;

	err = nulink_read_flash(OT8051_FLASH_CONFIG_ADDR, OT8051_FLASH_CONFIG_SIZE, (uint8_t *)&config[0]);
	if (err != ERROR_OK) {
		return err;
	}
	LOG_DEBUG("config0: 0x%x, config1: 0x%x\n", config[0], config[1]);
	
	if ((config[0] & OT8051_CONFIG_OCDEN) != 0) {
		config[0] &= ~OT8051_CONFIG_OCDEN;
		nulink_erase_flash(OT8051_FLASH_CONFIG_ADDR, OT8051_FLASH_CONFIG_SIZE);
		nulink_write_flash(OT8051_FLASH_CONFIG_ADDR, OT8051_FLASH_CONFIG_SIZE, (unsigned char *)config);
	}

	nulink_mcu_reset(h, RESET_NONE2, CONNECT_ICP_MODE, EXTMODE_NORMAL);	// ICP Mode Exit
	g_bICPMode = false;
	nulink_mcu_reset(h, RESET_NONE_NULINK, CONNECT_NORMAL, EXTMODE_NORMAL);	//OCD Mode Entry
	g_bOCDMode = true;

	err = nulink_usb_halt(h);
	if (err != ERROR_OK) {
		return err;
	}

	return ERROR_OK;

error_open:

	if (h && h->fd)
		jtag_libusb_close(h->fd);

	free(h);

	return ERROR_FAIL;
}

int nulink_config_trace(void *handle, bool enabled, enum tpio_pin_protocol pin_protocol,
			uint32_t port_size, unsigned int *trace_freq)
{
	return ERROR_OK;
}

struct hl_layout_api_s nulink_usb_layout_api = {

	.open = nulink_usb_open,

	.close = nulink_usb_close,

	.idcode = nulink_usb_idcode,

	.state = nulink_usb_state,

	.reset = nulink_8051_reset,

	.assert_srst = nulink_usb_assert_srst,

	.run = nulink_usb_run,

	.halt = nulink_usb_halt,

	.step = nulink_usb_step,

	.read_regs = nulink_usb_read_regs,

	.read_reg = nulink_usb_read_reg,

	.write_reg = nulink_usb_write_reg,

	.read_mem = nulink_usb_read_mem,

	.write_mem = nulink_usb_write_mem,

	.write_debug_reg = nulink_usb_write_debug_reg,

	.override_target = nulink_usb_override_target,

	.speed = nulink_speed,

	.config_trace = nulink_config_trace,

	.poll_trace = nulink_usb_trace_read,
};
