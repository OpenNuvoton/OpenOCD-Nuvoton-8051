/***************************************************************************
 *   Copyright (C) 2011 by James K. Larson                                 *
 *   jlarson@pacifier.com                                                  *
 *                                                                         *
 *   Copyright (C) 2013 Cosmin Gorgovan                                    *
 *   cosmin [at] linux-geek [dot] org                                      *
 *                                                                         *
 *   Copyright (C) 2014 Pawel Si                                           *
 *   stawel+openocd@gmail.com                                              *
 *                                                                         *
 *   Copyright (C) 2015 Nemui Trinomius                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
 *                                                                         *
 *   Copyright (C) 2017 Zale Yu                                            *
 *   CYYU@nuvoton.com                                                      *
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
#include <config.h>
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/cortex_m.h>
#include <target/numicro8051.h>
#include "jtag/hla/hla_transport.h"
#include "jtag/hla/hla_interface.h"
#include "jtag/hla/hla_layout.h"

/* Nuvoton NuMicro register locations */
#define NUMICRO_SYS_BASE        0x50000000UL
#define NUMICRO_SYS_WRPROT      0x50000100UL
#define NUMICRO_SYS_IPRSTC1     0x50000008UL

#define NUMICRO_SYSCLK_BASE     0x50000200UL
#define NUMICRO_SYSCLK_PWRCON   0x50000200UL
#define NUMICRO_SYSCLK_CLKSEL0  0x50000210UL
#define NUMICRO_SYSCLK_CLKDIV   0x50000218UL
#define NUMICRO_SYSCLK_AHBCLK   0x50000204UL

#define NUMICRO_SCS_BASE        0xE000E000UL
#define NUMICRO_SCS_AIRCR       0xE000ED0CUL
#define NUMICRO_SCS_DHCSR       0xE000EDF0UL
#define NUMICRO_SCS_DEMCR       0xE000EDFCUL
#define NUMICRO_CPUID           0xE000ED00UL

#define NUMICRO_APROM_BASE      0x00000000UL
#define NUMICRO_LDROM_BASE      0x00010000UL
#define NUMICRO_SPROM_BASE      0x00030100UL
#define NUMICRO_CONFIG_BASE     0x00030000UL

/* Command register bits */
#define PWRCON_OSC22M         (1 << 2)
#define PWRCON_XTL12M         (1 << 0)

#define IPRSTC1_CPU_RST       (1 << 1)
#define IPRSTC1_CHIP_RST      (1 << 0)

#define AHBCLK_ISP_EN         (1 << 2)
#define AHBCLK_SRAM_EN        (1 << 4)
#define AHBCLK_TICK_EN        (1 << 5)

#define ISPCON_ISPEN          (1 << 0)
#define ISPCON_BS_AP          (0 << 1)
#define ISPCON_BS_LP          (1 << 1)
#define ISPCON_BS_MASK        (1 << 1)
#define ISPCON_APUEN          (1 << 3)
#define ISPCON_CFGUEN         (1 << 4)
#define ISPCON_LDUEN          (1 << 5)
#define ISPCON_ISPFF          (1 << 6)

#define CONFIG0_LOCK_MASK     (1 << 1)

#define DHCSR_S_SDE           (1 << 20)

/* isp commands */
#define ISPCMD_READ           0x00U
#define ISPCMD_WRITE          0x21U
#define ISPCMD_ERASE          0x22U
#define ISPCMD_CHIPERASE      0x26U
#define ISPCMD_READ_CID       0x0BU
#define ISPCMD_READ_UID       0x04U
#define ISPCMD_VECMAP         0x2EU
#define ISPTRG_ISPGO          (1 << 0)

/* access unlock keys */
#define REG_KEY1              0x59U
#define REG_KEY2              0x16U
#define REG_KEY3              0x88U
#define REG_LOCK              0x00U

/* flash page size */
#define OT8051_FLASH_APROM_ADDR			0x00000
#define OT8051_FLASH_CONFIG_ADDR		0x30000
#define OT8051_FLASH_CONFIG_SIZE		8
#define OT8051_FLASH_SECTOR_SIZE		128
#define OT8051_FLASH_PAGE_SIZE			32
#define OT8051_FLASH_CRC_ALIGN_SIZE		256
#define OT8051_CONFIG_OCDEN				0x10

#define OT8051_SID_N76E003				0x3600
#define OT8051_SID_ML51_16K				0x4700
#define OT8051_SID_ML51_32K				0x4800
#define OT8051_SID_ML51_64K				0x4900
#define OT8051_SID_MS51_16K				0x4B00
#define OT8051_SID_MS51_8K				0x5200
#define OT8051_SID_MS51_32K				0x5300
#define OT8051_SID_ML56_64K				0x5700
#define OT8051_SID_MUG51_16K			0x6300
#define OT8051_SID_N76S003_MG51			0x6700

#define N76E_SCODE_ADDR			0x4800	// N76E003
#define ML51_SCODE_ADDR			0xFF80	// ML51, MS51, MUG51, MG51

/* flash MAX banks */
#define NUMICRO_MAX_FLASH_BANKS 4
/* flash mask */
#define NUMICRO_SPROM_MASK        0x00000001UL
#define NUMICRO_FLASH_OFFSET_MASK 0x00000004UL
#define NUMICRO_SPROM_ISPDAT      0x55AA03UL
extern m_nulink_usb_handle;
bool g_bErase = false;
bool g_bUpdateConfig = false;

/* flash bank structs */
struct numicro_flash_bank_type {
	uint32_t base;
	uint32_t size;
};

/* part structs */
struct numicro_cpu_type {
	char *partname;
	uint32_t partid;
};

/* Get Flash Info by DID */
typedef struct
{
	unsigned int uProgramMemorySize;
	unsigned int uLDSize;
	unsigned int uRAMSize;
	unsigned int uDID;
	union
	{
		unsigned int uFlashType;
		struct
		{
			unsigned int uFlashMode : 2;
			unsigned int uFlashPageSize : 2;
			unsigned int bSupportPID : 1;
			unsigned int bSupportUID : 1;
			unsigned int bSupportUCID : 1;
			unsigned int bSupportHIRCOff : 1;
			unsigned int uIOVoltage : 4;
			unsigned int uReserved0 : 12;
			unsigned int bSupportSCode : 1;
			unsigned int bSupportCRC8 : 1;
			unsigned int uReserved1 : 6;
		} feature;
	};
} FLASH_INFO_BY_DID_T;

typedef struct
{
	unsigned int uAPROM_Addr;
	unsigned int uAPROM_Size;
	unsigned int uNVM_Addr;
	unsigned int uNVM_Size;
	unsigned int uLDROM_Addr;
	unsigned int uLDROM_Size;
	unsigned int uLIBROM_Addr;
	unsigned int uLIBROM_Size;
	unsigned int uSPROM_Addr;
	unsigned int uSPROM_Size;
	unsigned int uSPROM_Mode;
	unsigned int uSCODE_Addr;
	unsigned int uCONFIG_Addr;
	unsigned int uCONFIG_Size;
	unsigned int uSFR_PageNum;
	unsigned int uSFR_TKAddr;		// Touch Key
	unsigned int uSFR_TKSize;
	const char *pszSeriesName;
} FLASH_DID_INFO_T;

static const struct numicro_cpu_type NuMicroParts[] = {
	/*PART Name*//*PART ID*/ /*Banks*/
	{"N76E003", 0x00003650},

	{"ML51TC0AE", 0x0C104832},
	{"ML51PC0AE", 0x01104832},
	{"ML51EC0AE", 0x02104832},
	{"ML51UC0AE", 0x03104832},
	{"ML51TB9AE", 0x0C104821},
	{"ML51PB9AE", 0x01104821},

	{"ML51UB9AE", 0x03104721},
	{"ML51EB9AE", 0x02104721},
	{"ML51XB9AE", 0x0D104721},
	{"ML51OB9AE", 0x04104721},
	{"ML51FB9AE", 0x0B104721},
	{"ML51DB9AE", 0x09104721},
	{"ML51BB9AE", 0x06104721},

	{"MS51FB9AE", 0x0B004B21},
	{"MS51XB9AE", 0x05004B21},
	{"MS51XB9BE", 0x0E004B21},

	{"NM18002Y", 0x1D004B21},

	{"MS51DA9AE", 0x0A005211},
	{"MS51BA9AE", 0x06005211},

	{"MS51FC0AE", 0x0B005332},
	{"MS51XC0BE", 0x0E005332},
	{"MS51EC0AE", 0x02005332},
	{"MS51TC0AE", 0x0C005332},
	{"MS51PC0AE", 0x01005332},

	{"MS51EB0AE", 0x02005322},

	{"ML56SD1AE", 0x08125744},
	{"ML56LD1AE", 0x00125744},
	{"ML56MD1AE", 0x07125744},

	{"ML54SD1AE", 0x08115744},
	{"ML54LD1AE", 0x00115744},
	{"ML54MD1AE", 0x07115744},

	{"ML51SD1AE", 0x08105744},
	{"ML51LD1AE", 0x00105744},
	{"ML51TD1AE", 0x0C105744},

	{"MUG51TB9AE", 0x0C106321},
	{"MUG51W",     0xFFFF6321},

	{"MG51XB9AE", 0x05006721},
	{"MG51FB9AE", 0x0B006721},
	{"MG51XC9AE", 0x05006731},
	{"MG51FC9AE", 0x0B006731},
	{"N76S003AQ20", 0x05006750},
	{"N76S003AT20", 0x0B006750},
	{"UNKNOWN", 0x00000000},
};

FLASH_INFO_BY_DID_T g_FlashDIDs[] = {
	// N76E003
	{ 18*1024, 0, 0x0300, 0x00003650, ((0 << 25) | (0 << 24) | (0x0E << 8) | (0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 2) | (0 << 0))},
	{ 18*1024, 0, 0x0300, 0x00003640, ((0 << 25) | (0 << 24) | (0x0E << 8) | (0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 2) | (1 << 0))},

	// ML51
	{  8*1024, 0, 0x0400, 0x00004711, ((0 << 25) | (1 << 24) | (0x0F << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},
	{ 16*1024, 0, 0x0400, 0x00004721, ((0 << 25) | (1 << 24) | (0x0F << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},

	{ 16*1024, 0, 0x0400, 0x00004821, ((0 << 25) | (1 << 24) | (0x0F << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},
	{ 32*1024, 0, 0x0800, 0x00004832, ((0 << 25) | (1 << 24) | (0x0F << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},

	{ 32*1024, 0, 0x0800, 0x00004932, ((0 << 25) | (1 << 24) | (0x0F << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},
	{ 64*1024, 0, 0x1000, 0x00004944, ((0 << 25) | (1 << 24) | (0x0F << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},

	// MS51
	{  8*1024, 0, 0x0200, 0x00004B10, ((0 << 25) | (1 << 24) | (0x0E << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},
	{  8*1024, 0, 0x0400, 0x00004B11, ((0 << 25) | (1 << 24) | (0x0E << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},
	{ 16*1024, 0, 0x0200, 0x00004B20, ((0 << 25) | (1 << 24) | (0x0E << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},
	{ 16*1024, 0, 0x0400, 0x00004B21, ((0 << 25) | (1 << 24) | (0x0E << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},

	{  8*1024, 0, 0x0400, 0x00005211, ((0 << 25) | (1 << 24) | (0x0E << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},

	{ 16*1024, 0, 0x0800, 0x00005322, ((0 << 25) | (1 << 24) | (0x0E << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},
	{ 32*1024, 0, 0x0800, 0x00005332, ((0 << 25) | (1 << 24) | (0x0E << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},

	// ML56
	{ 32*1024, 0, 0x1000, 0x00005734, ((1 << 25) | (1 << 24) | (0x07 << 8) | (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},
	{ 64*1024, 0, 0x1000, 0x00005744, ((1 << 25) | (1 << 24) | (0x07 << 8) | (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},

	// MUG51
	{ 16*1024, 0, 0x0400, 0x00006321, ((1 << 25) | (1 << 24) | (0x0F << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},

	// MG51, N76S003
	{ 18*1024, 0, 0x0400, 0x00006750, ((0 << 25) | (0 << 24) | (0x0E << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},
	{ 16*1024, 0, 0x0400, 0x00006721, ((0 << 25) | (1 << 24) | (0x0E << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},
	{ 32*1024, 0, 0x0400, 0x00006731, ((0 << 25) | (1 << 24) | (0x0E << 8) | (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 2) | (0 << 0))},
};

/* Private bank information for NuMicro. */
struct  numicro_flash_bank {
	struct working_area *write_algorithm;
	int probed;
	const struct numicro_cpu_type *cpu;
};

/* Private variables */
char *m_target_name = "";

void *GetInfo_8051_1T(unsigned int uDID,
					  FLASH_INFO_BY_DID_T *pInfo)
{
	size_t i;
	for(i = 0; i < sizeof(g_FlashDIDs) / sizeof(g_FlashDIDs[0]); ++i)
	{
		if(g_FlashDIDs[i].uDID == uDID)
		{
			*pInfo = g_FlashDIDs[i];
			break;
		}
	}

	if(!(i < sizeof(g_FlashDIDs) / sizeof(g_FlashDIDs[0])))
		return NULL;

	return pInfo;
}

void GetFlashSize_OT8051(unsigned int uConfig0,
						 unsigned int uProgramMemorySize,
						 unsigned int uFlashMode,
						 unsigned int *puLDROM_Addr,
						 unsigned int *puLDROM_Size,
						 unsigned int *puAPROM_Size,
						 unsigned int *puNVM_Size)
{
	unsigned int uLDROM_Size, uNVM_Size;
	unsigned int uLDSEL = (uConfig0 >> 8) & 0x07;

	uLDROM_Size = (0x07 - uLDSEL) * 1024;
	if (uLDROM_Size > 4096)
		uLDROM_Size = 4096;

	uNVM_Size = 0;
	if (uFlashMode)
		uNVM_Size = 0x2800 - uLDROM_Size;

	*puLDROM_Addr = uProgramMemorySize - uLDROM_Size;
	*puLDROM_Size = uLDROM_Size;
	*puAPROM_Size = uProgramMemorySize - uLDROM_Size - uNVM_Size;
	*puNVM_Size   = uNVM_Size;
}

FLASH_DID_INFO_T *GetInfo(unsigned int uDID,
							unsigned int uPID,
							unsigned int uConfig0,
							FLASH_DID_INFO_T *pInfo,
							FLASH_INFO_BY_DID_T *pInfo2)
{
	if (GetInfo_8051_1T(uDID, pInfo2) == NULL)
		return NULL;

	GetFlashSize_OT8051(uConfig0,
						pInfo2->uProgramMemorySize,
						pInfo2->feature.uFlashMode,
						&pInfo->uLDROM_Addr,
						&pInfo->uLDROM_Size,
						&pInfo->uAPROM_Size,
						&pInfo->uNVM_Size);

	pInfo->uAPROM_Addr		= OT8051_FLASH_APROM_ADDR;
	pInfo->uNVM_Addr		= pInfo->uAPROM_Size;
	pInfo->uSPROM_Size		= OT8051_FLASH_SECTOR_SIZE;
	pInfo->uCONFIG_Addr		= OT8051_FLASH_CONFIG_ADDR;
	pInfo->uCONFIG_Size		= OT8051_FLASH_CONFIG_SIZE;

	pInfo->uLIBROM_Addr		= 0xFFFFFFFF;
	pInfo->uLIBROM_Size		= 0;

	pInfo->uSFR_TKAddr		= 0;
	pInfo->uSFR_TKSize		= 0;

	unsigned int uSID = uDID & 0xFF00;

	switch (uSID)
	{
		case OT8051_SID_N76E003:
		{
			pInfo->uSPROM_Addr		= 0x30100;
			pInfo->uSPROM_Mode		= 1;
			pInfo->uSCODE_Addr		= N76E_SCODE_ADDR;
			pInfo->uSFR_PageNum		= 0;
			pInfo->pszSeriesName	= "N76E003";
			break;
		}
		case OT8051_SID_ML51_16K:
		case OT8051_SID_ML51_32K:
		case OT8051_SID_ML51_64K:
		{
			pInfo->uSPROM_Addr		= 0x20180;
			pInfo->uSPROM_Mode		= 0;
			pInfo->uSCODE_Addr		= ML51_SCODE_ADDR;
			pInfo->uSFR_PageNum		= 3;
			pInfo->pszSeriesName	= "ML51";
			break;
		}
		case OT8051_SID_MS51_16K:
		case OT8051_SID_MS51_8K:
		case OT8051_SID_MS51_32K:
		{
			pInfo->uSPROM_Addr		= 0x20180;
			pInfo->uSPROM_Mode		= 0;
			pInfo->uSCODE_Addr		= ML51_SCODE_ADDR;
			pInfo->uSFR_PageNum		= (uSID == OT8051_SID_MS51_32K) ? 3 : 0;
			pInfo->pszSeriesName	= "MS51";

			if (((uPID >> 12) & 0x0F) == 0x01)
				pInfo->pszSeriesName= "NM1800";

			break;
		}
		case OT8051_SID_ML56_64K:
		{
			pInfo->uSPROM_Addr		= 0x20180;
			pInfo->uSPROM_Mode		= 0;
			pInfo->uSCODE_Addr		= ML51_SCODE_ADDR;

			pInfo->uSFR_PageNum		= 4;

			if ((uPID & 0x0F) == 0x02)
			{
				pInfo->uSFR_TKAddr	= 0x8000;
				pInfo->uSFR_TKSize	= 0x60;
			}

			pInfo->pszSeriesName	= "ML51";
			break;
		}
		case OT8051_SID_MUG51_16K:
		{
			pInfo->uSPROM_Addr		= 0x20180;
			pInfo->uSPROM_Mode		= 0;
			pInfo->uSCODE_Addr		= ML51_SCODE_ADDR;
			pInfo->uLIBROM_Addr		= 0x4000;
			pInfo->uLIBROM_Size		= 0x1000;
			pInfo->uSFR_PageNum		= 4;
			pInfo->pszSeriesName	= "MUG51";
			break;
		}
		case OT8051_SID_N76S003_MG51:
		{
			pInfo->uSPROM_Addr		= 0x20180;
			pInfo->uSPROM_Mode		= 0;
			pInfo->uSCODE_Addr		= ML51_SCODE_ADDR;
			pInfo->uSFR_PageNum		= 0;
			if (((uDID >> 4) & 0x0F) == 0x05)
				pInfo->pszSeriesName= "N76S003";
			else
				pInfo->pszSeriesName= "MG51";
			break;
		}
	}

	return pInfo;
}
/* Program LongWord Block Write */
static int numicro_writeblock(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	int retval = ERROR_OK;

	unsigned int uLen_this;
	uint32_t nAdr = bank->base + offset;
	uint32_t nMany = count;

	nAdr &= 0xFFFFFF;

	unsigned int uAddr_c = nAdr, uAddr_s = numicro8051->uSCodeAddr;
	unsigned int uLen_c = 0, uLen_ec = 0, uLen_s = 0, uLen_es = 0;

	if (numicro8051->uSCodeAddr >= numicro8051->uProgramFlashSize)
	{
		if (nAdr < numicro8051->uProgramFlashSize)
		{
			if ((nAdr + nMany) > (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
			{
				uLen_c	= numicro8051->uProgramFlashSize - nAdr;
				uLen_ec	= numicro8051->uSCodeAddr - numicro8051->uProgramFlashSize;
				uLen_s	= numicro8051->uSPROMSize;
				uLen_es	= (nAdr + nMany) - (numicro8051->uSCodeAddr + numicro8051->uSPROMSize);
			}
			else if ((nAdr + nMany) > numicro8051->uSCodeAddr)
			{
				uLen_c	= numicro8051->uProgramFlashSize - nAdr;
				uLen_ec	= numicro8051->uSCodeAddr - numicro8051->uProgramFlashSize;
				uLen_s	= (nAdr + nMany) - numicro8051->uSCodeAddr;
				uLen_es	= 0;
			}
			else if ((nAdr + nMany) > numicro8051->uProgramFlashSize)
			{
				uLen_c	= numicro8051->uProgramFlashSize - nAdr;
				uLen_ec	= (nAdr + nMany) - numicro8051->uProgramFlashSize;
				uLen_s	= 0;
				uLen_es	= 0;
			}
			else
			{
				uLen_c	= nMany;
				uLen_ec	= 0;
				uLen_s	= 0;
				uLen_es	= 0;
			}
		}
		else if (nAdr < numicro8051->uSCodeAddr)
		{
			if ((nAdr + nMany) > (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
			{
				uLen_c	= 0;
				uLen_ec	= numicro8051->uSCodeAddr - nAdr;
				uLen_s	= numicro8051->uSPROMSize;
				uLen_es	= (nAdr + nMany) - (numicro8051->uSCodeAddr + numicro8051->uSPROMSize);
			}
			else if ((nAdr + nMany) > numicro8051->uSCodeAddr)
			{
				uLen_c	= 0;
				uLen_ec	= numicro8051->uSCodeAddr - nAdr;
				uLen_s	= (nAdr + nMany) - numicro8051->uSCodeAddr;
				uLen_es	= 0;
			}
			else
			{
				uLen_c	= 0;
				uLen_ec	= nMany;
				uLen_s	= 0;
				uLen_es	= 0;
			}
		}
		else if (nAdr < (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
		{
			uAddr_s = nAdr;

			if ((nAdr + nMany) > (numicro8051->uSCodeAddr + numicro8051->uSPROMSize))
			{
				uLen_c	= 0;
				uLen_ec	= 0;
				uLen_s	= (numicro8051->uSCodeAddr + numicro8051->uSPROMSize) - nAdr;
				uLen_es	= (nAdr + nMany) - (numicro8051->uSCodeAddr + numicro8051->uSPROMSize);
			}
			else
			{
				uLen_c	= 0;
				uLen_ec	= 0;
				uLen_s	= nMany;
				uLen_es	= 0;
			}
		}
		else
		{
			uLen_c	= 0;
			uLen_ec	= 0;
			uLen_s	= 0;
			uLen_es	= nMany;
		}
	}
	else
	{
		if (nAdr < numicro8051->uProgramFlashSize)
		{
			{
				if (nAdr < numicro8051->uSCodeAddr)
				{
					if ((nAdr + nMany) > numicro8051->uProgramFlashSize)
					{
						uLen_c	= numicro8051->uSCodeAddr - nAdr;
						uLen_ec	= 0;
						uLen_s	= numicro8051->uProgramFlashSize - numicro8051->uSCodeAddr;
						uLen_es	= (nAdr + nMany) - numicro8051->uProgramFlashSize;
					}
					else if ((nAdr + nMany) > numicro8051->uSCodeAddr)
					{
						uLen_c	= numicro8051->uSCodeAddr - nAdr;
						uLen_ec	= 0;
						uLen_s	= (nAdr + nMany) - numicro8051->uSCodeAddr;
						uLen_es	= 0;
					}
					else
					{
						uLen_c	= nMany;
						uLen_ec	= 0;
						uLen_s	= 0;
						uLen_es	= 0;
					}
				}
				else
				{
					uAddr_s = nAdr;

					if ((nAdr + nMany) > numicro8051->uProgramFlashSize)
					{
						uLen_c	= 0;
						uLen_ec	= 0;
						uLen_s	= numicro8051->uProgramFlashSize - nAdr;
						uLen_es	= (nAdr + nMany) - numicro8051->uProgramFlashSize;
					}
					else
					{
						uLen_c	= 0;
						uLen_ec	= 0;
						uLen_s	= nMany;
						uLen_es	= 0;
					}
				}
			}
		}
		else
		{
			uLen_c	= 0;
			uLen_ec	= 0;
			uLen_s	= 0;
			uLen_es	= nMany - uLen_ec;
		}
	}

	if ((uLen_ec > 0) || (uLen_es > 0))
	{
		return ERROR_FAIL;
	}

	if (uLen_s)
	{
		if (!(numicro8051->bSupportSCode ))
		{
			return ERROR_FAIL;
		}
	}

	if (uLen_c)
	{
		while (uLen_c)
		{
			uLen_this = uLen_c;

			if (uLen_this > 32)
				uLen_this = 32;

			retval = nulink_write_flash((uAddr_c + numicro8051->uProgramFlashAddr), uLen_this, (unsigned char *)buffer);
			if(retval != ERROR_OK) {
				LOG_DEBUG("write flash failed");
				return retval;
			}

			buffer	+= uLen_this;
			uAddr_c	+= uLen_this;
			uLen_c	-= uLen_this;
		}

		retval = nulink_set_flash_mode();
		if(retval != ERROR_OK) {
			LOG_DEBUG("nulink_set_flash_mode failed");
			return retval;
		}
	}

	if (uLen_s)
	{
		if (numicro8051->uSPROMMode)
		{
			retval = nulink_flash_sprom_init();
			if(retval != ERROR_OK) {
				LOG_DEBUG("flash init failed");
				return retval;
			}
		}

		while (uLen_s)
		{
			uLen_this = uLen_s;

			if (uLen_this > 32)
				uLen_this = 32;

			retval = nulink_write_flash((uAddr_s - numicro8051->uSCodeAddr + numicro8051->uSPROMAddr), uLen_this, (unsigned char *)buffer);
			if(retval != ERROR_OK) {
				LOG_DEBUG("write flash failed");
				return retval;
			}

			buffer	+= uLen_this;
			uAddr_s	+= uLen_this;
			uLen_s	-= uLen_this;
		}

		retval = nulink_set_flash_mode();
		if(retval != ERROR_OK) {
			LOG_DEBUG("nulink_set_flash_mode failed");
			return retval;
		}

		if (numicro8051->uSPROMMode)
		{
			retval = nulink_flash_sprom_uninit(numicro8051->uSPROMMode);
			if(retval != ERROR_OK) {
				LOG_DEBUG("disconnect failed");
				return retval;
			}
		}
	}

	return ERROR_OK;
}

/* Flash Lock checking - examines the lock bit. */
static int numicro_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t set, config[2];
	int i, retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("Nuvoton NuMicro: Flash Lock Check...");

	/* Read CONFIG0,CONFIG1 */
	retval = nulink_read_flash(NUMICRO_CONFIG_BASE, OT8051_FLASH_CONFIG_SIZE, (uint8_t *)&config[0]);
	if(retval != ERROR_OK) {
		LOG_DEBUG("Read config failed");
		return retval;	
	}

	LOG_DEBUG("CONFIG0: 0x%" PRIx32 ",CONFIG1: 0x%" PRIx32 "", config[0], config[1]);

	if ((config[0] & (1<<7)) == 0)
		LOG_INFO("CBS=0: Boot From LDROM");
	else
		LOG_INFO("CBS=1: Boot From APROM");

	if ((config[0] & CONFIG0_LOCK_MASK) == 0) {

		LOG_INFO("Flash is secure locked!");
		LOG_INFO("TO UNLOCK FLASH,EXECUTE chip_erase COMMAND!!");
		set = 1;
	}
	else {
		LOG_INFO("Flash is not locked!");
		set = 0;
	}

	for (i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = set;

	return ERROR_OK;
}

static int numicro_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("Nuvoton NuMicro: Sector Erase ... (%d to %d)", first, last);

	retval = nulink_flash_init();
	if(retval != ERROR_OK) {
		return retval;	
	}

	if (bank->base == ML51_SCODE_ADDR) {
		retval = nulink_erase_flash(numicro8051->uSPROMAddr, numicro8051->uSPROMSize);
		nulink_flash_sprom_uninit(numicro8051->uSPROMMode);
		nulink_flash_init();
	}
	else {
		if (!g_bErase) {
			retval = nulink_erase_flash(first * bank->sectors[0].size, (last - first + 1) * bank->sectors[0].size);
			g_bErase = true;
		}
	}

	/* done */
	LOG_DEBUG("Erase done.");

	return retval;
}

/* The write routine stub. */
static int numicro_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("Nuvoton NuMicro: Flash Write ...");

	retval = nulink_flash_init();

	if(retval != ERROR_OK)
		return retval;	

	/* try using a block write */
	retval = numicro_writeblock(bank, buffer, offset, count);
	if(retval != ERROR_OK)
		return retval;	

	/* done. */
	LOG_DEBUG("Write done.");

	return retval;
}

static int numicro_get_cpu_type(struct target *target, const struct numicro_cpu_type** cpu)
{
	struct hl_interface_s *adapter = target->tap->priv;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);
	int retval;
	uint32_t uCheckID = 0, uDeviceID = 0, uPID = 0, config[2];
	numicro8051->uPartID = 0x3650;

	retval = nulink_flash_init();

	if(retval != ERROR_OK)
		return retval;	

	retval = adapter->layout->api->idcode(adapter->handle, &uCheckID);
	if (retval != ERROR_OK)
		return retval;

	uCheckID = uCheckID & 0xFFFFFF;
	uDeviceID	= uCheckID & 0xFFFF;

	FLASH_DID_INFO_T flashInfo;
	FLASH_INFO_BY_DID_T flashInfo2;

	if (GetInfo_8051_1T(uDeviceID, &flashInfo2) != NULL) {
		if (((uCheckID >> 16) & 0xFF) != 0xDA) {
			nulink_mcu_disconnect();
			LOG_DEBUG("chip protected");
			return ERROR_FAIL;
		}
		else {
			uPID = 0;
			numicro8051->uPartID = uDeviceID;

			if (flashInfo2.feature.bSupportPID)
			{
				uPID = 2;
				retval = adapter->layout->api->idcode(adapter->handle, &uPID);
				if (retval != ERROR_OK)
					return retval;

				numicro8051->uPartID |= ((uPID & 0xFFFF) << 16) + uDeviceID;
			}
			else {
				numicro8051->uPartID = uDeviceID;
			}

			retval = nulink_read_flash(NUMICRO_CONFIG_BASE, OT8051_FLASH_CONFIG_SIZE, (uint8_t *)&config[0]);
			LOG_DEBUG("Config0: 0x%x, Config1: 0x%x\n", config[0], config[1]);
			if(retval != ERROR_OK) {
				LOG_DEBUG("Read config failed");
				return retval;	
			}

			if(retval != ERROR_OK)
				return retval;	

			GetInfo(uDeviceID, uPID, *(unsigned int *)&config[0], &flashInfo, &flashInfo2);

			if ((config[0] & 0x80) != 0x00)	// Boot from APROM
			{
				numicro8051->uProgramFlashAddr	= flashInfo.uAPROM_Addr;
				numicro8051->uProgramFlashSize	= flashInfo.uAPROM_Size;
			}
			else									// Boot from LDROM
			{
				numicro8051->uProgramFlashAddr	= flashInfo.uLDROM_Addr;
				numicro8051->uProgramFlashSize	= flashInfo.uLDROM_Size;
			}

			numicro8051->uXRAMSize	= flashInfo2.uRAMSize;

			numicro8051->uSPROMAddr	= flashInfo.uSPROM_Addr;
			numicro8051->uSPROMSize	= flashInfo.uSPROM_Size;
			numicro8051->uSPROMMode	= flashInfo.uSPROM_Mode;
			numicro8051->uSCodeAddr	= flashInfo.uSCODE_Addr;

			numicro8051->uLIBROMAddr	= flashInfo.uLIBROM_Addr;
			numicro8051->uLIBROMSize	= flashInfo.uLIBROM_Size;

			numicro8051->uSFR_PageNum	= flashInfo.uSFR_PageNum;
			numicro8051->uSFR_TKAddr	= flashInfo.uSFR_TKAddr;
			numicro8051->uSFR_TKSize	= flashInfo.uSFR_TKSize;

			numicro8051->bSupportSCode	= flashInfo2.feature.bSupportSCode;

			/* search part numbers */
			for (size_t i = 0; i < sizeof(NuMicroParts) / sizeof(NuMicroParts[0]); i++) {
				if (numicro8051->uPartID == NuMicroParts[i].partid) {
					*cpu = &NuMicroParts[i];
					LOG_INFO("Device Name: %s", (*cpu)->partname);
					return ERROR_OK;
				}
			}
		}
	}
	else {
		nulink_mcu_disconnect();
		LOG_DEBUG("unknown ID");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int numicro_probe(struct flash_bank *bank)
{
	struct numicro_flash_bank *numicro_info = bank->driver_priv;

	uint32_t flash_size = 0x4000, offset = 0, page_size = OT8051_FLASH_SECTOR_SIZE;
	int num_pages;
	const struct numicro_cpu_type *cpu;
	struct target *target = bank->target;
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	int retval = ERROR_OK;

	retval = numicro_get_cpu_type(target, &cpu);
	if (retval != ERROR_OK) {
		LOG_WARNING("NuMicro flash driver: Failed to detect a known part");
	}

	if (bank->base == ML51_SCODE_ADDR) {
		flash_size = numicro8051->uSPROMSize;
	}
	else {
		flash_size = numicro8051->uProgramFlashSize;
	}

	num_pages = flash_size / page_size;
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
	bank->size = flash_size;

	for (int i = 0; i < num_pages; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
		offset += page_size;
	}

	numicro_info->probed = true;
	g_bUpdateConfig = false;
	numicro_info->cpu = cpu;
	LOG_DEBUG("Nuvoton NuMicro: Probed ...");

	return ERROR_OK;
}

/* Standard approach to auto-probing. */
static int numicro_auto_probe(struct flash_bank *bank)
{
	struct numicro_flash_bank *numicro_info = bank->driver_priv;

	if (numicro_info->probed && (g_bUpdateConfig == false))
		return ERROR_OK;
	return numicro_probe(bank);
}

/* This is the function called in the config file. */
FLASH_BANK_COMMAND_HANDLER(numicro_flash_bank_command)
{
	struct numicro_flash_bank *bank_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_DEBUG("add flash_bank numicro %s", bank->name);

	bank_info = malloc(sizeof(struct numicro_flash_bank));

	memset(bank_info, 0, sizeof(struct numicro_flash_bank));

	bank->driver_priv = bank_info;

	return ERROR_OK;
}

COMMAND_HANDLER(numicro_handle_update_config_command)
{
	uint32_t config[2];
	int retval = ERROR_OK;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], config[0]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], config[1]);

	struct target *target = get_current_target(CMD_CTX);
	struct hl_interface_s *adapter = target->tap->priv;

	retval = nulink_flash_init();
	if(retval != ERROR_OK)
		return retval;

	LOG_DEBUG("config0: 0x%x, config1: 0x%x\n", config[0], config[1]);
	
	config[0] &= ~OT8051_CONFIG_OCDEN;
	nulink_erase_flash(OT8051_FLASH_CONFIG_ADDR, OT8051_FLASH_CONFIG_SIZE);
	nulink_write_flash(OT8051_FLASH_CONFIG_ADDR, OT8051_FLASH_CONFIG_SIZE, (unsigned char *)config);
	nulink_set_flash_mode();

	LOG_INFO("numicro update_config 0x%08" PRIx32 " 0x%08" PRIx32, config[0], config[1]);

	retval = adapter->layout->api->halt(adapter->handle);

	g_bUpdateConfig = true;

	return retval;
}

COMMAND_HANDLER(numicro_handle_chip_erase_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t config[2];
	int retval = ERROR_OK;

	retval = nulink_flash_init();
	if(retval != ERROR_OK)
		return retval;

	retval = nulink_read_flash(NUMICRO_CONFIG_BASE, OT8051_FLASH_CONFIG_SIZE, (uint8_t *)&config[0]);
	if(retval != ERROR_OK) {
		LOG_DEBUG("Read config failed");
		return retval;	
	}

	LOG_DEBUG("config0: 0x%x, config1: 0x%x\n", config[0], config[1]);

	retval = nulink_usb_chip_erase();
	if (retval != ERROR_OK) {
		command_print(CMD_CTX, "numicro M2351_erase failed");
		return retval;
	}

	config[0] &= ~OT8051_CONFIG_OCDEN;
	nulink_write_flash(OT8051_FLASH_CONFIG_ADDR, OT8051_FLASH_CONFIG_SIZE, (unsigned char *)config);
	nulink_set_flash_mode();

	command_print(CMD_CTX, "numicro chip_erase complete");

	g_bUpdateConfig = true;

	return ERROR_OK;
}
COMMAND_HANDLER(numicro_handle_memory_space_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct numicro8051_common *numicro8051 = target_to_numicro8051(target);

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if(strcmp(CMD_ARGV[0], "D") == 0)
	{
		numicro8051->uMemorySpace = amDATA;
	}
	else if(strcmp(CMD_ARGV[0], "I") == 0)
	{
		numicro8051->uMemorySpace = amIDATA;
	}
	else if(strcmp(CMD_ARGV[0], "X") == 0)
	{
		numicro8051->uMemorySpace = amXDATA;
	}
	else
	{
		numicro8051->uMemorySpace = amCODE;
	}

	command_print(CMD_CTX, "numicro memory_space complete");

	return ERROR_OK;	
}

static const struct command_registration numicro_exec_command_handlers[] = {
	{
		.name = "update_config",
		.handler = numicro_handle_update_config_command,
		.usage = "config0 config1",
		.mode = COMMAND_EXEC,
		.help = "update config0 config1.",
	},
	{
		.name = "chip_erase",
		.handler = numicro_handle_chip_erase_command,
		.mode = COMMAND_EXEC,
		.help = "whole chip erase.",
	},
	{
		.name = "memory_space",
		.handler = numicro_handle_memory_space_command,
		.mode = COMMAND_EXEC,
		.help = "change memory space.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration numicro_command_handlers[] = {
	{
		.name = "numicro",
		.mode = COMMAND_ANY,
		.help = "numicro flash command group",
		.usage = "",
		.chain = numicro_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver numicro_flash = {
	.name = "numicro",
	.commands = numicro_command_handlers,
	.flash_bank_command = numicro_flash_bank_command,
	.erase = numicro_erase,
	.write = numicro_write,
	.read = default_flash_read,
	.probe = numicro_probe,
	.auto_probe = numicro_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = numicro_protect_check,
};