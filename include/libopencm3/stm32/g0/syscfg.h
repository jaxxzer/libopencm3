/** @defgroup syscfg_defines SYSCFG Defines
 *
 * @ingroup STM32G0xx_defines
 *
 * @brief <b>Defined Constants and Types for the STM32G0xx System Config</b>
 *
 * @version 1.0.0
 *
 * LGPL License Terms @ref lgpl_license
 */
/*
 * This file is part of the libopencm3 project.
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

/**@{*/
#ifndef LIBOPENCM3_SYSCFG_H
#define LIBOPENCM3_SYSCFG_H

/* --- SYSCFG registers ----------------------------------------------------- */
#define SYSCFG_CFGR1			MMIO32(SYSCFG_BASE + 0x00)
#define SYSCFG_CFGR2			MMIO32(SYSCFG_BASE + 0x18)

#define SYSCFG_ITLINE(x)		MMIO32(SYSCFG_BASE + 0x80 + (x)*4)
#define SYSCFG_ITLINE0			MMIO32(SYSCFG_BASE + 0x80)
#define SYSCFG_ITLINE1			MMIO32(SYSCFG_BASE + 0x84)
#define SYSCFG_ITLINE2			MMIO32(SYSCFG_BASE + 0x88)
#define SYSCFG_ITLINE3			MMIO32(SYSCFG_BASE + 0x8c)
#define SYSCFG_ITLINE4			MMIO32(SYSCFG_BASE + 0x90)
#define SYSCFG_ITLINE5			MMIO32(SYSCFG_BASE + 0x94)
#define SYSCFG_ITLINE6			MMIO32(SYSCFG_BASE + 0x98)
#define SYSCFG_ITLINE7			MMIO32(SYSCFG_BASE + 0x9c)
#define SYSCFG_ITLINE8			MMIO32(SYSCFG_BASE + 0xa0)
#define SYSCFG_ITLINE9			MMIO32(SYSCFG_BASE + 0xa4)
#define SYSCFG_ITLINE10			MMIO32(SYSCFG_BASE + 0xa8)
#define SYSCFG_ITLINE11			MMIO32(SYSCFG_BASE + 0xac)
#define SYSCFG_ITLINE12			MMIO32(SYSCFG_BASE + 0xb0)
#define SYSCFG_ITLINE13			MMIO32(SYSCFG_BASE + 0xb4)
#define SYSCFG_ITLINE14			MMIO32(SYSCFG_BASE + 0xb8)
#define SYSCFG_ITLINE15			MMIO32(SYSCFG_BASE + 0xbc)
#define SYSCFG_ITLINE16			MMIO32(SYSCFG_BASE + 0xc0)
#define SYSCFG_ITLINE17			MMIO32(SYSCFG_BASE + 0xc4)
#define SYSCFG_ITLINE18			MMIO32(SYSCFG_BASE + 0xc8)
#define SYSCFG_ITLINE19			MMIO32(SYSCFG_BASE + 0xcc)
#define SYSCFG_ITLINE20			MMIO32(SYSCFG_BASE + 0xd0)
#define SYSCFG_ITLINE21			MMIO32(SYSCFG_BASE + 0xd4)
#define SYSCFG_ITLINE22			MMIO32(SYSCFG_BASE + 0xd8)
#define SYSCFG_ITLINE23			MMIO32(SYSCFG_BASE + 0xdc)
#define SYSCFG_ITLINE24			MMIO32(SYSCFG_BASE + 0xe0)
#define SYSCFG_ITLINE25			MMIO32(SYSCFG_BASE + 0xe4)
#define SYSCFG_ITLINE26			MMIO32(SYSCFG_BASE + 0xe8)
#define SYSCFG_ITLINE27			MMIO32(SYSCFG_BASE + 0xec)
#define SYSCFG_ITLINE28			MMIO32(SYSCFG_BASE + 0xf0)
#define SYSCFG_ITLINE29			MMIO32(SYSCFG_BASE + 0xf4)
#define SYSCFG_ITLINE30			MMIO32(SYSCFG_BASE + 0xf8)
#define SYSCFG_ITLINE31			MMIO32(SYSCFG_BASE + 0xfc)

/* --- SYSCFG_CFGR1 Values ---------------------------------------------------*/

#define SYSCFG_CFGR1_I2C_PAx_FMP_SHIFT		22
#define SYSCFG_CFGR1_I2C_PAx_FMP_MASK		0x03

#define SYSCFG_CFGR1_I2C2_FMP				(1 << 21)

#define SYSCFG_CFGR1_I2C1_FMP				(1 << 20)

#define SYSCFG_CFGR1_I2C_PBx_FMP_SHIFT		16
#define SYSCFG_CFGR1_I2C_PBx_FMP_MASK		0x0f

#define SYSCFG_CFGR1_UCPD2_STROBE			(1 << 10)

#define SYSCFG_CFGR1_UCPD1_STROBE			(1 << 9)

#define SYSCFG_CFGR1_BOOSTEN				(1 << 8)
#define SYSCFG_CFGR1_IR_MOD_SHIFT			6
#define SYSCFG_CFGR1_IR_MOD_MASK			0x03
/** @defgroup syscfg_cfgr1_ir_mod IR MOD
* @brief IR Modulation Envelope signal selection.
@{*/
#define SYSCFG_CFGR1_IR_MOD_TIM16			0
#define SYSCFG_CFGR1_IR_MOD_USART1			1
#define SYSCFG_CFGR1_IR_MOD_USART4			2
/*@}*/

#define SYSCFG_CFGR1_IR_POL					(1 << 5)

#define SYSCFG_CFGR1_PA11_PA12_RMP			(1 << 4)

#define SYSCFG_CFGR1_MEM_MODE_SHIFT			0
#define SYSCFG_CFGR1_MEM_MODE_MASK			0x03
/** @defgroup syscfg_cfgr1_mem_mode MEM MODE
* @brief Memory mapping selection bits
@{*/
#define SYSCFG_CFGR1_MEM_MODE_FLASH			0
#define SYSCFG_CFGR1_MEM_MODE_SYSTEM		1
#define SYSCFG_CFGR1_MEM_MODE_SRAM			3
/*@}*/

/* --- SYSCFG_CFGR2 Values ---------------------------------------------------*/

#define SYSCFG_CFGR2_SRAM_PEF				(1 << 8) 
#define SYSCFG_CFGR2_ECC_LOCK				(1 << 3)
#define SYSCFG_CFGR2_PVD_LOCK				(1 << 2)
#define SYSCFG_CFGR2_SRAM_PARITY_LOCK		(1 << 1)
#define SYSCFG_CFGR2_LOCKUP_LOCK			(1 << 0)

/* --- SYSCFG_ITLINE0 values ------------------------------------------------*/
#define SYSCFG_ITLINE0_WWDG					(1 << 0)

/* --- SYSCFG_ITLINE1 values ------------------------------------------------*/
#define SYSCFG_ITLINE1_PVDOUT				(1 << 0)

/* --- SYSCFG_ITLINE2 values ------------------------------------------------*/
#define SYSCFG_ITLINE2_RTC					(1 << 1)
#define SYSCFG_ITLINE2_TAMP					(1 << 0)

/* --- SYSCFG_ITLINE3 values ------------------------------------------------*/
#define SYSCFG_ITLINE3_FLASH_ECC			(1 << 1)
#define SYSCFG_ITLINE3_FLASH_ITF			(1 << 0)

/* --- SYSCFG_ITLINE4 values ------------------------------------------------*/
#define SYSCFG_ITLINE4_RCC					(1 << 0)

/* --- SYSCFG_ITLINE5 values ------------------------------------------------*/
#define SYSCFG_ITLINE5_EXTI1				(1 << 1)
#define SYSCFG_ITLINE5_EXTI0				(1 << 0)

/* --- SYSCFG_ITLINE6 values ------------------------------------------------*/
#define SYSCFG_ITLINE6_EXTI3				(1 << 1)
#define SYSCFG_ITLINE6_EXTI2				(1 << 0)

/* --- SYSCFG_ITLINE7 values ------------------------------------------------*/
#define SYSCFG_ITLINE7_EXTI15				(1 << 11)
#define SYSCFG_ITLINE7_EXTI14				(1 << 10)
#define SYSCFG_ITLINE7_EXTI13				(1 << 9)
#define SYSCFG_ITLINE7_EXTI12				(1 << 8)
#define SYSCFG_ITLINE7_EXTI11				(1 << 7)
#define SYSCFG_ITLINE7_EXTI10				(1 << 6)
#define SYSCFG_ITLINE7_EXTI9    		   	(1 << 5)
#define SYSCFG_ITLINE7_EXTI8    		   	(1 << 4)
#define SYSCFG_ITLINE7_EXTI7    	   		(1 << 3)
#define SYSCFG_ITLINE7_EXTI6    	   		(1 << 2)
#define SYSCFG_ITLINE7_EXTI5    	   		(1 << 1)
#define SYSCFG_ITLINE7_EXTI4    	   		(1 << 0)

/* --- SYSCFG_ITLINE8 values -------------------------------------------------*/
#define SYSCFG_ITLINE8_UCPD2				(1 << 1)
#define SYSCFG_ITLINE8_UCPD1				(1 << 0)

/* --- SYSCFG_ITLINE9 values -------------------------------------------------*/
#define SYSCFG_ITLINE9_DMA1_CH1				(1 << 0)

/* --- SYSCFG_ITLINE10 values ------------------------------------------------*/
#define SYSCFG_ITLINE10_DMA1_CH3			(1 << 1)
#define SYSCFG_ITLINE10_DMA1_CH2			(1 << 0)

/* --- SYSCFG_ITLINE11 values ------------------------------------------------*/
#define SYSCFG_ITLINE11_DMA1_CH7			(1 << 4)
#define SYSCFG_ITLINE11_DMA1_CH6			(1 << 3)
#define SYSCFG_ITLINE11_DMA1_CH5			(1 << 2)
#define SYSCFG_ITLINE11_DMA1_CH4			(1 << 1)
#define SYSCFG_ITLINE11_DMAMUX				(1 << 0)

BEGIN_DECLS

END_DECLS

#endif
/**@}*/
