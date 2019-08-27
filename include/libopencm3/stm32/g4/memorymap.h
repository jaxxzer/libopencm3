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

#ifndef LIBOPENCM3_MEMORYMAP_H
#define LIBOPENCM3_MEMORYMAP_H

#include <libopencm3/cm3/memorymap.h>

#define PERIPH_BASE			(0x40000000U)
#define PERIPH_BASE_APB1		(PERIPH_BASE + 0x0000000)
#define PERIPH_BASE_APB2		(PERIPH_BASE + 0x0010000)
#define PERIPH_BASE_AHB1		(PERIPH_BASE + 0x0020000)
#define PERIPH_BASE_AHB2		(PERIPH_BASE + 0x8000000)

/* APB1 */
#define TIM2_BASE			(PERIPH_BASE_APB1 + 0x0000)
#define TIM3_BASE			(PERIPH_BASE_APB1 + 0x0400)
#define TIM4_BASE			(PERIPH_BASE_APB1 + 0x0800)
#define TIM5_BASE			(PERIPH_BASE_APB1 + 0x0c00)
#define TIM6_BASE			(PERIPH_BASE_APB1 + 0x1000)
#define TIM7_BASE			(PERIPH_BASE_APB1 + 0x1400)
#define CRS_BASE			(PERIPH_BASE_APB1 + 0x2000)
#define TAMP_BASE			(PERIPH_BASE_APB1 + 0x2400)
#define RTC_BASE			(PERIPH_BASE_APB1 + 0x2800)
#define WWDG_BASE			(PERIPH_BASE_APB1 + 0x2c00)
#define IWDG_BASE			(PERIPH_BASE_APB1 + 0x3000)
#define SPI2_BASE			(PERIPH_BASE_APB1 + 0x3800)
#define SPI3_BASE			(PERIPH_BASE_APB1 + 0x3c00)
#define USART2_BASE			(PERIPH_BASE_APB1 + 0x4400)
#define USART3_BASE			(PERIPH_BASE_APB1 + 0x4800)
#define UART4_BASE			(PERIPH_BASE_APB1 + 0x4c00)
#define UART5_BASE			(PERIPH_BASE_APB1 + 0x5000)
#define I2C1_BASE			(PERIPH_BASE_APB1 + 0x5400)
#define I2C2_BASE			(PERIPH_BASE_APB1 + 0x5800)
#define USB_FS_device_BASE		(PERIPH_BASE_APB1 + 0x5c00)
#define FDCAN1_BASE			(PERIPH_BASE_APB1 + 0x6400)
#define FDCAN2_BASE			(PERIPH_BASE_APB1 + 0x6800)
#define FDCAN3_BASE			(PERIPH_BASE_APB1 + 0x6c00)
#define PWR_BASE			(PERIPH_BASE_APB1 + 0x7000)
#define I2C3_BASE			(PERIPH_BASE_APB1 + 0x7800)
#define LPTIM1_BASE			(PERIPH_BASE_APB1 + 0x7c00)
#define LPUART1_BASE			(PERIPH_BASE_APB1 + 0x8000)
#define I2C4_BASE			(PERIPH_BASE_APB1 + 0x8400)
#define UCPD1_BASE			(PERIPH_BASE_APB1 + 0xa000)
#define FDCAN_BASE			(PERIPH_BASE_APB1 + 0xa400)

/* APB2 */
#define SYSCFG_BASE			(PERIPH_BASE_APB2 + 0x0000)
#define VREFBUF_BASE			(PERIPH_BASE_APB2 + 0x0030)
#define COMP_BASE			(PERIPH_BASE_APB2 + 0x0200)
#define OPAMP_BASE			(PERIPH_BASE_APB2 + 0x0300)
#define EXTI_BASE			(PERIPH_BASE_APB2 + 0x0400)
#define TIM1_BASE			(PERIPH_BASE_APB2 + 0x2c00)
#define SPI1_BASE			(PERIPH_BASE_APB2 + 0x3000)
#define TIM8_BASE			(PERIPH_BASE_APB2 + 0x3400)
#define USART1_BASE			(PERIPH_BASE_APB2 + 0x3800)
#define SPI4_BASE			(PERIPH_BASE_APB2 + 0x3c00)
#define TIM15_BASE			(PERIPH_BASE_APB2 + 0x4000)
#define TIM16_BASE			(PERIPH_BASE_APB2 + 0x4400)
#define TIM17_BASE			(PERIPH_BASE_APB2 + 0x4800)
#define TIM20_BASE			(PERIPH_BASE_APB2 + 0x5000)
#define SAI_BASE			(PERIPH_BASE_APB2 + 0x5400)
#define HRTIM_BASE		(PERIPH_BASE_APB2 + 0x6800)
#define HRTIM_TIMA_BASE			(PERIPH_BASE_APB2 + 0x6880)
#define HRTIM_TIMB_BASE			(PERIPH_BASE_APB2 + 0x6900)
#define HRTIM_TIMC_BASE			(PERIPH_BASE_APB2 + 0x6980)
#define HRTIM_TIMD_BASE			(PERIPH_BASE_APB2 + 0x6a00)
#define HRTIM_TIME_BASE			(PERIPH_BASE_APB2 + 0x6a80)
#define HRTIM_TIMF_BASE			(PERIPH_BASE_APB2 + 0x6b00)
#define HRTIM_Common_BASE		(PERIPH_BASE_APB2 + 0x6b80)

/* AHB1 */
#define DMA1_BASE			(PERIPH_BASE_AHB1 + 0x0000)
#define DMA2_BASE			(PERIPH_BASE_AHB1 + 0x0400)
#define DMAMUX_BASE			(PERIPH_BASE_AHB1 + 0x0800)
#define CORDIC_BASE			(PERIPH_BASE_AHB1 + 0x0c00)
#define RCC_BASE			(PERIPH_BASE_AHB1 + 0x1000)
#define FMAC_BASE			(PERIPH_BASE_AHB1 + 0x1400)
#define FLASH_BASE			(PERIPH_BASE_AHB1 + 0x2000)
#define CRC_BASE			(PERIPH_BASE_AHB1 + 0x3000)

/* AHB2 */
#define GPIO_PORT_A_BASE	(PERIPH_BASE_AHB2 + 0x0000)
#define GPIO_PORT_B_BASE	(PERIPH_BASE_AHB2 + 0x0400)
#define GPIO_PORT_C_BASE	(PERIPH_BASE_AHB2 + 0x0800)
#define GPIO_PORT_D_BASE	(PERIPH_BASE_AHB2 + 0x0c00)
#define GPIO_PORT_E_BASE	(PERIPH_BASE_AHB2 + 0x1000)
#define GPIO_PORT_F_BASE	(PERIPH_BASE_AHB2 + 0x1400)
#define GPIO_PORT_G_BASE	(PERIPH_BASE_AHB2 + 0x1800)
/* PERIPH_BASE_AHB2 + 0x1c00 (0x4800 1c00 - 0x4FFF FFFF): Reserved */
#define ADC1_BASE			(PERIPH_BASE_AHB2 + 0x80000000)
#define ADC2_BASE			(PERIPH_BASE_AHB2 + 0x80000100)
#define ADC12_Common_BASE	(PERIPH_BASE_AHB2 + 0x80000200) /* Warning apparently at 0x50000300 on g471 ??? */
#define ADC3_BASE			(PERIPH_BASE_AHB2 + 0x80000400)
#define ADC4_BASE			(PERIPH_BASE_AHB2 + 0x80000500)
#define ADC5_BASE			(PERIPH_BASE_AHB2 + 0x80000600)
#define ADC345_Common_BASE	(PERIPH_BASE_AHB2 + 0x80000700)
#define DAC1_BASE			(PERIPH_BASE_AHB2 + 0x80000800)
#define DAC2_BASE			(PERIPH_BASE_AHB2 + 0x80000c00)
#define DAC3_BASE			(PERIPH_BASE_AHB2 + 0x80001000)
#define DAC4_BASE			(PERIPH_BASE_AHB2 + 0x80001400)
#define AES_BASE			(PERIPH_BASE_AHB2 + 0x80060000)
#define RNG_BASE			(PERIPH_BASE_AHB2 + 0x80060800)

#define FMC_BASE			(0xa0000000)
#define QUADSPI_BASE		(0xa0001000)

#define DBGMCU_BASE			(0xe0042000)

/* Device Electronic Signature */
#define DESIG_FLASH_SIZE_BASE		(0x1FFFF75e0)
#define DESIG_UNIQUE_ID_BASE		(0x1FFFF7590)
#define DESIG_UNIQUE_ID0		MMIO32(DESIG_UNIQUE_ID_BASE)
#define DESIG_UNIQUE_ID1		MMIO32(DESIG_UNIQUE_ID_BASE + 4)
#define DESIG_UNIQUE_ID2		MMIO32(DESIG_UNIQUE_ID_BASE + 8)

#endif
