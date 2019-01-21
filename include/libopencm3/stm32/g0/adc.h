/** @defgroup adc_defines ADC Defines
 *
 * @ingroup STM32G0xx_defines
 *
 * @brief <b>Defined Constants and Types for the STM32STM32G0xxL4xx Analog to Digital Converter</b>
 *
 * @version 1.0.0
 *
 * LGPL License Terms @ref lgpl_license
 *  */
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

#ifndef LIBOPENCM3_ADC_H
#define LIBOPENCM3_ADC_H

#include <libopencm3/stm32/common/adc_common_v2.h>
#include <libopencm3/stm32/common/adc_common_v2_single.h>

/** @defgroup adc_reg_base ADC register base addresses
 * @ingroup adc_defines
 *
 *@{*/
#define ADC1				ADC1_BASE
/**@}*/

/** @defgroup adc_channel ADC Channel Numbers
 * @ingroup adc_defines
 *
 *@{*/
#define ADC_CHANNEL_TEMP	12
#define ADC_CHANNEL_VREF	13
#define ADC_CHANNEL_VBAT	14
/**@}*/

/* ----- ADC registers  -----------------------------------------------------*/
/* Watchdog 1 Threshold register */
#define ADC_AWD1TR(adc)		MMIO32((adc) + 0x20)
/* Watchdog 2 Threshold register */
#define ADC_AWD2TR(adc)		MMIO32((adc) + 0x22)
/* Watchdog 3 Threshold register */
#define ADC_AWD3TR(adc)		MMIO32((adc) + 0x2c)

/* Watchdog 2 Configuration register */
#define ADC_AWD2CR(adc)		MMIO32((adc) + 0xA0)
/* Watchdog 3 Configuration register */
#define ADC_AWD3CR(adc)		MMIO32((adc) + 0xA4)

/* Calibration factor register */
#define ADC_CALFACT(adc)	MMIO32((adc) + 0xB4)

/* Option register */
#define ADC_OR(adc)			MMIO32((adc) + 0xD0)

/* --- Register values -------------------------------------------------------*/

/* ADC_IER Values ------------------------------------------------------------*/

/** CCRDYIE: Channel Configuration Ready Interrupt enable bit */
#define ADC_IER_CCRDYIE				(1 << 13)

/* PRESC: ADC clock mode */
#define ADC_CCR_PRESC_MASK		(0xf)
#define ADC_CCR_PRESC_SHIFT		(18)
/** @defgroup adc_ccr_presc ADC clock prescaler
 * @ingroup adc_defines
 *@{*/
#define ADC_CCR_PRESC_NODIV		(0x0)
#define ADC_CCR_PRESC_DIV1		(0x1)
#define ADC_CCR_PRESC_DIV2		(0x2)
#define ADC_CCR_PRESC_DIV6		(0x3)
#define ADC_CCR_PRESC_DIV8		(0x4)
#define ADC_CCR_PRESC_DIV10		(0x5)
#define ADC_CCR_PRESC_DIV12		(0x6)
#define ADC_CCR_PRESC_DIV16		(0x7)
#define ADC_CCR_PRESC_DIV32		(0x8)
#define ADC_CCR_PRESC_DIV64		(0x9)
#define ADC_CCR_PRESC_DIV128	(0x10)
#define ADC_CCR_PRESC_DIV256	(0x11)
/**@}*/

/* ADC_CR Values -------------------------------------------------------------*/

/** ADVREGEN: Voltage regulator enable bit */
#define ADC_CR_ADVREGEN				(1 << 28)

/* ADC_CFGR1 Values ----------------------------------------------------------*/

/** CHSELRMOD: Mode Selection of the ADC_CHSELR register */
#define ADC_CFGR1_CHSELRMOD_SHIFT	(21)
#define ADC_CFGR1_CHSELRMOD			(1 << ADC_CFGR1_CHSELRMOD_SHIFT)
#define ADC_CFGR1_CHSELRMOD_VAL(x)	((x) << ADC_CFGR1_CHSELRMOD_SHIFT)

/* ADC_CFGR2 Values ----------------------------------------------------------*/

/** CKMODE: ADC Clock Mode */
#define ADC_CFGR2_CKMODE_SHIFT		(30)
#define ADC_CFGR2_CKMODE_MASK		(0x3)
/** @defgroup adc_cfgr2_ckmode ADC Clock mode 
 * @ingroup adc_defines
 *@{*/
#define ADC_CFGR2_CKMODE_ADCCLK		(0x0)
#define ADC_CFGR2_CKMODE_PCLK_DIV2	(0x1)
#define ADC_CFGR2_CKMODE_PCLK_DIV4	(0x2)
#define ADC_CFGR2_CKMODE_PCLK		(0x3)
/**@}*/

/** LFTRIG: Low Frequency Trigger Mode enable bit */
#define ADC_CFGR2_LFTRIG			(1 << 29)

/* ADC_SMPR Values -----------------------------------------------------------*/

/* SMP1 ADC Channel Sample Time selection */ 
#define ADC_SMPR_SMPSEL_SHIFT			0x8
#define ADC_SMPR_SMPSEL_MASK			0x7ffff
#define ADC_SMPR_SMPSEL_CHANNEL_SHIFT(channel)	((channel) + ADC_SMPR_SMPSEL_SHIFT)
#define ADC_SMPR_SMPSEL_CHANNEL_MASK			(1)
/** @defgroup adc_smpr_smpsel ADC Sample Time selection
@ingroup adc_defines
@{*/
#define ADC_SMPR_SMPSEL_SMP1	0x0
#define ADC_SMPR_SMPSEL_SMP2	0x1
/**@}*/

/** SMP1 ADC Sample Time #1 selection */ 
#define ADC_SMPR_SMP1_SHIFT			0x0
#define ADC_SMPR_SMP1_MASK			0x7

/** SMP1 ADC Sample Time #2 selection */ 
#define ADC_SMPR_SMP2_SHIFT			0x4
#define ADC_SMPR_SMP2_MASK			0x7

/** @defgroup adc_smpr_smp ADC Sample Time selection values
@ingroup adc_defines
@{*/
#define ADC_SMPR_SMPx_001DOT5CYC		0x0
#define ADC_SMPR_SMPx_003DOT5CYC		0x1
#define ADC_SMPR_SMPx_007DOT5CYC		0x2
#define ADC_SMPR_SMPx_012DOT5CYC		0x3
#define ADC_SMPR_SMPx_019DOT5CYC		0x4
#define ADC_SMPR_SMPx_039DOT5CYC		0x5
#define ADC_SMPR_SMPx_079DOT5CYC		0x6
#define ADC_SMPR_SMPx_160DOT5CYC		0x7
/**@}*/

/* AWD1TR Values -----------------------------------------------------------*/

#define ADC_AWDTR1_LT_SHIFT		0
#define ADC_AWDTR1_LT			(0xFFF << ADC_TR1_LT_SHIFT)
#define ADC_AWDTR1_LT_VAL(x)		((x) << ADC_TR1_LT_SHIFT)

#define ADC_AWDTR1_HT_SHIFT		16
#define ADC_AWDTR1_HT			(0xFFF << ADC_TR1_HT_SHIFT)
#define ADC_AWDTR1_HT_VAL(x)		((x) << ADC_TR1_HT_SHIFT)

/* AWD2TR Values -----------------------------------------------------------*/

#define ADC_AWDTR2_LT_SHIFT		0
#define ADC_AWDTR2_LT			(0xFFF << ADC_TR2_LT_SHIFT)
#define ADC_AWDTR2_LT_VAL(x)		((x) << ADC_TR2_LT_SHIFT)

#define ADC_AWDTR2_HT_SHIFT		16
#define ADC_AWDTR2_HT			(0xFFF << ADC_TR2_HT_SHIFT)
#define ADC_AWDTR2_HT_VAL(x)		((x) << ADC_TR2_HT_SHIFT)

/* CHSELR Values -----------------------------------------------------------*/

#define ADC_CHSELR_MAX_CHANNELS	18

/* AWD3TR Values -----------------------------------------------------------*/

#define ADC_AWDTR3_LT_SHIFT		0
#define ADC_AWDTR3_LT			(0xFFF << ADC_TR3_LT_SHIFT)
#define ADC_AWDTR3_LT_VAL(x)		((x) << ADC_TR3_LT_SHIFT)

#define ADC_AWDTR3_HT_SHIFT		16
#define ADC_AWDTR3_HT			(0xFFF << ADC_TR3_HT_SHIFT)
#define ADC_AWDTR3_HT_VAL(x)		((x) << ADC_TR3_HT_SHIFT)

/* AWD2CR Values -----------------------------------------------------------*/

/** AWD2CR Analog watchdog channel selection*/ 
#define ADC_AW2CR_AWD2CHx_EN(x)			(1 << x)

/* AWD3CR Values -----------------------------------------------------------*/

/** AWD3CR Analog watchdog channel selection*/ 
#define ADC_AW3CR_AWD3CHx_EN(x)			(1 << x)

/* --- API definition ----------------------------------------------------- */

/** @defgroup adc_api_clksource ADC clock source
 * @ingroup adc_defines
 *
 *@{*/
#define ADC_CLKSOURCE_ADC			ADC_CFGR2_CKMODE_ADCCLK
#define ADC_CLKSOURCE_PCLK			ADC_CFGR2_CKMODE_PCLK
#define ADC_CLKSOURCE_PCLK_DIV2		ADC_CFGR2_CKMODE_PCLK_DIV2
#define ADC_CLKSOURCE_PCLK_DIV4		ADC_CFGR2_CKMODE_PCLK_DIV4
/**@}*/

/** @defgroup adc_api_smptime ADC sampling time
 * @ingroup adc_defines
 *
 *@{*/
#define ADC_SMPTIME_001DOT5 		ADC_SMPR_SMPx_001DOT5CYC
#define ADC_SMPTIME_003DOT5 		ADC_SMPR_SMPx_003DOT5CYC
#define ADC_SMPTIME_007DOT5 		ADC_SMPR_SMPx_007DOT5CYC
#define ADC_SMPTIME_012DOT5 		ADC_SMPR_SMPx_012DOT5CYC
#define ADC_SMPTIME_019DOT5 		ADC_SMPR_SMPx_019DOT5CYC
#define ADC_SMPTIME_039DOT5 		ADC_SMPR_SMPx_039DOT5CYC
#define ADC_SMPTIME_079DOT5 		ADC_SMPR_SMPx_079DOT5CYC
#define ADC_SMPTIME_160DOT5 		ADC_SMPR_SMPx_160DOT5CYC
/**@}*/

/* --- Function prototypes ------------------------------------------------- */

BEGIN_DECLS

void adc_set_clk_source(uint32_t adc, uint32_t source);
void adc_set_clk_prescale(uint32_t adc, uint32_t prescale);

void adc_set_channel_sample_time_selection(uint32_t adc, uint8_t channel, uint8_t selection);
void adc_set_selection_sample_time(uint32_t adc, uint8_t selection, uint8_t time);

void adc_enable_regulator(uint32_t adc);
void adc_disable_regulator(uint32_t adc);

END_DECLS

/**@}*/

#endif

