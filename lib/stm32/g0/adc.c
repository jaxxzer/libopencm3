/** @addtogroup adc_file ADC peripheral API
 * @ingroup peripheral_apis
 *
 * @author @htmlonly &copy; @endhtmlonly 2019 Guillaume Revaillot <g.revaillot@gmail.com>
 *
 * @date 10 January 2019
 * 
 * @code
 * TODO Add example.
 * @endcode
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

#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/assert.h>

/** @brief ADC Set Clock Source
 *
 * @param[in] adc Unsigned int32. ADC base address (@ref adc_reg_base)
 * @param[in] source Unsigned int32. Source (@ref adc_cfgr2_ckmode)
 */

void adc_set_clk_source(uint32_t adc, uint32_t source)
{
	uint32_t reg32 = ADC_CFGR2(adc);

	reg32 &= ~(ADC_CFGR2_CKMODE_MASK << ADC_CFGR2_CKMODE_SHIFT);
	ADC_CFGR2(adc) = (reg32 | (source << ADC_CFGR2_CKMODE_SHIFT));
}

/** @brief ADC Set Clock Prescale
 *
 * @param[in] adc Unsigned int32. ADC base address (@ref adc_reg_base)
 * @param[in] prescale Unsigned int32. Prescale value for ADC Async Clock @ref adc_ccr_presc
*/
void adc_set_clk_prescale(uint32_t adc, uint32_t prescale)
{
	uint32_t reg32 = ADC_CCR(adc);

	reg32  &= ~(ADC_CCR_PRESC_MASK << ADC_CCR_PRESC_SHIFT);
	ADC_CCR(adc) = (reg32 | (prescale << ADC_CCR_PRESC_SHIFT));
}

/** @brief ADC Set the Sample Time for All Channels
 *
 * @param[in] adc ADC block register address base @ref adc_reg_base
 * @param[in] time Sampling time selection from @ref adc_sample
 */
void adc_set_sample_time_on_all_channels(uint32_t adc, uint8_t time)
{
	uint32_t reg32;

	reg32 = ADC_SMPR1(adc);
	// set all channel on ADC_SMPR_SMPSEL_SMP1 and clear ADC_SMPR_SMPSEL_SMP1
	reg32 &= ~((ADC_SMPR_SMPSEL_MASK << ADC_SMPR_SMP1_SHIFT) | (ADC_SMPR_SMP1_MASK << ADC_SMPR_SMP1_SHIFT));
	// setup ADC_SMPR_SMPSEL_SMP1
	reg32 |= (time << ADC_SMPR_SMP1_SHIFT);
	ADC_SMPR1(adc) = reg32;
}

/** @brief ADC Set the Sample Time Selection for a Single Channel
 *
 * @param[in] adc Unsigned int32. ADC block base address @ref adc_reg_base
 * @param[in] channel uint8. ADC Channel integer 0..18 or from @ref adc_channel
 * @param[in] selection Unsigned int8. Sampling time selection @ref adc_smpr_smpsel
*/
void adc_set_channel_sample_time_selection(uint32_t adc, uint8_t channel, uint8_t selection)
{
	uint32_t reg32;

	reg32 = ADC_SMPR1(adc);
	reg32 &= ~(ADC_SMPR_SMPSEL_CHANNEL_MASK << ADC_SMPR_SMPSEL_CHANNEL_SHIFT(channel));
	reg32 |= (selection << ADC_SMPR_SMPSEL_CHANNEL_SHIFT(channel));
	ADC_SMPR1(adc) = reg32;
}

/** @brief ADC Set the Sample Time for given Selection.
 *
 * @param[in] adc Unsigned int32. ADC block base address @ref adc_reg_base.
 * @param[in] selection Unsigned int8. Sampling time selection @ref adc_smpr_smpsel
 * @param[in] time Unsigned int8. Sampling time selection from @ref adc_smpr_smp
*/
void adc_set_selection_sample_time(uint32_t adc, uint8_t selection, uint8_t time)
{
	uint32_t reg32;

	reg32 = ADC_SMPR1(adc);
	switch (selection) {
		case ADC_SMPR_SMPSEL_SMP1:
			reg32 &= ~(ADC_SMPR_SMP1_MASK << ADC_SMPR_SMP1_SHIFT);
			reg32 |= (time << ADC_SMPR_SMP1_SHIFT);
			break;
		case ADC_SMPR_SMPSEL_SMP2:
			reg32 &= ~(ADC_SMPR_SMP2_MASK << ADC_SMPR_SMP2_SHIFT);
			reg32 |= (time << ADC_SMPR_SMP2_SHIFT);
			break;
	}
	ADC_SMPR1(adc) = reg32;
}

/** @brief ADC Set a Regular Channel Conversion Sequence
 *
 * @param[in] adc Unsigned int32. ADC block base address @ref adc_reg_base.
 * @param[in] length Unsigned int8 Number of channels in the group, range 0..18
 * @param[in] channel Unisigned int8 array Set of channels in sequence, range @ref adc_channel
 */
void adc_set_regular_sequence(uint32_t adc, uint8_t length, uint8_t channel[])
{
	uint32_t reg32 = 0;
	bool stepup = false, stepdn = false;

	if (length > ADC_CHSELR_MAX_CHANNELS) {
		return;
	}

	if (length == 0) {
		ADC_CHSELR(adc) = 0;
		return;
	}

	reg32 |= (1 << channel[0]);

	for (uint8_t i = 1; i < length; i++) {
		reg32 |= ADC_CHSELR_CHSEL(channel[i]);
		stepup |= channel[i-1] < channel[i];
		stepdn |= channel[i-1] > channel[i];
	}

	/* Check if the channel list is in order */
	if (stepup && stepdn) {
		cm3_assert_not_reached();
	}

	if (stepdn) {
		ADC_CFGR1(adc) |= ADC_CFGR1_SCANDIR;
	} else {
		ADC_CFGR1(adc) &= ~ADC_CFGR1_SCANDIR;
	}

	ADC_CHSELR(adc) = reg32;
}

/**
 * @brief Enable the ADC Voltage regulator
 *
 * @param[in] adc Unsigned int32. ADC base address (@ref adc_reg_base)
 */
void adc_enable_regulator(uint32_t adc)
{
	ADC_CR(adc) |= ADC_CR_ADVREGEN;
}

/**
 * @brief Disable the ADC Voltage regulator
 *
 * @param[in] adc Unsigned int32. ADC base address (@ref adc_reg_base)
 */
void adc_disable_regulator(uint32_t adc)
{
	ADC_CR(adc) &= ~ADC_CR_ADVREGEN;
}

/**@}*/

