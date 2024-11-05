/*
 * LED_pattern.hpp
 *
 *  Created on: Jun 27, 2024
 *      Author: gomas
 */

#ifndef LED_PATTERN_HPP_
#define LED_PATTERN_HPP_

#include "programable_PWM.hpp"

namespace SabaneLib::LEDPattern{

	inline constexpr PWMState ok[] = {
		{1.0f,10},
		{0.0f,10},
		ProgramablePWM::end_of_pwm_sequence
	};
	inline constexpr PWMState setting[]={
		{1.0f,100},
		{0.0f,100},
		{1.0f,700},
		{0.0f,100},
		ProgramablePWM::end_of_pwm_sequence
	};
}


#endif /* LED_PATTERN_HPP_ */
