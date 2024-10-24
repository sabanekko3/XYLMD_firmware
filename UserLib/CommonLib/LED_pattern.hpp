/*
 * LED_pattern.hpp
 *
 *  Created on: Jun 27, 2024
 *      Author: gomas
 */

#ifndef LED_PATTERN_HPP_
#define LED_PATTERN_HPP_

#include "programable_LED.hpp"
namespace SabaneLib::LEDPattern{

	inline constexpr LEDState ok[] = {
		{1.0f,10},
		{0.0f,10},
		end
	};
	inline constexpr LEDState setting[]={
		{1.0f,100},
		{0.0f,100},
		{1.0f,700},
		{0.0f,100},
		end
	};
}


#endif /* LED_PATTERN_HPP_ */
