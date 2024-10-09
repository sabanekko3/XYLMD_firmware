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

	inline const LEDState ok[] = {
		{0.2f,100},
		{0.0f,100},
		end
	};
	inline const LEDState error[]={
		{0.2f,100},
		{0.0f,100},
		{0.2f,700},
		{0.0f,100},
		end
	};
}


#endif /* LED_PATTERN_HPP_ */
