/*
 * gpio_handler.hpp
 *
 *  Created on: Nov 4, 2024
 *      Author: gomas
 */

#ifndef GPIO_HANDLER_HPP_
#define GPIO_HANDLER_HPP_

#include "CommonLib/programable_PWM.hpp"

namespace BoardLib{
class PWMDummy:public SabaneLib::IPWM{
	GPIO_TypeDef *port;
	const uint16_t pin;
	const float duty_threshold;

public:
	PWMDummy(GPIO_TypeDef *_port,uint16_t _pin,float _duty_threshold = 0.1f)
		: port(_port),pin(_pin),duty_threshold(_duty_threshold){
	}

	void operator()(float duty)override{
		port->BSRR = pin << (16*(duty_threshold > duty));
//		if(duty_threshold < duty) LL_GPIO_SetOutputPin(port,pin);
//		else LL_GPIO_ResetOutputPin(port,pin);
	}
};
}


#endif /* GPIO_HANDLER_HPP_ */
