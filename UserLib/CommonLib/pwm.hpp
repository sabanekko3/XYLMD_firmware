/*
 * pwm.hpp
 *
 *  Created on: Jun 25, 2024
 *      Author: gomas
 */

#ifndef PWM_HPP_
#define PWM_HPP_

#include "main.h"

#ifdef HAL_TIM_MODULE_ENABLED
namespace SabaneLib{

	class IPWM{
	public:
		virtual void out(float val) = 0;

		virtual ~IPWM(){}
	};

	////////////////////////////////////////////////////////////
	//Hardware PWM class////////////////////////////////////////
	////////////////////////////////////////////////////////////
	class PWMHard:public IPWM{
	protected:
		TIM_HandleTypeDef *tim;
		const uint32_t ch;
	public:
		PWMHard(TIM_HandleTypeDef *_tim,uint32_t _ch)
			: tim(_tim),
			  ch(_ch){
		}

		void out(float val) override{
			__HAL_TIM_SET_COMPARE(tim, ch, tim->Init.Period*val);
		}

		void out_toggle(float val = 1.0f){
			if(__HAL_TIM_GET_COMPARE(tim, ch)!=0){
				__HAL_TIM_SET_COMPARE(tim, ch,0);
			}else{
				__HAL_TIM_SET_COMPARE(tim, ch,val*tim->Init.Period);
			}
		}

		void start(void){
			HAL_TIM_PWM_Start(tim, ch);
			HAL_TIMEx_PWMN_Start(tim,ch);
			__HAL_TIM_SET_COMPARE(tim, ch,0);
		}

		void stop(void){
			HAL_TIM_PWM_Stop(tim, ch);
			HAL_TIMEx_PWMN_Stop(tim,ch);
			__HAL_TIM_SET_COMPARE(tim, ch,0);
		}

	};
}
#endif //HAL_TIM_MODULE_ENABLED

#endif /* PWM_HPP_ */
