/*
 * pwm.hpp
 *
 *  Created on: Jun 25, 2024
 *      Author: gomas
 */

#ifndef PWM_HPP_
#define PWM_HPP_

#include "main.h"


namespace SabaneLib{

	class IPWM{
	public:
		//duty比は0.0f~1.0fの範囲で入力
		//duty setter
		virtual void operator()(float val) = 0;
		//duty getter
		virtual float operator()(void)const = 0;

		virtual void set_period(uint32_t _period) = 0;

		virtual uint32_t get_period(void)const = 0;

		//更新処理(ソフトウェアPWM用)
		virtual void update(void) = 0;

		virtual ~IPWM(){}
	};

	////////////////////////////////////////////////////////////
	//Hardware PWM class////////////////////////////////////////
	////////////////////////////////////////////////////////////
#ifdef HAL_TIM_MODULE_ENABLED
	class PWMHard:public IPWM{
	private:
		TIM_HandleTypeDef* const tim;
		const uint32_t ch;
	public:
		PWMHard(TIM_HandleTypeDef *_tim,uint32_t _ch)
			: tim(_tim),
			  ch(_ch){
		}

		void operator()(float duty) override{//0.0~1.0f
			__HAL_TIM_SET_COMPARE(tim, ch, static_cast<float>(__HAL_TIM_GET_AUTORELOAD(tim))*duty);
		}

		float operator()(void)const override{
			return static_cast<float>(__HAL_TIM_GET_COMPARE(tim, ch))/static_cast<float>(__HAL_TIM_GET_AUTORELOAD(tim));
		}

		void set_period(uint32_t _period)override{
			__HAL_TIM_SET_AUTORELOAD(tim, _period);
		}

		uint32_t get_period(void)const override{
			return __HAL_TIM_GET_AUTORELOAD(tim);
		}

		void update(void)override{
			//nop
		}

		void start(void){
			HAL_TIM_PWM_Start(tim, ch);
			HAL_TIMEx_PWMN_Start(tim,ch);
			tim->Instance->CR1 |= TIM_CR1_ARPE;//ARPEをセット 次回カウントリセット時にARRを適用
			__HAL_TIM_SET_COMPARE(tim, ch,0);
		}

		void stop(void){
			HAL_TIM_PWM_Stop(tim, ch);
			HAL_TIMEx_PWMN_Stop(tim,ch);
			__HAL_TIM_SET_COMPARE(tim, ch,0);
		}

	};
#endif //HAL_TIM_MODULE_ENABLED


	class PWMSoft:public IPWM{
	private:
		GPIO_TypeDef* const port;
		const uint_fast16_t pin;

		uint32_t count = 0;
		uint32_t period = 0;
		uint32_t duty = 0xFFFFFFFF;

	public:
		PWMSoft(GPIO_TypeDef *_port,uint_fast16_t _pin,uint32_t _period)
			: port(_port),pin(_pin),period(_period){
		}

		void operator()(float val)override{//0.0f=0.1f
			duty = val*static_cast<float>(period);
		}

		float operator()(void)const override{
			return static_cast<float>(duty)/static_cast<float>(period);
		}

		void set_period(uint32_t _period)override{
			count = 0;
			period = _period;
		}

		uint32_t get_period(void)const override{
			return period;
		}

		//timer interrupt function
		void update(void)override{
			//bool*intの計算となっているので諸説
			//条件分岐より早いので採用
			//C言語においてstatic_cast<int>(bool)は0 or 1が保証されているようだ
			count = static_cast<int>(count < period)*(count+1);
			port->BSRR = pin << (16*static_cast<int>(count >= duty));
		}


	};

}


#endif /* PWM_HPP_ */
