/*
 * programmable_pwm.hpp
 *
 *  Created on: Jun 27, 2024
 *      Author: gomas
 */

#ifndef PROGRAMABLE_LED_HPP_
#define PROGRAMABLE_LED_HPP_

#include "pwm.hpp"
#include "main.h"
namespace SabaneLib{
	struct LEDState{
		float power;
		uint32_t length;
	};

	constexpr LEDState end{0.0f,0};

	class ILED{
	public:
		virtual void play(const LEDState *pattern) = 0;
		virtual bool is_playing(void) = 0;
		virtual ~ILED(){}
	};

	/////////////////////////////////
	//LED control with hardware pwm//
	/////////////////////////////////
#ifdef HAL_TIM_MODULE_ENABLED
	class LEDPWMHard:public ILED,public PWMHard{
	private:
		const LEDState *playing_pattern = nullptr;
		uint32_t pattern_count = 0;
		uint32_t length_count = 0;

	public:
		LEDPWMHard(TIM_HandleTypeDef *tim,uint32_t ch):
			PWMHard(tim,ch){
		}

		void play(const LEDState *pattern) override{
			playing_pattern = pattern;
			pattern_count = 0;
			length_count = 0;

			length_count = playing_pattern[pattern_count].length;

			out(playing_pattern[pattern_count].power);
		}

		bool is_playing(void)override{return playing_pattern!=nullptr ? true:false;}

		void update(void){
			if(playing_pattern == nullptr){
				return;
			}
			length_count  --;
			if(length_count <= 0){
				pattern_count ++;

				if(playing_pattern[pattern_count].length == 0){
					playing_pattern = nullptr;
					out(0.0f);
					return;
				}
				length_count = playing_pattern[pattern_count].length;
				out(playing_pattern[pattern_count].power);
			}
		}

		void out_weak(float val){
			if(not is_playing()) out(val);
		}
	};
#endif

	////////////////////////////////
	//LED control with GPIO LL Lib//
	////////////////////////////////
#ifdef STM32G4xx_LL_GPIO_H
	class LEDLLGpio:public ILED{
	private:
		GPIO_TypeDef *port;
		const uint32_t pin;

		const LEDState *playing_pattern = nullptr;
		uint32_t pattern_count = 0;
		uint32_t length_count = 0;

	public:
		LEDLLGpio(GPIO_TypeDef *_port,uint32_t _pin):port(_port),pin(_pin){
		}

		void play(const LEDState *pattern) override{
			playing_pattern = pattern;
			pattern_count = 0;
			length_count = 0;

			length_count = playing_pattern[pattern_count].length;
			if(playing_pattern[pattern_count].power > 0.5f){
				LL_GPIO_SetOutputPin(port,pin);
			}else{
				LL_GPIO_ResetOutputPin(port,pin);
			}
		}

		bool is_playing(void)override{return playing_pattern!=nullptr ? true:false;}

		void update(void){
			if(playing_pattern == nullptr){
				return;
			}
			length_count  --;
			if(length_count <= 0){
				pattern_count ++;

				if(playing_pattern[pattern_count].length == 0){
					playing_pattern = nullptr;
					HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET);
					return;
				}
				length_count = playing_pattern[pattern_count].length;
				if(playing_pattern[pattern_count].power > 0.5f){
					LL_GPIO_SetOutputPin(port,pin);
				}else{
					LL_GPIO_ResetOutputPin(port,pin);
				}
			}
		}
	};
#endif //STM32G4xx_LL_GPIO_H

	/////////////////////////////////
	//LED control with GPIO HAL Lib//
	/////////////////////////////////
#ifdef STM32G4xx_HAL_GPIO_H
	class LEDHALGpio:public ILED{
	private:
		GPIO_TypeDef *port;
		const uint32_t pin;

		const LEDState *playing_pattern = nullptr;
		uint32_t pattern_count = 0;
		uint32_t length_count = 0;

	public:
		LEDHALGpio(GPIO_TypeDef *_port,uint32_t _pin):port(_port),pin(_pin){
		}

		void play(const LEDState *pattern) override{
			playing_pattern = pattern;
			pattern_count = 0;
			length_count = 0;

			length_count = playing_pattern[pattern_count].length;

			HAL_GPIO_WritePin(port,pin,(playing_pattern[pattern_count].power > 0.5) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		}

		bool is_playing(void)override{return playing_pattern!=nullptr ? true:false;}

		void update(void){
			if(playing_pattern == nullptr){
				return;
			}

			length_count  --;
			if(length_count <= 0){
				pattern_count ++;

				if(playing_pattern[pattern_count].length == 0){
					playing_pattern = nullptr;
					HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET);
					return;
				}
				length_count = playing_pattern[pattern_count].length;
				HAL_GPIO_WritePin(port,pin,(playing_pattern[pattern_count].power > 0.5) ? GPIO_PIN_SET : GPIO_PIN_RESET);
			}
		}
	};
#endif //STM32G4xx_HAL_GPIO_H
}



#endif /* PROGRAMABLE_LED_HPP_ */
