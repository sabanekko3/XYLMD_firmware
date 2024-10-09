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
		virtual void start_sequence(const LEDState *pattern) = 0;
		virtual bool is_playing(void) = 0;
	};

	//LED control with hardware pwm
	class LEDPWMHard:public ILED,public PWMHard{
	private:
		const LEDState *playing_pattern = nullptr;
		uint32_t pattern_count = 0;
		uint32_t length_count = 0;

	public:
		LEDPWMHard(TIM_HandleTypeDef *tim,uint32_t ch):
			PWMHard(tim,ch){
		}

		void start_sequence(const LEDState *pattern) override{
			playing_pattern = pattern;
			pattern_count = 0;
			length_count = 0;

			length_count = playing_pattern[pattern_count].length;

			out(playing_pattern[pattern_count].power);
		}

		bool is_playing(void)override{return playing_pattern!=nullptr ? true:false;}

		void update(void){
			if(playing_pattern != nullptr){
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
			}else{

			}
		}

		void out_weak(float val){
			if(not is_playing()) out(val);
		}

	};
}



#endif /* PROGRAMABLE_LED_HPP_ */
