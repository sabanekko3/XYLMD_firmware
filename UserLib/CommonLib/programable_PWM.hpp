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

#include <memory>

namespace SabaneLib{
	struct PWMState{
		float power;
		uint32_t interval;
	};
}

inline bool operator==(const SabaneLib::PWMState& s1,const SabaneLib::PWMState& s2){
	return (s1.power == s2.power) && (s1.interval == s2.interval);
}

namespace SabaneLib{
	class ProgramablePWM{
	private:
		const PWMState *playing_pattern = nullptr;
		uint32_t pattern_count = 0;
		uint32_t interval_count = 0;

	public:
		static constexpr PWMState end_of_pwm_sequence{0.0f,0};
		std::unique_ptr<IPWM> pwm;

		ProgramablePWM(std::unique_ptr<IPWM> _pwm):
			pwm(std::move(_pwm)){
		}

		void play(const PWMState *pattern){
			playing_pattern = pattern;
			pattern_count = 0;
			interval_count = 0;

			interval_count = playing_pattern[pattern_count].interval;

			(*pwm)(playing_pattern[pattern_count].power);
		}

		bool is_playing(void){
			return playing_pattern!=nullptr;
		}

		void update(void){
			if(playing_pattern == nullptr){
				return;
			}
			interval_count  --;
			if(interval_count <= 0){
				++pattern_count;

				if(playing_pattern[pattern_count] == end_of_pwm_sequence){
					playing_pattern = nullptr;
					(*pwm)(0.0f);
					return;
				}
				interval_count = playing_pattern[pattern_count].interval;
				(*pwm)(playing_pattern[pattern_count].power);
			}
		}

		void out_weak(float val){
			if(not is_playing())(*pwm)(val);
		}
	};
}





#endif /* PROGRAMABLE_LED_HPP_ */
