/*
 * motor.hpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include "CommonLib/pwm.hpp"
#include "CommonLib/math.hpp"
#include "CommonLib/encoder.hpp"

#include <functional>

namespace s = SabaneLib;

namespace LMDLib{
	class Motor{
		s::PWMHard u,v,w;
		std::function<s::MotorMath::UVW(float)> f;
		const s::IEncoder &enc;

	public:
		Motor(s::PWMHard _u,s::PWMHard _v, s::PWMHard _w,std::function<s::MotorMath::UVW (float) > _f,const s::IEncoder &_enc)
			:u(_u),
			 v(_v),
			 w(_w),
			 f(_f),
			 enc(_enc){
		}

		void start(void){
			u.start();
			v.start();
			w.start();
			HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
		}

		void move(float power){
			float rad = SabaneLib::MotorMath::q15_to_rad(enc.get_angle()) + (M_PI/2.0f)*(power>0.0f?1.0f:-1.0f);

			s::MotorMath::UVW uvw_phase = f(rad);
			u.out(uvw_phase.u*0.4f*abs(power) + 0.5f);
			v.out(uvw_phase.v*0.4f*abs(power) + 0.5f);
			w.out(uvw_phase.w*0.4f*abs(power) + 0.5f);
		}
	};
}




#endif /* MOTOR_HPP_ */
