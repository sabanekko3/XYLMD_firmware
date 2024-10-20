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
		std::function<s::MotorMath::SinCos(q15_t)> f;
		const s::IEncoder &enc;

	public:
		Motor(s::PWMHard _u,s::PWMHard _v, s::PWMHard _w,std::function<s::MotorMath::SinCos(q15_t)>  _f,const s::IEncoder &_enc)
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
			SabaneLib::MotorMath::UVW tmp;
			SabaneLib::MotorMath::dq_to_uvw({0, power},f(static_cast<q15_t>(enc.get_angle())),tmp);
			u.out(tmp.u*0.4f + 0.5f);
			v.out(tmp.v*0.4f + 0.5f);
			w.out(tmp.w*0.4f + 0.5f);
		}
	};
}




#endif /* MOTOR_HPP_ */
