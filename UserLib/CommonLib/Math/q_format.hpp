/*
 * q_format.hpp
 *
 *  Created on: Oct 21, 2024
 *      Author: gomas
 */

#ifndef COMMONLIB_Q_FORMAT_HPP_
#define COMMONLIB_Q_FORMAT_HPP_

#include "arm_math.h"

namespace SabaneLib::Math{

	template<typename T>
	concept QFromat = std::is_same_v<T, q15_t> || std::is_same_v<T, q31_t>;

	////////////////////////////////
	//Q31 format support funcitons//
	////////////////////////////////
	namespace Q31Def{
		constexpr q31_t pi = 0x7FFF'FFFF; //厳密には0x8000'0000がpiだが-piと混ざり支障が出るため0x7FFF'FFFFとする
		constexpr q31_t pi_2 = pi/2; // PI/2
		constexpr q31_t pi_3 = pi/3; // PI/3
	}

	constexpr q31_t float_to_q31(float f){
		constexpr float coef = static_cast<float>(0x8000'0000);
		return static_cast<q31_t>(f * coef);
	}

	constexpr float q31_to_float(q31_t q){
		constexpr float coef = 1.0f / static_cast<float>(0x8000'0000);
		return static_cast<float>(q) * coef;
	}

	constexpr q31_t rad_to_q31(float rad){ //-PI~PI
		constexpr float coef = static_cast<float>(0x8000'0000)/M_PI;
		return static_cast<q31_t>(rad * coef);
	}

	constexpr float q31_to_rad(q31_t rad){  //-PI~PI
		constexpr float coef = M_PI/static_cast<float>(0x8000'0000);
		return static_cast<float>(rad) * coef;
	}


	constexpr q31_t rad_to_q31_normalized(float rad){  //入力範囲の制限無し(-32768~32768 精度は16bit分のみ
		constexpr float coef = static_cast<float>(0x8000)/M_PI;
		return static_cast<q31_t>(rad * coef) << 16;
	}

	////////////////////////////////
	//Q15 format support funcitons//
	////////////////////////////////
	namespace Q15Def{
		constexpr q15_t pi = 0x7FFF; //厳密には0x8000がpiだが-piと混ざり支障が出るため0x7FFFとする
		constexpr q15_t pi_2 = pi/2; // PI/2
		constexpr q15_t pi_3 = pi/3; // PI/3
	}

	constexpr q15_t float_to_q15(float f){
		constexpr float coef = static_cast<float>(0x8000);
		return static_cast<q15_t>( f * coef);
	}

	constexpr float q15_to_float(q15_t q){
		constexpr float coef = 1.0f / static_cast<float>(0x8000);
		return static_cast<float>(q) *coef;
	}

	constexpr q15_t rad_to_q15(float rad){
		constexpr float coef = static_cast<float>(0x8000)/(M_PI);
		return static_cast<q15_t>(static_cast<int32_t>(rad * coef));
	}

	constexpr float q15_to_rad(q15_t angle){
		constexpr float coef = M_PI / static_cast<float>(0x8000);
		return static_cast<float>(angle)*coef;
	}
}



#endif /* COMMONLIB_Q_FORMAT_HPP_ */
