/*
 * math.hpp
 *
 *  Created on: Jun 26, 2024
 *      Author: gomas
 */

#ifndef MATH_HPP_
#define MATH_HPP_

#include "arm_math.h"

#define __FPU_PRESENT
#define ARM_MATH_CM4
#include <cmath>
#include <cassert>
#include <functional>

//#define SIN_TABLE_COMPACT_MODE

namespace SabaneLib::MotorMath{
	struct DQ{
		float d;
		float q;
	};

	struct AB{
		float a;
		float b;
	};

	struct UVW{
		float u;
		float v;
		float w;
	};

	struct SinCos{
		float sin;
		float cos;
	};

	////////////////////////////////
	//Q31 format support funcitons//
	////////////////////////////////
	constexpr q31_t float_to_q31(float f){
		constexpr float coef = static_cast<float>(0x8000'0000);
		return static_cast<q31_t>( f * coef);
	}

	constexpr float q31_to_float(q31_t q){
		constexpr float coef = 1.0f / static_cast<float>(0x8000'0000);
		return static_cast<float>(q) * coef;
	}

	constexpr q31_t PI_Q31 = 0x7FFF'FFFF; //厳密には0x8000'0000がpiだが-piと混ざり支障が出るため0x7FFF'FFFFとする

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
	constexpr q15_t float_to_q15(float f){
		constexpr float coef = static_cast<float>(0x8000);
		return static_cast<q15_t>( f * coef);
	}

	constexpr float q15_to_float(q15_t q){
		constexpr float coef = 1.0f / static_cast<float>(0x8000);
		return static_cast<float>(q) *coef;
	}

	constexpr q15_t PI_Q15 = 0x7FFF; //厳密には0x8000がpiだが-piと混ざり支障が出るため0x7FFFとする

	constexpr q15_t rad_to_q15(float rad){
		constexpr float coef = static_cast<float>(0x8000)/(M_PI);
		return static_cast<q15_t>(static_cast<int32_t>(rad * coef));
	}

	constexpr float q15_to_rad(q15_t angle){
		constexpr float coef = M_PI / static_cast<float>(0x8000);
		return static_cast<float>(angle)*coef;
	}

	///////////////////////
	//sin/cos table class//
	///////////////////////
	template<int PERIOD_N>
	class SinTable{
	private:
		static constexpr size_t PERIOD = 1u << PERIOD_N;
#ifdef SIN_TABLE_COMPACT_MODE
		float table[PERIOD/4];
#else
		float table[PERIOD];
#endif
	public:
		SinTable(void){}

		void generate(std::function<float(float)> func = [](float rad)->float{return std::sin(rad);}){

#ifdef SIN_TABLE_COMPACT_MODE
			for(size_t i = 0; i < PERIOD/4; i++) { table[i] = func(static_cast<float>(i)/static_cast<float>(PERIOD)*2*M_PI); }
#else
			for(size_t i = 0; i < PERIOD; i++) { table[i] = func(static_cast<float>(i)/static_cast<float>(PERIOD)*2*M_PI); }
#endif
		}

#ifdef SIN_TABLE_COMPACT_MODEf
		float sin(q31_t rad)const{
			size_t index = (rad >> (32-PERIOD_N)) & (PERIOD-1);
			if(index >= PERIOD*3/4){
				return -table[PERIOD - index - 1];
			}else if(index >= PERIOD/2){
				return -table[index - PERIOD/2];
			}else if(index >= PERIOD/4){
				return table[PERIOD/2 - index - 1];
			}else{
				return table[index];
			}
		}
#else
		float sin(q15_t rad)const{
			size_t index = (rad >> (16-PERIOD_N)) & (PERIOD-1);
			return table[index];
		}
#endif

		float cos(q15_t rad) const { return this->sin(static_cast<q15_t>(rad + PI_Q15/2));}

		float sin(float rad) const { return this->sin(rad_to_q15(rad)); }
		float cos(float rad) const  { return this->cos(rad_to_q15(rad)); }

		SinCos sin_cos(q31_t rad)const{return {this->sin(rad),this->cos(rad)};}
		SinCos sin_cos(float rad)const{return this->sin_cos(rad_to_q15(rad));}

		UVW uvw_phase(float rad)const{
			return {this->cos(rad),
					this->cos(rad - static_cast<float>(2*M_PI)/3.0f),
					this->cos(rad + static_cast<float>(2*M_PI)/3.0f)};
		}
	};



//	void dq_to_uvw(DQ input,SinCos sincos,UVW &output){
//		//inv park
//		AB ab_data;
//		arm_inv_park_f32(input.d,input.q,&ab_data.a,&ab_data.b,sincos.sin,sincos.cos);
//
//		//inv clarke
//		arm_inv_clarke_f32(ab_data.a,ab_data.b,&output.u,&output.v);
//		output.w = -output.u - output.v;
//	}
//
//	void uvw_to_dq(UVW input,SinCos sincos,DQ &output){
//		//clarke
//		AB ab_data;
//		arm_clarke_f32(input.u,input.v,&ab_data.a,&ab_data.b);
//		//park
//		arm_park_f32(ab_data.a,ab_data.b,&output.d,&output.q,sincos.sin,sincos.cos);
//	}
}



#endif /* MATH_HPP_ */
