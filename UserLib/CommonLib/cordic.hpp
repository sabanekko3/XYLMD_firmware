/*
 * cordic.hpp
 *
 *  Created on: Aug 2, 2024
 *      Author: gomas
 */

#ifndef CORDIC_HPP_
#define CORDIC_HPP_

#include "main.h"

#include <cassert>
#include <utility>

#ifdef HAL_CORDIC_MODULE_ENABLED

namespace SabaneLib{

enum class CordicMode:uint32_t{
	COS_SIN,
	SIN_COS,
	PHASE_MODULUS,
	ATAN,
	COSH_SINH,
	SINH_CONH,
	ATANH,
	LN,
	SQRT,
};

template<Math::QFromat T>
class CordicHandler{
protected:
	CORDIC_TypeDef *const cordic;

	constexpr uint32_t generate_CSR(CordicMode mode,bool input_2param,bool output_2param,uint32_t precision,uint32_t scale){
		assert(1 <= precision || precision <= 15);
		assert(scale <= 15);

		//DMAWEN,DMAREN and IEN are disable.
		if constexpr(std::is_same<T,q15_t>()){
			return ((0b1 << 22) |                      //CSR.ARGSIZE, input  16bit
					(0b1 << 21) |                      //CSR.RESSIZE, output 16bit
					((input_2param ? 0u:0u) << 20) |   //CSR.NARGS
					((output_2param ? 0u:0u) << 19) |  //CSR.NRES
					((scale & 0b111) << 8) |
					((precision & 0b1111) << 4) |
					((static_cast<uint32_t>(mode) & 0b1111) << 0)
			);
		}else if(std::is_same<T,q31_t>()){
			return ((0b0 << 22) |                      //CSR.ARGSIZE, input  32bit
					(0b0 << 21) |                      //CSR.RESSIZE, output 32bit
					((input_2param ? 1u:0u) << 20) |   //CSR.NARGS
					((output_2param ? 1u:0u) << 19) |  //CSR.NRES
					((scale & 0b111) << 8) |
					((precision & 0b1111) << 4) |
					((static_cast<uint32_t>(mode) & 0b1111) << 0)
			);
		}
		return 0;
	}

public:
	CordicHandler(CORDIC_TypeDef *_cordic)
	:cordic(_cordic){
		//nop
	}

	void set_mode(CordicMode mode,bool input_2param,bool output_2param,uint32_t precision = 4,uint32_t scale = 0){ //precision:1~15
		cordic->CSR = generate_CSR(mode,input_2param,output_2param,precision,scale);
	}

	void set_param(T input1){
		cordic->WDATA = input1;
	}

	void set_param(T input1,T input2){
		if constexpr(std::is_same<T,q15_t>()){
			cordic->WDATA = static_cast<uint16_t>(input1) | (input2)<<16;
		}else if(std::is_same<T,q31_t>()){
			cordic->WDATA = input1;
			cordic->WDATA = input2;
		}
	}

	bool is_available(void)const{
		return (cordic->CSR&0x8000'0000)==0x8000'0000; //CSR::RRDY. 1 is ready
	}

	T read_ans(void){
		return cordic->RDATA;
	}

	std::pair<T,T> read_ans_pair(void){
		if constexpr(std::is_same<T,q15_t>()){
			uint32_t tmp = cordic->RDATA;
			return std::pair<T,T>{tmp&0xFFFF,(tmp>>16)&0xFFFF};
		}else if(std::is_same<T,q31_t>()){
			T tmp1 = cordic->RDATA;
			T tmp2 = cordic->RDATA;
			return std::pair<T,T>{tmp1,tmp2};
		}
	}
};

class FastMathCordic{
public:
	CordicHandler<q15_t> handler;

	FastMathCordic(CORDIC_TypeDef *_cordic)
	:handler(_cordic){
	}

	void start_sincos(float rad){
		handler.set_mode(CordicMode::SIN_COS,true,true,4,0);
		handler.set_param(Math::rad_to_q15(rad),0x7FFF);
	}

	Math::SinCos get_sincos(void){
		auto [s,c] = handler.read_ans_pair();
		return Math::SinCos{.sin = Math::q15_to_float(s),.cos=Math::q15_to_float(c)};
	}

	void start_atan2(float x,float y){
		handler.set_mode(CordicMode::PHASE_MODULUS,true,false,4,0);
		handler.set_param(Math::float_to_q15(x),Math::float_to_q15(y));
	}
	void start_atan2(q15_t x,q15_t y){
		handler.set_mode(CordicMode::PHASE_MODULUS,true,false,4,0);
		handler.set_param(x,y);
	}

	float get_atan2(void){
		return Math::q15_to_rad(handler.read_ans());
	}
	q15_t get_atan2_q15(void){
		return handler.read_ans();
	}
};

}

#endif


#endif /* CORDIC_HPP_ */
