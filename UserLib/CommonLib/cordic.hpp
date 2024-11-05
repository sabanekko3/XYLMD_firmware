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

class CordicHandler{
protected:
	CORDIC_TypeDef *const cordic;

	constexpr uint32_t generate_CSR(CordicMode mode,bool input_2param,bool output_2param,uint32_t precision,uint32_t scale){
		assert(1 <= precision || precision <= 15);
		assert(scale <= 15);

		return ((0b0 << 22) |      //CSR.ARGSIZE, input  16bit
				(0b1 << 21) |      //CSR.RESSIZE, output 16bit
				((input_2param?0b1:0) << 20) |          //CSR.NARGS
//				((output_2param?0b1:0) << 19) |          //CSR.NRES
				((scale & 0b111) << 8) |
				((precision & 0b1111) << 4) |
				((static_cast<uint32_t>(mode) & 0b1111) << 0)
			); //Q15format DMAWEN,DMAREN and IEN are disable.
	}

public:
	CordicHandler(CORDIC_TypeDef *_cordic)
	:cordic(_cordic){
		//nop
	}

	void set_mode(CordicMode mode,bool input_2param,bool output_2param,uint32_t precision = 4,uint32_t scale = 0){ //precision:1~15
		cordic->CSR = generate_CSR(mode,input_2param,output_2param,precision,scale);
	}

	void set_param(q15_t input1){
		cordic->WDATA = input1 << 16;
	}

	void set_param(q15_t input1,q15_t input2){
		cordic->WDATA = input1 << 16;
		cordic->WDATA = input2 << 16;
	}

	bool is_avilable(void)const{
		return (cordic->CSR&0x8000'0000)==0x8000'0000; //CSR::RRDY. 1 is ready
	}

	q15_t read_ans(void){
		return cordic->RDATA;
	}

	std::pair<q15_t,q15_t> read_ans_pair(void){
		uint32_t tmp = cordic->RDATA;
		return std::pair<q15_t,q15_t>{tmp,tmp>>16};
	}
};



class FastMathCordic{
public:
	CordicHandler handler;

	FastMathCordic(CORDIC_TypeDef *_cordic)
	:handler(_cordic){
	}

	void start_sincos(float rad){
		handler.set_mode(CordicMode::SIN_COS,false,true,4,0);
		handler.set_param(Math::rad_to_q15(rad));
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
