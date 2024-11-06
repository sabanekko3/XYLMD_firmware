/*
 * motor_math.hpp
 *
 *  Created on: Oct 21, 2024
 *      Author: gomas
 */

#ifndef COMMONLIB_MOTOR_MATH_HPP_
#define COMMONLIB_MOTOR_MATH_HPP_

#include "arm_math.h"

#define __FPU_PRESENT
#define ARM_MATH_CM4

namespace SabaneLib::Math{
	struct DQ;
	struct AB;
	struct UVW;

	//高速化のため0初期化を省略
	//必要な場合はその都度初期化せよ
	struct SinCos{
		float sin;
		float cos;
	};

	//高速化のため0初期化を省略
	//必要な場合はその都度初期化せよ
	struct DQ{
		float d;
		float q;

		AB to_ab(SinCos sincos);
		UVW to_uvw(SinCos sincos);
	};

	//高速化のため0初期化を省略
	//必要な場合はその都度初期化せよ
	struct AB{
		float a;
		float b;

		DQ to_dq(SinCos sincos);
		UVW to_uvw(void);
	};

	//高速化のため0初期化を省略
	//必要な場合はその都度初期化せよ
	struct UVW{
		float u;
		float v;
		float w;

		DQ to_dq(SinCos sincos);
		AB to_ab(void);
		UVW& sv_modulation(void);//空間ベクトル変調
		UVW& sperimposition(float sub);//信号重畳
	};

	//struct DQ functions
	inline AB DQ::to_ab(SinCos sincos){
		AB ab_data;
		arm_inv_park_f32(d,q,&ab_data.a,&ab_data.b,sincos.sin,sincos.cos);
		return ab_data;
	}

	inline UVW DQ::to_uvw(SinCos sincos){
		AB ab_data;
		UVW uvw_data;

		arm_inv_park_f32(d,q,&ab_data.a,&ab_data.b,sincos.sin,sincos.cos);
		arm_inv_clarke_f32(ab_data.a,ab_data.b,&uvw_data.u,&uvw_data.v);
		uvw_data.w = -uvw_data.u - uvw_data.v;
		return uvw_data;
	}

	//struct AB functions
	inline DQ AB::to_dq(SinCos sincos){
		DQ dq_data;
		arm_park_f32(a,b,&dq_data.d,&dq_data.q,sincos.sin,sincos.cos);
		return dq_data;
	}
	inline UVW AB::to_uvw(void){
		UVW uvw_data;
		arm_inv_clarke_f32(a,b,&uvw_data.u,&uvw_data.v);
		uvw_data.w = -uvw_data.u - uvw_data.v;
		return uvw_data;
	}

	//struct UVW functions
	inline DQ UVW::to_dq(SinCos sincos){
		AB ab_data;
		DQ dq_data;

		arm_clarke_f32(u,v,&ab_data.a,&ab_data.b);
		arm_park_f32(ab_data.a,ab_data.b,&dq_data.d,&dq_data.q,sincos.sin,sincos.cos);
		return dq_data;
	}

	inline AB UVW::to_ab(void){
		AB ab_data;
		arm_clarke_f32(u,v,&ab_data.a,&ab_data.b);
		return ab_data;
	}

	inline UVW& UVW::sv_modulation(void){
		float max = u>v ? u : v;
		max = max>w ? max : w;
		float min = u<v ? u : v;
		min = min<w ? min : w;

		float ave = (max + min)*0.5f;
		u -= ave;
		v -= ave;
		w -= ave;

		return *this;
	}
	inline UVW& UVW::sperimposition(float bias){
		u += bias;
		v += bias;
		w += bias;
		return *this;
	}
}


#endif /* COMMONLIB_MOTOR_MATH_HPP_ */
