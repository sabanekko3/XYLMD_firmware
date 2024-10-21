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

namespace SabaneLib::MotorMath{
	struct DQ{
		float d = 0.0f;
		float q = 0.0f;
	};

	struct AB{
		float a = 0.0f;
		float b = 0.0f;
	};

	struct UVW{
		float u = 0.0f;
		float v = 0.0f;
		float w = 0.0f;
	};

	struct SinCos{
		float sin = 0.0f;
		float cos = 0.0f;
	};

	inline AB uvw_to_ab(const UVW &input){
		AB tmp;
		arm_clarke_f32(input.u,input.v,&tmp.a,&tmp.b);
		return tmp;
	}

	inline DQ ab_to_dq(AB input,SinCos sincos){
		DQ tmp;
		arm_park_f32(input.a,input.b,&tmp.d,&tmp.q,sincos.sin,sincos.cos);
		return tmp;
	}

	inline UVW dq_to_uvw(DQ input,SinCos sincos){
		AB ab_data;
		UVW uvw_data;

		arm_inv_park_f32(input.d,input.q,&ab_data.a,&ab_data.b,sincos.sin,sincos.cos);
		arm_inv_clarke_f32(ab_data.a,ab_data.b,&uvw_data.u,&uvw_data.v);
		uvw_data.w = -uvw_data.u - uvw_data.v;
		return uvw_data;
	}

	inline DQ uvw_to_dq(UVW input,SinCos sincos){
		AB ab_data;
		DQ dq_data;

		arm_clarke_f32(input.u,input.v,&ab_data.a,&ab_data.b);
		arm_park_f32(ab_data.a,ab_data.b,&dq_data.d,&dq_data.q,sincos.sin,sincos.cos);
		return dq_data;
	}
}


#endif /* COMMONLIB_MOTOR_MATH_HPP_ */
