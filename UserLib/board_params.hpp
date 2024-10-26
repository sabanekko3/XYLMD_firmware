/*
 * board_params.hpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#ifndef BOARD_PARAMS_HPP_
#define BOARD_PARAMS_HPP_

#include "CommonLib/byte_reader_writer.hpp"

#include "main.h"

namespace BoardLib{
	enum class Axis{
		X,
		Y
	};

	enum class Command{
		SET_ORIGIN,
		TARGET_POS,
		POWER,
		GAIN_P,
		GAIN_I,
		GAIN_D
	};

	namespace Coef{
		constexpr float mm_to_q15rad = static_cast<float>(0xFFFF) / 30.0f;
		constexpr float q15rad_to_mm = 30.0f / static_cast<float>(0xFFFF);
	}


	constexpr float adc_to_current(uint16_t adc_val){
		constexpr float rl = (2.2f*1.5f)/(2.2f+1.5f); //基板上の並列接続された抵抗の合成抵抗
		constexpr float v_bias = rl/(rl+22.0f)*3.3f;  //バイアス電圧
		constexpr float amp_gain_inv = 1.0f/8.0f; //cube mxで設定するオペアンプのゲインの逆数
		constexpr float shant_r_inv = 1.0f/0.005f;         //シャント抵抗値の逆数
		constexpr float bias = (amp_gain_inv + 1.0f)*v_bias;

		float v = adc_val*3.3f/static_cast<float>(1<<12);

		return -(bias - amp_gain_inv*v)*shant_r_inv;
	}

	constexpr float adc_to_voltage(uint16_t adc_val,float gain = 1.0f){
		constexpr float coef = 3.3f/static_cast<float>(0xFFF);
		return adc_val * coef * gain;
	}

	inline float data_select(Axis xy,SabaneLib::ByteReader &r){
		auto data_x = r.read<float>();
		auto data_y = r.read<float>();

		switch(xy){
		case Axis::X:
			return data_x.has_value() ? data_x.value() : 0.0f;
			break;
		case Axis::Y:
			return data_y.has_value() ? data_y.value() : 0.0f;
			break;
		}
		return 0.0f;
	}
}

#endif /* BOARD_PARAMS_HPP_ */
