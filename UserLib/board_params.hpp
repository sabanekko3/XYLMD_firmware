/*
 * board_params.hpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#ifndef BOARD_PARAMS_HPP_
#define BOARD_PARAMS_HPP_

namespace LSMParam{
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

	constexpr float mm_to_q15rad = static_cast<float>(0xFFFF) / 30.0f;
	constexpr float q15rad_to_mm = 30.0f / static_cast<float>(0xFFFF);
}

#endif /* BOARD_PARAMS_HPP_ */
