/*
 * board_params.hpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#ifndef BOARD_PARAMS_HPP_
#define BOARD_PARAMS_HPP_

namespace LSMParam{
	enum class Shaft{
		X,
		Y
	};

	enum class Config{
		POS,
		POWER,
		GAIN_P,
		GAIN_I,
		GAIN_D
	};
}



#endif /* BOARD_PARAMS_HPP_ */
