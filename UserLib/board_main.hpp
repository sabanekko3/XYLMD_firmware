/*
 * board_main.hpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#ifndef BOARD_MAIN_HPP_
#define BOARD_MAIN_HPP_

#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "fdcan.h"
#include "i2c.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "board_params.hpp"
#include "motor.hpp"

#include "CommonLib/pwm.hpp"
#include "CommonLib/pid.hpp"
#include "CommonLib/math.hpp"
#include "CommonLib/programable_LED.hpp"
#include "CommonLib/LED_pattern.hpp"
#include "CommonLib/sincos_enc.hpp"
#include "CommonLib/cordic.hpp"
#include "CommonLib/fdcan_control.hpp"

namespace LMDBoard{

	inline auto table = SabaneLib::MotorMath::SinTable<12>{};
	inline auto cordic = SabaneLib::MotorMath::FastMathCordic{CORDIC};

	inline auto atan_enc = SabaneLib::ContinuableEncoder{16,1000.f};

	inline auto motor = LMDLib::Motor{
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_2},
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_3},
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_1},
		[](q15_t r)->SabaneLib::MotorMath::SinCos {return table.sin_cos(r);},
		atan_enc
	};

	inline auto position_pid = SabaneLib::PIDBuilder(1000.0f)
			.set_gain(0.000'01f, 0.000'007f, 0.0f)
			.set_limit(0.1f)
			.build();

	inline auto can = SabaneLib::FdCanComm{&hfdcan1,
		std::make_unique<SabaneLib::RingBuffer<SabaneLib::CanFrame,5> >(),
		std::make_unique<SabaneLib::RingBuffer<SabaneLib::CanFrame,5> >(),
		SabaneLib::FdCanRxFifo::no0
	};

	inline constexpr auto my_axis = LSMParam::Axis::X;

	inline q15_t qsin;
	inline q15_t qcos;

	inline SabaneLib::MotorMath::UVW uvw_i;
	inline SabaneLib::MotorMath::DQ dq_i;

	inline float target_mm;

	inline int32_t atan_enc_bias = 0;

	inline volatile uint16_t adc_val[3]={0};
	inline volatile uint16_t vref_val = 0;
}


#endif /* BOARD_MAIN_HPP_ */
