/*
 * board_main.hpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#ifndef BOARD_MAIN_HPP_
#define BOARD_MAIN_HPP_

#include "board_params.hpp"
#include "motor.hpp"

#include "CommonLib/MotorMath/sin_table.hpp"
#include "CommonLib/pwm.hpp"
#include "CommonLib/pid.hpp"
#include "CommonLib/programable_LED.hpp"
#include "CommonLib/LED_pattern.hpp"
#include "CommonLib/cordic.hpp"
#include "CommonLib/fdcan_control.hpp"

#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "fdcan.h"
#include "i2c.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>

namespace LMDBoard{

	inline auto table = SabaneLib::MotorMath::SinTable<12>{};
	inline auto cordic = SabaneLib::MotorMath::FastMathCordic{CORDIC};

	inline auto atan_enc = SabaneLib::ContinuableEncoder{16,18000.f};

	inline auto motor = LMDLib::Motor{
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_2},
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_3},
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_1}
	};

	namespace PIDIns{
		inline auto position = SabaneLib::PIDBuilder(18000.0f)
				.set_gain(0.000'1f, 0.000'1f, 0.0f)
				.set_limit(0.0f)
				.build();

		inline auto d_current = SabaneLib::PIDBuilder(18000.0f)
				.set_gain(0.1f, 0.8f, 0.0f)
				.set_limit(1.0f)
				.build();

		inline auto q_current = SabaneLib::PIDBuilder(18000.0f)
				.set_gain(0.1f, 0.8f, 0.0f)
				.set_limit(1.0f)
				.build();
	}


	inline auto can = SabaneLib::FdCanComm{&hfdcan1,
		std::make_unique<SabaneLib::RingBuffer<SabaneLib::CanFrame,5> >(),
		std::make_unique<SabaneLib::RingBuffer<SabaneLib::CanFrame,5> >(),
		SabaneLib::FdCanRxFifo[0]
	};

	inline auto led = SabaneLib::LEDLLGpio{LED_GPIO_Port,LED_Pin};

	inline constexpr auto my_axis = LSMParam::Axis::Y;

	inline q15_t qsin;
	inline q15_t qcos;

	inline SabaneLib::MotorMath::UVW uvw_i;
	inline SabaneLib::MotorMath::DQ dq_i;

	inline SabaneLib::MotorMath::DQ target_i;

	inline float target_mm = 0.0f;

	inline int32_t atan_enc_bias = 0;

	inline volatile uint16_t adc_val[3]={0};
	inline volatile uint16_t vref_val = 0;
}


#endif /* BOARD_MAIN_HPP_ */
