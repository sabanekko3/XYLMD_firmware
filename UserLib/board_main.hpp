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
#include "CommonLib/filter.hpp"

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

namespace BoardElement{

	inline constexpr auto my_axis = BoardLib::Axis::Y;

	inline auto table = SabaneLib::MotorMath::SinTable<12>{};
	inline auto cordic = SabaneLib::MotorMath::FastMathCordic{CORDIC};

	inline q15_t e_angle;
	inline auto atan_enc = SabaneLib::ContinuableEncoder{16,18000.f};
	inline auto enc_filter = SabaneLib::LowpassFilter<float>{0.05};
	inline auto target_filter = SabaneLib::LowpassFilter<float>{0.05};

	inline auto motor = LMDLib::Motor{
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_2},
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_3},
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_1}
	};

	namespace PIDIns{
		inline auto position = SabaneLib::PIDBuilder(1000.0f)
				.set_gain(0.000'1f, 0.000'05f, 0.0f)
				.set_limit(0.0f)
				.build();

		inline auto d_current = SabaneLib::PIBuilder(18000.0f)
				.set_gain(0.1f, 0.8f)
				.set_limit(1.0f)
				.build();

		inline auto q_current = SabaneLib::PIBuilder(18000.0f)
				.set_gain(0.1f, 0.8f)
				.set_limit(1.0f)
				.build();
	}

	//静的にunique_ptrを生成する
	//make_uniqueはnewを使用しているため微妙
	inline auto can_tx_buff = SabaneLib::RingBuffer<SabaneLib::CanFrame,5>{};
	inline auto can_rx_buff = SabaneLib::RingBuffer<SabaneLib::CanFrame,5>{};
	inline auto can = SabaneLib::FdCanComm{&hfdcan1,
		std::unique_ptr<SabaneLib::RingBuffer<SabaneLib::CanFrame,5>>(&can_tx_buff),
		std::unique_ptr<SabaneLib::RingBuffer<SabaneLib::CanFrame,5>>(&can_rx_buff),
		SabaneLib::FdCanRxFifo[0]
	};

	inline auto led = SabaneLib::LEDLLGpio{LED_GPIO_Port,LED_Pin};


	//変数
	inline float vbus_voltage = 0.0f;

	inline SabaneLib::MotorMath::UVW uvw_i = {.u=0.0f, .v=0.0f, .w=0.0f};
	inline SabaneLib::MotorMath::AB ab_i = {.a = 0.0f, .b = 0.0f};
	inline SabaneLib::MotorMath::DQ dq_i = {.d = 0.0f, .q = 0.0f};

	inline SabaneLib::MotorMath::DQ target_i = {.d = 0.0f, .q =0.0f};

	inline float target_angle = 0.0f;

	inline int32_t atan_enc_bias = 0;

	namespace TestFunctions{
		void print_param(void);
		void move_test(void);
	}
}


#endif /* BOARD_MAIN_HPP_ */
