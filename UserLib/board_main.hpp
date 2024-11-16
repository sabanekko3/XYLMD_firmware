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
#include "LED_pattern.hpp"
#include "timer_help.hpp"

#include "CommonLib/Math/sin_table.hpp"
#include "CommonLib/Math/filter.hpp"
#include "CommonLib/pwm.hpp"
#include "CommonLib/pid.hpp"
#include "CommonLib/programable_PWM.hpp"
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

namespace BoardElement{

	inline constexpr auto my_axis = BoardLib::Axis::Y;

	inline constexpr auto table = SabaneLib::Math::SinTable<12>{};
	inline auto cordic = SabaneLib::FastMathCordic{CORDIC};

	inline q15_t e_angle;
	inline auto atan_enc = SabaneLib::ContinuableEncoder{16,1000.f};
	inline auto target_filter = SabaneLib::Math::LowpassFilter<float>{0.05};

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

	//配置newするためのメモリープール
	namespace TmpMemoryPool{
		inline uint8_t can_tx_buff[sizeof(SabaneLib::RingBuffer<SabaneLib::CanFrame,5>)];
		inline uint8_t can_rx_buff[sizeof(SabaneLib::RingBuffer<SabaneLib::CanFrame,5>)];

		inline uint8_t led_pwm[sizeof(SabaneLib::PWMSoft)];
	}

	inline auto can = SabaneLib::FdCanComm{
		&hfdcan1,
		std::unique_ptr<SabaneLib::RingBuffer<SabaneLib::CanFrame,5>>(
				new(TmpMemoryPool::can_tx_buff) SabaneLib::RingBuffer<SabaneLib::CanFrame,5>{}),
		std::unique_ptr<SabaneLib::RingBuffer<SabaneLib::CanFrame,5>>(
				new(TmpMemoryPool::can_rx_buff) SabaneLib::RingBuffer<SabaneLib::CanFrame,5>{}),
		SabaneLib::FdCanRxFifo0
	};

	inline auto led = SabaneLib::ProgramablePWM{
		std::unique_ptr<SabaneLib::PWMSoft> (new(TmpMemoryPool::led_pwm) SabaneLib::PWMSoft{LED_GPIO_Port,LED_Pin,10})
	};

	//変数
	inline float vbus_voltage = 0.0f;

	inline SabaneLib::Math::UVW uvw_i = {.u=0.0f, .v=0.0f, .w=0.0f};
	inline SabaneLib::Math::AB ab_i = {.a = 0.0f, .b = 0.0f};
	inline SabaneLib::Math::DQ dq_i = {.d = 0.0f, .q = 0.0f};

	inline SabaneLib::Math::DQ target_i = {.d = 0.0f, .q =0.0f};

	inline float target_angle = 0.0f;

	inline int32_t atan_enc_bias = 0;

	namespace TestFunctions{
		void print_param(void);
		void move_test(void);
		void cordic_test(void);
	}
}


#endif /* BOARD_MAIN_HPP_ */
