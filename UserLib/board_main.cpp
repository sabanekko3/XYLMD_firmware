/*
 * board_main.cpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */


#include "board_main.hpp"

#include "main.h"

namespace b = BoardElement;
namespace blib = BoardLib;
namespace slib = SabaneLib;

extern "C" int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len,100);
	return len;
}
extern uint32_t SystemCoreClock;

extern "C" void main_(void){
//	b::TestFunctions::cordic_test();
	printf("clock:%d\r\n",blib::get_timer_freq(&htim1));

	//アナログ系初期化
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);

	//CAN初期化
	LL_GPIO_SetOutputPin(CAN_R_GPIO_Port,CAN_R_Pin);
	LL_GPIO_ResetOutputPin(CAN_SHDN_GPIO_Port,CAN_SHDN_Pin);
	b::can.start();
	b::can.set_filter_free(0);

	//タイマー系
	b::motor.start();
	HAL_TIM_Base_Start_IT(&htim17);

	HAL_ADC_Start(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc2);

	//制御用初期化
	HAL_Delay(1);//念のため
	b::PIDIns::position.set_limit(0.0f);
	b::atan_enc_bias = b::atan_enc.get_angle();

	b::led.play(blib::LEDPattern::setting);

	while(1){

		if(not HAL_GPIO_ReadPin(SW_GPIO_Port,SW_Pin)){
			b::atan_enc_bias = b::atan_enc.get_angle();
			b::PIDIns::position.set_limit(2.0f);
			b::led.play(blib::LEDPattern::setting);
			b::target_angle = 0.0f * blib::Coef::mm_to_q15rad;
			//move_test();
		}

		if(b::can.rx_available()){
			  slib::CanFrame rx_frame;
			  b::can.rx(rx_frame);
			  auto reader = rx_frame.reader();

			  switch(static_cast<blib::Command>(rx_frame.id)){
			  case blib::Command::SET_ORIGIN:
				  b::atan_enc_bias = b::atan_enc.get_angle();
				  break;
			  case blib::Command::TARGET_POS:
				  b::target_angle = blib::data_select(b::my_axis,reader) * blib::Coef::mm_to_q15rad;
				  break;
			  case blib::Command::POWER:
				  b::PIDIns::position.set_limit(blib::data_select(b::my_axis,reader));
				  break;
			  case blib::Command::GAIN_P:
				  b::PIDIns::position.set_p_gain(blib::data_select(b::my_axis,reader));
				  break;
			  case blib::Command::GAIN_I:
				  b::PIDIns::position.set_i_gain(blib::data_select(b::my_axis,reader));
				  break;
			  case blib::Command::GAIN_D:
				  b::PIDIns::position.set_d_gain(blib::data_select(b::my_axis,reader));
				  break;
			  default:
				  break;
			  }
		}

//		b::TestFunctions::print_param();
	}
}

void b::TestFunctions::print_param(void){
	printf("%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\r\n",
			b::target_i.d,
			b::target_i.q,
			b::dq_i.d,
			b::dq_i.q,
			b::atan_enc.get_speed() * blib::Coef::q15rad_to_mm,
			b::vbus_voltage
	);
	HAL_Delay(1);
}

void b::TestFunctions::move_test(){
	while(1){
		b::target_angle = 0.0f * blib::Coef::mm_to_q15rad;
		HAL_Delay(500);
		b::target_angle = 50.0f * blib::Coef::mm_to_q15rad;
		HAL_Delay(500);
		b::target_angle = 100.0f * blib::Coef::mm_to_q15rad;
		HAL_Delay(500);
	}
}

void b::TestFunctions::cordic_test(void){
	auto ch = slib::CordicHandler<q15_t>{CORDIC};
	q31_t angle = 0;
	while(1){
		ch.set_mode(slib::CordicMode::SIN_COS,true,true,4);
		ch.set_param(angle,0x7FFF);
		auto [s,c] = ch.read_ans_pair();

		ch.set_mode(slib::CordicMode::PHASE_MODULUS,true,true,4);
		ch.set_param(c,s);
		while(not ch.is_available());
		auto [r,m] = ch.read_ans_pair();

//		printf("%4.3f,%4.3f,%4.3f,%4.3f\r\n",
//				slib::Math::q15_to_rad(angle),
//				slib::Math::q15_to_float(s),
//				slib::Math::q15_to_float(c),
//				slib::Math::q15_to_rad(r));

		printf("%4.3f,%4.3f\r\n",
				b::table.sin(angle),
				b::table.cos(angle));

//		printf("%4x,%4x,%8x,%4.3f,%4.3f,%4.3f,%4.3f,%d,%d\r\n",
//				c,s,static_cast<int16_t>(c) | (static_cast<int32_t>(s)<<16),
//				slib::Math::q15_to_rad(angle),
//				slib::Math::q15_to_float(s),
//				slib::Math::q15_to_float(c),
//				slib::Math::q15_to_rad(r),
//				t1,
//				t2);
		angle += 0xFFFFFF;
		HAL_Delay(10);
	}
}

