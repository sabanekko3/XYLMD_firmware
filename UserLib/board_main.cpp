/*
 * board_main.cpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */


#include "board_main.hpp"

#include "main.h"

namespace b = LMDBoard;


static float data_select(LSMParam::Axis xy,SabaneLib::ByteReader &r){
	auto data_x = r.read<float>();
	auto data_y = r.read<float>();

	switch(xy){
	case LSMParam::Axis::X:
		return data_x.has_value() ? data_x.value() : 0.0f;
	case LSMParam::Axis::Y:
		return data_y.has_value() ? data_y.value() : 0.0f;
	}
	return 0.0f;
}

extern "C" int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len,100);
	return len;
}

static void print_param(void){
	constexpr float q15rad_to_mm = 30.0f/static_cast<float>(0xFFFF);
	printf("%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\r\n",
			b::target_i.d,
			b::target_i.q,
			b::dq_i.d,
			b::dq_i.q,
			b::atan_enc.get_speed()*q15rad_to_mm
	);
	HAL_Delay(1);
}

static void move_test(void){
	while(1){
		b::target_mm = 0.0f;
		HAL_Delay(500);
		b::target_mm = 50.0f;
		HAL_Delay(500);
		b::target_mm = 100.0f;
		HAL_Delay(500);
	}
}

extern "C" void main_(void){
	//アナログ系初期化
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);

	//テーブル初期化
	LMDBoard::table.generate([](float rad)->float{
	  b::cordic.start_sincos(rad);
	  while(not b::cordic.is_avilable());
	  return b::cordic.get_sincos().sin;
	});

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
	b::atan_enc_bias = b::atan_enc.get_angle();

	b::led.play(SabaneLib::LEDPattern::ok);

	while(1){

		if(not HAL_GPIO_ReadPin(SW_GPIO_Port,SW_Pin)){
			b::atan_enc_bias = b::atan_enc.get_angle();
			b::PIDIns::position.set_limit(4.0f);
			b::led.play(SabaneLib::LEDPattern::setting);
			//move_test();
		}

		if(b::can.rx_available()){
			  SabaneLib::CanFrame rx_frame;
			  b::can.rx(rx_frame);
			  auto reader = rx_frame.reader();

			  switch(static_cast<LSMParam::Command>(rx_frame.id)){
			  case LSMParam::Command::SET_ORIGIN:
				  b::atan_enc_bias = b::atan_enc.get_angle();
				  break;
			  case LSMParam::Command::TARGET_POS:
				  b::target_mm = data_select(b::my_axis,reader);
				  break;
			  case LSMParam::Command::POWER:
				  b::PIDIns::position.set_limit(data_select(b::my_axis,reader));
				  break;
			  case LSMParam::Command::GAIN_P:
				  b::PIDIns::position.set_p_gain(data_select(b::my_axis,reader));
				  break;
			  case LSMParam::Command::GAIN_I:
				  b::PIDIns::position.set_i_gain(data_select(b::my_axis,reader));
				  break;
			  case LSMParam::Command::GAIN_D:
				  b::PIDIns::position.set_d_gain(data_select(b::my_axis,reader));
				  break;
			  default:
				  break;
			  }
		}

		print_param();

	}
}
