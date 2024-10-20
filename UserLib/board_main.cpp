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
	printf("%4.3f,%4.3f,%4.3f,%4.3f,%d\r\n",
			b::target_i.d,
			b::target_i.q,
			b::dq_i.d,
			b::dq_i.q,
			__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2)
	);
	HAL_Delay(1);
}

extern "C" void main_(void){
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);

	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

	LMDBoard::table.generate([](float rad)->float{
	  b::cordic.start_sincos(rad);
	  while(not b::cordic.is_avilable());
	  return b::cordic.get_sincos().sin;
	});

	HAL_GPIO_WritePin(CAN_R_GPIO_Port,CAN_R_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CAN_SHDN_GPIO_Port,CAN_SHDN_Pin,GPIO_PIN_RESET);
	b::can.start();
	b::can.set_filter_free(0);

	b::motor.start();

	HAL_TIM_Base_Start_IT(&htim17);

	HAL_ADC_Start(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc2);

	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

	b::target_mm = 0.0f;
	b::PIDIns::position.set_limit(0.0f);

	HAL_Delay(10);
	while(HAL_GPIO_ReadPin(SW_GPIO_Port,SW_Pin));

	b::atan_enc_bias = b::atan_enc.get_angle();
	b::PIDIns::position.set_limit(4.0f);
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

	while(1){
		if(b::can.rx_available()){
			  SabaneLib::CanFrame rx_frame;
			  b::can.rx(rx_frame);
			  auto reader = rx_frame.reader();

			  switch(static_cast<LSMParam::Config>(rx_frame.id)){
			  case LSMParam::Config::POS:
				  b::target_mm = data_select(b::my_axis,reader);
				  break;
			  case LSMParam::Config::POWER:
				  b::PIDIns::position.set_limit(data_select(b::my_axis,reader));
				  break;
			  case LSMParam::Config::GAIN_P:
				  b::PIDIns::position.set_p_gain(data_select(b::my_axis,reader));
				  break;
			  case LSMParam::Config::GAIN_I:
				  b::PIDIns::position.set_i_gain(data_select(b::my_axis,reader));
				  break;
			  case LSMParam::Config::GAIN_D:
				  b::PIDIns::position.set_d_gain(data_select(b::my_axis,reader));
				  break;
			  default:
				  break;
			  }
		}

		//print_param();

	}
}
