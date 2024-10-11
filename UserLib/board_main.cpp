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
	b::position_pid.set_limit(0.0f);

	HAL_Delay(10);
	while(HAL_GPIO_ReadPin(SW_GPIO_Port,SW_Pin));
	printf("start\r\n");

	b::atan_enc_bias = b::atan_enc.get_angle();
	b::position_pid.set_limit(0.3f);
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
				  b::position_pid.set_limit(data_select(b::my_axis,reader));
				  break;
			  case LSMParam::Config::GAIN_P:
				  b::position_pid.set_p_gain(data_select(b::my_axis,reader));
				  break;
			  case LSMParam::Config::GAIN_I:
				  b::position_pid.set_i_gain(data_select(b::my_axis,reader));
				  break;
			  case LSMParam::Config::GAIN_D:
				  b::position_pid.set_d_gain(data_select(b::my_axis,reader));
				  break;
			  default:
				  break;
			  }

		  }else{
		  }
	}
}
