/*
 * board_interrupt_funcs.cpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#include "board_main.hpp"

namespace b = LMDBoard;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	b::can.rx_interrupt_task();
}
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes){
	b::can.tx_interrupt_task();
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
      if(hadc == &hadc1){
    	  b::adc_val[0]=ADC1->JDR1;
    	  b::qcos = -(static_cast<q15_t>(ADC1->JDR3)-static_cast<q15_t>(2154))*16;
    	  b::qsin =  (static_cast<q15_t>(ADC1->JDR2)-static_cast<q15_t>(2142))*16;
    	  b::vref_val = ADC1->JDR4;

      }else if(hadc == &hadc2){
    	  b::adc_val[1]=ADC2->JDR1;
    	  b::adc_val[2]=ADC2->JDR2;
      }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim17){
		b::cordic.start_atan2(b::qcos,b::qsin);
		while(not b::cordic.is_avilable()){}
		b::atan_enc.update(b::cordic.read_ans());

		constexpr float mm_to_q15rad = static_cast<float>(0xFFFF) / 30.0f;
		float target_angle = b::target_mm * mm_to_q15rad;
		b::motor.move(b::position_pid(target_angle,b::atan_enc.get_angle()-b::atan_enc_bias));
	}
}

