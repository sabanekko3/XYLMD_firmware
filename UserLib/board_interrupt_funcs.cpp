/*
 * board_interrupt_funcs.cpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#include "board_main.hpp"

namespace b = BoardElement;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	b::led.play(SabaneLib::LEDPattern::ok);
	b::can.rx_interrupt_task();
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes){
	b::led.play(SabaneLib::LEDPattern::ok);
	b::can.tx_interrupt_task();
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
	LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
	static int adc_flag = 0;
	if(hadc == &hadc1){
		q15_t qcos = -(static_cast<q15_t>(ADC1->JDR2)-static_cast<q15_t>(2154))*16;
		q15_t qsin =  (static_cast<q15_t>(ADC1->JDR1)-static_cast<q15_t>(2142))*16;

		b::cordic.start_atan2(qcos,qsin);

		adc_flag |= 0b01;
	}else if(hadc == &hadc2){
		b::uvw_i.u = BoardParam::adc_to_current(ADC2->JDR1);
		b::uvw_i.v = BoardParam::adc_to_current(ADC2->JDR2);
		b::ab_i = SabaneLib::MotorMath::uvw_to_ab(b::uvw_i);

		b::vbus_voltage = BoardParam::adc_to_voltage(ADC2->JDR3);
		adc_flag |= 0b10;
	}

	if(adc_flag == 0b11){
		//Cordicの読み込みとuvw->dq変換
		while(not b::cordic.is_avilable()){}
		b::e_angle = b::cordic.read_ans();
		b::dq_i = SabaneLib::MotorMath::ab_to_dq(b::ab_i, b::table.sin_cos(b::e_angle));

		//位置制御
		b::target_i.d = 0.0f;
		b::target_i.q = b::PIDIns::position(b::target_angle, b::atan_enc.update(b::e_angle) - b::atan_enc_bias);

		//電流制御
		b::motor.move(
				{b::PIDIns::d_current(b::target_i.d,b::dq_i.d),
					b::PIDIns::q_current(b::target_i.q,b::dq_i.q)},
				b::table.sin_cos(b::e_angle)
		);

		adc_flag = 0;
	}
	LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim17){
		//位置制御
//		LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
//		b::target_i.d = 0.0f;
//		b::target_i.q = b::PIDIns::position(b::target_angle, b::atan_enc.update(b::e_angle) - b::atan_enc_bias);
//		LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);

		//LED制御
		b::led.update();
	}
}

