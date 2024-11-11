/*
 * board_interrupt_funcs.cpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#include "board_main.hpp"

namespace b = BoardElement;
namespace blib = BoardLib;
namespace slib = SabaneLib;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	b::led.play(blib::LEDPattern::ok);
	b::can.rx_interrupt_task();
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes){
	b::led.play(blib::LEDPattern::ok);
	b::can.tx_interrupt_task();
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){

	static int adc_flag = 0;
	if(hadc == &hadc1){
//		LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
		q15_t qcos = -(static_cast<q15_t>(ADC1->JDR2)-static_cast<q15_t>(2154))*16;
		q15_t qsin =  (static_cast<q15_t>(ADC1->JDR1)-static_cast<q15_t>(2142))*16;

		b::cordic.start_atan2(qcos,qsin);

		adc_flag |= 0b01;
//		LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
	}else if(hadc == &hadc2){
//		LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
		b::uvw_i.u = blib::adc_to_current(ADC2->JDR1);
		b::uvw_i.v = blib::adc_to_current(ADC2->JDR2);
		b::ab_i = b::uvw_i.to_ab();

		b::vbus_voltage = blib::adc_to_voltage(ADC2->JDR3, blib::Coef::vbus_r_gain);
		adc_flag |= 0b10;
//		LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
	}

	if(adc_flag == 0b11){
//		LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
		//Cordicの読み込みとuvw->dq変換
		while(not b::cordic.handler.is_available()){}
		b::e_angle = b::cordic.handler.read_ans();

		//電流制御
		b::dq_i = b::ab_i.to_dq(b::table.sin_cos(b::e_angle));
		auto dq_v = slib::Math::DQ{
				.d = b::PIDIns::d_current(b::target_i.d,b::dq_i.d),
				.q = b::PIDIns::q_current(b::target_i.q,b::dq_i.q)};

		constexpr float coef = -1.0f/6.0f;
		b::motor.move(dq_v.to_uvw(b::table.sin_cos(b::e_angle)).sperimposition(b::table.sin(static_cast<q15_t>(b::e_angle*3))*coef));

		b::led.pwm->update();
		adc_flag = 0;
//		LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim17){
		//位置制御
//		LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
		b::target_i.d = 0.0f;
		b::target_i.q = b::PIDIns::position(b::target_filter(b::target_angle), b::atan_enc.update(b::e_angle) - b::atan_enc_bias);
//		LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);

		//LED制御
		b::led.update();
	}
}

