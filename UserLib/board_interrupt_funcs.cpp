/*
 * board_interrupt_funcs.cpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#include "board_main.hpp"

namespace b = LMDBoard;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){

	b::can.rx_interrupt_task();
}
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes){
	b::can.tx_interrupt_task();
}

static float adc_to_current(uint16_t adc_val){
	constexpr float rl = (2.2f*1.5f)/(2.2f+1.5f); //基板上の並列接続された抵抗の合成抵抗
	constexpr float v_bias = rl/(rl+22.0f)*3.3f;  //バイアス電圧
	constexpr float amp_gain_inv = 1.0f/8.0f; //cube mxで設定するオペアンプのゲインの逆数
	constexpr float shant_r_inv = 1.0f/0.005f;         //シャント抵抗値の逆数
	float v = adc_val*3.3f/static_cast<float>(1<<12);

	return -((amp_gain_inv + 1.0f)*v_bias - amp_gain_inv*v)*shant_r_inv;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	static int adc_flag = 0;
	if(hadc == &hadc1){
		b::uvw_i.w = adc_to_current(ADC1->JDR1);
		b::qcos = -(static_cast<q15_t>(ADC1->JDR3)-static_cast<q15_t>(2154))*16;
		b::qsin =  (static_cast<q15_t>(ADC1->JDR2)-static_cast<q15_t>(2142))*16;
		b::vref_val = ADC1->JDR4;

		b::cordic.start_atan2(b::qcos,b::qsin);

		adc_flag ++;
	}else if(hadc == &hadc2){
	  b::uvw_i.u = adc_to_current(ADC2->JDR1);
	  b::uvw_i.v = adc_to_current(ADC2->JDR2);

	  adc_flag ++;
	}

	if(adc_flag >= 2){
		//position control loop
		constexpr float mm_to_q15rad = static_cast<float>(0xFFFF) / 30.0f;
		float target_angle = b::target_mm * mm_to_q15rad;

		b::target_i.d = 0.0f;
		b::target_i.q = b::PIDIns::position(target_angle, b::atan_enc.get_angle() - b::atan_enc_bias);

		//current control loop
		SabaneLib::MotorMath::AB ab_i = SabaneLib::MotorMath::uvw_to_ab(b::uvw_i);

		//TODO:delete while loop
		while(not b::cordic.is_avilable()){}

		b::atan_enc.update(b::cordic.read_ans());
		b::dq_i = SabaneLib::MotorMath::ab_to_dq(ab_i, b::table.sin_cos(static_cast<q15_t>(b::atan_enc.get_angle())));

		b::motor.move(
				{b::PIDIns::d_current(b::target_i.d,b::dq_i.d),
					b::PIDIns::q_current(b::target_i.q,b::dq_i.q)},
				b::table.sin_cos(static_cast<q15_t>(b::atan_enc.get_angle()))
		);

		adc_flag = 0;
	}
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim17){
//		constexpr float mm_to_q15rad = static_cast<float>(0xFFFF) / 30.0f;
//		float target_angle = b::target_mm * mm_to_q15rad;
//		b::motor.move(b::position_pid(target_angle,b::atan_enc.get_angle()-b::atan_enc_bias));
	}
}

