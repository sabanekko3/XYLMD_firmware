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
    	  b::enc_val[0]=ADC1->JDR2;
    	  b::enc_val[1]=ADC1->JDR3;
    	  b::vref_val = ADC1->JDR4;

      }else if(hadc == &hadc2){
    	  b::adc_val[1]=ADC2->JDR1;
    	  b::adc_val[2]=ADC2->JDR2;
    	  //enc_val[1]=ADC1->JDR3;
      }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim17){
		b::qsin =  (static_cast<q15_t>(b::enc_val[0])-static_cast<q15_t>(2142));//-static_cast<q15_t>((1.7/3.2)*(0xFFF)));
		b::qcos = -(static_cast<q15_t>(b::enc_val[1])-static_cast<q15_t>(2154));//-static_cast<q15_t>((1.7/3.2)*(0xFFF)));

		b::cordic.start_atan2(static_cast<q15_t>(b::qcos*16),static_cast<q15_t>(b::qsin*16));
		while(not b::cordic.is_avilable());
		b::atan_enc.update(b::cordic.read_ans());

		constexpr float mm_to_q15rad = static_cast<float>(0xFFFF) / 30.0f;
		float target_angle = b::target_mm * mm_to_q15rad;
		b::motor.move(b::position_pid(target_angle,b::atan_enc.get_angle()-b::atan_enc_bias));
	}
}



float adc_to_current(uint16_t adc_val){
	constexpr float rl = (2.2*1.5)/(2.2+1.5); //基板上�????��?��??��?��???��?��??��?��?圧抵抗�????��?��??��?��???��?��??��?��?
	constexpr float v_bias = rl/(rl+22)*3.3;  //バイアス電圧
	constexpr float amp_gain_inv = 1.0f/7.0f; //cube mxで設定するオペアンプ�????��?��??��?��ゲインの???��?��??��?��?数
	constexpr float shant_r_inv = 1.0f/0.005f;         //シャント抵抗�????��?��??��?��値
	float v = adc_val*3.3/static_cast<float>(1<<12);

	return ((amp_gain_inv + 1.0f)*v_bias - amp_gain_inv*v)*shant_r_inv;
}

