/*
 * timer_help.hpp
 *
 *  Created on: Nov 14, 2024
 *      Author: gomas
 */

#ifndef TIMER_HELP_HPP_
#define TIMER_HELP_HPP_

#include "main.h"

namespace BoardLib{

#ifdef HAL_RCC_MODULE_ENABLED
	inline uint32_t get_timer_freq(TIM_HandleTypeDef* tim){
		uint32_t tim_clock = 0;

		//タイマーのクロックソースを特定
		//STM32F3,G4,H7を参考に決定
		switch(reinterpret_cast<uint32_t>(tim->Instance)){
#ifdef TIM1
		case reinterpret_cast<uint32_t>(TIM1_BASE):
			tim_clock = HAL_RCC_GetPCLK2Freq();
			break;
#endif //TIM1
#ifdef TIM2
		case reinterpret_cast<uint32_t>(TIM2_BASE):
			tim_clock = HAL_RCC_GetPCLK1Freq();
			break;
#endif //TIm2

#ifdef TIM3
		case reinterpret_cast<uint32_t>(TIM3_BASE):
			tim_clock = HAL_RCC_GetPCLK1Freq();
			break;
#endif //TIm3

#ifdef TIM4
		case reinterpret_cast<uint32_t>(TIM4_BASE):
			tim_clock = HAL_RCC_GetPCLK1Freq();
			break;
#endif //TIm4

#ifdef TIM5
		case TIM5:
			bus = ClockBus::APB1;
			tim_clock = HAL_RCC_GetPCLK1Freq();
			break;
#endif //TIm5

#ifdef TIM6
		case reinterpret_cast<uint32_t>(TIM6_BASE):
			tim_clock = HAL_RCC_GetPCLK1Freq();
			break;
#endif //TIm6

#ifdef TIM7
		case reinterpret_cast<uint32_t>(TIM7_BASE):
			tim_clock = HAL_RCC_GetPCLK1Freq();
			break;
#endif //TIm7

#ifdef TIM8
		case reinterpret_cast<uint32_t>(TIM8_BASE):
			tim_clock = HAL_RCC_GetPCLK2Freq();
			break;
#endif //TIm8

#ifdef TIM12
		case reinterpret_cast<uint32_t>(TIM12_BASE):
			tim_clock = HAL_RCC_GetPCLK1Freq();
			break;
#endif //TIm12

#ifdef TIM13
		case reinterpret_cast<uint32_t>(TIM13_BASE):
			tim_clock = HAL_RCC_GetPCLK1Freq();
			break;
#endif //TIM13

#ifdef TIM14
		case reinterpret_cast<uint32_t>(TIM14_BASE):
			tim_clock = HAL_RCC_GetPCLK1Freq();
			break;
#endif //TIm14

#ifdef TIM15
		case reinterpret_cast<uint32_t>(TIM15_BASE):
			tim_clock = HAL_RCC_GetPCLK2Freq();
			break;
#endif //TIm15

#ifdef TIM16
		case reinterpret_cast<uint32_t>(TIM16_BASE):
			tim_clock = HAL_RCC_GetPCLK2Freq();
			break;
#endif //TIM16

#ifdef TIM17
		case reinterpret_cast<uint32_t>(TIM17_BASE):
			tim_clock = HAL_RCC_GetPCLK2Freq();
			break;
#endif //TIM17
		default:
			Error_Handler();
		}

		if(HAL_RCC_GetHCLKFreq() == tim_clock){
			//nop
		}else{
			tim_clock *=2;
		}

		return tim_clock/__HAL_TIM_GET_AUTORELOAD(tim);
	}
#endif //HAL_RCC_MODULE_ENABLED

}



#endif /* TIMER_HELP_HPP_ */
