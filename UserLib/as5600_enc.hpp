/*
 * as5600_enc.hpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#ifndef COMMONLIB_AS5600_ENC_HPP_
#define COMMONLIB_AS5600_ENC_HPP_

#include "main.h"

#ifdef HAL_I2C_MODULE_ENABLED

#include "encoder.hpp"

namespace LMDLib{
	//AS5600による制御
	class AS5600State:public ContinuableEncoder{
	private:
		static constexpr uint16_t as5600_id = 0x36;
		static constexpr size_t as5600_resolution = 12;

		I2C_HandleTypeDef* const i2c;

		uint8_t enc_val[2] = {0};

		const int32_t inv = 1;

	public:
		AS5600State(I2C_HandleTypeDef* _i2c,float _freq,bool is_inv = false)
			:i2c(_i2c),
			 ContinuableEncoder(as5600_resolution,_freq),
			 inv(is_inv?-1:1){
		}

		void read_start(void){
			HAL_I2C_Mem_Read_IT(i2c, as5600_id<<1, 0x0c, I2C_MEMADD_SIZE_8BIT, enc_val, 2);
		}

		void i2c_rx_interrupt_task(void){
			uint16_t raw_angle = enc_val[0]<<8 | enc_val[1];
			update(raw_angle);
		}
	};
}
#endif

#endif /* COMMONLIB_AS5600_ENC_HPP_ */
