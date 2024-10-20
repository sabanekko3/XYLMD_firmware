/*
 * encoder.hpp
 *
 *  Created on: Jun 24, 2024
 *      Author: gomas
 */

#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include "main.h"

#include <numeric>
#include <cmath>
#include <iterator>
#include <functional>

namespace SabaneLib{

	class IEncoder{
	public:
		virtual int32_t get_angle(void)const = 0;
		virtual int32_t get_speed(void)const = 0;
	};

	//エンコーダーの連続化クラス
	class ContinuableEncoder : public IEncoder{
	private:
		const size_t resolution_bit;
		const size_t resolution;

		uint32_t angle = 0;
		uint32_t speed = 0;
		int32_t turn_count = 0;

		const int32_t k_speed;
		int32_t angle_buff[4];
		uint32_t head = 0;
		int32_t angle_sum_old = 0;

	public:
		ContinuableEncoder(size_t _resolution_bit,float freq):
			resolution_bit(_resolution_bit),
			resolution(1<<resolution_bit),
			k_speed(freq/(sizeof(angle_buff)/sizeof(int32_t))){
		}

		int32_t get_angle(void)const override{return angle;}
		int32_t get_speed(void)const override{return speed;}

		virtual int32_t update(uint32_t _angle){
			int32_t new_angle = _angle&(resolution-1);

			int32_t angle_top_2 = (new_angle>>(resolution_bit-2))&0b11;
			int32_t old_angle_top_2 = (angle>>(resolution_bit-2))&0b11;

			if(old_angle_top_2 == 3 && angle_top_2 == 0){
				turn_count ++;
			}else if(old_angle_top_2 == 0 && angle_top_2 == 3){
				turn_count --;
			}

			angle = new_angle + resolution*turn_count;

			angle_buff[head] = angle;
			head = (head+1)&(sizeof(angle_buff)/sizeof(float) - 1);
			int32_t angle_sum = std::reduce(std::begin(angle_buff), std::end(angle_buff));
			speed = (angle_sum - angle_sum_old)*k_speed;
			angle_sum_old = angle_sum;

			return angle;
		}

		void set_turn_count(int32_t _turn_count){turn_count = _turn_count;}
		int32_t get_turn_count(void)const{return turn_count;}

		virtual ~ContinuableEncoder(){}
	};


}

#endif /* ENCODER_HPP_ */
