/*
 * sin_table.hpp
 *
 *  Created on: Jun 26, 2024
 *      Author: gomas
 */

#ifndef SIN_TABLE_HPP_
#define SIN_TABLE_HPP_

#include "q_format.hpp"

#include <cmath>
#include <cassert>

namespace SabaneLib::Math{
	enum class TableMode: size_t{
		NORMAL = 1,
		COMPACT = 4
	};

	template<size_t PeriodN,TableMode M = TableMode::NORMAL>
	class SinTable{
	private:
		static constexpr size_t period = 1u << PeriodN;

		float table[period/static_cast<size_t>(M)];

	public:
		constexpr SinTable(void){
			static_assert((1 <= PeriodN) && (PeriodN <= 16));

			if constexpr(M == TableMode::NORMAL){
				for(size_t i = 0; i < period; i++) {
					table[i] = std::sin(static_cast<float>(i)/static_cast<float>(period)*2*M_PI);
				}
			}else if(M==TableMode::COMPACT){
				for(size_t i = 0; i < period/static_cast<size_t>(M); i++) {
					table[i] = std::sin(static_cast<float>(i)/static_cast<float>(period)*2*M_PI);
				}
			}
		}


		//q15_t functions
		float sin(q15_t rad) const {
			if constexpr(M == TableMode::NORMAL){
				size_t index = (rad >> (16-PeriodN)) & (period-1);
				return table[index];

			}else if(M == TableMode::COMPACT){
				size_t index = (rad >> (16-PeriodN)) & (period-1);
				if(index >= period*3/4){
					return -table[period - index - 1];
				}else if(index >= period/2){
					return -table[index - period/2];
				}else if(index >= period/4){
					return table[period/2 - index - 1];
				}else{
					return table[index];
				}
			}
		}
		float cos(q15_t rad) const {
			return this->sin(static_cast<q15_t>(rad + Q15Def::pi_2));
		}
		SinCos sin_cos(q15_t rad) const {
			return SinCos{.sin = this->sin(rad),.cos = this->cos(rad)};
		}
		UVW uvw_phase(q15_t rad) const {
			return {this->cos(rad),
					this->cos(rad - 2*Q15Def::pi_3),
					this->cos(rad + 2*Q15Def::pi_3)};
		}


		//q31_t functions
		float sin(q31_t rad) const {
			if constexpr(M == TableMode::NORMAL){
				size_t index = (rad >> (32-PeriodN)) & (period-1);
				return table[index];

			}else if(M == TableMode::COMPACT){
				size_t index = (rad >> (32-PeriodN)) & (period-1);
				if(index >= period*3/4){
					return -table[period - index - 1];
				}else if(index >= period/2){
					return -table[index - period/2];
				}else if(index >= period/4){
					return table[period/2 - index - 1];
				}else{
					return table[index];
				}
			}
		}
		float cos(q31_t rad) const {
			return this->sin(static_cast<q31_t>(rad + Q31Def::pi_2));
		}
		SinCos sin_cos(q31_t rad) const {
			return SinCos{.sin = this->sin(rad),.cos = this->cos(rad)};
		}
		UVW uvw_phase(q31_t rad) const {
			return {this->cos(rad),
					this->cos(rad - 2*Q31Def::pi_3),
					this->cos(rad + 2*Q31Def::pi_3)};
		}


		//float
		float sin(float rad) const {
			return this->sin(rad_to_q15(rad));
		}
		float cos(float rad) const  {
			return this->cos(rad_to_q15(rad));
		}
		SinCos sin_cos(float rad) const {
			return this->sin_cos(rad_to_q15(rad));
		}

		UVW uvw_phase(float rad) const {
			return {this->cos(rad),
					this->cos(rad - static_cast<float>(2*M_PI)/3.0f),
					this->cos(rad + static_cast<float>(2*M_PI)/3.0f)};
		}
	};
}



#endif /* SIN_TABLE_HPP_ */
