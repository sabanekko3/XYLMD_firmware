/*
 * sincos_enc.hpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#ifndef COMMONLIB_SINCOS_ENC_HPP_
#define COMMONLIB_SINCOS_ENC_HPP_

#include "math.hpp"
#include "encoder.hpp"

namespace SabaneLib{
	class SinCosEncoder:public ContinuableEncoder{
	private:
		std::function<q15_t(q15_t,q15_t)> qatan;

	public:
		SinCosEncoder(float freq,std::function<q15_t(q15_t,q15_t)> _qatan)
			:ContinuableEncoder(16,freq),
			 qatan(_qatan){
		}

		int32_t update(q15_t cos,q15_t sin){
			return ContinuableEncoder::update(qatan(cos,sin));
		}
	};
}

#endif /* COMMONLIB_SINCOS_ENC_HPP_ */
