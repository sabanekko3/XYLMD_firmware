/*
 * filter.hpp
 *
 *  Created on: Oct 24, 2024
 *      Author: gomas
 */

#ifndef COMMONLIB_FILTER_HPP_
#define COMMONLIB_FILTER_HPP_


namespace SabaneLib{
	class LowpassFilter{
	private:
		float data = 0;
		const float k = 0;
	public:
		//set gain 0~100
		LowpassFilter(float _k):k(_k){}
		float operator() (float input){
			data = input*k + (1.0f-k)*data;
			return data;
		}
		float get(void)const{
			return data;
		}
		void reset(void){
			data = 0;
		}
	};


	class HighpassFilter{
	private:
		float data = 0;
		const float k = 0;
	public:
		HighpassFilter(float _k):k(_k){}

		//inline functions
		float operator() (float input){
			data = input*k + (1.0f-k)*data;
			return input - data;
		}
		float get(void)const{
			return data;
		}
		void reset(void){
			data = 0;
		}
	};
}


#endif /* COMMONLIB_FILTER_HPP_ */
