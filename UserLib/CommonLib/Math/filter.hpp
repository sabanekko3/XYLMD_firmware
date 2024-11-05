/*
 * filter.hpp
 *
 *  Created on: Oct 24, 2024
 *      Author: gomas
 */

#ifndef COMMONLIB_FILTER_HPP_
#define COMMONLIB_FILTER_HPP_

#include <concepts>
#include <type_traits>

namespace SabaneLib::Math{

	template <class T>
	concept Arithmetic = std::is_arithmetic_v<T>;

	//指数移動平均による実装
	template<Arithmetic T>
	class LowpassFilter{
	private:
		T data = 0;
		const float k = 0;
	public:
		LowpassFilter(float _k):k(_k){}

		T operator() (T input){
			data = input*k + (1.0f-k)*data;
			return data;
		}
		T get(void)const{
			return data;
		}
		void reset(void){
			data = 0;
		}
	};

	//指数移動平均による実装
	template<Arithmetic T>
	class HighpassFilter{
	private:
		T data = 0;
		const float k = 0;
	public:
		HighpassFilter(float _k):k(_k){}

		T operator() (T input){
			data = input*k + (1.0f-k)*data;
			return input - data;
		}
		T get(void)const{
			return data;
		}
		void reset(void){
			data = 0;
		}
	};


	//単純移動平均
	template<Arithmetic T,size_t n>
	class MovingAverage{
	private:
		static constexpr size_t buff_size = 1<<n;
		static constexpr size_t mask = buff_size - 1;

		T buff[buff_size] = {0};
		T sum = 0;
		size_t head = 0;

	public:
		void push(T val){
			sum -= buff[head];
			buff[head] = val;
			sum += val;
			head = (head + 1) & mask;
		}

		T get_average(void)const{
			return sum / static_cast<T>(buff_size);
		}

		void reset(void){
			for(auto &b : buff){
				b = 0;
			}
		}
	};
}


#endif /* COMMONLIB_FILTER_HPP_ */
