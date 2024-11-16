/*
 * ring_buffer.hpp
 *
 *  Created on: Sep 26, 2024
 *      Author: gomas
 */

#ifndef RING_BUFFER_HPP_
#define RING_BUFFER_HPP_

#include <stdbool.h>

namespace SabaneLib{

	template<typename T>
	class IRingBuffer{
	public:
		bool virtual push(const T& input) = 0;
		bool virtual pop(T& output) = 0;
		size_t virtual get_free_level(void)const = 0;
		size_t virtual get_busy_level(void)const = 0;
		void virtual reset(void) = 0;

		virtual ~IRingBuffer(){}
	};

	template<typename T, size_t n>
	class RingBuffer : public IRingBuffer<T>{
	private:
		static constexpr size_t size = 1<<n;
		static constexpr size_t mask = size-1;
		size_t head = 0;
		size_t tail = 0;
		size_t data_count = 0;

		T data_buff[size] = {0};
	public:
		bool push(const T &input)override{
			data_buff[head] = input;
			head = (head+1) & mask;
			data_count ++;
			if(data_count > size){
				data_count = size;
				tail = head;
				return false;
			};
			return true;
		}

		bool pop(T &output)override{
			if(data_count > 0){
				output = data_buff[tail];
				tail = (tail + 1) & mask;
				data_count --;
				if(data_count < 0) data_count = 0;
				return true;
			}else{
				return false;
			}
		}

		size_t get_free_level(void)const override{
			return size - data_count;
		}
		size_t get_busy_level(void)const override{
			return data_count;
		}
		void reset(void)override{
			head = 0;
			tail = 0;
			data_count = 0;
		}

	};

}



#endif /* RING_BUFFER_HPP_ */
