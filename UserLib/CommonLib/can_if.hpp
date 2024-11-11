/*
 * can.hpp
 *
 *  Created on: Sep 26, 2024
 *      Author: gomas
 */

#ifndef CAN_HPP_
#define CAN_HPP_

#include "main.h"
#include "ring_buffer.hpp"
#include "byte_reader_writer.hpp"



namespace SabaneLib{
	struct CanFrame{
		uint8_t data[8]={0};
		size_t data_length=0;
		uint32_t id=0;
		bool is_ext_id=false;
		bool is_remote=false;

		ByteWriter writer(void){
			return ByteWriter(data,sizeof(data), data_length);
		}
		ByteReader reader(void)const{
			return ByteReader(data,sizeof(data));
		}
	};

	enum class CanFilterMode{
		ONLY_STD,
		ONLY_EXT,
		STD_AND_EXT,
	};

	class ICan{
	public:
		uint32_t virtual tx_available(void)const = 0;
		bool virtual tx(const CanFrame &tx_frame) = 0;

		uint32_t virtual rx_available(void)const = 0;
		bool virtual rx(CanFrame &rx_frame) = 0;

		virtual ~ICan(){}
	};
}



#endif /* CAN_HPP_ */
