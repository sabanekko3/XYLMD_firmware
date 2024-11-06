/*
 * can_control.hpp
 *
 *  Created on: Sep 27, 2024
 *      Author: gomas
 */

#ifndef CAN_CONTROL_HPP_
#define CAN_CONTROL_HPP_

#include "can_if.hpp"

#include <memory>

namespace SabaneLib{
#ifdef HAL_CAN_MODULE_ENABLED
	class CanComm:public ICan{
	private:
		CAN_HandleTypeDef* const can;
		const uint32_t rx_fifo;
		const uint32_t rx_filter_fifo;
		const uint32_t rx_fifo_it;

		std::unique_ptr<IRingBuffer<CanFrame>> rx_buff;
		std::unique_ptr<IRingBuffer<CanFrame>> tx_buff;
	public:
		CanComm(CAN_HandleTypeDef *_can,std::unique_ptr<IRingBuffer<CanFrame>> _rx_buff,std::unique_ptr<IRingBuffer<CanFrame>> &&_tx_buff,uint32_t _rx_fifo,uint32_t _rx_filter_fifo,uint32_t _rx_fifo_it)
		:can(_can),
		 rx_buff(std::move(_rx_buff)),
		 tx_buff(std::move(_tx_buff)),
		 rx_fifo(_rx_fifo),
		 rx_filter_fifo(_rx_filter_fifo),
		 rx_fifo_it(_rx_fifo_it){
		}

		void start(void){
			HAL_CAN_Start(can);
			HAL_CAN_ActivateNotification(can, rx_fifo_it);
			HAL_CAN_ActivateNotification(can, CAN_TX_MAILBOX0);
			HAL_CAN_ActivateNotification(can, CAN_TX_MAILBOX1);
			HAL_CAN_ActivateNotification(can, CAN_TX_MAILBOX2);
		}

		CAN_HandleTypeDef *get_can_handle(void)const{
			return can;
		}

		////////////////////
		//can tx functions//
		////////////////////
		uint32_t tx_available(void)const override {
			return tx_buff->get_free_level();
		}

		void tx_interrupt_task(void){
			while(HAL_CAN_GetTxMailboxesFreeLevel(can) && tx_buff->get_busy_level()){
				CanFrame tx_frame;

				if(!tx_buff->pop(tx_frame)){
					break;
				}

				uint32_t mailbox_num;
				CAN_TxHeaderTypeDef tx_header;

				if(tx_frame.is_ext_id){
					tx_header.ExtId = tx_frame.id;
					tx_header.IDE = CAN_ID_EXT;
				}else{
					tx_header.StdId = tx_frame.id;
					tx_header.IDE = CAN_ID_STD;
				}

				if(tx_frame.is_remote){
					tx_header.RTR = CAN_RTR_REMOTE;
				}else{
					tx_header.RTR = CAN_RTR_DATA;
				}

				tx_header.DLC = tx_frame.data_length;
				tx_header.TransmitGlobalTime = DISABLE;

				HAL_CAN_AddTxMessage(can, &tx_header, tx_frame.data, &mailbox_num);
			}
		}

		bool tx(const CanFrame &tx_frame)override{
			if(HAL_CAN_GetTxMailboxesFreeLevel(can)){
				uint32_t mailbox_num;
				CAN_TxHeaderTypeDef tx_header;

				if(tx_frame.is_ext_id){
					tx_header.ExtId = tx_frame.id;
					tx_header.IDE = CAN_ID_EXT;
				}else{
					tx_header.StdId = tx_frame.id;
					tx_header.IDE = CAN_ID_STD;
				}

				if(tx_frame.is_remote){
					tx_header.RTR = CAN_RTR_REMOTE;
				}else{
					tx_header.RTR = CAN_RTR_DATA;
				}

				tx_header.DLC = tx_frame.data_length;
				tx_header.TransmitGlobalTime = DISABLE;

				HAL_CAN_AddTxMessage(can, &tx_header, const_cast<uint8_t*>(tx_frame.data), &mailbox_num);
			}else{
				if(!tx_buff->push(tx_frame)){
					return false;
				}
			}

			return true;
		}

		////////////////////
		//can rx functions//
		////////////////////
		uint32_t rx_available(void)const override{ return rx_buff->get_busy_level(); }
		void rx_interrupt_task(void){
			CAN_RxHeaderTypeDef rx_header;
			CanFrame rx_frame;

			HAL_CAN_GetRxMessage(can, rx_fifo, &rx_header, rx_frame.data);
			rx_frame.is_remote = (rx_header.RTR == CAN_RTR_DATA)? false : true;
			rx_frame.id = (rx_header.IDE == CAN_ID_STD)? rx_header.StdId : rx_header.ExtId;
			rx_frame.is_ext_id = (rx_header.IDE == CAN_ID_STD)? false : true;
			rx_frame.data_length = rx_header.DLC;

			rx_buff->push(rx_frame);
		}
		bool rx(CanFrame &rx_frame)override{
			if(rx_buff->pop(rx_frame)){
				return true;
			}else{
				return false;
			}
		}

		//////////////////////
		//can filter setting//
		//////////////////////
		void set_filter_mask(uint32_t filter_no,uint32_t id,uint32_t mask,CanFilterMode mode,bool as_std = true){
			CAN_FilterTypeDef filter;
			uint32_t filter_id;
			uint32_t filter_mask;
			switch(mode){
			case FilterMode::ONLY_STD:
				if(!as_std){
					filter_id = id << 21;
					filter_mask = mask << 21 | 0x4;
				}else{
					filter_id = id << 3;
					filter_mask = mask << 3 | 0x4;
				}
				break;
			case FilterMode::ONLY_EXT:
				if(!as_std){
					filter_id = id << 21 | 0x4;
					filter_mask = mask << 21 | 0x4;
				}else{
					filter_id = id << 3 | 0x4;
					filter_mask = mask << 3 | 0x4;
				}
				break;
			case FilterMode::STD_AND_EXT:
				if(!as_std){
					filter_id = id << 21;
					filter_mask = mask << 21;
				}else{
					filter_id = id << 3;
					filter_mask = mask << 3;
				}
			}

			filter.FilterIdHigh         = filter_id >> 16;
			filter.FilterIdLow          = filter_id;
			filter.FilterMaskIdHigh     = filter_mask >> 16;
			filter.FilterMaskIdLow      = filter_mask;
			filter.FilterScale          = CAN_FILTERSCALE_32BIT; // 32モード
			filter.FilterFIFOAssignment = rx_fifo;      // FIFO0へ格納
			filter.FilterBank           = filter_no;
			filter.FilterMode           = CAN_FILTERMODE_IDMASK; // IDマスクモード
			filter.SlaveStartFilterBank = 14;
			filter.FilterActivation     = ENABLE;

			HAL_CAN_ConfigFilter(can, &filter);
		}

		void set_filter_free(uint32_t filter_no){
			CAN_FilterTypeDef filter;
			filter.FilterIdHigh         = 0;
			filter.FilterIdLow          = 0;
			filter.FilterMaskIdHigh     = 0;
			filter.FilterMaskIdLow      = 0;
			filter.FilterScale          = CAN_FILTERSCALE_32BIT;
			filter.FilterFIFOAssignment = rx_fifo;
			filter.FilterBank           = filter_no;
			filter.FilterMode           = CAN_FILTERMODE_IDMASK;
			filter.SlaveStartFilterBank = 14;
			filter.FilterActivation     = ENABLE;

			HAL_CAN_ConfigFilter(can, &filter);
		}
	};

#endif
}

#endif /* CAN_CONTROL_HPP_ */
