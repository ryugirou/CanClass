/*
 * CanClass.hpp
 *
 *  Created on: Jun 30, 2019
 *      Author: ryu
 */

#ifndef CANCLASS_HPP_
#define CANCLASS_HPP_

#include "stm32f1xx_hal.h"
#include "stm32f103xb.h"

enum can_bitrate {
    CAN_BITRATE_10K,
    CAN_BITRATE_20K,
    CAN_BITRATE_50K,
    CAN_BITRATE_100K,
    CAN_BITRATE_125K,
    CAN_BITRATE_250K,
    CAN_BITRATE_500K,
    CAN_BITRATE_750K,
    CAN_BITRATE_1000K,
};

enum can_bus_state {
    OFF_BUS,
    ON_BUS
};

#define CAN_MTU 8
#define LED_DURATION 10

template<typename T>
union _Encapsulator
{
    T data;
    uint64_t i;
};

class CanClass {
private:
	CAN_HandleTypeDef hcan;
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_payload[CAN_MTU];
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_payload[CAN_MTU];
	uint32_t status;
	uint8_t rx_flag = 0;
	uint32_t led_laston = 0;
	uint32_t led_lastoff = 0;
	CAN_FilterTypeDef filter;
	uint32_t prescaler;
	enum can_bus_state bus_state;
	void can_set_filter(uint32_t id, uint32_t mask);
	void can_enable(void);
	void can_disable(void);
	void can_set_bitrate(enum can_bitrate bitrate);
	void can_set_silent(uint8_t silent);
	uint32_t can_tx(CAN_TxHeaderTypeDef *tx_header, uint8_t (&buf)[CAN_MTU]);
	uint32_t can_rx(CAN_RxHeaderTypeDef *rx_header, uint8_t (&buf)[CAN_MTU]);
	uint8_t is_can_msg_pending(uint8_t fifo);
	void Error_Handler(void);

public:
	CanClass();
	template<typename T>
	void send(T data,uint32_t id);
	template<typename T>
	void receive(T &data,uint32_t id);
	void init(void);
	void endit(void);
	void led_on(void);
	void led_process(void);
};

template<typename T>
void CanClass::send(T data,uint32_t id)
{
    _Encapsulator<T> _e;
    _e.data = data;

    for (int i = sizeof(T); i > 0;)
    {
        i--;
        tx_payload[i] = _e.i & 0xff;
        _e.i >>= 8;
    }
    tx_header.StdId = id;
    can_tx(&tx_header, tx_payload);
}

template<typename T>
void CanClass::receive(T &data,uint32_t id)
{

	status = can_rx(&rx_header, rx_payload);

	if (status == HAL_OK or rx_flag == 1)
	{
		if(rx_header.StdId == id){
			_Encapsulator<T> _e;

			for (int i = 0; i < sizeof(T); i++)
			{
				_e.i = (_e.i << 8) | (uint64_t) (rx_payload[i]);
			}

			data = _e.data;
		}
		else{
			rx_flag=1;
		}
	}
}

#endif /* CANCLASS_HPP_ */
