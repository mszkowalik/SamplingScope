/*
 * Pin.cpp
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#include "Pin.h"

Pin::Pin(uint16_t PinNumber, GPIO_TypeDef * PortNumber) {

	Pin(PinNumber,PortNumber,GPIO_MODE_INPUT,GPIO_PIN_RESET ); /// by default set pin to INPUT
}

Pin::Pin(uint16_t PinNumber, GPIO_TypeDef * PortNumber, uint32_t Mode, GPIO_PinState Default)
{

	pinNumber_ = PinNumber;
	portNumber_	= PortNumber;
	default_ = Default;
	mode_ = Mode;

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pinNumber_;
	GPIO_InitStruct.Mode = mode_;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	if(Mode == GPIO_MODE_OUTPUT_PP)
	{
		HAL_GPIO_WritePin(portNumber_, pinNumber_, default_);

		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	}
	if(Mode == GPIO_MODE_INPUT)
	{


	}

	HAL_GPIO_Init(portNumber_, &GPIO_InitStruct);
}
Pin::~Pin() {
	// TODO Auto-generated destructor stub
}

GPIO_PinState Pin::read()
{
	HAL_GPIO_ReadPin(portNumber_,pinNumber_);
}

GPIO_PinState Pin::write(GPIO_PinState state)
{
	HAL_GPIO_WritePin(portNumber_, pinNumber_, state);
	return state;
}
