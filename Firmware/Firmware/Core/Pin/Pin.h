/*
 * Pin.h
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#ifndef PIN_PIN_H_
#define PIN_PIN_H_

#include "stm32h7xx.h"
class Pin {
public:
	Pin(uint16_t PinNumber, GPIO_TypeDef * PortNumber);
	Pin(uint16_t PinNumber, GPIO_TypeDef * PortNumber, uint32_t Mode,GPIO_PinState Default);
	virtual ~Pin();

	GPIO_PinState read();
	GPIO_PinState write(GPIO_PinState state);

private:
	uint32_t mode_;
	GPIO_PinState default_;
	uint16_t pinNumber_;
	GPIO_TypeDef* portNumber_;
};

#endif /* PIN_PIN_H_ */
