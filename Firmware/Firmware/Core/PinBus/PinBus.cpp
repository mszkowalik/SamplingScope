/*
 * PinBus.cpp
 *
 *  Created on: Apr 25, 2021
 *      Author: mszko
 */

#include "PinBus.h"

PinBus::PinBus(uint8_t width, uint32_t mode, Pin** busPins) {
	// TODO Auto-generated constructor stub
	width_ = width;
	mode_ = mode;
	pins_ = busPins;

}

PinBus::~PinBus() {
	// TODO Auto-generated destructor stub
}
void PinBus::write(uint32_t data)
{
	data = data & (0xFFFFFFFF >> (32-width_)); // limit number
	for(uint8_t i = 0; i < width_; i++)
		{
			GPIO_PinState val  = static_cast<GPIO_PinState>((data >> i)&1);
			pins_[i]->write(val);
		}
}

