/*
 * SY89296.cpp
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#include "DelayLine.h"

DelayLine::DelayLine(Pin** bus, DAC* DAC ) {
	DAC_ = DAC;
	Bus_ = bus;

}

DelayLine::~DelayLine() {
	// TODO Auto-generated destructor stub
}

void DelayLine::setDelay(uint32_t word)
{

	// upper 11 bits are used for digital control, lower 12 are used for DAC
	word = word & 0x7FFFFF; //use only 23 bits (11 + 12)

	uint64_t logic_word = word >>12;
	for(uint8_t i = 0; i <= 11; i++)
	{
		GPIO_PinState val  = static_cast<GPIO_PinState>((logic_word >> i)&1);
		Bus_[i]->write(val);
	}

	DAC_->setVoltage(word & 4095 );  //lower 12 bits of data
}
