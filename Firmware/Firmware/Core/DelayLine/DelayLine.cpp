/*
 * SY89296.cpp
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#include "DelayLine.h"

DelayLine::DelayLine(PinBus* bus, DAC* DAC ) {
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

	Bus_->write(word >>12);


	DAC_->setVoltage(word & 4095 );  //lower 12 bits of data
}
