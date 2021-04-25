/*
 * FreqCounter.cpp
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#include "FreqCounter.h"

FreqCounter::FreqCounter(PinBus* bus, Pin* reset) {
	bus_ = bus;
	reset_ = reset;

}

FreqCounter::~FreqCounter() {
	// TODO Auto-generated destructor stub
}


void FreqCounter::reset()
{
}
