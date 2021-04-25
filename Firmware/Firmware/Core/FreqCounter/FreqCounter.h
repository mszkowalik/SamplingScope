/*
 * FreqCounter.h
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#ifndef FREQCOUNTER_FREQCOUNTER_H_
#define FREQCOUNTER_FREQCOUNTER_H_

#include "../PinBus/PinBus.h"

class FreqCounter {
public:
	FreqCounter(PinBus* bus, Pin* reset);
	virtual ~FreqCounter();

	void reset();

private:
	PinBus* bus_;
	Pin* reset_;
};

#endif /* FREQCOUNTER_FREQCOUNTER_H_ */
