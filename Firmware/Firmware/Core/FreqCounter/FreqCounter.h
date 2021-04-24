/*
 * FreqCounter.h
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#ifndef FREQCOUNTER_FREQCOUNTER_H_
#define FREQCOUNTER_FREQCOUNTER_H_

#include "../Pin/Pin.h"

class FreqCounter {
public:
	FreqCounter(Pin** bus, Pin* Reset);
	virtual ~FreqCounter();
};

#endif /* FREQCOUNTER_FREQCOUNTER_H_ */
