/*
 * SY89296.h
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#ifndef DELAYLINE_DELAYLINE_H_
#define DELAYLINE_DELAYLINE_H_

#include "../DAC/DAC.h"
#include "../PinBus/PinBus.h"

class DelayLine {
public:
	DelayLine(PinBus* bus, DAC* DAC);
	virtual ~DelayLine();

	void setDelay(uint32_t word);
private:
	PinBus* Bus_;
	DAC* DAC_;
};

#endif /* DELAYLINE_DELAYLINE_H_ */
