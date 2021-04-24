/*
 * SY89296.h
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#ifndef DELAYLINE_DELAYLINE_H_
#define DELAYLINE_DELAYLINE_H_

#include "../DAC/DAC.h"
#include "../Pin/Pin.h"

class DelayLine {
public:
	DelayLine(Pin** bus, DAC* DAC);
	virtual ~DelayLine();

	void setDelay(uint32_t word);
private:
	Pin** Bus_;
	DAC* DAC_;
};

#endif /* DELAYLINE_DELAYLINE_H_ */
