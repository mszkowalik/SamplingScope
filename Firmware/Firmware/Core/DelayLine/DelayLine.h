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
	float Coeffs[21] = {0.00000000e+000,  1.49187859e+000, -2.29744869e-003,  1.78800501e-006,
			 -5.79895328e-010,  6.98725519e-014, -7.95454130e-019,  4.69708485e-024,
			 -1.83653101e-029,  5.23569605e-035, -1.14182645e-040,  1.95452719e-046,
			 -2.66028582e-052,  2.89108071e-058, -2.50047099e-064,  1.70252423e-070,
			 -8.94212108e-077,  3.50047727e-083, -9.62633639e-090,  1.66063667e-096,
			 -1.35312729e-103};

};

#endif /* DELAYLINE_DELAYLINE_H_ */
