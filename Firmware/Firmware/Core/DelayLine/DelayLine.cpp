/*
 * SY89296.cpp
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#include "DelayLine.h"
#include <math.h>

DelayLine::DelayLine(PinBus* bus, DAC* DAC ) {
	DAC_ = DAC;
	Bus_ = bus;

}

DelayLine::~DelayLine() {
	// TODO Auto-generated destructor stub
}

void DelayLine::setDelay(uint32_t word)
{

	// upper 11 bits are used for digital control, lower 10 are used for DAC
	word = word & 0x7FFFFF; //use only 21 bits (11 + 10)

	Bus_->write(word >>12);

/* Delay line has max of 1.5V input voltage
	0V -> 47ps
	1.5V -> 0ps
	additional delay goes only or 3000 LSB.

*/
	//DAC_->setVoltage(uint16_t((word & 4095) * (3000.0 / 4096.0)) );  //lower 10 bits of data
	uint16_t coeffNbr = sizeof(Coeffs)/sizeof(Coeffs[0]);
	float ret = 0;
	for(uint16_t i=0; i < coeffNbr; i++)
	{
		ret+= pow(float(word),float(i))*Coeffs[i];
	}
	uint16_t DACWord = uint16_t(ret);

	DAC_->setVoltage(DACWord & 0xFFF);  //lower 10 bits of data
}
