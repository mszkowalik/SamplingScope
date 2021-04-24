/*
 * PinBus.h
 *
 *  Created on: Apr 25, 2021
 *      Author: mszko
 */

#ifndef PINBUS_PINBUS_H_
#define PINBUS_PINBUS_H_
#include "../Pin/Pin.h"

class PinBus {
public:
	PinBus(uint8_t width, uint32_t mode, Pin** busPins);
	virtual ~PinBus();
	void write(uint32_t data);
	uint32_t read();
private:
	uint32_t mode_;
	Pin** pins_;
	uint8_t width_;

};

#endif /* PINBUS_PINBUS_H_ */
