/*
 * MCP4921.h
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#ifndef DAC_DAC_H_
#define DAC_DAC_H_

#include "stm32h7xx.h"

class DAC {
public:
	DAC(SPI_HandleTypeDef *hspi, float VREF);
	virtual ~DAC();
	uint8_t Write(bool BUF, bool nGA, bool nSHDN, uint16_t data);
	uint16_t setVoltage(uint16_t voltage);

private:
	SPI_HandleTypeDef *spi_;
	float vref_ = 2.048;
	float coeffs[12] = {1,0,0,0,0,0,0,0,0,0,0,0};
};

#endif /* DAC_DAC_H_ */
