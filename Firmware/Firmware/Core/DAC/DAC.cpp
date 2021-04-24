/*
 * MCP4921.cpp
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#include "DAC.h"
//#include "stm32h7xx_hal_spi.h"

DAC::DAC(SPI_HandleTypeDef *hspi, float VREF=2.048) {
	spi_ = hspi;
	vref_ = VREF;

}

DAC::~DAC() {
	// TODO Auto-generated destructor stub
}

uint8_t DAC::Write(bool BUF, bool nGA, bool nSHDN, uint16_t data)
{
	  uint8_t brx[20];
	  uint8_t btx[2] = {0b00111100 , 0b11111111};

	  data = data & 0b111111111111;
	  uint8_t payload[2] = {  uint8_t(BUF) << 6 | uint8_t(nGA) <<5 | uint8_t(nSHDN) <<4 | data>>8 , data & 0xFF };

	  HAL_SPI_TransmitReceive( spi_, payload, brx, 2, 5 ); // timeout 5msec;
	  while( spi_->State == HAL_SPI_STATE_BUSY );  // wait for xmission complete
	  return 0;
}

uint16_t DAC::setVoltage(uint16_t Voltage)
{
	Write(0, 1, 1, Voltage);
}
