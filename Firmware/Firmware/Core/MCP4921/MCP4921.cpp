/*
 * MCP4921.cpp
 *
 *  Created on: Apr 24, 2021
 *      Author: mszko
 */

#include "MCP4921.h"
//#include "stm32h7xx_hal_spi.h"

MCP4921::MCP4921(SPI_HandleTypeDef *hspi, float VREF=2.048) {
	spi_ = hspi;
	vref_ = VREF;

}

MCP4921::~MCP4921() {
	// TODO Auto-generated destructor stub
}

uint8_t MCP4921::Write(bool BUF, bool nGA, bool nSHDN, uint16_t data)
{
	  uint8_t brx[20];
	  uint8_t btx[2] = {0b00111100 , 0b11111111};

	  data = data & 0b111111111111;
	  uint8_t payload[2] = {  uint8_t(BUF) << 6 | uint8_t(nGA) <<5 | uint8_t(nSHDN) <<4 | data>>8 , data & 0xFF };

	  HAL_SPI_TransmitReceive( spi_, payload, brx, 2, 5 ); // timeout 5msec;
	  while( spi_->State == HAL_SPI_STATE_BUSY );  // wait for xmission complete
	  return 0;
}

uint16_t MCP4921::setVoltage(uint16_t Voltage)
{
	Write(0, 1, 1, Voltage);
}
