/*
 * SamplingScope.h
 *
 *  Created on: Apr 25, 2021
 *      Author: mszko
 */

#ifndef SAMPLINGSCOPE_H_
#define SAMPLINGSCOPE_H_
#include "../PinBus/PinBus.h"
#include "../DAC/DAC.h"
#include "../FreqCounter/FreqCounter.h"
#include "../DelayLine/DelayLine.h"
#include "stm32h7xx.h"
#include "main.h"

class SamplingScope {
public:
	SamplingScope(SPI_HandleTypeDef *vref1_hspi,SPI_HandleTypeDef *vref2_hspi,SPI_HandleTypeDef *delay_main_dac_hspi,SPI_HandleTypeDef *delay_step_dac_hspi, TIM_HandleTypeDef *timer);
	virtual ~SamplingScope();

	uint32_t measureDelay(uint32_t controlWord);
	void precDelay_us(uint64_t delay);
	void delayTest();
private:
	Pin *MAIN_DELAY_Q0, *MAIN_DELAY_Q1, *MAIN_DELAY_Q2, *MAIN_DELAY_Q3, *MAIN_DELAY_Q4, *MAIN_DELAY_Q5, *MAIN_DELAY_Q6, *MAIN_DELAY_Q7, *MAIN_DELAY_Q8, *MAIN_DELAY_Q9, *MAIN_DELAY_Q10;
	Pin *STEP_GEN_DELAY_Q0 ,*STEP_GEN_DELAY_Q1 , *STEP_GEN_DELAY_Q2, *STEP_GEN_DELAY_Q3, *STEP_GEN_DELAY_Q4, *STEP_GEN_DELAY_Q5, *STEP_GEN_DELAY_Q6, *STEP_GEN_DELAY_Q7, *STEP_GEN_DELAY_Q8, *STEP_GEN_DELAY_Q9, *STEP_GEN_DELAY_Q10;
	Pin *CNTR_Q0, *CNTR_Q1, *CNTR_Q2, *CNTR_Q3, *CNTR_Q4, *CNTR_Q5, *CNTR_Q6, *CNTR_Q7, *CNTR_Q8, *CNTR_Q9, *CNTR_Q10, *CNTR_Q11, *CNTR_Q12, *CNTR_Q13, *CNTR_Q14, *CNTR_Q15;
	Pin *CLK_EN_1 , *CLK_EN_0, *CLK_SEL_1, *CLK_SEL_0, *CNTR_RESET;

	Pin** PROG_DELAY_MAIN;
	Pin** PROG_DELAY_STEP_GEN;
	Pin** CNTR;

	PinBus *MAIN_DELAY_BUS, *STEP_GEN_DELAY_BUS, *CNTR_BUS;

	DAC *VREF1;
	DAC *VREF2;
	DAC *DELAY_MAIN_DAC;
	DAC *DELAY_STEP_DAC;
	DelayLine *MAIN_DELAY;
	DelayLine *STEP_DELAY;

	FreqCounter *COUNTER;
	TIM_HandleTypeDef* timer_;


};

#endif /* SAMPLINGSCOPE_H_ */
