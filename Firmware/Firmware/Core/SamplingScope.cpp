/*
 * SamplingScope.cpp
 *
 *  Created on: Apr 25, 2021
 *      Author: mszko
 */

#include "SamplingScope.h"
#include "math.h"

SamplingScope::SamplingScope(SPI_HandleTypeDef *vref1_hspi,SPI_HandleTypeDef *vref2_hspi,SPI_HandleTypeDef *delay_main_dac_hspi,SPI_HandleTypeDef *delay_step_dac_hspi, TIM_HandleTypeDef *timer, uint32_t* ADC_RAW) {

	ADC = ADC_RAW;

	MAIN_DELAY_Q0 = new Pin(MAIN_DELAY_Q0_Pin, MAIN_DELAY_Q0_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q1 = new Pin(MAIN_DELAY_Q1_Pin, MAIN_DELAY_Q1_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q2 = new Pin(MAIN_DELAY_Q2_Pin, MAIN_DELAY_Q2_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q3 = new Pin(MAIN_DELAY_Q3_Pin, MAIN_DELAY_Q3_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q4 = new Pin(MAIN_DELAY_Q4_Pin, MAIN_DELAY_Q4_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q5 = new Pin(MAIN_DELAY_Q5_Pin, MAIN_DELAY_Q5_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q6 = new Pin(MAIN_DELAY_Q6_Pin, MAIN_DELAY_Q6_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q7 = new Pin(MAIN_DELAY_Q7_Pin, MAIN_DELAY_Q7_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q8 = new Pin(MAIN_DELAY_Q8_Pin, MAIN_DELAY_Q8_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q9 = new Pin(MAIN_DELAY_Q9_Pin, MAIN_DELAY_Q9_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q10 = new Pin(MAIN_DELAY_Q10_Pin, MAIN_DELAY_Q10_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);

	STEP_GEN_DELAY_Q0 = new Pin(STEP_GEN_DELAY_Q0_Pin, STEP_GEN_DELAY_Q0_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q1 = new Pin(STEP_GEN_DELAY_Q1_Pin, STEP_GEN_DELAY_Q1_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q2 = new Pin(STEP_GEN_DELAY_Q2_Pin, STEP_GEN_DELAY_Q2_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q3 = new Pin(STEP_GEN_DELAY_Q3_Pin, STEP_GEN_DELAY_Q3_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q4 = new Pin(STEP_GEN_DELAY_Q4_Pin, STEP_GEN_DELAY_Q4_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q5 = new Pin(STEP_GEN_DELAY_Q5_Pin, STEP_GEN_DELAY_Q5_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q6 = new Pin(STEP_GEN_DELAY_Q6_Pin, STEP_GEN_DELAY_Q6_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q7 = new Pin(STEP_GEN_DELAY_Q7_Pin, STEP_GEN_DELAY_Q7_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q8 = new Pin(STEP_GEN_DELAY_Q8_Pin, STEP_GEN_DELAY_Q8_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q9 = new Pin(STEP_GEN_DELAY_Q9_Pin, STEP_GEN_DELAY_Q9_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q10 = new Pin(STEP_GEN_DELAY_Q10_Pin, STEP_GEN_DELAY_Q10_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);

	CNTR_Q0 = new Pin(CNTR_Q0_Pin, CNTR_Q0_GPIO_Port, GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q1 = new Pin(CNTR_Q1_Pin, CNTR_Q1_GPIO_Port, GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q2 = new Pin(CNTR_Q2_Pin, CNTR_Q2_GPIO_Port, GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q3 = new Pin(CNTR_Q3_Pin, CNTR_Q3_GPIO_Port, GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q4 = new Pin(CNTR_Q4_Pin, CNTR_Q4_GPIO_Port, GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q5 = new Pin(CNTR_Q5_Pin, CNTR_Q5_GPIO_Port,GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q6 = new Pin(CNTR_Q6_Pin, CNTR_Q6_GPIO_Port,GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q7 = new Pin(CNTR_Q7_Pin, CNTR_Q7_GPIO_Port,GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q8 = new Pin(CNTR_Q8_Pin, CNTR_Q8_GPIO_Port,GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q9 = new Pin(CNTR_Q9_Pin, CNTR_Q9_GPIO_Port,GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q10 = new Pin(CNTR_Q10_Pin, CNTR_Q10_GPIO_Port,GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q11 = new Pin(CNTR_Q11_Pin, CNTR_Q11_GPIO_Port,GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q12 = new Pin(CNTR_Q12_Pin, CNTR_Q12_GPIO_Port,GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q13 = new Pin(CNTR_Q13_Pin, CNTR_Q13_GPIO_Port,GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q14 = new Pin(CNTR_Q14_Pin, CNTR_Q14_GPIO_Port,GPIO_MODE_INPUT,GPIO_PIN_RESET);
	CNTR_Q15 = new Pin(CNTR_Q15_Pin, CNTR_Q15_GPIO_Port,GPIO_MODE_INPUT,GPIO_PIN_RESET);

	CLK_EN_1 = new Pin(CLK_EN_1_Pin, CLK_EN_1_GPIO_Port,GPIO_MODE_OUTPUT_PP, GPIO_PIN_SET);
	CLK_EN_0 = new Pin(CLK_EN_0_Pin, CLK_EN_0_GPIO_Port,GPIO_MODE_OUTPUT_PP, GPIO_PIN_SET);
	CLK_SEL_1 = new Pin(CLK_SEL_1_Pin, CLK_SEL_1_GPIO_Port,GPIO_MODE_OUTPUT_PP, GPIO_PIN_SET);
	CLK_SEL_0 = new Pin(CLK_SEL_0_Pin, CLK_SEL_0_GPIO_Port,GPIO_MODE_OUTPUT_PP, GPIO_PIN_RESET);
	CNTR_RESET = new Pin(CNTR_RESET_Pin, CNTR_RESET_GPIO_Port,GPIO_MODE_OUTPUT_PP, GPIO_PIN_SET);

	PROG_DELAY_MAIN = new Pin*[11] {
				MAIN_DELAY_Q0,
				MAIN_DELAY_Q1,
				MAIN_DELAY_Q2,
				MAIN_DELAY_Q3,
				MAIN_DELAY_Q4,
				MAIN_DELAY_Q5,
				MAIN_DELAY_Q6,
				MAIN_DELAY_Q7,
				MAIN_DELAY_Q8,
				MAIN_DELAY_Q9,
				MAIN_DELAY_Q10
		};

	PROG_DELAY_STEP_GEN = new Pin* [11]{
				STEP_GEN_DELAY_Q0,
				STEP_GEN_DELAY_Q1,
				STEP_GEN_DELAY_Q2,
				STEP_GEN_DELAY_Q3,
				STEP_GEN_DELAY_Q4,
				STEP_GEN_DELAY_Q5,
				STEP_GEN_DELAY_Q6,
				STEP_GEN_DELAY_Q7,
				STEP_GEN_DELAY_Q8,
				STEP_GEN_DELAY_Q9,
				STEP_GEN_DELAY_Q10
		};

	CNTR = new Pin*[16]{
				CNTR_Q0,
				CNTR_Q1,
				CNTR_Q2,
				CNTR_Q3,
				CNTR_Q4,
				CNTR_Q5,
				CNTR_Q6,
				CNTR_Q7,
				CNTR_Q8,
				CNTR_Q9,
				CNTR_Q10,
				CNTR_Q11,
				CNTR_Q12,
				CNTR_Q13,
				CNTR_Q14,
				CNTR_Q15
			};

	VREF1 = new DAC(vref1_hspi, 2.048f);
	VREF2 = new DAC(vref2_hspi, 2.048f);
	DELAY_MAIN_DAC = new DAC(delay_main_dac_hspi, 2.048f);
	DELAY_STEP_DAC = new DAC(delay_step_dac_hspi, 2.048f);

	MAIN_DELAY_BUS = new PinBus(11,GPIO_MODE_OUTPUT_PP, PROG_DELAY_MAIN);
	STEP_GEN_DELAY_BUS = new PinBus(11,GPIO_MODE_OUTPUT_PP, PROG_DELAY_STEP_GEN);
	CNTR_BUS = new PinBus(16,GPIO_MODE_INPUT, CNTR);

	MAIN_DELAY = new DelayLine(MAIN_DELAY_BUS, DELAY_MAIN_DAC);
	STEP_DELAY = new DelayLine(STEP_GEN_DELAY_BUS, DELAY_STEP_DAC);

	//COUNTER = new FreqCounter(CNTR_BUS, CNTR_RESET);
	timer_ = timer;
	HAL_TIM_Base_Start_IT(timer_);
	CNTR_BUS->read();
	MAIN_DELAY_BUS->write(0b10101111000);

}
void SamplingScope::delayTest(){
	while(1){
		CNTR_RESET->write(GPIO_PIN_RESET);
		precDelay_us(0);
		CNTR_RESET->write(GPIO_PIN_SET);
		precDelay_us(0);

	}
}

SamplingScope::~SamplingScope() {
	// TODO Auto-generated destructor stub
}

float SamplingScope::measureDelay(uint32_t controlWord)
{
	CLK_SEL_1->write(GPIO_PIN_RESET);
	CNTR_RESET->write(GPIO_PIN_RESET);

	//set DelayLine delay
	MAIN_DELAY->setDelay(controlWord);
	precDelay_us(10); // DAC settling time
	//connectFeedback Loop
	CLK_SEL_0->write(GPIO_PIN_SET); // select CLK1 input
	//let the loop settle
	precDelay_us(10);
	//reset counter
	CNTR_RESET->write(GPIO_PIN_RESET);
	precDelay_us(10);
	CNTR_RESET->write(GPIO_PIN_SET);
	//delay 1ms
	precDelay_us(1000);
	//disconnect feedback loop
	CLK_SEL_0->write(GPIO_PIN_RESET); //deselect CLK1
	//read counter value
	uint32_t delay = CNTR_BUS->read();

	float delay_s = 1/float(delay*1E3);


	return delay_s;

}

void SamplingScope::precDelay_us(uint64_t delay)
{
	__HAL_TIM_SET_COUNTER(timer_,0);
	delay = delay*275;
	while(__HAL_TIM_GET_COUNTER(timer_) < ((delay)));
}

float SamplingScope::getDACVoltage(uint8_t CH)
{

	if (CH == _PD_MAIN || CH ==_PD_STEP || CH ==_VREF1 || CH ==_VREF2)
	{
		float ret = ((ADC[CH]/1024.0)*(3.3)/65536); //divide by 1024 -> oversampling
	}
	else
		return 0.0;
}
float SamplingScope::getTemp(uint8_t CH)
{
	if (CH == _RT0 || CH ==_RT1 || CH ==_RT2 || CH ==_RT3)
	{ //Nominal B-Constant 3350e3399K = 3375
		const float B = 3375.0;
		float volt = ((ADC[CH]/1024.0)*(3.3)/65536); //divide by 1024 -> oversampling
		float res = volt/((3.3-volt)/10000.0);
		float Temp = (1/((log(res/10000.0)/B)+(1/298.15)))-273.15;
		return Temp;
	}
	else
		return 0.0;
}


void SamplingScope::idle()
{
	MAIN_DELAY->setDelay(0);
	CLK_SEL_0->write(GPIO_PIN_SET); // select CLK1 input
}
