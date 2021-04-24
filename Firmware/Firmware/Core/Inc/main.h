/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SIMO_EN_Pin GPIO_PIN_3
#define SIMO_EN_GPIO_Port GPIOE
#define PROG_DELAY_STEP_GEN_CS_Pin GPIO_PIN_4
#define PROG_DELAY_STEP_GEN_CS_GPIO_Port GPIOE
#define VREF2_LDAC_Pin GPIO_PIN_13
#define VREF2_LDAC_GPIO_Port GPIOC
#define USB_PD_ALERT_Pin GPIO_PIN_14
#define USB_PD_ALERT_GPIO_Port GPIOC
#define ECL_EN_Pin GPIO_PIN_15
#define ECL_EN_GPIO_Port GPIOC
#define MAIN_DELAY_Q10_Pin GPIO_PIN_1
#define MAIN_DELAY_Q10_GPIO_Port GPIOF
#define MAIN_DELAY_Q9_Pin GPIO_PIN_2
#define MAIN_DELAY_Q9_GPIO_Port GPIOF
#define MAIN_DELAY_Q8_Pin GPIO_PIN_3
#define MAIN_DELAY_Q8_GPIO_Port GPIOF
#define MAIN_DELAY_Q7_Pin GPIO_PIN_4
#define MAIN_DELAY_Q7_GPIO_Port GPIOF
#define MAIN_DELAY_Q6_Pin GPIO_PIN_5
#define MAIN_DELAY_Q6_GPIO_Port GPIOF
#define MAIN_DELAY_Q5_Pin GPIO_PIN_6
#define MAIN_DELAY_Q5_GPIO_Port GPIOF
#define MAIN_DELAY_Q4_Pin GPIO_PIN_7
#define MAIN_DELAY_Q4_GPIO_Port GPIOF
#define MAIN_DELAY_Q3_Pin GPIO_PIN_8
#define MAIN_DELAY_Q3_GPIO_Port GPIOF
#define MAIN_DELAY_Q2_Pin GPIO_PIN_9
#define MAIN_DELAY_Q2_GPIO_Port GPIOF
#define MAIN_DELAY_Q1_Pin GPIO_PIN_10
#define MAIN_DELAY_Q1_GPIO_Port GPIOF
#define MAIN_DELAY_Q0_Pin GPIO_PIN_0
#define MAIN_DELAY_Q0_GPIO_Port GPIOC
#define USB_PD_RESET_Pin GPIO_PIN_3
#define USB_PD_RESET_GPIO_Port GPIOC
#define COMP_OUT_P_Pin GPIO_PIN_0
#define COMP_OUT_P_GPIO_Port GPIOA
#define PD_MAIN_LDAC_Pin GPIO_PIN_2
#define PD_MAIN_LDAC_GPIO_Port GPIOA
#define VREF2_FB_Pin GPIO_PIN_6
#define VREF2_FB_GPIO_Port GPIOA
#define RT3_Pin GPIO_PIN_7
#define RT3_GPIO_Port GPIOA
#define RT2_Pin GPIO_PIN_4
#define RT2_GPIO_Port GPIOC
#define RT1_Pin GPIO_PIN_5
#define RT1_GPIO_Port GPIOC
#define RT0_Pin GPIO_PIN_0
#define RT0_GPIO_Port GPIOB
#define PD_MAIN_FB_Pin GPIO_PIN_1
#define PD_MAIN_FB_GPIO_Port GPIOB
#define VREF1_FB_Pin GPIO_PIN_11
#define VREF1_FB_GPIO_Port GPIOF
#define PD_STEP_FB_Pin GPIO_PIN_12
#define PD_STEP_FB_GPIO_Port GPIOF
#define STEP_GEN_DELAY_Q10_Pin GPIO_PIN_13
#define STEP_GEN_DELAY_Q10_GPIO_Port GPIOF
#define STEP_GEN_DELAY_Q9_Pin GPIO_PIN_14
#define STEP_GEN_DELAY_Q9_GPIO_Port GPIOF
#define STEP_GEN_DELAY_Q8_Pin GPIO_PIN_15
#define STEP_GEN_DELAY_Q8_GPIO_Port GPIOF
#define STEP_GEN_DELAY_Q0_Pin GPIO_PIN_0
#define STEP_GEN_DELAY_Q0_GPIO_Port GPIOG
#define STEP_GEN_DELAY_Q3_Pin GPIO_PIN_1
#define STEP_GEN_DELAY_Q3_GPIO_Port GPIOG
#define STEP_GEN_DELAY_Q4_Pin GPIO_PIN_7
#define STEP_GEN_DELAY_Q4_GPIO_Port GPIOE
#define STEP_GEN_DELAY_Q5_Pin GPIO_PIN_8
#define STEP_GEN_DELAY_Q5_GPIO_Port GPIOE
#define STEP_GEN_DELAY_Q6_Pin GPIO_PIN_9
#define STEP_GEN_DELAY_Q6_GPIO_Port GPIOE
#define STEP_GEN_DELAY_Q7_Pin GPIO_PIN_10
#define STEP_GEN_DELAY_Q7_GPIO_Port GPIOE
#define STEP_GEN_DELAY_Q2_Pin GPIO_PIN_11
#define STEP_GEN_DELAY_Q2_GPIO_Port GPIOE
#define STEP_GEN_DELAY_Q1_Pin GPIO_PIN_12
#define STEP_GEN_DELAY_Q1_GPIO_Port GPIOE
#define CLK_SEL_0_Pin GPIO_PIN_13
#define CLK_SEL_0_GPIO_Port GPIOE
#define INT_CLK_Pin GPIO_PIN_14
#define INT_CLK_GPIO_Port GPIOE
#define CLK_SEL_1_Pin GPIO_PIN_15
#define CLK_SEL_1_GPIO_Port GPIOE
#define CLK_EN_1_Pin GPIO_PIN_11
#define CLK_EN_1_GPIO_Port GPIOB
#define VREF2_CS_Pin GPIO_PIN_12
#define VREF2_CS_GPIO_Port GPIOB
#define CLK_EN_0_Pin GPIO_PIN_13
#define CLK_EN_0_GPIO_Port GPIOB
#define VREF1_LDAC_Pin GPIO_PIN_14
#define VREF1_LDAC_GPIO_Port GPIOB
#define SAMPLE_CLK_Pin GPIO_PIN_2
#define SAMPLE_CLK_GPIO_Port GPIOG
#define CNTR_RESET_Pin GPIO_PIN_3
#define CNTR_RESET_GPIO_Port GPIOG
#define COMP_OUT_N_Pin GPIO_PIN_4
#define COMP_OUT_N_GPIO_Port GPIOG
#define PD_STEP_LDAC_Pin GPIO_PIN_5
#define PD_STEP_LDAC_GPIO_Port GPIOG
#define CNTR_Q3_Pin GPIO_PIN_6
#define CNTR_Q3_GPIO_Port GPIOG
#define CNTR_Q2_Pin GPIO_PIN_7
#define CNTR_Q2_GPIO_Port GPIOG
#define CNTR_Q1_Pin GPIO_PIN_8
#define CNTR_Q1_GPIO_Port GPIOG
#define CNTR_Q0_Pin GPIO_PIN_6
#define CNTR_Q0_GPIO_Port GPIOC
#define CNTR_Q14_Pin GPIO_PIN_7
#define CNTR_Q14_GPIO_Port GPIOC
#define CNTR_Q13_Pin GPIO_PIN_8
#define CNTR_Q13_GPIO_Port GPIOC
#define CNTR_Q11_Pin GPIO_PIN_9
#define CNTR_Q11_GPIO_Port GPIOC
#define CNTR_Q12_Pin GPIO_PIN_8
#define CNTR_Q12_GPIO_Port GPIOA
#define CNTR_Q4_Pin GPIO_PIN_9
#define CNTR_Q4_GPIO_Port GPIOA
#define CNTR_Q15_Pin GPIO_PIN_10
#define CNTR_Q15_GPIO_Port GPIOA
#define PROG_DELAY_MAIN_CS_Pin GPIO_PIN_15
#define PROG_DELAY_MAIN_CS_GPIO_Port GPIOA
#define CNTR_Q5_Pin GPIO_PIN_12
#define CNTR_Q5_GPIO_Port GPIOC
#define TM_IN1H_Pin GPIO_PIN_10
#define TM_IN1H_GPIO_Port GPIOG
#define TM_IN1L_Pin GPIO_PIN_11
#define TM_IN1L_GPIO_Port GPIOG
#define TM_IN2H_Pin GPIO_PIN_12
#define TM_IN2H_GPIO_Port GPIOG
#define TM_IN2L_Pin GPIO_PIN_13
#define TM_IN2L_GPIO_Port GPIOG
#define TM_nSLEEP_Pin GPIO_PIN_14
#define TM_nSLEEP_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
