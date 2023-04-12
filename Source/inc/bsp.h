/**
 * @author  Bernhard Gleich
 * @version v1.0
 *
 * \par Changelog
 * 20-12-22     V1.0        Bernhard Gleich
 */
 
/**
 * @addtogroup CORE   System Core
 * @{
 *
 */ 

/**
 * @addtogroup BSO   Board Supply Package
 * @{
 *
 * \par Description
 * This headerfile contains the hardware abstraction.
 * 
 *
 * \bug
 *      
 *
 * \todo
 *
 * \copyright
 * Copyright (c) 2022 Munich Institute of Biomedical Engineering (MIBE),
 * Technische Universitaet Muenchen,
 * Boltzmannstrasse 11, 85748 Garching
 *
 * <b>All rights reserved.</b>
 */

  
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
 extern "C" {
#endif

//---- Include Section ---------------------------------------------------------
#include <stdint.h>


//---- Define Section ----------------------------------------------------------


/**
 * @addtogroup  MISC_GEN General usefull macros
 * @brief       Macros used throughout the library. 
 *
 * These macros can be used generally, independent of the used hardware
 *
 * @{
 */


#define MISC_BIT(x)	                    (1 << (x))
#define MISC_BITS_PER_LONG              32
#define MISC_GENMASK(h, l) ({ 					\
		uint32_t t = (uint32_t)(~0UL);			\
		t = t << (MISC_BITS_PER_LONG - (h - l + 1));		\
		t = t >> (MISC_BITS_PER_LONG - (h + 1));		\
		t;						\
})


/**
 * @}  
 */
 


/**
 * @addtogroup  BSP_MACROS Macro Definitions
 * @brief       Macros used throughout the library. 
 *
 * These macros are used for basic hardware abstraction and basic GPIO operations
 *
 * @{
 */

/******************************************************************************/
/*      			                  UC family                                     */ 
/******************************************************************************/
#define STM32F4xx                       STM32F4xx
#define STM32F401xx                     1
#define HSE_VALUE                       25000000

/******************************************************************************/
/*      			                 TASK Comm                                      */ 
/******************************************************************************/
#define FLAG_EXTI_DRDY                            0x00000001U
#define FLAG_EXTI_TRIGGER_IN                      0x00000010U
#define FLAG_SYSTEM_CONFIG_CHANGED                0x00000100U
#define FLAG_START_DAQ                            0x00001000U


/******************************************************************************/
/*  			                  CLOCK configuration                               */ 
/******************************************************************************/
#define RCC_OSCILLATORTYPE              RCC_OSCILLATORTYPE_HSE
#define RCC_PLLM                        25
#define RCC_PLLN                        336
#define RCC_PLLP                        4
#define RCC_PLLQ                        7           

/******************************************************************************/
/*  			                  Communication Protocol                            */ 
/******************************************************************************/
#define COM_RX_LENGTH                         9
#define COM_TX_LENGTH                         22
#define COM_CMD_SET_ACOUSTIC_PARAM            0x10
#define COM_CMD_SET_TRIGGER_PARAM             0x20
#define COM_CMD_SET_ADXL355_PARAM             0x30
#define COM_CMD_SET_ADXL355_OFFSET            0x40
#define COM_CMD_SET_STATE_MODE                0x50
#define COM_CMD_GET_SAMPLES                   0x60

#define COM_VAL_IT_FALLING                    0x00
#define COM_VAL_IT_BOTH                       0x01

#define COM_ACK                               0x06
#define COM_NACK                              0x15

#define COM_ERROR_CODE_1                      0x01

#define MAX_INT_OF_MODES                      3 
#define MIN_INT_OF_MODES                      0 
#define MAX_INT_OF_STATES                     3
#define MIN_INT_OF_STATES                     0
#define MAX_INT_OF_HPF                        6
#define MIX_INT_OF_HPF                        0
#define MAX_INT_OF_ODR_LPF                    10
#define MIX_INT_OF_ODR_LPF                    0
#define MAX_INT_OF_RANGE                      3
#define MIX_INT_OF_RANGE                      1

/******************************************************************************/
/*  			          PORTS and PINS for digital outputs                        */ 
/******************************************************************************/
#define CS_Pin 																GPIO_PIN_4
#define CS_GPIO_Port													GPIOA

#define LED0_Pin 															GPIO_PIN_13
#define LED0_GPIO_Port												GPIOC


/******************************************************************************/
/*  			          PORTS and PINS for digital inputs                         */ 
/******************************************************************************/
#define	TRIGGER_IN_Pin 							  				GPIO_PIN_3
#define TRIGGER_IN_GPIO_Port					  			GPIOA
#define TRIGGER_IN_EXTI_Line                  EXTI3_IRQn

#define	DRDY_Pin 							  							GPIO_PIN_0
#define DRDY_IN_GPIO_Port					  					GPIOA
#define DRDY_IN_EXTI_Line                     EXTI0_IRQn


#define	INT1_Pin 							  							GPIO_PIN_1
#define INT1_IN_GPIO_Port					  					GPIOA
#define INT1_EXTI_Line                        EXTI1_IRQn

#define	INT2_Pin 							  							GPIO_PIN_2
#define INT2_IN_GPIO_Port					  					GPIOA
#define INT2_EXTI_Line                        EXTI2_IRQn

/******************************************************************************/
/*  			          PORTS and PINS for SPI interface                          */ 
/******************************************************************************/
#define	SCLK_Pin 							  							GPIO_PIN_5
#define	MISO_Pin 							  							GPIO_PIN_6
#define	MOSI_Pin 							  							GPIO_PIN_7
#define SPI_ALTERNATE_FUNCTION								GPIO_AF5_SPI1
#define SPI_GPIO_Port							        		GPIOA
#define SPI 																	SPI1
#define SPI_PRESCALER                         SPI_BAUDRATEPRESCALER_32 /*!< leads to 4.5 Mbit/s */
#define SPI_IRQ                               SPI1_IRQn
#define SPI_CLK_ENAB                          __HAL_RCC_SPI1_CLK_ENABLE
#define SPI_IRQ_HANDLER                       SPI1_IRQHandler

/******************************************************************************/
/*  			          PORTS and PINS for PWM generation                         */ 
/******************************************************************************/
#define BUZZER_Pin 							        			GPIO_PIN_8  				//TIM1_CH1
#define BUZZER_GPIO_Port					        		GPIOA
#define BUZZER_ALTERNATE_FUNCTION							GPIO_AF1_TIM1
#define BUZZER_RESONANCE_FREQUENCY_KHZ				4
#define BUZZER_TIMER                          TIM1
#define BUZZER_TIMER_CHANNEL                  TIM_CHANNEL_1
#define BUZZER_TIMER_CLK_ENABLE               __HAL_RCC_TIM1_CLK_ENABLE
#define BUZZER_TIMER_CLK_MHZ                  84
#define BUZZER_TIMER_PSC                      (BUZZER_TIMER_CLK_MHZ/1)-1
#define BUZZER_TIMER_ARR                      ((BUZZER_TIMER_CLK_MHZ*1000)/(BUZZER_RESONANCE_FREQUENCY_KHZ*(BUZZER_TIMER_PSC+1)))-1
#define BUZZER_TIMER_PULSE                    (BUZZER_TIMER_ARR+1)/2



/**
 * @}
 */


//---- Global Section ----------------------------------------------------------
//---- Enumeration/Typedef Section ---------------------------------------------
//---- Prototype Section -------------------------------------------------------

 
/**
 * @}  
 */
 

/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

/* recrusive #include prevention */
#endif

/* END OF FILE */

