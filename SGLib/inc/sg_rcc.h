/**
 * @author  Bernhard Gleich
 * @version v1.0
 *
 * \par Changelog
 * 03-27-21     V1.0        Bernhard Gleich
 */
 
/**
 * @addtogroup CORE   System Core
 * @{
 *
 */   

#ifndef SG_RCC_H
#define SG_RCC_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif 
 
/**
 * @addtogroup RCC   RCC
 * @{
 *
 * RCC library provides initialization of the very basic clock at the beginning. Function @ref SG_RCC_Init should be called at beginning of the main 
 * function to initialize system.
 *
 * The lib sets the APB1, APB2 Prescaler to te lowest possible factor. The AHB Prescaler will be set to one to get maximum speed at all peripherial 
 * units.
 * Use your own implementation in case you want something special.
 *
 * @note  In case of STM32F7xx is used, this library also enables CACHE for Instructions and Data.
 * @note  In case there is no specification in the bsp.h file the following parameters will be used
 *          - #define RCC_OSCILLATORTYPE    RCC_OSCILLATORTYPE_HSE
 *          - #define RCC_PLLM              8
 *          - #define RCC_PLLN              360
 *          - #define RCC_PLLP              2
 *          - #define RCC_PLLQ              7
 * To use the internal oscillator just use
 *          - #define RCC_OSCILLATORTYPE    something else
 *
 * @note  The LSE clock is not configured within this lib.
 * @note  Peripherial Clocks are not configured here.
 * @note  No MCO source nor I2S Mux will be selected here.
 *
 * \par Dependencies
 *
 *  - sg_lib_conf.h
 *  - bsp.h.h
 *
 * \todo Test on several boards
 *
 */  

//---- Include Section ---------------------------------------------------------
#include "bsp.h"     
#include "sg_lib_conf.h"

//---- Define Section ----------------------------------------------------------
/* Set default values if not defined by user */
#if !defined(RCC_OSCILLATORTYPE) || !defined(RCC_PLLM) || !defined(RCC_PLLN) || !defined(RCC_PLLP) || !defined(RCC_PLLQ) 
#define RCC_OSCILLATORTYPE    RCC_OSCILLATORTYPE_HSE
#define RCC_PLLM              8
#define RCC_PLLN              360
#define RCC_PLLP              2
#define RCC_PLLQ              7
#endif

//---- Global Section ----------------------------------------------------------

//---- Enumeration/Typedef Section ---------------------------------------------

/**
 * @addtogroup RCC_Typedefs     Type definitions and enumerations
 * @brief    Enumerations of pin combination and results 
 * @{
 */

/**
 * @brief  RCC result enumeration
 */
typedef enum {
	SG_RCC_Ok = 0x00, /*!< Everything OK */
	SG_RCC_Error      /*!< An error occurred */
} sg_rcc_result_t;

//addtogroup 
/**
 * @}
 */
//---- Prototype Section -------------------------------------------------------

/**
 * @addtogroup RCC_Functions    Exported functions
 * @brief    Exported functions to use the Clock System
 * @{
 */

/**
 * @brief  Initializes system clocks
 * @note   This function should be called on the start of main program
 * @note   When STM32F7xx target is used, D and I caches are also enabled with this function
 * @param  None
 * @retval RCC System status as definedn in \ref sg_rcc_result_t
 */
sg_rcc_result_t SG_RCC_Init(void); 

//addtogroup 
/**
 * @}
 */
 
//addtogroup 
/**
 * @}  
 */
 
//addtogroup 
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
