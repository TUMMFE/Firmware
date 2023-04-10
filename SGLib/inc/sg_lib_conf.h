/**
 * @author  Bernhard Gleich
 * @version v1.0
 *
 * \par Changelog
 * 03-27-21     V1.0        Bernhard Gleich
 */
 
/**
 * @addtogroup GENERAL   General
 * @{
 *
 */   

#ifndef SG_LIB_CONF_H
#define SG_LIB_CONF_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif 
 
/**
 * @addtogroup CONFIG   Configuration
 * @{
 *
 * \par Structure of the Library
 * Each library is structured as follows:
 *  - Header file:
 *    - Library description
 *    - Includes section
 *    - Defines section for different configurations in library
 *    - Globa section where global variables are defined (if they exist).
 *    - Enumerations and structures section
 *    - Function declaration section
 *  - Source file:
 *    - Function implementations
 *
 * \par Prerequisists
 * Each library includes at least these 2 files:
 *  - sg_lib_conf.h file:
 *    This file was done to include the correct HAL and CMSIS versions of the STM32 families supported by this library.
 *    It sets some basic defines and includes some standard libs.
 *
 *  - bsp.h file:
 *    <b>This file has to be created by the user.</b> 
 *    It contains the board supply package were the users can translate their schematic into more "talking" names. It is also needed to define the    
 *    used STM32 familiy as well as some clock settings. It is used for library configuration settings, so you don't have to edit library file. 
 *    Use the following defines for the different uC families:
 *    -  STM32F7xx or STM32F7XX for F7 series
 *    -  STM32F4xx or STM32F4XX for F4 series
 *    -  STM32F0xx or STM32F0XX for F0 series
 *
 *    Use the following defines to specify your uC
 *    -  STM32F0xx
 *    -  STM32F401xx
 *    -  STM32F405xx
 *    -  STM32F407xx
 *    -  STM32F410Cx
 *    -  STM32F410Rx
 *    -  STM32F410Tx
 *    -  STM32F415xx
 *    -  STM32F417xx
 *    -  STM32F427xx
 *    -  STM32F429xx
 *    -  STM32F437xx
 *    -  STM32F439xx
 *    -  STM32F446xx
 *    -  STM32F469xx
 *    -  STM32F479xx
 *    -  STM32F7xx
 *
 *    Check example below for meaning.
\code

//------------------------------------
//bsp.h file: (set by user)
//------------------------------------

//User wants to change that, use defines.h file like this:
#ifndef BSP.H
#define BSP.H

//Define the used STM32 Family and 401 uC
#define STM32F4XX
#define STM32F401xx


#endif
\endcode

 * \par HAL Source from ST
 *
 * SG libraries works on STM32Cube provided from ST and are not included in package of libraries. You have to add HAL drivers in your project.
 */     

//---- Include Section ---------------------------------------------------------
#include "bsp.h"     
/* Check if HAL drivers enabled */
#ifndef USE_HAL_DRIVER
    #define USE_HAL_DRIVER
#endif

/* Include proper header file */
/* STM32F7xx */
#if defined(STM32F0xx) || defined(STM32F0XX)
#ifndef STM32F0xx
    #define STM32F0xx
#endif
#ifndef STM32F0XX
    #define STM32F0XX
#endif
    #include "stm32f0xx.h"
    #include "stm32f0xx_hal.h"
#endif

/* STM32F4xx */
#if defined(STM32F4xx) || defined(STM32F4XX)
#ifndef STM32F4xx
    #define STM32F4xx
#endif
#ifndef STM32F4XX
    #define STM32F4XX
#endif
    #include "stm32f4xx.h"
    #include "stm32f4xx_hal.h"
#endif

/* STM32F7xx */
#if defined(STM32F7xx) || defined(STM32F7XX)
#ifndef STM32F7xx
    #define STM32F7xx
#endif
#ifndef STM32F7XX
    #define STM32F7XX
#endif
    #include "stm32f7xx.h"
    #include "stm32f7xx_hal.h"
#endif

/* Check if anything defined */
#if !defined(STM32F0xx) && !defined(STM32F4xx) && !defined(STM32F7xx)
#error "There is not selected STM32 family used. Check stm32fxxx_hal.h file for configuration!"
#endif

/* Init main libraries used everywhere */
#include "bsp.h"
#include "sg_rcc.h"
#include "sg_io.h"

//---- Define Section ----------------------------------------------------------

//---- Global Section ----------------------------------------------------------

//---- Enumeration/Typedef Section ---------------------------------------------

//---- Prototype Section -------------------------------------------------------
 
 
//addtogroup CONFIG
/**
 * @}  
 */
 
//addtogroup GENERAL
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
