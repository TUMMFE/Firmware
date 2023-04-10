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
 * @addtogroup MAIN   Main
 * @{
 *
 * \par Description
 * This headerfile contains the main entry point into the applications as well as
 * functions for the general error handler and for getting the current GPIO state.
 * 
 *
 * \bug
 *      
 *
 * \todo
 *
 * \copyright
 * Copyright (c) 2022 Munich Institute of Biomedical Engineering (MIBE)
 * Technische Universitaet Muenchen,
 * Boltzmannstrasse 11, 85748 Garching
 *
 * <b>All rights reserved.</b>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

//---- Include Section ---------------------------------------------------------
#include "cmsis_os2.h"	

//---- Define Section ----------------------------------------------------------
#define APP_MAIN_STK_SZ         (1024U)


//---- Global Section ----------------------------------------------------------
extern uint64_t app_main_stk[];
extern const osThreadAttr_t app_main_attr;
extern osThreadId_t main_id;


//---- Enumeration/Typedef Section ---------------------------------------------

//---- Prototype Section -------------------------------------------------------

/**
 * \brief           Error handler for critical runtime errors
 */
void Error_Handler(void);

/**
 * \brief           Configuration of the system clock
 */
void SystemClock_Config(void);

/**
 * @}
 */

/**
 * @}  
 */
 

/**
 * @}
 */
 
#ifdef __cplusplus
}
#endif

/* recrusive #include prevention */
#endif

/* END OF FILE */
