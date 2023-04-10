/**
 * @author  Bernhard Gleich
 * @version v1.0
 *
 * \par Changelog
 * 03/10/23     V1.0        Bernhard Gleich
 */
 
/**
 * @addtogroup PER   System Peripherials
 * @{
 *
 */ 

/**
 * @addtogroup SOUND   Acoustic stimulus generation
 * @{
 *
 * \par Description
 * This headerfile contains the methods to create an acoustic stimulus and to
 * control the buzzer.
 * 
 * Subjects practice fastest possible thumb flexion movements by performing 
 * brief contractions of their left flexor pollicis brevis (FPB) muscle. 
 * Movements are externally paced by an acoustic stimulus at a certain 
 * frequency.
 *
 * Acoustic stimulus will be applied for several minutes, followed by a pause 
 * of again severel minutes. This blocks will be repeated several times.
 *
 *    ____________________________           ____________________________
 *   |acoustic stimulation (fstim)|pause ...|acoustic stimulation (fstim)| ....
 * --                              ----//----                             ----  
 *
 * This file controls the acoustic stimulation pattern with the following 
 * parameters.
 *
 * fstim      repetition rate of the acoustic stimulus in mHz, e.g. 250 mHz will 
 *            lead to one acoustic stimulus within 4 seconds
 *
 * btime      duration of one block in seconds
 *
 * duration   duration of one acoustic stimulus in milliseconds
 *
 * The pause between two subsequent blocks as well as the start of a block
 * will be controlled by the PC HOST and not by the microcontroller.
 *
 * \bug
 *      
 *
 * \todo
 *     - include a signal to start adxl355 measurment before acoustic stimulus
 *
 * \copyright
 * Copyright (c) 2023 Munich Insitute of Biomedical Engineering,
 * Technische Universitaet Muenchen,
 * Boltzmannstrasse 11, 85748 Garching
 *
 * <b>All rights reserved.</b>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SOUND_H
#define __SOUND_H

#ifdef __cplusplus
 extern "C" {
#endif

//---- Include Section ---------------------------------------------------------
#include <stdint.h>
#include <stdint.h>
#include <stdbool.h>
#include <bsp.h>
#include "cmsis_os2.h"

//---- Define Section ----------------------------------------------------------


//---- Global Section ----------------------------------------------------------
/**
 * @addtogroup SOUND_GLOBALS  Global Variables 
 * @brief    Thread variables and other global stuff 
 * @{
 */
extern osEventFlagsId_t evt_id; 
 
/**
  * @}
  */

//---- Enumeration/Typedef Section ---------------------------------------------

/**
 * @addtogroup SOUND_TYPEDEF  Type definitions
 * @brief    Type definitions and enumerations
 * @{
 */
 
/**
  * @brief Stimulation Pattern
  */
typedef struct {
	uint16_t fstim;           /*!< stimulation frequency in Millihertz          */
  uint16_t blockduration;   /*!< stimulation duration of one block in seconds */ 	
	uint16_t stimulusduration;/*!< duration of one stimulus in milliseconds     */
} SOUND_StimulationBlock_TypeDef;

/**
  * @}
  */


//---- Prototype Section -------------------------------------------------------

/**
 * @addtogroup SOUND_Functions     Exported functions
 * @brief    Exported functions to use the TIMING
 * @{
 */

/**
 * @brief Initalize the hardware for the acoustic stimulation
 * @note  Hardware consists of
 *        - the init of the GPIO output pin
 *        - the init of the PWM timer channel 
 *        - the init of the timer peripherial
 *        - the generation of the software timers
 */
extern void SOUND_Init(void);

/**
 * @brief Configruation of the acoustic stimulation block
 * @note  Configruation consists of
 *        - the configuration of the internal counters
 *        - the configuration of the timer for the stimulus duration
 *        - the configuration of the timer for block duration
 *
 * @param  block  Configuration of a stimulation block
 */
extern void SOUND_Configuration(SOUND_StimulationBlock_TypeDef block);

/**
 * @brief Start the stimulation block
 */
extern void SOUND_Run(void);

/**
 * @brief Pause the stimulation block
 */
extern void SOUND_Pause(void);

/**
 * @brief Resume the stimulation block
 */
extern void SOUND_Resume(void);

/**
 * @brief Cancel the stimulation block
 */
extern void SOUND_Cancel(void);

/**
 * @brief Generate one acoustic stmulus
 */
extern void SOUND_ReleaseStimulus(void);

 
/**
  * @}
  */



//---- Closings fors DOXYGEN groups ---------------------------------------------
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
