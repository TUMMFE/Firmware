/**
 * \file            sound.c
 * \brief           Acoustic stimulation
 */


//---- Include Section ---------------------------------------------------------
#include "sound.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp.h"
#include "main.h"
#include "sg_lib_conf.h"
#include "sg_io.h"

#include "cmsis_os2.h"



//---- Define Section ----------------------------------------------------------
//---- Global Section ----------------------------------------------------------

/**
  * @brief Some counters/timers to monitor the position inside the stimulation block
  */
static uint16_t StimulusCounter;   /*!< counter of already applied stimuli           */
static uint32_t TotalStimuli;      /*!< total number of stimuli to be applied        */
static bool isFinished;            /*!< true when stimulation block is finished      */
static TIM_HandleTypeDef htim1;    /*!< used for the PWM timer (stimulus control)    */
static osTimerId_t Stimulus_id;    /*!< one shot timer to control the duration of a stimulus */
static osTimerId_t Block_id;       /*!< periodic timer to control the stimuli inside a block */
static uint32_t BlockTicks;        /*!< Timer ticks for the block timer              */
static uint32_t StimuliTicks;      /*!< Timer ticks for the stimulus timer           */

osEventFlagsId_t evt_id; 

//---- Enumeration/Typedef Section ---------------------------------------------
//---- Prototype Section -------------------------------------------------------

/**
 * @brief Start the PWM output
 */
static void StartPWM(void);

/**
 * @brief Stop the PWM output
 */
static void StopPWM(void);


//---- Private Function Section ------------------------------------------------

static void StartPWM(void) {
  //BUZZER_TIMER->CR1 |= TIM_CR1_CEN;     //set the CEN bit in the CR1 register
  HAL_TIM_PWM_Start(&htim1, BUZZER_TIMER_CHANNEL);
}  // END OF 'StartPWM'
  
static void StopPWM(void) {
  //BUZZER_TIMER->CR1 &= ~TIM_CR1_CEN;    //clear the CEN bit in the CR1 register
  HAL_TIM_PWM_Stop(&htim1, BUZZER_TIMER_CHANNEL);
}  // END OF 'StopPWM'

//---- IRQ Handler -------------------------------------------------------------

static void StimulusTimer_Callback (void *argument) {
  StopPWM();          //stop the PWM generation
  StimulusCounter++;  //increase the stimulus counter
}  // END OF 'StimulusTimer_Callback'

static void BlockTimer_Callback (void *argument) {
  if (StimulusCounter < TotalStimuli) {
    osEventFlagsSet(evt_id, FLAG_START_DAQ);
    StartPWM();    //start the PWM generation
    osTimerStart(Stimulus_id, StimuliTicks);
  } else {
    isFinished = true;
    osTimerStop(Block_id);
  }  
}  // END OF 'BlockTimer_Callback'

//---- Exported functions ------------------------------------------------------

extern void SOUND_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  
  //Peripheral clock enable
	BUZZER_TIMER_CLK_ENABLE();
  
  //Basic timer configuration
  htim1.Instance                = BUZZER_TIMER;
  htim1.Init.Prescaler          = (uint16_t) BUZZER_TIMER_PSC; 
  htim1.Init.CounterMode        = TIM_COUNTERMODE_UP;
  htim1.Init.Period             = (uint16_t) BUZZER_TIMER_ARR; //400 kHz / 4 kHz - 1 = 99
  htim1.Init.ClockDivision      = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter  = 0;
  htim1.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  /* Common configuration for all channels */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (uint16_t) BUZZER_TIMER_PULSE; 
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, BUZZER_TIMER_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
  
  //Configuration of GPIO Pin
  SG_IO_InitAlternate(BUZZER_GPIO_Port, BUZZER_Pin, SG_IO_PushPull, SG_IO_NoPullUpOrDown, SG_IO_High, BUZZER_ALTERNATE_FUNCTION);
 
  
  /* Create two software timers. The first timer (Stimulus_id) is used to 
   * control the duration of one acoustic simulus. The other timer (Block_id) is
   * used to control the block itself and the release of the acoustic simuli.
   * The stimulus timer is a one shot timer which is started every fstim in the
   * callback of the block timer. 
   */
  
  Stimulus_id = osTimerNew(StimulusTimer_Callback, osTimerOnce, (void *)0, NULL);
  if (Stimulus_id == NULL) {
    Error_Handler();
  }
  
  Block_id = osTimerNew(BlockTimer_Callback, osTimerPeriodic, (void *)5, NULL);
  if (Block_id == NULL) {
    Error_Handler();
  }
  evt_id = osEventFlagsNew(NULL);
  if (evt_id == NULL) {
    Error_Handler();
  }

  
} // END OF 'SOUND_Init'

extern void SOUND_Configuration(SOUND_StimulationBlock_TypeDef block) {
  float temp;
  
  //SOUND_Cancel();
  /* Calculate the total number of stimuli inside one block. fstim is given in
   * Millihertz, thus this must be converted to Hz by dividing fstim by 1000.
   * The number of stimuli than is fstim (now in Hz) mulitplied by the block
   * duration given in seconds. Afterall cast it to an integer value.
   */
  temp = ((float)block.fstim * (float)block.blockduration) / (float)1000.0;
  TotalStimuli = (uint32_t) temp;
  
  StimulusCounter = 0;  //reset the stimulus counter  
  
  /* The number of ticks for the stimulus counter can be derived directly from 
   * "stimulusduration" as follows:
   * ticks = stimulusduration
   * because the Kernel Tick Frequency was set to 1 kHz.
   *
   * for the block timer, the ticks are related to fstim (given in Millihertz) 
   * as follows:
   * ticks = (1000 / fstim) * 1000.
   * e.g. a typical fstim would be 250 mHz (1 stimulus every 4 seconds).
   * ticks = (1000 / 250) * 1000 = 4000;
   */
  StimuliTicks = block.stimulusduration;
  temp = ((float) 1000.0/((float) block.fstim)) * (float)1000.0;
  BlockTicks = (uint32_t) temp;    
} // END OF 'SOUND_Configuration'

extern void SOUND_Run(void) {
  osStatus_t  status;
  
  isFinished = false;
  StimulusCounter = 0;
  status = osTimerStart(Block_id, BlockTicks);
  if (status != osOK) {
    Error_Handler();
  }   
} // END OF 'SOUND_Run'

extern void SOUND_Pause(void) {
  osStatus_t  status; 
  
  status = osTimerStop(Block_id);
  if (status != osOK) {
    Error_Handler();
  }     
} // END OF 'SOUND_Pause'

extern void SOUND_Resume(void) {
  osStatus_t  status;
  
  status = osTimerStart(Block_id, BlockTicks);
  if (status != osOK) {
    Error_Handler();
  }  
} // END OF 'SOUND_Resume'

extern void SOUND_Cancel(void) {
  osStatus_t  status; 
  isFinished = true;
  
  status = osTimerStop(Block_id);
  if (status != osOK) {
    Error_Handler();
  }    
} // END OF 'SOUND_Cancel'

extern void SOUND_ReleaseStimulus(void) {
  osStatus_t  status;
  
  StartPWM();    //start the PWM generation
  status = osTimerStart(Stimulus_id, StimuliTicks);
  if (status != osOK) {
    Error_Handler();
  }
} // END OF 'SOUND_ReleaseStimulus'

/* END OF FILE */
