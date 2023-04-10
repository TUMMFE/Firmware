#include "stm32f4xx_it.h"
#include "main.h"
#include "sg_lib_conf.h"
#include "bsp.h"
#include "adxl355.h"


extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/**
  * General IRQ Handler for EXTI lines
  */
extern void SG_IO_IRQGeneralHandler(uint16_t GPIO_Line) {
	switch (GPIO_Line) {
    case TRIGGER_IN_Pin:
      osThreadFlagsSet(main_id, FLAG_EXTI_TRIGGER_IN);
      break;
  	case DRDY_Pin:			
			osThreadFlagsSet(main_id, FLAG_EXTI_DRDY);
  		break;  
  	default:
  		break;
  }
} // END OF 'IRQ_GeneralHandler'

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable Interrupt.
  */
void NMI_Handler(void) {
  while (1) {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) {
  while (1) {
  }
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}


/**** END OF FILE ****/
