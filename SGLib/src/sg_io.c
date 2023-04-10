/**
 * \file            sg_io.c
 * \brief           GPIO and EXTI configuration
 */

//---- Include Section ---------------------------------------------------------
#include "sg_io.h"

//---- Define Section ----------------------------------------------------------
//---- Global Section ----------------------------------------------------------
/* The index of the array indicates the GPIOx, where as the value represents
 * the used pin pattern
 */
static uint16_t UsedPinsOfPort[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};

//---- Enumeration/Typedef Section ---------------------------------------------

//---- Prototype Section -------------------------------------------------------
/**
 * @brief  Enable the clock of the corresponding GPIO peripherial
 * @param  Pointer to GPIOx port you will use for initialization
 * @retval None
 */
static void EnableClock         (GPIO_TypeDef* GPIOx);

/**
 * @brief  Initializes GPIO pins(s) 
 * @note   This function also enables clock for GPIO port
 * @param  GPIOx: Pointer to GPIOx port you will use for initialization
 * @param  pin: GPIO pin(s) you will use for initialization (eg. GPIO_PIN_0)
 * @param  otype: Select GPIO Output type. This parameter can be a value of @ref sg_io_otype_t enumeration
 * @param  pupd: Select GPIO pull resistor. This parameter can be a value of @ref sg_io_pupd_t enumeration
 * @param  speed: Select GPIO speed. This parameter can be a value of @ref sg_io_speed_t enumeration
 * @retval None
 */
static void Init                (GPIO_TypeDef* GPIOx, uint16_t pin, sg_io_mode_t mode, sg_io_otype_t otype, sg_io_pupd_t pupd, sg_io_speed_t speed);

//---- Private Function Section ------------------------------------------------
static void EnableClock(GPIO_TypeDef* GPIOx) {
    /* Set bit according to the 1 << portsourcenumber */
    #if defined(STM32F0xx)
	    RCC->AHBENR |= (1 << (SG_IO_GetPortSource(GPIOx) + 17));
    #else
	    RCC->AHB1ENR |= (1 << SG_IO_GetPortSource(GPIOx));
    #endif
} // end of 'EnableClock'

static void Init(GPIO_TypeDef* GPIOx, uint16_t pin, sg_io_mode_t mode, sg_io_otype_t otype, sg_io_pupd_t pupd, sg_io_speed_t speed) {
	uint8_t pinpos;
	uint8_t ptr;
	
	ptr = SG_IO_GetPortSource(GPIOx);
	    
	//STM32F0xx series does not have FAST speed mode available 
	#if defined(STM32F0xx)
	    if (speed == SG_IO_Fast) {
		    // Set speed to high mode 
		    speed = SG_IO_High;
	    }
    #endif
	
	//Go through all pins. In total 16 (=0x10) pins are are possible 
	for (pinpos = 0; pinpos < 0x10; pinpos++) {
		//Check if pin available
		if ((pin & (1 << pinpos)) == 0) {
			continue;
		}
		/* The pin will not longer be used. The index of the array 
	     * indicates the GPIOx, where as the value represents the 
		 * used pin pattern
         */		
		UsedPinsOfPort[ptr] |= 1 << pinpos;
		
		//Set GPIO PUPD register 
		GPIOx->PUPDR = (GPIOx->PUPDR & ~(0x03 << (2 * pinpos))) | ((uint32_t)(pupd << (2 * pinpos)));
		
		//Set GPIO MODE register 
		GPIOx->MODER = (GPIOx->MODER & ~((uint32_t)(0x03 << (2 * pinpos)))) | ((uint32_t)(mode << (2 * pinpos)));
		
		/* 
         * All pins are configured as inputs by default normally. 
         * Thus, speed and output type must not be set at input pins.
         */
        /* but what happens to the analog pins*/
        
		//Set only if output or alternate functions 
		if ((mode == SG_IO_Output) || (mode == SG_IO_Alternate) || (mode == SG_IO_Analog)) {		
			/* Set GPIO OTYPE register */
			GPIOx->OTYPER = (GPIOx->OTYPER & ~(uint16_t)(0x01 << pinpos)) | ((uint16_t)(otype << pinpos));
			
			/* Set GPIO OSPEED register */
			/* Speed not valid for analog pin but doesn't matter*/
			GPIOx->OSPEEDR = (GPIOx->OSPEEDR & ~((uint32_t)(0x03 << (2 * pinpos)))) | ((uint32_t)(speed << (2 * pinpos)));
		}
	}
} // end of 'Init'



/******************************************************************/
/*              STM32F4xx and STM32F7xx IRQ handlers              */
/******************************************************************/

#if defined(STM32F4xx) || defined(STM32F7xx)
    #ifndef TM_EXTI_DISABLE_DEFAULT_HANDLER_0
        void EXTI0_IRQHandler(void) {
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR0)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR0;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_0);
        	}
        }
    #endif

    #ifndef EXTI_DISABLE_DEFAULT_HANDLER_1
        void EXTI1_IRQHandler(void) {
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR1)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR1;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_1);
        	}
        }
    #endif

    #ifndef EXTI_DISABLE_DEFAULT_HANDLER_2
        void EXTI2_IRQHandler(void) {
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR2)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR2;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_2);
        	}
        }
    #endif

    #ifndef EXTI_DISABLE_DEFAULT_HANDLER_3
        void EXTI3_IRQHandler(void) {
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR3)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR3;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_3);
        	}
        }
    #endif

    #ifndef EXTI_DISABLE_DEFAULT_HANDLER_4
        void EXTI4_IRQHandler(void) {
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR4)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR4;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_4);
        	}
        }
    #endif

    #ifndef EXTI_DISABLE_DEFAULT_HANDLER_9_5
        void EXTI9_5_IRQHandler(void) {
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR5)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR5;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_5);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR6)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR6;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_6);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR7)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR7;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_7);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR8)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR8;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_8);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR9)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR9;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_9);
        	}
        }
    #endif

    #ifndef EXTI_DISABLE_DEFAULT_HANDLER_15_10
        void EXTI15_10_IRQHandler(void) {
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR10)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR10;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_10);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR11)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR11;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_11);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR12)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR12;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_12);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR13)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR13;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_13);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR14)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR14;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_14);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR15)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR15;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_15);
        	}
        }
    #endif
#endif

/******************************************************************/
/*                     STM32F0xx IRQ handlers                     */
/******************************************************************/

#if defined(STM32F0xx)
    #ifndef EXTI_DISABLE_DEFAULT_HANDLER_0_1
        void EXTI0_1_IRQHandler(void) {
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR0)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR0;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_0);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR1)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR1;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_1);
        	}
        }
    #endif

    #ifndef EXTI_DISABLE_DEFAULT_HANDLER_2_3
        void EXTI2_3_IRQHandler(void) {
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR2)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR2;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_2);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR3)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR3;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_3);
        	}
        }
    #endif

    #ifndef EXTI_DISABLE_DEFAULT_HANDLER_4_15
        void EXTI4_15_IRQHandler(void) {
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR4)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR4;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_4);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR5)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR5;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_5);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR6)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR6;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_6);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR7)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR7;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_7);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR8)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR8;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_8);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR9)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR9;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_9);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR10)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR10;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_10);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR11)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR11;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_11);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR12)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR12;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_12);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR13)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR13;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_13);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR14)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR14;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_14);
        	}
        	/* Check status */
        	if (EXTI->PR & (EXTI_PR_PR15)) {
        		/* Clear bit */
        		EXTI->PR = EXTI_PR_PR15;
        		/* Call global function */
        		SG_IO_IRQGeneralHandler(GPIO_PIN_15);
        	}
        }
    #endif

#endif



//---- Extern Function Section -------------------------------------------------

void SG_IO_Init(GPIO_TypeDef* GPIOx, uint16_t pin, sg_io_mode_t mode, sg_io_otype_t otype, sg_io_pupd_t pupd, sg_io_speed_t speed) {
    //check input. A pin number of 0 will lead to a hard fault due to a division by zero
    if (pin == 0) {
        return;
    }
    EnableClock(GPIOx);
    Init(GPIOx, pin, mode, otype, pupd, speed); 
} // end of 'SG_IO_Init'

void SG_IO_InitAlternate(GPIO_TypeDef* GPIOx, uint16_t pin, sg_io_otype_t otype, sg_io_pupd_t pupd, sg_io_speed_t speed, uint8_t alternate) {
    uint8_t i;
    
    //check input. A pin number of 0 will lead to a hard fault due to a division by zero
    if (pin == 0) {
        return;
    }
    EnableClock(GPIOx);
    
    //set alternate functions for all pins
	//Go through all pins. In total there are 0x10 (= 16) possible pins available 
	for (i = 0; i < 0x10; i++) {
		/* check if pin is available */
		if ((pin & (1 << i)) == 0) {
			continue;            /* the pin is not available; jump to the next */
		}		
		//set alternate function 
		GPIOx->AFR[i >> 0x03] = (GPIOx->AFR[i >> 0x03] & ~(0x0F << (4 * (i & 0x07)))) | (alternate << (4 * (i & 0x07)));
	}    
    Init(GPIOx, pin, SG_IO_Alternate, otype, pupd, speed);     
} // end of 'SG_IO_InitAlternate'

void SG_IO_DeInit(GPIO_TypeDef* GPIOx, uint16_t pin) {
    uint8_t i;
	uint8_t ptr = SG_IO_GetPortSource(GPIOx);
	
	//Go through all pins. In total there are 0x10 (= 16) possible pins available 
	for (i = 0; i < 0x10; i++) {
		//Pin is set ?
		if (pin & (1 << i)) {
			//Set 11 bits combination for analog mode 
			GPIOx->MODER |= (0x03 << (2 * i));			
			/* 
             * The pin will not longer be used. The index of the array 
			 * indicates the GPIOx, where as the value represents the 
			 * used pin pattern
             */
			UsedPinsOfPort[ptr] &= ~(1 << i);
		}
	}    
} // end of 'SG_IO_DeInit'

void SG_IO_SetPinAsInput(GPIO_TypeDef* GPIOx, uint16_t pin) {
    uint8_t i;
    
	//Go through all pins. In total there are 0x10 (= 16) possible pins available 
	for (i = 0; i < 0x10; i++) {
		//Pin is set ?
		if (pin & (1 << i)) {
			//set 00 bits combination for input 
			GPIOx->MODER &= ~(0x03 << (2 * i));
		}
	}    
} // end of 'SG_IO_SetPinAsInput'

void SG_IO_SetPinAsOutput(GPIO_TypeDef* GPIOx, uint16_t pin) {
    uint8_t i;
    
	//Go through all pins. In total there are 0x10 (= 16) possible pins available 
	for (i = 0; i < 0x10; i++) {
		//Pin is set ?
		if (pin & (1 << i)) {
			//set 01 bits combination for output 
			GPIOx->MODER =  (GPIOx->MODER & ~(0x03 << (2 * i))) | (0x01 << (2 * i));
		}
	}     
} // end of 'SG_IO_SetPinAsOutput'
void SG_IO_SetPinAsAnalog(GPIO_TypeDef* GPIOx, uint16_t pin) {
    uint8_t i;
	//Go through all pins
	for (i = 0x00; i < 0x10; i++) {
		//Pin is set 
		if (pin & (1 << i)) {
			// Set 11 bits combination for analog mode 
			GPIOx->MODER |= (0x03 << (2 * i));
		}
	}   
} // end of 'SG_IO_SetPinAsAnalog'

void SG_IO_SetPinAsAlternate(GPIO_TypeDef* GPIOx, uint16_t pin) {
    uint8_t i;	
	//Set alternate functions for all pins
	for (i = 0; i < 0x10; i++) {	
		if ((pin & (1 << i)) == 0) {
			continue;
		}		
		GPIOx->MODER = (GPIOx->MODER & ~(0x03 << (2 * i))) | (0x02 << (2 * i));
	}
    
} // end of 'SG_IO_SetPinAsAlternate'
void SG_IO_SetPullResistor(GPIO_TypeDef* GPIOx, uint16_t pin, sg_io_pupd_t pupd) {
    uint8_t i;
	
	//go through all pins
	for (i = 0; i < 0x10; i++) {
		//Check if pin available 
		if ((pin & (1 << i)) == 0) {
			continue;
		}
		GPIOx->PUPDR = (GPIOx->PUPDR & ~(0x03 << (2 * i))) | ((uint32_t)(pupd << (2 * i)));
	}
    
} // end of 'SG_IO_SetPullResistor'
uint16_t SG_IO_GetPortSource(GPIO_TypeDef* GPIOx) {
    //Get port source number 
	//Offset from GPIOA                       Difference between 2 GPIO addresses 
	return ((uint32_t)GPIOx - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE));    
} // end of 'SG_IO_GetPortSource'

uint16_t SG_IO_GetPinSource(uint16_t pin) {
    uint16_t pinsource = 0;
	
	//Get pinsource
	while (pin > 1) {
		//Increase pinsource 
		pinsource++;
		//Shift right
		pin >>= 1;
	}	
	return pinsource;    
} // end of 'SG_IO_GetPinSource'

void SG_IO_Lock(GPIO_TypeDef* GPIOx, uint16_t pin) {
    uint32_t d;
	
	//Set GPIO pin with 16th bit set to 1 
	d = 0x00010000 | pin;
	
	//Write to LCKR register 
	GPIOx->LCKR = d;
	GPIOx->LCKR = pin;
	GPIOx->LCKR = d;
	
	//Read twice 
	(void)GPIOx->LCKR;
	(void)GPIOx->LCKR;  
    
} // end of 'SG_IO_Lock'
uint16_t SG_IO_GetUsedPins(GPIO_TypeDef* GPIOx) {
    return UsedPinsOfPort[SG_IO_GetPortSource(GPIOx)];
} // end of 'SG_IO_GetUsedPins'

uint16_t SG_IO_GetFreePins(GPIO_TypeDef* GPIOx) {
    return ~UsedPinsOfPort[SG_IO_GetPortSource(GPIOx)];
} // end of 'SG_IO_GetFreePins'

extern sg_io_result_t SG_IO_IRQAttach(GPIO_TypeDef* GPIOx, uint16_t line, sg_io_trigger_t trigger) {
  sg_io_pupd_t pupd;
	uint8_t pinsource, portsource;
	IRQn_Type irqchannel;
	
	/* Check if user wants to init more than one gpio pin for exti at a time */
	if (!(line && !(line & (line - 1)))) {
		uint8_t i;
		/* Check all pins */
		for (i = 0; i < 0x10; i++) {
			if (line & (1 << i)) {
				/* Attach one pin at a time */
				if (SG_IO_IRQAttach(GPIOx, 1 << i, trigger) != SG_IO_Ok) {
					/* If one failed, return error */
					return SG_IO_Error;
				}
			}
		}
				
		/* Return OK, all succedded */
		return SG_IO_Ok;
	}
	
	/* Check if linee is already in use */
	if (
		(EXTI->IMR & line) || /*!< Interrupt already attached */
		(EXTI->EMR & line)    /*!< Event already attached */
	) {
		/* Return error */
		return SG_IO_Error;
	}
	
#if defined(STM32F0xx)
	/* Get IRQ channel */
	switch (line) {
		case GPIO_PIN_0:
		case GPIO_PIN_1:
			irqchannel = EXTI0_1_IRQn;
			break;
		case GPIO_PIN_2:
		case GPIO_PIN_3:
			irqchannel = EXTI2_3_IRQn;
			break;
		case GPIO_PIN_4:
		case GPIO_PIN_5:
		case GPIO_PIN_6:
		case GPIO_PIN_7:
		case GPIO_PIN_8:
		case GPIO_PIN_9:
		case GPIO_PIN_10:
		case GPIO_PIN_11:
		case GPIO_PIN_12:
		case GPIO_PIN_13:
		case GPIO_PIN_14:
		case GPIO_PIN_15:
			irqchannel = EXTI4_15_IRQn;
			break;
		default:
			return SG_IO_Error;
	}
#else
	/* Get IRQ channel */
	switch (line) {
		case GPIO_PIN_0:
			irqchannel = EXTI0_IRQn;
			break;
		case GPIO_PIN_1:
			irqchannel = EXTI1_IRQn;
			break;
		case GPIO_PIN_2:
			irqchannel = EXTI2_IRQn;
			break;
		case GPIO_PIN_3:
			irqchannel = EXTI3_IRQn;
			break;
		case GPIO_PIN_4:
			irqchannel = EXTI4_IRQn;
			break;
		case GPIO_PIN_5:
		case GPIO_PIN_6:
		case GPIO_PIN_7:
		case GPIO_PIN_8:
		case GPIO_PIN_9:
			irqchannel = EXTI9_5_IRQn;
			break;
		case GPIO_PIN_10:
		case GPIO_PIN_11:
		case GPIO_PIN_12:
		case GPIO_PIN_13:
		case GPIO_PIN_14:
		case GPIO_PIN_15:
			irqchannel = EXTI15_10_IRQn;
			break;
		default:
			return SG_IO_Error;
	}
#endif

	/* Check pull settings */
	if (trigger == SG_IO_FallingEdge) {
		pupd = SG_IO_PullUp;
	} else if (trigger == SG_IO_RisingEdge) {
		pupd = SG_IO_PullDown;
	} else {
		pupd = SG_IO_NoPullUpOrDown;
	}
	
	/* Init GPIO pin */
	SG_IO_Init(GPIOx, line, SG_IO_Input, SG_IO_PushPull, pupd, SG_IO_Low);
	
	/* Calculate pinsource */
	pinsource = SG_IO_GetPinSource(line);
	
	/* Calculate portsource */
	portsource = SG_IO_GetPortSource(GPIOx);
	
	/* Enable SYSCFG clock */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	/* Connect proper GPIO to SYSCFG */
	SYSCFG->EXTICR[pinsource >> 2] &= ~(0x0F << (0x04 * (pinsource & 0x03)));
	SYSCFG->EXTICR[pinsource >> 2] |= (portsource << (0x04 * (pinsource & 0x03)));
	
	/* Clear first */
	EXTI->IMR &= ~line;
	EXTI->EMR &= ~line;
	
	/* Select interrupt mode */
	EXTI->IMR |= line;
	
	/* Clear first */
	EXTI->FTSR &= ~line;
	EXTI->RTSR &= ~line;
	
	/* Select edge */
	if (trigger == SG_IO_FallingEdge) {
		/* Write to fallineg edge register */
		EXTI->FTSR |= line;
	} else if (trigger == SG_IO_RisingEdge) {
		/* Write to rising edge register */
		EXTI->RTSR |= line;
	} else {
		/* Write to rising and fallineg edge registers */
		EXTI->FTSR |= line;
		EXTI->RTSR |= line;
	}
	
	/* Add to NVIC */
	HAL_NVIC_SetPriority(irqchannel, EXTI_NVIC_PRIORITY, pinsource);
	/* Enable interrupt */
	HAL_NVIC_EnableIRQ(irqchannel);
    return SG_IO_Ok;
} // end of 'SG_IO_IRQAttach'

extern sg_io_result_t SG_IO_IRQDetach(uint16_t line){
    // Disable EXTI for specific GPIO line 
	EXTI->IMR &= ~line;
	EXTI->EMR &= ~line;
	
	// Clear trigger edges 
	EXTI->FTSR &= ~line;
	EXTI->RTSR &= ~line;
	
	return SG_IO_Ok;    
} // end of 'SG_IO_IRQDetach'

extern void SG_IO_IRQDeInit(void) {
    // Clear EXTERNAL lines only = GPIO pins
	EXTI->IMR   &= 0xFFFF0000;
	EXTI->EMR   &= 0xFFFF0000;
	EXTI->FTSR  &= 0xFFFF0000;
	EXTI->RTSR  &= 0xFFFF0000;
	EXTI->PR    &= 0xFFFF0000;      
} // end of 'SG_IO_IRQDeInit'

__weak extern void SG_IO_IRQGeneralHandler(uint16_t line){
    /* NOTE:    This function Should not be modified, when the 
                callback is needed, the 'IRQ_GeneralHandler' 
                could be implemented in the user file
    */ 
} // end of 'SG_IO_IRQGeneralHandler'
 
/* END OF FILE */
