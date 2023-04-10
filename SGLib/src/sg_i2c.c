/**
 * \file            sg_i2c.c
 * \brief           I2C Library
 */

//---- Include Section ---------------------------------------------------------
#include "sg_i2c.h"
//---- Define Section ----------------------------------------------------------

//I2C2 AF fix for F0xx 
#if !defined(GPIO_AF4_I2C2) 
    #define GPIO_AF4_I2C2   GPIO_AF1_I2C2
#endif

//Timeout value
#define I2C_TIMEOUT_VALUE              1000
#define I2C_TIMEOUT_VALUE_DETECT       5
#define I2C_TRIALS                     2

//Handle values for I2C 
#ifdef I2C1
    static I2C_HandleTypeDef I2C1Handle = {I2C1};
#endif
#ifdef I2C2
    static I2C_HandleTypeDef I2C2Handle = {I2C2};
#endif
#ifdef I2C3
    static I2C_HandleTypeDef I2C3Handle = {I2C3};
#endif
#ifdef I2C4
    static I2C_HandleTypeDef I2C4Handle = {I2C4};
#endif

//---- Global Section ----------------------------------------------------------
//---- Enumeration/Typedef Section ---------------------------------------------
//---- Prototype Section -------------------------------------------------------

#ifdef I2C1
    static void I2C1_InitPins(sg_i2c_pinset_t pinset);
#endif
#ifdef I2C2
    static void I2C2_InitPins(sg_i2c_pinset_t pinset);
#endif
#ifdef I2C3
    static void I2C3_InitPins(sg_i2c_pinset_t pinset);
#endif
#ifdef I2C4
    static void I2C4_InitPins(sg_i2c_pinset_t pinset);
#endif

/**
 * @brief  Gets pointer to I2C handle structure for specific I2C
 * @param  *I2Cx: Pointer to I2Cx used for handle
 * @retval Pointer to I2C Handle structure
 */
static I2C_HandleTypeDef* GetHandle(I2C_TypeDef* I2Cx);

/**
 * @brief  fill the settings for the HAL init function
 * @param[out]  handle: I2C handle (can be obtainded by @ref GetHandle
 * @param[in]   speed:      clock speed in units of Hz for I2C communication
 * @retval none
 */
static void FillSettings(I2C_HandleTypeDef* handle, uint32_t speed);

//---- Private Function Section ------------------------------------------------
#ifdef I2C1
/**@brief   Init the pin, enable the clock for I2C1
 *
 * @param[in]   pinset:   Pin combination
 */
static void I2C1_InitPins(sg_i2c_pinset_t pinset) {
    #if defined(GPIOB)
        if (pinset == SG_I2C_PinSet_1) {
            //Port B, Pin 6/7
            SG_IO_InitAlternate(GPIOB, GPIO_PIN_6|GPIO_PIN_7, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C1);
        }
    #endif
    #if defined(GPIOB)
        if (pinset == SG_I2C_PinSet_2) {
            //Port B, Pin 8/9
            SG_IO_InitAlternate(GPIOB, GPIO_PIN_8|GPIO_PIN_9, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C1);
        }
    #endif
    #if defined(GPIOB)
        if (pinset == SG_I2C_PinSet_3) {
            //Port B, Pin 6/9
            SG_IO_InitAlternate(GPIOB, GPIO_PIN_6|GPIO_PIN_9, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C1);
        }
    #endif
    //Init pins 
	if (pinset == SG_I2C_PinSetCustom) {
		//Init custom pins, callback function
		SG_I2C_InitCustomPinsCallback(I2C1, GPIO_AF4_I2C1);
	}
} // end of 'I2C1_InitPins'
#endif

#ifdef I2C2
/**@brief   Init the pin, enable the clock for I2C2
 *
 * @param[in]   pinset:   Pin combination
 */
static void I2C2_InitPins(sg_i2c_pinset_t pinset) {
    #if defined(GPIOB)
        if (pinset == SG_I2C_PinSet_1) {
            //Port B, Pin 10/11
            SG_IO_InitAlternate(GPIOB, GPIO_PIN_10|GPIO_PIN_11, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C2);
        }
    #endif
    #if defined(GPIOF)
        if (pinset == SG_I2C_PinSet_2) {
            //Port F, Pin 1/0
            SG_IO_InitAlternate(GPIOF, GPIO_PIN_1|GPIO_PIN_0, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C2);
        }
    #endif
    #if defined(GPIOH)
        if (pinset == SG_I2C_PinSet_3) {
            //Port H, Pin 4/5
            SG_IO_InitAlternate(GPIOH, GPIO_PIN_4|GPIO_PIN_5, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C2);
        }
    #endif
    //Init pins 
	if (pinset == SG_I2C_PinSetCustom) {
		//Init custom pins, callback function
		SG_I2C_InitCustomPinsCallback(I2C2, GPIO_AF4_I2C2);
	}
} // end of 'I2C2_InitPins'
#endif

#ifdef I2C3
/**@brief   Init the pin, enable the clock for I2C3
 *
 * @param[in]   pinset:   Pin combination
 */
static void I2C3_InitPins(sg_i2c_pinset_t pinset) {;
    #if defined(GPIOA)
        if (pinset == SG_I2C_PinSet_1) {
            //Port A,C, Pin A8/C9
            SG_IO_InitAlternate(GPIOA, GPIO_PIN_8, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C3);
            SG_IO_InitAlternate(GPIOC, GPIO_PIN_9, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C3);           
        }
    #endif
    #if defined(GPIOH)
        if (pinset == SG_I2C_PinSet_2) {
            //Port H, Pin 7/8
            SG_IO_InitAlternate(GPIOH, GPIO_PIN_7|GPIO_PIN_8, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C3);
        }
    #endif
    //Init pins 
	if (pinset == SG_I2C_PinSetCustom) {
		//Init custom pins, callback function
		SG_I2C_InitCustomPinsCallback(I2C3, GPIO_AF4_I2C3);
	}
} // end of 'I2C3_InitPins'
#endif

#ifdef I2C4
/**@brief   Init the pin, enable the clock for I2C4
 *
 * @param[in]   pinset:   Pin combination
 */
static void I2C4_InitPins(sg_i2c_pinset_t pinset) {
    #if defined(GPIOD)
        if (pinset == SG_I2C_PinSet_1) {
            //Port D, Pin 12/13
            SG_IO_InitAlternate(GPIOD, GPIO_PIN_12|GPIO_PIN_13, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C4);
        }
    #endif
    #if defined(GPIOF)
        if (pinset == SG_I2C_PinSet_2) {
            //Port F, Pin 1/0
            SG_IO_InitAlternate(GPIOF, GPIO_PIN_1|GPIO_PIN_0, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C4);
        }
        if (pinset == SG_I2C_PinSet_3) {
            //Port F, Pin 14/15
            SG_IO_InitAlternate(GPIOF, GPIO_PIN_14|GPIO_PIN_15, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C4);
        }
    #endif
    #if defined(GPIOH)
        if (pinset == I2CLL_pinsetnation_4) {
            //Port H, Pin 11/12
            SG_IO_InitAlternate(GPIOH, GPIO_PIN_11|GPIO_PIN_12, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C4);
        }
    #endif
    #if defined(GPIOB) && defined(GPIOD)
    	if (pinset == SG_I2C_PinSet_5) {
        #if defined(GPIO_AF11_I2C4)
        		TM_GPIO_InitAlternate(GPIOB, GPIO_PIN_7, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF11_I2C4);
        #else
        		TM_GPIO_InitAlternate(GPIOB, GPIO_PIN_7, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C4);
        #endif
                TM_GPIO_InitAlternate(GPIOD, GPIO_PIN_12, SG_IO_OpenDrain, SG_IO_PullUp, SG_IO_Medium, GPIO_AF4_I2C4);
        	}    
    #endif
    //Init pins 
	if (pinset == SG_I2C_PinSetCustom) {
		//Init custom pins, callback function
		SG_I2C_InitCustomPinsCallback(I2C4, GPIO_AF4_I2C4);
	}    
} // end of 'I2C4_InitPins'
#endif

static I2C_HandleTypeDef* GetHandle(I2C_TypeDef* I2Cx) {
    #ifdef I2C1
    	if (I2Cx == I2C1) {
    		return &I2C1Handle;
    	}
    #endif
    #ifdef I2C2
        	if (I2Cx == I2C2) {
        		return &I2C2Handle;
        	}
    #endif
    #ifdef I2C3
    	if (I2Cx == I2C3) {
    		return &I2C3Handle;
    	}
    #endif
    #ifdef I2C4
    	if (I2Cx == I2C4) {
    		return &I2C4Handle;
    	}
    #endif
	//Return invalid
	return 0;
} // end of 'GetHandle'

static void FillSettings(I2C_HandleTypeDef* handle, uint32_t speed) {
    #if defined(STM32F7xx)
    	uint32_t timing;
    	
    	/* Future */
    	if (clock >= 3400000) {
    		/* 100kHz @ 50MHz APB clock */
    		timing = 0x40912732;
    	} else if (clock >= 1000000) {
    		/* 100kHz @ 50MHz APB clock */
    		timing = 0x40912732;
    	} else if (clock >= 400000) {
    		/* 100kHz @ 50MHz APB clock */
    		timing = 0x40912732;
    	} else {	
    		/* 100kHz @ 50MHz APB clock */
    		timing = 0x40912732;
    	}
    #endif
    #if defined(STM32F0xx)
    	/* 100kHz @ 48MHz APB clock */
    	uint32_t timing = 0x10805E89;
    #endif

	/* Fill settings */
	handle->Init.OwnAddress2 = 0x00;
	handle->Init.OwnAddress1 = 0x00;
	handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; 
    #if defined(STM32F7xx) || defined(STM32F0xx)
    	handle->Init.Timing = timing;
    #else
    	handle->Init.ClockSpeed = speed;
    	handle->Init.DutyCycle = I2C_DUTYCYCLE_2;
    #endif
} // end of 'FillSettings'

//---- Extern Function Section -------------------------------------------------
extern sg_i2c_result_t SG_I2C_Init(I2C_TypeDef* I2Cx, sg_i2c_pinset_t pinset, uint32_t speed) {
  I2C_HandleTypeDef* handle = GetHandle(I2Cx);
  
	handle->Instance = I2Cx;
	
    #ifdef I2C1
    	if (I2Cx == I2C1) {
    		__HAL_RCC_I2C1_CLK_ENABLE();
    		I2C1_InitPins(pinset);
    	}
    #endif
    #ifdef I2C2	
    	if (I2Cx == I2C2) {    		
    		__HAL_RCC_I2C2_CLK_ENABLE();    	
    		I2C2_InitPins(pinset);
    	} 
    #endif
    #ifdef I2C3
    	if (I2Cx == I2C3) {    		
    		__HAL_RCC_I2C3_CLK_ENABLE();
    		I2C3_InitPins(pinset);
    	}
    #endif
    #ifdef I2C4
    	if (I2Cx == I2C4) {    		
    		__HAL_RCC_I2C4_CLK_ENABLE();    		    		
    		I2C4_InitPins(pinset);
    	}
    #endif    	
    //Fill settings 
    FillSettings(handle, speed);	
    //Initialize I2C 
    if (HAL_I2C_Init(handle) != HAL_OK) {
        return SG_I2C_Error;
    }
    		
    //Enable analog filter
    #if defined(I2C_ANALOGFILTER_ENABLE)
    	if (HAL_I2CEx_ConfigAnalogFilter(handle, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    	    return SG_I2C_Error;
    	}
    #endif	
	return SG_I2C_Ok;
} // end of 'SG_I2C_Init'

extern sg_i2c_result_t SG_I2C_Read(I2C_TypeDef* I2Cx, uint8_t devadr, uint8_t regadr, uint8_t *data, uint16_t count) {
    I2C_HandleTypeDef* handle = GetHandle(I2Cx);
	
	//Send register address size = 1 byte
	if (HAL_I2C_Master_Transmit(handle, (uint16_t)devadr, &regadr, 1, I2C_TIMEOUT_VALUE) != HAL_OK) {
		return SG_I2C_Error;
	}
	
	// Receive multiple byte 
	if (HAL_I2C_Master_Receive(handle, devadr, data, count, I2C_TIMEOUT_VALUE) != HAL_OK) {	
		return SG_I2C_Error;
	}		
	return SG_I2C_Ok;
} // end of 'SG_I2C_Read'


extern sg_i2c_result_t SG_I2C_ReadNoRegister(I2C_TypeDef* I2Cx, uint8_t devadr, uint8_t *data, uint16_t count) {
    I2C_HandleTypeDef* handle = GetHandle(I2Cx);

	if (HAL_I2C_Master_Receive(handle, (uint16_t)devadr, data, count, I2C_TIMEOUT_VALUE) != HAL_OK) {
		return SG_I2C_Error;
	}
	return SG_I2C_Ok;
} // end of 'SG_I2C_ReadNoRegister'

extern sg_i2c_result_t SG_I2C_Read16(I2C_TypeDef* I2Cx, uint8_t devadr, uint16_t regadr, uint8_t *data) {
  uint8_t adr[2];
	I2C_HandleTypeDef* handle = GetHandle(I2Cx);
	
	// Format I2C address 
	adr[0] = (regadr >> 8) & 0xFF; /* High byte */
	adr[1] = (regadr) & 0xFF;      /* Low byte */
	
	// Send address 
	if (HAL_I2C_Master_Transmit(handle, (uint16_t)devadr, adr, 2, I2C_TIMEOUT_VALUE) != HAL_OK) {
		return SG_I2C_Error;
	}
	
	// Receive byte 
	if (HAL_I2C_Master_Receive(handle, devadr, data, 1, I2C_TIMEOUT_VALUE) != HAL_OK) {
		return SG_I2C_Error;
	}		
	return SG_I2C_Ok;
    //
} // end of 'SG_I2C_Read16'

extern sg_i2c_result_t SG_I2C_Write(I2C_TypeDef* I2Cx, uint8_t devadr, uint8_t regadr, uint8_t *data, uint16_t count) {
    I2C_HandleTypeDef* handle = GetHandle(I2Cx);
	
	if (HAL_I2C_Mem_Write(handle, devadr, regadr, (regadr > 0xFF) ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT, data, count, I2C_TIMEOUT_VALUE) != HAL_OK) {
		return SG_I2C_Error;
	}	
	// Return OK 
	return SG_I2C_Ok;
} // end of 'SG_I2C_Write'


extern sg_i2c_result_t SG_I2C_WriteNoRegister(I2C_TypeDef* I2Cx, uint8_t devadr, uint8_t *data, uint16_t count) {
    I2C_HandleTypeDef* handle = GetHandle(I2Cx);
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(handle, (uint16_t)devadr, data, count, I2C_TIMEOUT_VALUE) != HAL_OK) {
		return SG_I2C_Error;
	} 
	return SG_I2C_Ok;
} // end of 'SG_I2C_WriteNoRegister'

extern sg_i2c_result_t SG_I2C_Write16(I2C_TypeDef* I2Cx, uint8_t devadr, uint16_t regadr, uint8_t data) {
  I2C_HandleTypeDef* handle = GetHandle(I2Cx);
  uint8_t d[3];
		
	d[0] = (regadr >> 8) & 0xFF;    // High byte 
	d[1] = (regadr) & 0xFF;         // Low byte 
	d[2] = data;                    // Data byte 
	
	if (HAL_I2C_Master_Transmit(handle, (uint16_t)devadr, (uint8_t *)d, 3, I2C_TIMEOUT_VALUE) != HAL_OK) {
		return SG_I2C_Error;
	} 
	return SG_I2C_Ok;
} // end of 'SG_I2C_Write16'

extern sg_i2c_result_t SG_I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t devadr) {
    I2C_HandleTypeDef* handle = GetHandle(I2Cx);
	
    /* Check if device is ready for communication */
	if (HAL_I2C_IsDeviceReady(handle, devadr, I2C_TRIALS, I2C_TIMEOUT_VALUE_DETECT) != HAL_OK) {
		return SG_I2C_Error;
	}
	return SG_I2C_Ok;
} // end of 'SG_I2C_IsDeviceConnected'

__weak void SG_I2C_InitCustomPinsCallback(I2C_TypeDef* I2Cx, uint16_t alternate) {
    //Custom user function.
	//In case user needs functionality for custom pins, this function should be declared outside this library 
} // end of 'SG_I2C_InitCustomPinsCallback'



extern sg_i2c_result_t SG_I2C_WriteReadRepeatedStart(I2C_TypeDef* I2Cx, uint8_t devadr, uint8_t wregadr, uint8_t* wdata, uint16_t wcount, uint8_t rregadr, uint8_t* rdata, uint16_t rcount) {
    I2C_HandleTypeDef* handle = GetHandle(I2Cx);
    
    //Write command to device
	if (HAL_I2C_Mem_Write(handle, devadr, wregadr, I2C_MEMADD_SIZE_8BIT, wdata, wcount, I2C_TIMEOUT_VALUE) != HAL_OK) {	
		return SG_I2C_Error;
	}
	
	//Read data from controller
	if (HAL_I2C_Mem_Read(handle, devadr, rregadr, I2C_MEMADD_SIZE_8BIT, rdata, rcount, I2C_TIMEOUT_VALUE) != HAL_OK) {		
		return SG_I2C_Error;
	}
	return SG_I2C_Ok;    
} // end of 'SG_I2C_WriteReadRepeatedStart'
 
/* END OF FILE */
