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

#ifndef SG_IO_H
#define SG_IO_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif 
 
/**
 * @addtogroup GPIO   GPIO
 * @{
 *
 * \par Description
 * Library containing functions to initialize GPIO as input, output, analog or alternate as well as basic input and output methods. 
 * Library also provides an easy way to enable external interrupt on a specific pin. You only need to specify GPIOx, pin and interrupt 
 * trigger you want to use and you are ready to use. There are predefined function names that you use to handle this interrupts.
 * Library is a faster and full replacement for the HAL functions.
 *
 * \par External Interrupts
 * This library allows the use of external interrupts on specific pins. 
 * There are predefined function names that you use to handle this interrupts.
 * 
 * There are 16 external interrupts possible, GPIO lines 0 to 15.
 * Each GPIO_PIN_x from all GPIOx is connected to one line. 
 * PC0, PA0, PB0, PJ0,... are all connected to line 0, and so on. 
 * But you can use only one pin for one line at a time. 
 * So only PA0 at one time, or PD0 or PC0, but only one a time.
 *
 * You can still use more lines at the same time. So let's say PA0 is line0 and PC13 is line13.
 * This 2 interrupts can be used simultaneously without problems.
 * 
 * You can still use more lines at the same time. 
 * So let's say PA0 is line0 and PC13 is line13.
 * This 2 interrupts can be used simultaneously without problems.
 *
 * \par Interrupt handling
 * There are 16 interrupt lines, but only 7 interrupt handlers.
 * Lines 0 to 4 have each own handler, then lines 5 to 9 have one common
 * and lines 10 to 15 have one common. Please refere to reference sheet.
 * 
 * There is a common, but as weak defined IRQ handler in this file.
 * Sometimes there is need for use default handler names in your own, 
 * but 2 functions with same name can not be used, so you can disable the IRQ 
 * handler used in this lib by adding on ore more of the following
 * lines to the bsh.h file. In this case you must implement your own IRQ handler in your code.
 * 
 * @warning By default, all handlers for lines are enabled. Disabled them with adding lines below in bsp.h file:
 
\code
//These works for STM32F4xx and STM32F7xx series
//Disable EXTI0_IRQHandler function
#define EXTI_DISABLE_DEFAULT_HANDLER_0
//Disable EXTI1_IRQHandler function
#define EXTI_DISABLE_DEFAULT_HANDLER_1
//Disable EXTI2_IRQHandler function
#define EXTI_DISABLE_DEFAULT_HANDLER_2
//Disable EXTI3_IRQHandler function
#define EXTI_DISABLE_DEFAULT_HANDLER_3
//Disable EXTI4_IRQHandler function
#define EXTI_DISABLE_DEFAULT_HANDLER_4
//Disable EXTI9_5_IRQHandler function
#define EXTI_DISABLE_DEFAULT_HANDLER_9_5
//Disable EXTI15_10_IRQHandler function
#define EXTI_DISABLE_DEFAULT_HANDLER_15_10

//These works for STM32F0xx series
//Disable EXTI0_1_IRQHandler function
#define EXTI_DISABLE_DEFAULT_HANDLER_0_1
//Disable EXTI2_3_IRQHandler function
#define EXTI_DISABLE_DEFAULT_HANDLER_2_3
//Disable EXTI4_15_IRQHandler function
#define EXTI_DISABLE_DEFAULT_HANDLER_4_15
\endcode
 *
 * @note If you need higher priority for external interrupts in NVIC, add lines below 
 * in your bsp.h file
\code
//Set custom NVIC priority
#define EXTI_NVIC_PRIORITY      0x04
\endcode
 * \par Dependencies
 *
 *  - sg_lib_conf.h
 *  - bsp.h
 *
 * \todo Test on several boards
 */     

//---- Include Section ---------------------------------------------------------
#include "bsp.h" 
#include "sg_lib_conf.h"    

//---- Define Section ----------------------------------------------------------

/**
 * @addtogroup  IO_MACROS Macro Definitions
 * @brief       Macros used throughout the library. 
 *
 * These macros are used for basic IO functionality due to speed reasons.
 *
 * @{
 */

/**
 * @brief   Default EXTI preemption priority for EXTI lines used in NVIC
 * @note    If a higher priority is needed to note overwrite the value here,
 *          since this is a common uses library. Add somewhere in the project
 *          (best choice would be the board supply package) e.g.
 *          the following line:
 *          #define EXTI_NVIC_PRIORITY      0x04    
 */  
#ifndef EXTI_NVIC_PRIORITY
    #define EXTI_NVIC_PRIORITY     0x03
#endif

/**
 * @brief  Sets pin(s) low
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set pin low
 * @param  pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them low
 * @retval None
 */
#define SG_IO_SetPinLow(GPIOx, pin)			((GPIOx)->BSRR = (uint32_t)(((uint32_t)pin) << 16))

/**
 * @brief  Sets pin(s) high
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set pin high
 * @param  pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them high
 * @retval None
 */
#define SG_IO_SetPinHigh(GPIOx, pin)			((GPIOx)->BSRR = (uint32_t)(pin))

/**
 * @brief  Sets pin(s) value
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set pin value
 * @param  pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them value
 * @param  val: If parameter is 0 then pin will be low, otherwise high
 * @retval None
 */
#define SG_IO_SetPinValue(GPIOx, pin, val)	((val) ? SG_IO_SetPinHigh(GPIOx, pin) : SG_IO_SetPinLow(GPIOx, pin))

/**
 * @brief  Toggles pin(s)
 * Defined as macro to get maximum speed using register access.
 * With this instruction a interrupt which occurs between 
 * reading ODR and setting BSSR cannot affect the toggle 
 * function. BitSet has a higher priority than BitReset
 * (look at e.g. rm0008.pdf, page 168). 
 * The instruction GPIOA->ODR ^= GPIO_PIN_0 will not be IRQ safe (but might be faster)
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to toggle pin value
 * @param  pin: Select GPIO pin(s). You can select more pins with | (OR) operator to toggle them all at a time
 * @retval None
 */
#define SG_IO_PinToggle(GPIOx, pin)    ((GPIOx)->BSRR = (((GPIOx)->ODR ^ (uint32_t)(pin)) & (uint32_t)(pin))|(((uint32_t)(pin)) << 16));

/**
 * @brief  Sets value to entire GPIO PORT
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set value
 * @param  value: Value for GPIO OUTPUT data
 * @retval None
 */
#define SG_IO_SetPortValue(GPIOx, value)			((GPIOx)->ODR = (value))

/**
 * @brief  Gets input data bit
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to read input bit value
 * @param  pin: GPIO pin where you want to read value
 * @retval 1 in case pin is high, or 0 if low
 */
#define SG_IO_GetInputPinValue(GPIOx, pin)	(((GPIOx)->IDR & (pin)) == 0 ? 0 : 1)

/**
 * @brief  Gets output data bit
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to read output bit value
 * @param  pin: GPIO pin where you want to read value
 * @retval 1 in case pin is high, or 0 if low
 */
#define SG_IO_GetOutputPinValue(GPIOx, pin)	(((GPIOx)->ODR & (pin)) == 0 ? 0 : 1)

/**
 * @brief  Gets input value from entire GPIO PORT
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to read input data value
 * @retval Entire PORT INPUT register
 */
#define SG_IO_GetPortInputValue(GPIOx)			((GPIOx)->IDR)

/**
 * @brief  Gets output value from entire GPIO PORT
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to read output data value
 * @retval Entire PORT OUTPUT register
 */
#define SG_IO_GetPortOutputValue(GPIOx)			((GPIOx)->ODR)

/**
 * @brief   Creates software interrupt for specific external GPIO line
 * @note    This also works for others EXTI lines from 16 to 23
 * @note    Defined as macro for faster execution
 *
 * @param[in]   GPIO_Line:  GPIO line where you want software interrupt
 *
 */
#define SG_IO_CreateSoftwareInterrupt(GPIO_Line)	    (EXTI->SWIER |= (GPIO_Line))
/**
 * @}
 */

//---- Global Section ----------------------------------------------------------

//---- Enumeration/Typedef Section ---------------------------------------------

/**
 * @addtogroup IO_Typedefs  Type definitions and enumerations
 * @brief    Enumerations of pin combination and results  
 * @{
 */

/**
 * @brief GPIO Mode enumeration
 */
typedef enum {
	SG_IO_Input       = 0x00,    /*!< GPIO Pin as General Purpose Input */
	SG_IO_Output      = 0x01,    /*!< GPIO Pin as General Purpose Output */
	SG_IO_Alternate   = 0x02,    /*!< GPIO Pin as Alternate Function */
	SG_IO_Analog      = 0x03,    /*!< GPIO Pin as Analog input/output */
} sg_io_mode_t;

/**
 * @brief GPIO Output type enumeration
 */
typedef enum {
	SG_IO_PushPull    = 0x00, /*!< GPIO Output Type Push-Pull */
	SG_IO_OpenDrain   = 0x01  /*!< GPIO Output Type Open-Drain */
} sg_io_otype_t;

/**
 * @brief  GPIO Speed enumeration
 */
typedef enum {
	SG_IO_Low     = 0x00,     /*!< GPIO Speed Low */
	SG_IO_Medium  = 0x01,     /*!< GPIO Speed Medium */
	SG_IO_Fast    = 0x02,     /*!< GPIO Speed Fast, not available on STM32F0xx devices */
	SG_IO_High    = 0x03      /*!< GPIO Speed High */
} sg_io_speed_t;

/**
 * @brief GPIO pull resistors enumeration
 */
typedef enum {
	SG_IO_NoPullUpOrDown  = 0x00,     /*!< No pull resistor */
	SG_IO_PullUp          = 0x01,     /*!< Pull up resistor enabled */
	SG_IO_PullDown        = 0x02      /*!< Pull down resistor enabled */
} sg_io_pupd_t;

/**
 * @brief  Interrupt trigger enumeration	
 */
typedef enum {
	SG_IO_RisingEdge      = 0x00,     /*!< Trigger interrupt on rising edge on line, pull down resistor active */
	SG_IO_FallingEdge,                /*!< Trigger interrupt on falling edge on line, pull up resistor active */
	SG_IO_AnyEdge                     /*!< Trigger interrupt on any edge on line, no pull resistor active */
} sg_io_trigger_t;

/**
 * @brief  Result enumeration
 */
typedef enum {
	SG_IO_Ok      = 0x00,     /*!< Everything ok */
	SG_IO_Error               /*!< An error has occured */
} sg_io_result_t;

/**
 * @}
 */
 
//---- Prototype Section -------------------------------------------------------

/**
 * @addtogroup IO_Functions     Exported functions
 * @brief    Exported functions to use the GPIO and EXTI
 * @{
 */

/**
 * @brief  Initializes GPIO pins(s)
 * @note   This function also enables clock for GPIO port
 * @param  GPIOx: Pointer to GPIOx port you will use for initialization
 * @param  pin: GPIO pin(s) you will use for initialization (eg. GPIO_PIN_0)
 * @param  mode: Select GPIO mode. This parameter can be a value of @ref sg_io_mode_t enumeration
 * @param  otype: Select GPIO Output type. This parameter can be a value of @ref sg_io_otype_t enumeration
 * @param  pupd: Select GPIO pull resistor. This parameter can be a value of @ref sg_io_pupd_t enumeration
 * @param  speed: Select GPIO speed. This parameter can be a value of @ref sg_io_speed_t enumeration
 * @retval None
 */
void SG_IO_Init(GPIO_TypeDef* GPIOx, uint16_t pin, sg_io_mode_t mode, sg_io_otype_t otype, sg_io_pupd_t pupd, sg_io_speed_t speed);

/**
 * @brief  Initializes GPIO pins(s) as alternate function
 * @note   This function also enables clock for GPIO port
 * @param  GPIOx: Pointer to GPIOx port you will use for initialization
 * @param  pin: GPIO pin(s) you will use for initialization (eg. GPIO_PIN_0)
 * @param  otype: Select GPIO Output type. This parameter can be a value of @ref sg_io_otype_t enumeration
 * @param  pupd: Select GPIO pull resistor. This parameter can be a value of @ref sg_io_pupd_t enumeration
 * @param  speed: Select GPIO speed. This parameter can be a value of @ref sg_io_speed_t enumeration
 * @param  alternate: Alternate function you will use - look at datasheet for values
 * @retval None
 */
void SG_IO_InitAlternate(GPIO_TypeDef* GPIOx, uint16_t pin, sg_io_otype_t otype, sg_io_pupd_t pupd, sg_io_speed_t speed, uint8_t alternate);

/**
 * @brief  Deinitializes pin(s)
 * @note   Pins(s) will be set as analog mode to get low power consumption
 * @param  GPIOx: GPIOx PORT where you want to set pin as input
 * @param  pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them as input
 * @retval None
 */
void SG_IO_DeInit(GPIO_TypeDef* GPIOx, uint16_t pin);

/**
 * @brief  Sets pin(s) as input 
 * @note   Pins HAVE to be initialized first using @ref SG_IO_Init() or @ref SG_IO_InitAlternate() function
 * @note   This is just an option for fast input mode
 * @param  GPIOx: GPIOx PORT where you want to set pin as input
 * @param  pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them as input
 * @retval None
 */
void SG_IO_SetPinAsInput(GPIO_TypeDef* GPIOx, uint16_t pin);

/**
 * @brief  Sets pin(s) as output
 * @note   Pins HAVE to be initialized first using @ref SG_IO_Init() or @ref SG_IO_InitAlternate() function
 * @note   This is just an option for fast output mode 
 * @param  GPIOx: GPIOx PORT where you want to set pin as output
 * @param  pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them as output
 * @retval None
 */
void SG_IO_SetPinAsOutput(GPIO_TypeDef* GPIOx, uint16_t pin);

/**
 * @brief  Sets pin(s) as analog
 * @note   Pins HAVE to be initialized first using @ref SG_IO_Init() or @ref SG_IO_InitAlternate() function
 * @note   This is just an option for fast analog mode 
 * @param  GPIOx: GPIOx PORT where you want to set pin as analog
 * @param  pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them as analog
 * @retval None
 */
void SG_IO_SetPinAsAnalog(GPIO_TypeDef* GPIOx, uint16_t pin);

/** 
 * @brief  Sets pin(s) as alternate function
 * @note   For proper alternate function, you should first init pin using @ref SG_IO_InitAlternate() function.
 *            This functions is only used for changing GPIO mode
 * @param  GPIOx: GPIOx PORT where you want to set pin as alternate
 * @param  pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them as alternate
 * @retval None
 */
void SG_IO_SetPinAsAlternate(GPIO_TypeDef* GPIOx, uint16_t pin);

/**
 * @brief  Sets pull resistor settings to GPIO pin(s)
 * @note   Pins HAVE to be initialized first using @ref SG_IO_Init() or @ref SG_IO_InitAlternate() function
 * @param  *GPIOx: GPIOx PORT where you want to select pull resistor
 * @param  pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them as output
 * @param  pupd: Pull resistor option. This parameter can be a value of @ref sg_io_pupd_t enumeration
 * @retval None
 */
void SG_IO_SetPullResistor(GPIO_TypeDef* GPIOx, uint16_t pin, sg_io_pupd_t pupd);


/**
 * @brief  Gets port source from desired GPIOx PORT
 * @note   Meant for private use, unless you know what are you doing
 * @param  GPIOx: GPIO PORT for calculating port source
 * @retval Calculated port source for GPIO
 */
uint16_t SG_IO_GetPortSource(GPIO_TypeDef* GPIOx);

/**
 * @brief  Gets pin source from desired GPIO pin
 * @note   Meant for private use, unless you know what are you doing
 * @param  pin: GPIO pin for calculating port source
 * @retval Calculated pin source for GPIO pin
 */
uint16_t SG_IO_GetPinSource(uint16_t pin);

/**
 * @brief  Locks GPIOx register for future changes
 * @note   You are not able to config GPIO registers until new MCU reset occurs
 * @param  *GPIOx: GPIOx PORT where you want to lock config registers
 * @param  pin: GPIO pin(s) where you want to lock config registers
 * @retval None
 */
void SG_IO_Lock(GPIO_TypeDef* GPIOx, uint16_t pin);

/** 
 * @brief  Gets bit separated pins which were used at least once in library and were not deinitialized
 * @param  *GPIOx: Pointer to GPIOx peripheral where to check used GPIO pins
 * @retval Bit values for used pins
 */
uint16_t SG_IO_GetUsedPins(GPIO_TypeDef* GPIOx);

/** 
 * @brief  Gets bit separated pins which were not used at in library or were deinitialized
 * @param  *GPIOx: Pointer to GPIOx peripheral where to check used GPIO pins
 * @retval Bit values for free pins
 */
uint16_t SG_IO_GetFreePins(GPIO_TypeDef* GPIOx); 

/**@brief   Attach external interrupt on specific GPIO pin 
 * @note    This function automatically enables the clock for GPIO peripheral 
 *          and also sets pull resistors depending on trigger you use.
 *          - Falling edge: pull up is enabled
 *          - Rising edge:  pull down is enabled
 * 	        - Any edge:     no pull activated
 * @note    You can attach only one GPIOx to specific GPIO_PIN.
 *          In other words, GPIO_PIN_5 can not be attached to GPIOA and GPIOB 
 *          at the same time. In case of an try, the function will return
 *          error, because you have to detach GPIO_Line first and attach back 
 *          on other GPIO port.
 * @note    If you use more than one GPIO_Pin with OR (|) operator at single call 
 *          and if GPIO_Pin can't be attached because there is already one 
 *          GPIO_Pin at this line, function will return error and other pins 
 *          might not be initialized.
 * @note    If function return @arg IRQ_Ok, then all pins are attached correctly.
 *
 * @param[in]   *GPIOx:     GPIO port where you want EXTI interrupt line
 * @param[in]   line:  GPIO pin where you want EXTI interrupt line. 
 *                          Use OR (|) operator if you want to attach
 *                          interrupt on more than one GPIO pin at the same 
 *                          GPIOx at the same time.
 * @param[in]   trigger:    Pin trigger source. This parameter can be a value 
 *                          of @ref sg_io_trigger_t enumeration
 * @return      Attach result which can be a value of @ref sg_io_result_t enumeration
 *
 */
extern sg_io_result_t SG_IO_IRQAttach(GPIO_TypeDef* GPIOx, uint16_t line, sg_io_trigger_t trigger);

/**@brief   Detach GPIO pin from interrupt lines
 *
 * @param[in]   line:  GPIO line you want to disable. 
 *                          Valid GPIO is GPIO_Pin_0 to GPIO_Pin_15. 
 *                          Use OR (|) operator if you want to detach 
 *                          interrupt in more than one GPIO pin at the 
 *                          same GPIOx at the same time.
 * @return      Attach result which can be a value of @ref sg_io_result_t enumeration
 *
 */
extern sg_io_result_t SG_IO_IRQDetach(uint16_t line);

/**@brief   Clears all interrupts on EXTI line
 * @note    It clears bits for external pins (bit 0 to bit 15) only!
 *          It has no effect for internally connected peripherals 
 *          (like RTC) to EXTI line.
 *
 */
extern void SG_IO_IRQDeInit(void);

/**@brief   EXTI Global handler
 * @note    This function is called from library each time any 
 *          interrupt occurs on EXTI line.
 * @note    With __weak parameter to prevent link errors if not 
 *          defined by user.
 *
 * @param[in]   line:   GPIO Line where interrupt occurred so you can identify what to do
 *
 */
extern void SG_IO_IRQGeneralHandler(uint16_t line);
/**
 * @}
 */

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
