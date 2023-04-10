/**
 * @author  Bernhard Gleich
 * @version v1.0
 *
 * \par Changelog
 * 03-31-21     V1.0        Bernhard Gleich
 */
 
/**
 * @addtogroup CONNECTIVITY   Connectivity
 * @{
 *
 */   

#ifndef SG_I2C_H
#define SG_I2C_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif 
 
/**
 * @addtogroup I2C   I2C
 * @{
 *
 * \par Description
 * I2C Communication with the SSG32F microcontollers.
 *
 * \par Pinout
 *<table>
 * <tr>
 *   <th rowspan="2">I2Cx</th>
 *   <th colspan="2">Pin Set 1<br></th>
 *   <th colspan="2">Pin Set 2<br></th>
 *   <th colspan="2">Pin Set 3<br></th>
 *   <th colspan="2">Pin Set 4<br></th>
 *   <th colspan="2">Pin Set 5<br></th>
 * </tr>
 * <tr>
 *   <td><b>SCL</b></td>
 *   <td><b>SDA</b></td>
 *   <td><b>SCL</b></td>
 *   <td><b>SDA</b></td>
 *   <td><b>SCL</b></td>
 *   <td><b>SDA</b></td>
 *   <td><b>SCL</b></td>
 *   <td><b>SDA</b></td>
 *   <td><b>SCL</b></td>
 *   <td><b>SDA</b></td> 
 * </tr>
 * <tr>
 *   <td><b>I2C1</b><br></td>
 *   <td>PB6</td>
 *   <td>PB7</td>
 *   <td>PB8</td>
 *   <td>PB9</td>
 *   <td>PB6</td>
 *   <td>PB9<br></td>
 *   <td>n.a.</td>
 *   <td>n.a.</td>
 *   <td>n.a.</td>
 *   <td>n.a.</td> 
 * </tr>
 * <tr>
 *   <td><b>I2C2</b></td>
 *   <td>PB10</td>
 *   <td>PB11</td>
 *   <td>PF1</td>
 *   <td>PF0</td>
 *   <td>PH4</td>
 *   <td>PH5</td>
 *   <td>n.a.</td>
 *   <td>n.a.</td>
 *   <td>n.a.</td>
 *   <td>n.a.</td> 
 * </tr>
 * <tr>
 *   <td><b>I2C3</b></td>
 *   <td>PA8</td>
 *   <td>PC9</td>
 *   <td>PH7</td>
 *   <td>PH8</td>
 *   <td>n.a.</td>
 *   <td>n.a.</td>
 *   <td>n.a.</td>
 *   <td>n.a.</td>
 *   <td>n.a.</td>
 *   <td>n.a.</td> 
 * </tr>
 * <tr>
 *   <td><b>I2C4</b></td>
 *   <td>PD12<br></td>
 *   <td>PD13</td>
 *   <td>PF1</td>
 *   <td>PF0</td>
 *   <td>PF14</td>
 *   <td>PF15</td>
 *   <td>PH11</td>
 *   <td>PH12</td>
 *   <td>PD12</td>
 *   <td>PB7</td>
 * </tr>
 *</table>
 *
 * @warning
 * I2C4 is not available on all devices. Please check if it is available for the 
 * device before using it!
 *
 * @note
 * All pins are set to OpenDrain, Alternate Function with PullUp 
 *
 * \par Custom pinout
 *
 * You can use other combinations with @ref SG_I2C_PinsSetCustom in @ref SG_I2C_Init() function and callback function will be called,
 * where you can initialize your custom pinout for your I2C peripheral
 *
 * \par Dependencies
 *
 *  - sg_lib_conf.h
 *  - sg_io.h
 *  - bsp.h
 *
 * \todo Test on several boards
 */  
     

//---- Include Section ---------------------------------------------------------
#include "bsp.h"     
#include "sg_lib_conf.h"
#include "sg_io.h"

//---- Define Section ----------------------------------------------------------

/**
 * @defgroup    SG_I2C_MACROS Macro Definitions
 * @brief       Macros used throughout the library. 
 *
 * These macros define the standard clock speed of the I2C bus. Standards are
 * 100 kHz, 400 kHz, 1 MHz, 3.4 MHz
 *
 * @{
 */
#define SG_I2C_CLOCK_STANDARD        100000  /*!< I2C standard speed 100 kHz */
#define SG_I2C_CLOCK_FAST            400000  /*!< I2C fast speed  400 kHz */
#define SG_I2C_CLOCK_FAST_PLUS       1000000 /*!< I2C fast plus speed 1 MHz */
#define SG_I2C_CLOCK_HIGH_SPEED      3400000 /*!< I2C high speed 3.4 MHz */

/**
 * @}
 */

//---- Global Section ----------------------------------------------------------

//---- Enumeration/Typedef Section ---------------------------------------------
/**
 * @defgroup    SG_I2C_TYPEDEF  Type definitions and enumerations
 * @brief       Enumerations of pin combination and results
 * 
 * Enumeration are used for the definitions of the pin combinations and 
 * for error signaling.
 *
 * @{
 */
 
/**
 * @brief  Definition of possible I2C pin combinations
 */ 
typedef enum {
    SG_I2C_PinSet_1  = 0x00,         /*!< see pinout table for details */
    SG_I2C_PinSet_2,                 /*!< see pinout table for details */
    SG_I2C_PinSet_3,                 /*!< see pinout table for details */
    SG_I2C_PinSet_4,                  /*!< see pinout table for details */
    SG_I2C_PinSetCustom              /*!< Use custom pins for I2Cx */
} sg_i2c_pinset_t;


 /**
 * @brief  Enumeration for errors and results
 */ 
typedef enum {
    SG_I2C_Ok = 0x00,                        /*!< Everything OK */
    SG_I2C_Error                             /*!< An error has occurred */
} sg_i2c_result_t;

/**
 * @}
 */


//---- Prototype Section -------------------------------------------------------

/**
 * @defgroup    SG_I2C_FUNCTIONS     Exported functions
 * @brief       Exported functions to use the I2C bus
 * 
 * In this version the functions only provide a direct peripherial communication
 * without the uses of the DMA.
 *
 * The I2C peripherial must be initialized properly before using. For 
 * initialization the choide of the proper pin combination is essential.
 *
 * The functions uses the SSG32F4 HAL library provided by SSG.
 * @{
 */

/**@brief   Init the I2C peripherial (including GPIO Port, Clock)
 *
 * @param[in]   *I2Cx:      pointer to I2Cx peripheral used for intialization
 * @param[in]   PinCombi:   pin used for I2C. Value of @ref SG_I2C_PinCombinations
 * @param[in]   speed:      clock speed in units of Hz for I2C communication
 * @return      SG_I2C_Ok if successfull, SG_I2C_Error on error
 */ 
extern sg_i2c_result_t SG_I2C_Init(I2C_TypeDef* I2Cx, sg_i2c_pinset_t pinset, uint32_t speed);

/**@brief   Reads multiple bytes from device
 *
 * @param[in]   *I2Cx:  pointer to I2Cx peripheral used for intialization
 * @param[in]   devadr: 7-bit, left aligned device address used for communication
 * @param[in]   regadr: Register address from where read operation will start
 * @param[out]  *data:  Pointer to variable where data will be stored from read operation
 * @param[in]   count:  Number of elements to read from device
 * @return      SG_I2C_Ok if successfull, SG_I2C_Error on error
 */
extern sg_i2c_result_t SG_I2C_Read(I2C_TypeDef* I2Cx, uint8_t devadr, uint8_t regadr, uint8_t *data, uint16_t count);


/**@brief   Reads multiple bytes from device without specifying register address
 *
 * @param[in]   *I2Cx:  pointer to I2Cx peripheral used for intialization
 * @param[in]   devadr: 7-bit, left aligned device address used for communication
 * @param[out]  *data:  Pointer to variable where data will be stored from read operation
 * @param[in]   count:  Number of elements to read from device
 * @return      SG_I2C_Ok if successfull, SG_I2C_Error on error
 */
extern sg_i2c_result_t SG_I2C_ReadNoRegister(I2C_TypeDef* I2Cx, uint8_t devadr, uint8_t *data, uint16_t count);

/**@brief   Reads single byte from device with 16-bit register address
 *
 * @param[in]   *I2Cx:  pointer to I2Cx peripheral used for intialization
 * @param[in]   devadr: 7-bit, left aligned device address used for communication
 * @param[out]  *data:  Pointer to variable where data is stored 
 * @param[in]   regadr: Register address from where read operation will start
 * @return      SG_I2C_Ok if successfull, SG_I2C_Error on error
 */
extern sg_i2c_result_t SG_I2C_Read16(I2C_TypeDef* I2Cx, uint8_t devadr, uint16_t regadr, uint8_t *data);

/**@brief   Writes multiple bytes to the device
 *
 * @param[in]   *I2Cx:  pointer to I2Cx peripheral used for intialization
 * @param[in]   devadr: 7-bit, left aligned device address used for communication
 * @param[in]   regadr: Register address from where read operation will start
 * @param[out]  *data:  Pointer to variable where data is stored 
 * @param[in]   count:  Number of elements to write to the device
 * @return      SG_I2C_Ok if successfull, SG_I2C_Error on error
 */
extern sg_i2c_result_t SG_I2C_Write(I2C_TypeDef* I2Cx, uint8_t devadr, uint8_t regadr, uint8_t *data, uint16_t count);


/**@brief   Writes multiple bytes to the device without specifying 
 *          register address
 *
 * @param[in]   *I2Cx:  pointer to I2Cx peripheral used for intialization
 * @param[in]   devadr: 7-bit, left aligned device address used for communication
 * @param[out]  *data:  Pointer to variable where data is stored 
 * @param[in]   count:  Number of elements to write to the device
 * @return      SG_I2C_Ok if successfull, SG_I2C_Error on error
 */
extern sg_i2c_result_t SG_I2C_WriteNoRegister(I2C_TypeDef* I2Cx, uint8_t devadr, uint8_t *data, uint16_t count);



/**@brief   Write single byte to the device with 16-bit register address
 *
 * @param[in]   *I2Cx:  pointer to I2Cx peripheral used for intialization
 * @param[in]   devadr: 7-bit, left aligned device address used for communication
 * @param[out]  *data:  Pointer to variable where data is stored 
 * @param[in]   regadr: Register address from where read operation will start
 * @return      SG_I2C_Ok if successfull, SG_I2C_Error on error
 */
extern sg_i2c_result_t SG_I2C_Write16(I2C_TypeDef* I2Cx, uint8_t devadr, uint16_t regadr, uint8_t data);

/**
 * @brief  Checks if device is connected to I2C port and ready to use
 * @param  *I2Cx: Pointer to I2Cx peripheral to be used in communication
 * @param  devadr: 7-bit, left aligned device address used for communication
 * @retval Member of @ref SG_I2C_Result_t enumeration
 */
extern sg_i2c_result_t SG_I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t devadr);

/**
 * @brief  Callback for custom pins initialization.
 * 
 *         When you call SG_I2C_Init() function, and if you pass SG_I2C_PinsPack_Custom to function,
 *         then this function will be called where you can initialize custom pins for I2C peripheral.
 * @param  *I2Cx: I2C for which initialization will be set
 * @param  alternate: Alternate function which should be used for GPIO initialization
 * @retval None
 * @note   With __weak parameter to prevent link errors if not defined by user
 */
void SG_I2C_InitCustomPinsCallback(I2C_TypeDef* I2Cx, uint16_t alternate);

/**
 * @brief  Writes and receives amount of data via I2C using repeated start condition
 * @param  *I2Cx: Pointer to I2Cx peripheral to be used in communication
 * @param  devadr: 7-bit, left aligned device address used for communication
 * @param  wregadr: Register address to start writing to
 * @param  *wdata: Pointer to data array where data for write are stored
 * @param  wcount: Number of elements to write
 * @param  rregadr: Register address where reading will start
 * @param  *rdata: Pointer to array where data will be saved
 * @param  rcount: Number of elements to read
 * @retval Member of @ref SG_I2C_Result_t enumeration
 */
extern sg_i2c_result_t SG_I2C_WriteReadRepeatedStart(
	I2C_TypeDef* I2Cx,
	uint8_t devadr, 
	uint8_t wregadr, 
	uint8_t* wdata,
	uint16_t wcount, 
	uint8_t rregadr, 
	uint8_t* rdata,
	uint16_t rcount
);


//addtogroup 
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
