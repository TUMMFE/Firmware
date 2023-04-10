/**
 * @author  Bernhard Gleich
 * @version v1.0
 *
 * \par Changelog
 * 09/03/23     V1.0        Bernhard Gleich - start of development
 */
 
/**
 * @addtogroup PER   System Peripherials
 * @{
 *
 */ 

/**
 * @addtogroup ADXL355   Implementation of ADXL355
 * @{
 *
 * \par Description
 * This headerfile contains the methods to control the ADXL355. 
 * The ADXL355 is a low noise, low drift, low power, 3-Axis MEMS 
 * accelerometer with an integrated 20-bit sigma-delta ADC. The device
 * can be connected to the host microcontroller via SPI or I2C. 
 * This file can be used with SPI only. 
 *
 * \bug
 *      
 *
 * \todo
 *     
 *
 * \copyright
 * Copyright (c) 2023 Munich Insitute of Biomedical Engineering,
 * Technische Universitaet Muenchen,
 * Boltzmannstrasse 11, 85748 Garching 
 *
 * <b>All rights reserved.</b>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADXL355_H
#define __ADXL355_H

#ifdef __cplusplus
 extern "C" {
#endif

//---- Include Section ---------------------------------------------------------
#include <stdint.h>
#include <stdint.h>
#include <stdbool.h>
#include "bsp.h"
#include "lwrb.h"


//---- Define Section ----------------------------------------------------------
/**
 * @addtogroup ADXL355_DEFINES  Definitions
 * @brief    Register and configuration defines
 * @{
 */


/**
  * @brief Bit Stuffing macros
  */
#define ADXL355_ADDR(x)			      ((x) & 0xFF)              /*!< Get the register adress from register map */
#define ADXL355_GET_TRANSF_LEN(x) (((x) >>  8) & 0x0000FF)  /*!< Get the transfer data length from register map */
#define ADXL355_SET_TRANSF_LEN(x) (((x) <<  8) & 0x00FF00)  /*!< Set the transfer data length from register map */
#define ADXL355_GET_RESET_VAL(x)  (((x) >> 16) & 0x0000FF)  /*!< Get the reset value from register map */
#define ADXL355_SET_RESET_VAL(x)  (((x) << 16) & 0xFF0000)  /*!< Set the reset value from register map */
      

/**
  * @}
  */


/**
  * @brief Register map of the ADXL355
  */
#define ADXL355_DEVID_AD      (ADXL355_ADDR(0x00) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0xAD))  /*!< This register contains the Analog Devices ID */
#define ADXL355_DEVID_MST 	  (ADXL355_ADDR(0x01) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x1D))  /*!< This register contains the Analog Devices MEMS ID */ 
#define ADXL355_PARTID        (ADXL355_ADDR(0x02) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0xED))  /*!< This register contains the device ID */    
#define ADXL355_REVID 		    (ADXL355_ADDR(0x03) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x01))  /*!< This register contains the product revision ID, beginning with 0x00 and incrementing for each subsequent revision. */
#define ADXL355_STATUS        (ADXL355_ADDR(0x04) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x00))  /*!< This register includes bits that describe the various conditions of the ADXL355. */  
#define ADXL355_FIFO_ENTRIES  (ADXL355_ADDR(0x05) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x00))  /*!< This register indicates the number of valid data samples present in the FIFO buffer. This number ranges from 0 to 96.*/  
#define ADXL355_TEMP          (ADXL355_ADDR(0x06) | ADXL355_SET_TRANSF_LEN(2))                                /*!< These two registers contain the uncalibrated temperature data. The nominal intercept is 1885 LSB at 25°C and the nominal slope is -9.05 LSB/°C. TEMP2 contains the four most significant bits, and TEMP1 contains the eight least significant bits of the 12-bit value. The ADXL355 temperature value is not double buffered, meaning the value can update between reading of the two registers.*/  
#define ADXL355_XDATA         (ADXL355_ADDR(0x08) | ADXL355_SET_TRANSF_LEN(3))                                /*!< These three registers contain the x-axis acceleration data. Data is left justified and formatted as twos complement.*/                                                       
#define ADXL355_YDATA         (ADXL355_ADDR(0x0B) | ADXL355_SET_TRANSF_LEN(3))                                /*!< These three registers contain the y-axis acceleration data. Data is left justified and formatted as twos complement.*/                                                       
#define ADXL355_ZDATA         (ADXL355_ADDR(0x0E) | ADXL355_SET_TRANSF_LEN(3))                                /*!< These three registers contain the z-axis acceleration data. Data is left justified and formatted as twos complement.*/                                                       
#define ADXL355_FIFO_DATA     (ADXL355_ADDR(0x11) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x00))  /*!< Read this register to access data stored in the FIFO. */
#define ADXL355_OFFSET_X      (ADXL355_ADDR(0x1E) | ADXL355_SET_TRANSF_LEN(2))                                /*!< Offset added to x-axis data after all other signal processing. Data is in twos complement format. */
#define ADXL355_OFFSET_Y      (ADXL355_ADDR(0x20) | ADXL355_SET_TRANSF_LEN(2))                                /*!< Offset added to y-axis data after all other signal processing. Data is in twos complement format. */
#define ADXL355_OFFSET_Z      (ADXL355_ADDR(0x22) | ADXL355_SET_TRANSF_LEN(2))                                /*!< Offset added to z-axis data after all other signal processing. Data is in twos complement format. */
#define ADXL355_ACT_EN        (ADXL355_ADDR(0x24) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x00))  /*!< Activity enable register. */
#define ADXL355_ACT_THRESH    (ADXL355_ADDR(0x25) | ADXL355_SET_TRANSF_LEN(2))                                /*!< Threshold for activity detection. */
#define ADXL355_ACT_CNT       (ADXL355_ADDR(0x27) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x01))  /*!< Number of consecutive events above threshold (from ACT_THRESH) required to detect activity. */
#define ADXL355_FILTER        (ADXL355_ADDR(0x28) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x00))  /*!< Register to specify parameters for the internal high-pass and low-pass filters.  */
#define ADXL355_FIFO_SAMPLES  (ADXL355_ADDR(0x29) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x60))  /*!< Watermark number of samples stored in the FIFO that triggers a FIFO_FULL condition. Values range from 1 to 96. */
#define ADXL355_INT_MAP       (ADXL355_ADDR(0x2A) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x00))  /*!< The INT_MAP register configures the interrupt pins. */
#define ADXL355_SYNC          (ADXL355_ADDR(0x2B) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x00))  /*!< Register to control the external timing triggers. */
#define ADXL355_RANGE         (ADXL355_ADDR(0x2C) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x81))  /*!< I2C polarity, interrupt polarity and range settings. */
#define ADXL355_POWER_CTL     (ADXL355_ADDR(0x2D) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x01))  /*!< Power Control */
#define ADXL355_SELF_TEST     (ADXL355_ADDR(0x2E) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x00))  /*!< Operation of the self test feature. */
#define ADXL355_RESET         (ADXL355_ADDR(0x2F) | ADXL355_SET_TRANSF_LEN(1) | ADXL355_SET_RESET_VAL(0x00))  /*!< Reset register */
 

/**
  * @brief Shadow Registers of the ADXL355
  * @note  In case of a software reset, an unlikely race condition may occur in products with REVID = 0x01 or earlier. If the race condition occurs,
  *        some factory settings in the NVM load incorrectly to shadow registers (the registers from which the internal logic configures the sensor
  *        and calculates the output after a power-on or a software reset). The incorrect loading of the NVM affects overall performance of the
  *        sensor, such as an incorrect 0 g bias and other performance issues. The incorrect loading of NVM does not occur from a power-on or
  *        after a power cycle. To guarantee reliable operation of the sensor after a software reset, the user can access the shadow registers after a
  *        power-on, read and store the values on the host microprocessor, and compare the values read from the same shadow registers after a
  *        software reset. This method guarantees proper operation in all devices and under all conditions. The recommended steps are as follows:
  *           1. Read the shadow registers, Register 0x50 to Register 0x54 (five 8-bit registers) after power-up, but before any software reset.
  *           2. Store these values in a host device (for example, a host microprocessor).
  *           3. After each software reset, read the same five registers. If the values differ, perform a software reset again until they match. 
  */
#define ADXL355_SHADOW_REGISTER_BASE_ADDR   (ADXL355_ADDR(0x50) | ADXL355_SET_TRANSF_LEN(5))

/**
  * @}
  */

/**
  * @brief ADXL355 Sensor properties according to datasheet
  */

#define ADXL355_MAX_FIFO_SAMPLES_VAL      0x60    /*!< maximum size of FIFO buffer */
#define ADXL355_SELF_TEST_TRIGGER_VAL     0x03    /*!< enable self test force and self test mode */
#define ADXL355_RESET_CODE                0x52    /*!< Code to generate a rest of the sensor */

#define ADXL355_SPI_READ                  0x01
#define ADXL355_SPI_WRITE                 0x00


/*
 * At +/- 2g with 20-bit resolution, scale is given in datasheet as
 * 3.9ug/LSB = 0.0000039 * 9.80665 = 0.00003824593 m/s^2.
 * For +/- 4g range a multiplier with value 2 is used.
 * For +/-8g range, a multiplier with value 4 is used.
 */
#define ADXL355_ACC_SCALE_FACTOR_MUL      (int64_t) 38245         /*!< Converstion to m/s^2 */
#define ADXL355_ACC_SCALE_FACTOR_DIV      (int32_t) 1000000000    /*!< Converstion to m/s^2 */

/*
 * The datasheet defines an intercept of 1852 LSB at 25 degC
 * and a slope of -9.05 LSB/C. The following formula can be used to find the
 * temperature:
 * Temp = ((RAW - 1852)/(-9.05)) + 25 
 */
#define ADXL355_TEMP_OFFSET           -1852.0 /*!< Offset of the temperature sensor */
#define ADXL355_TEMP_SCALE_FACTOR     -9.05   /*!< Slope of the temperature sensor */
#define ADXL355_TEMP_SCALE_FACTOR_DIV 100


#define ADXL355_NEG_ACC_MSK         MISC_GENMASK(31,20) /*!< Bitmask (Bits 20-31) are used for conversion from 2s complement */  
#define ADXL355_RANGE_FIELD_MSK     MISC_GENMASK(1,0)   /*!< Bitmask (Bits 0-1) are used for the sensor range in the range register */
#define ADXL355_ODR_LPF_FIELD_MSK   MISC_GENMASK(3,0)   /*!< Bitmask (Bits 0-3) are used for the output data rate in the filter register */
#define ADXL355_HPF_FIELD_MSK       MISC_GENMASK(6,4)   /*!< Bitmask (Bits 4-6) are used for the highpassfilter in the filter register */
#define ADXL355_IRQ_POL_FIELD_MSK   MISC_BIT(6)         /*!< Bit 6 is used for the interrupt polarity in the range register */

/**
  * @}
  */


//---- Global Section ----------------------------------------------------------
/**
 * @addtogroup ADXL355_GLOBALS  Global Variables 
 * @brief    Thread variables and other global stuff 
 * @{
 */

extern uint32_t ADXL355_SpiTransferState;




/**
  * @}
  */

//---- Enumeration/Typedef Section ---------------------------------------------

/**
 * @addtogroup adxl355_type_tDEF  Type definitions
 * @brief    Type definitions and enumerations for the usage of the ADXL3555
 * @{
 */

/**
  * @brief ID for ADXL355 
  */
typedef enum  {
	ID_ADXL355
} adxl355_type_t;

/**
  * @brief Error Codes 
  */
typedef enum  {
  ADXL355_OK,
	ADXL355_COMMUNICATION_ERROR,
  ADXL355_EAGAIN,
  ADXL355_OUT_OF_RANGE,
  ADXL355_GPIO_ERROR,
  ADXL355_WRONG_DEVICE_AD,
  ADXL355_WRONG_DEVICE_MST,
  ADXL355_WRONG_DEVICE_PARTID,
  ADXL355_WRONG_SHADOW_REGISTER
} adxl355_error_t;


/**
  * @brief Operation Mode for ADXL355 
  */
typedef enum {
	ADXL355_MEAS_TEMP_ON_DRDY_ON    = 0,    /*!< Mode: measurement, temperature: on, data ready: on */
	ADXL355_STDBY_TEMP_ON_DRDY_ON   = 1,    /*!< Mode: standby, temperature: on, data ready: on */
	ADXL355_MEAS_TEMP_OFF_DRDY_ON   = 2,    /*!< Mode: measurement, temperature: off, data ready: on */
	ADXL355_STDBY_TEMP_OFF_DRDY_ON  = 3,    /*!< Mode: standby, temperature: off, data ready: on */
	ADXL355_MEAS_TEMP_ON_DRDY_OFF   = 4,    /*!< Mode: measurement, temperature: on, data ready: off */
	ADXL355_STDBY_TEMP_ON_DRDY_OFF  = 5,    /*!< Mode: standby, temperature: on, data ready: off */
	ADXL355_MEAS_TEMP_OFF_DRDY_OFF  = 6,    /*!< Mode: measurement, temperature: off, data ready: off */
	ADXL355_STDBY_TEMP_OFF_DRDY_OFF = 7     /*!< Mode: standby, temperature: off, data ready: off */
} adxl355_opmode_t;

 
/**
  * @brief Communication Mode of ADXL355 
  */
typedef enum {
	ADXL355_SPI_COMM,         /*!< SPI communication */
	ADXL355_I2C_COMM          /*!< I2c communication */
} adxl355_comm_type;

/**
  * @brief Corner frequencies for the high pass filter.
  */
typedef enum {
	ADXL355_HPF_OFF,        /*!< high pass filter is turned off */
	ADXL355_HPF_24_7,       /*!< f(3 db) = 24.7 x 10^-4 x ODR e.g. 9.88 Hz @ 4 kHz ODR */
	ADXL355_HPF_6_2084,     /*!< f(3 db) = 6.2084 x 10^-4 x ODR e.g. 2.48 Hz @ 4 kHz ODR */
	ADXL355_HPF_1_5545,     /*!< f(3 db) = 1.5545 x 10^-4 x ODR e.g. 0.62 Hz @ 4 kHz ODR */
	ADXL355_HPF_0_3862,     /*!< f(3 db) = 0.3862 x 10^-4 x ODR e.g. 0.1545 Hz @ 4 kHz ODR */
	ADXL355_HPF_0_0954,     /*!< f(3 db) = 0.0954 x 10^-4 x ODR e.g. 0.03816 Hz @ 4 kHz ODR */
	ADXL355_HPF_0_0238      /*!< f(3 db) = 0.02387 x 10^-4 x ODR e.g. 0.00952 Hz @ 4 kHz ODR */
} adxl355_hpf_corner_t;

/**
  * @brief Corner frequencies for the low pass filter & output data rate settings
  */
typedef enum {
	ADXL355_ODR_4000HZ,     /*!< ODR = 4000 Hz */
	ADXL355_ODR_2000HZ,     /*!< ODR = 2000 Hz */
	ADXL355_ODR_1000HZ,     /*!< ODR = 1000 Hz */
	ADXL355_ODR_500HZ,      /*!< ODR = 500 Hz */
	ADXL355_ODR_250HZ,      /*!< ODR = 250 Hz */
	ADXL355_ODR_125HZ,      /*!< ODR = 125 Hz */
	ADXL355_ODR_62_5HZ,     /*!< ODR = 62.5 Hz */
	ADXL355_ODR_31_25HZ,    /*!< ODR = 31.25 Hz */
	ADXL355_ODR_15_625HZ,   /*!< ODR = 15.625 Hz */
	ADXL355_ODR_7_813HZ,    /*!< ODR = 7.813 Hz */
	ADXL355_ODR_3_906HZ     /*!< ODR = 3.906 Hz */
} adxl355_odr_lpf_t;

/**
  * @brief Measurement range of the sensor
  */
typedef enum {
	 ADXL355_RANGE_2G = 1, /*!< +/-2g range */
	 ADXL355_RANGE_4G = 2, /*!< +/-4g range */
	 ADXL355_RANGE_8G = 3, /*!< +/-8g range */
} adxl355_range_t;

/**
  * @brief Interrupt polarity
  */
typedef enum {
	ADXL355_INT_ACTIVE_LOW  = 0,  /*!< Active low polarity */
	ADXL355_INT_ACTIVE_HIGH = 1   /*!< Active high polarity */
} adxl355_irq_polarity_t;

/**
  * @brief State Machine for SPI data transmission
  */
typedef enum {
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
} adxl355_spi_transfer_state_t;

/**
 * @struct adxl355_init_param_t
 * @brief Structure holding the parameters for ADXL355 device initialization.
 */
typedef struct {
	adxl355_comm_type comm_type;    /*!< Device Communication type: ADXL355_SPI_COMM, ADXL355_I2C_COMM */
	adxl355_type_t dev_type;          /*!< Device type: ADXL355 or ... */
} adxl355_init_param_t; 

struct _adxl355_irq_mask_t  {
	uint8_t RDY_EN1 : 1;
	uint8_t FULL_EN1 : 1;
	uint8_t OVR_EN1 : 1;
	uint8_t ACT_EN1 : 1;
	uint8_t RDY_EN2 : 1;
	uint8_t FULL_EN2 : 1;
	uint8_t OVR_EN2 : 1;
	uint8_t ACT_EN2 : 1;
};

typedef union {
	struct _adxl355_irq_mask_t fields;
	uint8_t value;
} adxl355_irq_mask_t;

struct _adxl355_sts_reg_flags {
	uint8_t DATA_RDY : 1;
	uint8_t FIFO_FULL : 1;
	uint8_t FIFO_OVR : 1;
	uint8_t Activity : 1;
	uint8_t NVM_BUSY : 1;
	uint8_t reserved : 3;
};

typedef union {
	struct _adxl355_sts_reg_flags fields;
	uint8_t value;
} adxl355_status_register_flags_t;

struct _adxl355_act_en_flags {
	uint8_t ACT_X    : 1;
	uint8_t ACT_Y    : 1;
	uint8_t ACT_Z    : 1;
	uint8_t reserved : 4;
};

typedef union {
	struct _adxl355_act_en_flags fields;
	uint8_t value;
} adxl355_act_enab_flags_t;

typedef struct {
	int64_t integer;
	int32_t fractional;
} adxl355_frac_representation_t;


/**
 * @struct adxl355_dev
 * @brief ADXL355 Device structure.
 */
typedef struct {
	/** Device type */
	adxl355_type_t dev_type;
	adxl355_comm_type comm_type;
	adxl355_opmode_t op_mode;
	adxl355_odr_lpf_t odr_lpf;
	adxl355_hpf_corner_t hpf_corner;
	adxl355_range_t range;
	uint16_t x_offset;
	uint16_t y_offset;
	uint16_t z_offset;
	uint8_t fifo_samples;
	adxl355_act_enab_flags_t act_en;
	uint8_t act_cnt;
	uint16_t act_thr;
	uint8_t comm_buff[289];
} adxl355_device_t;

/**
  * @}
  */


//---- Prototype Section -------------------------------------------------------

/**
 * @addtogroup ADXL355_Functions     Exported functions
 * @brief    Exported functions to use the ADXL355
 * @{
 */

/**
 * @brief Initializes the communication peripheral and checks if the ADXL355
 *        part is present.
 *
 * @param device      The device structure.
 *
 * @return            Result of the initialization procedure.
 */
 
extern adxl355_error_t adxl355_init(adxl355_device_t *device);

/**
 * @brief Updates the device struct 
 *
 * @param device      The device structure.
 *
 */
extern void adxl355_update_device(adxl355_device_t *device);
 
/** 
 * @brief Places the device into the given operation mode.
 *
 * @param op_mode   Operation mode mode.
 *
 * @return          Result of the setting operation procedure.
 */
extern adxl355_error_t adxl355_set_op_mode(adxl355_opmode_t op_mode);

/** 
 * @brief Gets the current operation mode of the device
 *
 * @param op_mode   Operation mode mode.
 *
 * @return          Result of the setting operation procedure.
 */
extern adxl355_error_t adxl355_get_op_mode(adxl355_opmode_t *op_mode);

/** 
 * @brief Performs a soft reset of the device.
 *
 *
 * @return          Result of the setting operation procedure.
 */
extern adxl355_error_t adxl355_softreset(void);

/**
 * @brief Reads the status register value.
 *
 * @param flags   Register value.
 *
 * @return        Result of the reading procedure.
 */
extern adxl355_error_t adxl355_get_status_register(adxl355_status_register_flags_t *flags);

/**
 * @brief Triggers the self-test feature.
 *
 *
 * @return        Result of the writing procedure.
 */
extern adxl355_error_t adxl355_set_selftest(void);

/**
 * @brief Sets the measurement range register value.
 *
 * @param range   Selected range.
 *
 * @return        Result of the writing procedure.
 */
extern adxl355_error_t adxl355_set_range(adxl355_range_t range);

/**
 * @brief  Writes the low-pass filter settings.
 *
 * @param odr_lpf filter settings
 *
 * @return        Result of the writing procedure.
 */
extern adxl355_error_t adxl355_set_odr_lpf(adxl355_odr_lpf_t odr_lpf);


/**
 * @brief  Writes the high-pass filter settings.
 *
 * @param hpf_corner  filter settings
 *
 * @return            Result of the writing procedure.
 */
extern adxl355_error_t adxl355_set_hpf_corner(adxl355_hpf_corner_t hpf_corner);

/**
 * @brief Sets an offset value for each axis (Offset Calibration).
 *
 * @param x_offset  X-axis's offset.
 * @param y_offset  Y-axis's offset.
 * @param z_offset  Z-axis's offset.
 *
 * @return          Result of the writing procedure.
 */
extern adxl355_error_t adxl355_set_offset(uint16_t x_offset, uint16_t y_offset, uint16_t z_offset);

/**
 * @brief Reads the raw output data.
 *
 * @param raw_x   X-axis's raw output data.
 * @param raw_y   Y-axis's raw output data.
 * @param raw_z   Z-axis's raw output data.
 *
 * @return        Result of the reading procedure.
 */
extern adxl355_error_t adxl355_get_raw_xyz(uint32_t *raw_x, uint32_t *raw_y, uint32_t *raw_z);

/**
 * @brief Reads the raw output data of each axis and converts it to g.
 *
 * @param x       X-axis's output data.
 * @param y       Y-axis's output data.
 * @param z       Z-axis's output data.
 *
 * @return        Result of the reading procedure.
 */
extern adxl355_error_t adxl355_get_xyz(adxl355_frac_representation_t *x, adxl355_frac_representation_t *y, adxl355_frac_representation_t *z);

/**
 * @brief Reads the raw temperature.
 *
 * @param raw_temp  Raw temperature output data.
 *
 * @return          Result of the reading procedure.
 */
extern adxl355_error_t adxl355_get_raw_temp(uint16_t *raw_temp);

/**
 * @brief Reads the raw temperature data and converts it to millidegrees Celsius.
 *
 * @param temp    Temperature output data.
 *
 * @return        Result of the reading procedure.
 */
extern adxl355_error_t adxl355_get_temp(adxl355_frac_representation_t *temp);

/**
 * @brief Reads the number of FIFO entries register value.
 *
 * @param reg_value   Register value.
 *
 * @return            Result of the reading procedure.
 */
extern adxl355_error_t adxl355_get_nb_fifo_entries(uint8_t *reg_value);

/**
 * @brief Sets the number of FIFO samples register value.
 *
 * @param reg_value   Register value.
 *
 * @return            Result of the writing procedure.
 */
extern adxl355_error_t adxl355_set_nb_fifo_entries(uint8_t reg_value);

/**
 * @brief Reads fifo data and returns the raw values.
 *
 * @param fifo_entries  The number of fifo entries.
 * @param raw_x         Raw x-axis data.
 * @param raw_y         Raw y-axis data.
 * @param raw_z         Raw z-axis data.
 *
 * @return              Result of the configuration procedure.
 */
extern adxl355_error_t adxl355_get_raw_fifo_data(uint8_t *fifo_entries, uint32_t *raw_x, uint32_t *raw_y, uint32_t *raw_z);

/**
 * @brief Reads fifo data and returns the values converted in m/s^2.
 *
 * @param fifo_entries  The number of fifo entries.
 * @param x             Converted x-axis data.
 * @param y             Converted y-axis data.
 * @param z             Converted z-axis data.
 *
 * @return              Result of the configuration procedure.
 */
extern adxl355_error_t adxl355_get_fifo_data(uint8_t *fifo_entries, adxl355_frac_representation_t *x, adxl355_frac_representation_t *y, adxl355_frac_representation_t *z);


/**
 * @brief Configures the activity enable register.
 *
 * @param act_config  Activity enable mapping.
 *
 * @return            Result of the configuration procedure.
 */
extern adxl355_error_t adxl355_config_act_enab(adxl355_act_enab_flags_t act_config);

/**
 * @brief Configures the activity threshold registers.
 *
 * @param act_thr   Activity threshold value.
 *
 * @return          Result of the configuration procedure.
 */
extern adxl355_error_t adxl355_config_act_thres(uint16_t act_thr);

/**
* @brief Writes the activity count register value.
 *
 * @param act_cnt   Register value.
 *
 * @return          Result of the writing procedure.
 */
extern adxl355_error_t adxl355_set_act_cnt(uint8_t act_cnt);

/**
 * @brief Configures the interrupt map for INT1 and INT2 pins.
 *
 * @param int_conf  Interrupt mapping.
 *
 * @return          Result of the configuration procedure.
 */
extern adxl355_error_t adxl355_config_int_pins(adxl355_irq_mask_t int_conf);

/**
 * @brief Sets the interrupt polarity.
 *
 * @param int_pol   Interrupt polarity to be set.
 *
 * @return          Result of the reading procedure.
 */
extern adxl355_error_t adxl355_set_irq_polarity(adxl355_irq_polarity_t int_pol);



extern void adxl355_read_to_buffer();


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
