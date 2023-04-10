/**
 * \file            adxl355.c
 * \brief           ADXL355 acceleration sensor
 */


//---- Include Section ---------------------------------------------------------
#include "adxl355.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sg_lib_conf.h"
#include "sg_io.h"
#include "RTE_Components.h"
#include "cmsis_os2.h"

#include "lwrb.h"


//---- Define Section ----------------------------------------------------------
#define BUFFERSIZE          289 //maxi. 96 values with 3 bytes inside the buffer 
#define SPI_TIMEOUT         15
//---- Enumeration/Typedef Section ---------------------------------------------


//---- Global Section ----------------------------------------------------------

uint32_t ADXL355_SpiTransferState = TRANSFER_WAIT;








//---- Private Section ----------------------------------------------------------
static SPI_HandleTypeDef spiHandle;
static uint8_t shadow_reg_val[5] = {0, 0, 0, 0, 0};
static const uint8_t adxl355_scale_mul[4] = {0, 1, 2, 4};
static const uint8_t adxl355_part_id[] = {
	[ID_ADXL355] = ADXL355_GET_RESET_VAL(ADXL355_PARTID),
};


static adxl355_device_t device_handle;




//---- Prototype Section -------------------------------------------------------

/**
 * @brief Converts array of raw acceleration to uint32 data raw acceleration
 *
 * @param raw_array   Raw acceleration array.
 *
 * @return            Converted data.
 */
static uint32_t accel_array_conv(uint8_t *raw_array);

/**
 * @brief Converts raw acceleration value to m/s^2 value.
 *
 * @param raw_accel   Raw acceleration value
 *
 * @return            Converted data.
 */
static int64_t accel_conv(uint32_t raw_accel);

/**
 * @brief Converts raw temperature data to degrees Celsius data.
 *
 * @param raw_temp    Raw temperature data.
 *
 * @return            Converted data.
 */
static int64_t temp_conv(uint16_t raw_temp);

/**
 * @brief Configuration of SPI using DMA transfer 
 * @note  Configuration contains
 *        - the DMA
 *        - the SPI peripherial
 *        The method will call the HAL_SPI_MspInit function via the HAL
 *
 * @return ret          false in case of error
 */
static bool spi_configuration(void);


/**
 * @brief Writes to the device
 *
 * @param base_address  Address of the base register.
 * @param size          The number of bytes to be written. It is the size
 *                      of the write_data buffer.
 * @param write_data    The data which is going to be written.
 *
 * @return ret          false in case of error
 */
static bool write_device_data(uint8_t base_address, uint16_t size, uint8_t *write_data);

/**
 * @brief Reads from the device.
 *
 * @param base_address  Address of the base register.
 * @param size          The number of bytes to be read and returned in read_data.
 * @param read_data     The read data buffer
 *
 * @return ret          false in case of error
 */
static bool read_device_data(uint8_t base_address, uint16_t size, uint8_t *read_data);

static uint32_t get_unaligned_be32(uint8_t *buf);

/**
 * @brief Signed 64bit divide with 32bit divisor with remainder
 */
static int64_t div_s64_rem(int64_t dividend, int32_t divisor, int32_t *remainder);

//---- Private Function Section ------------------------------------------------


static int64_t div_s64_rem(int64_t dividend, int32_t divisor, int32_t *remainder) {
	*remainder = dividend % divisor;
	return dividend / divisor;
}


static uint32_t get_unaligned_be32(uint8_t *buf) {
  return buf[2] | ((uint16_t)buf[1] << 8) | ((uint32_t)buf[0] << 16);
} // END OF 'get_unaligned_be32'	


static uint32_t accel_array_conv(uint8_t *raw_array) {
  uint32_t raw_accel;

	raw_accel = get_unaligned_be32(raw_array);

	return (raw_accel >> 4);  
} // END OF 'accel_array_conv'	

static int64_t accel_conv(uint32_t raw_accel) {
  int accel_data;

	// Raw acceleration is in two's complement
	// Convert from two's complement to int

	if ((raw_accel & MISC_BIT(19)) == MISC_BIT(19))
		accel_data = raw_accel | ADXL355_NEG_ACC_MSK;
	else
		accel_data = raw_accel;

	// Apply scale factor based on the selected range
	switch (device_handle.dev_type) {
	case ID_ADXL355:
		return ((int64_t)(accel_data * ADXL355_ACC_SCALE_FACTOR_MUL * adxl355_scale_mul[device_handle.range]));
	default:
		return 0;
	}  
} // END OF 'accel_conv'	

static int64_t temp_conv(uint16_t raw_temp) {
  switch(device_handle.dev_type) {
	case ID_ADXL355:
    return ((raw_temp +  ADXL355_TEMP_OFFSET) * (int64_t)ADXL355_TEMP_SCALE_FACTOR);
	default:
		return 0;
	}  
} // END OF 'temp_conv'	

static bool spi_configuration(void) {
  bool ret;
  
  ret = true;
  
  SPI_CLK_ENAB();
   /* configuration of SPI pins */
  SG_IO_InitAlternate(SPI_GPIO_Port, MISO_Pin, SG_IO_PushPull, SG_IO_NoPullUpOrDown, SG_IO_High, SPI_ALTERNATE_FUNCTION);
  SG_IO_InitAlternate(SPI_GPIO_Port, SCLK_Pin, SG_IO_PushPull, SG_IO_NoPullUpOrDown, SG_IO_High, SPI_ALTERNATE_FUNCTION);
  SG_IO_InitAlternate(SPI_GPIO_Port, MOSI_Pin, SG_IO_PushPull, SG_IO_NoPullUpOrDown, SG_IO_High, SPI_ALTERNATE_FUNCTION);

  spiHandle.Instance                = SPI;
  spiHandle.Init.Mode               = SPI_MODE_MASTER;
  spiHandle.Init.Direction          = SPI_DIRECTION_2LINES;
  spiHandle.Init.DataSize           = SPI_DATASIZE_8BIT;
  spiHandle.Init.CLKPolarity        = SPI_POLARITY_LOW;         //CPOL = 0
  spiHandle.Init.CLKPhase           = SPI_PHASE_1EDGE;          //CPHA = 0
  spiHandle.Init.NSS                = SPI_NSS_SOFT;
  spiHandle.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_32; //this leads to 84 MHz/32 = 2.625 MHz (valid 100 kHz - 10 MHz)
  spiHandle.Init.FirstBit           = SPI_FIRSTBIT_MSB;
  spiHandle.Init.TIMode             = SPI_TIMODE_DISABLE;
  spiHandle.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
  spiHandle.Init.CRCPolynomial      = 10;
  if (HAL_SPI_Init(&spiHandle) != HAL_OK)
  {
    ret = false;
  }  
  
  HAL_NVIC_SetPriority(SPI_IRQ, 0, 0);
  HAL_NVIC_EnableIRQ(SPI_IRQ);
  __HAL_SPI_ENABLE(&spiHandle);
       
  return ret;
} // END OF 'spi_configuration'	





static bool write_device_data(uint8_t base_address, uint16_t size, uint8_t *write_data) {
  bool ret = true;
  uint8_t aTxBuffer[BUFFERSIZE] = {0};
  uint8_t aRxBuffer[BUFFERSIZE] = {0};
 
  for (uint16_t idx = 0; idx < size; idx++) {
		aTxBuffer[1+idx] = write_data[idx];
  }  
  if (device_handle.comm_type == ADXL355_SPI_COMM) {
    SG_IO_SetPinLow(CS_GPIO_Port, CS_Pin);    //select the SPI slave device
    aTxBuffer[0] = ADXL355_SPI_WRITE | (base_address << 1);   //set the R/W bit according to datasheet
    if (HAL_SPI_TransmitReceive_IT(&spiHandle, (uint8_t *) aTxBuffer, (uint8_t *)aRxBuffer, size+1) != HAL_OK) {
      ret = false;
    }         
     //Wait for the end of the previous transfer
    while (HAL_SPI_GetState(&spiHandle) != HAL_SPI_STATE_READY) {
    } 
    SG_IO_SetPinHigh(CS_GPIO_Port, CS_Pin);    //deselect the SPI slave device
  } else {
    ret = false;
  }    
  return ret;
} // END OF 'write_device_data'

static bool read_device_data(uint8_t base_address, uint16_t size, uint8_t *read_data) {
  bool ret = true;
  uint8_t aTxBuffer[BUFFERSIZE] = {0};
  uint8_t aRxBuffer[BUFFERSIZE] = {0};
    
  if (device_handle.comm_type == ADXL355_SPI_COMM) {
    SG_IO_SetPinLow(CS_GPIO_Port, CS_Pin);    //select the SPI slave device
    aTxBuffer[0] = ADXL355_SPI_READ | (base_address << 1);   //set the R/W bit according to datasheet
    if (HAL_SPI_TransmitReceive_IT(&spiHandle, (uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, size +1) != HAL_OK) {
      ret = false;
    }     
    //Wait for the end of the previous transfer
    while (HAL_SPI_GetState(&spiHandle) != HAL_SPI_STATE_READY) {
    } 
    SG_IO_SetPinHigh(CS_GPIO_Port, CS_Pin);    //deselect the SPI slave device
    for (uint16_t idx = 0; idx < size; idx++) {
          read_data[idx] = aRxBuffer[idx+1];
    }
  }
  return ret;
} // END OF 'read_device_data'

//---- IRQ Handler -------------------------------------------------------------
void SPI_IRQ_HANDLER() {  
  /* 
   * Reception of data via SPI is finished.
   */
  HAL_SPI_IRQHandler(&spiHandle);    
} //END OF 'SPI_IRQ_HANDLER

/**
  * @brief  Tx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  ADXL355_SpiTransferState = TRANSFER_COMPLETE;
} // END OF 'HAL_SPI_TxCpltCallback'	

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi) {
  ADXL355_SpiTransferState = TRANSFER_COMPLETE;
} //END OF 'HAL_SPI_RxCpltCallback'

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  ADXL355_SpiTransferState = TRANSFER_ERROR;
} // END OF 'HAL_SPI_ErrorCallback'	

//---- Exported functions ------------------------------------------------------
extern adxl355_error_t adxl355_init(adxl355_device_t *device) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  uint8_t reg_value;
  
  device_handle = *device;
  
  if (device->comm_type == ADXL355_SPI_COMM) {
    if (spi_configuration() == false) {
      ret = ADXL355_COMMUNICATION_ERROR;
      goto error_com;
    }
  } else {
    ret = ADXL355_COMMUNICATION_ERROR;
    goto error_com;
  }

  //all transfers are completet after init
  ADXL355_SpiTransferState = TRANSFER_COMPLETE;
  
  //check the id registers for correct content
  comm_result = read_device_data(ADXL355_ADDR(ADXL355_DEVID_AD), ADXL355_GET_TRANSF_LEN(ADXL355_DEVID_AD), &reg_value);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
    goto error_com;
  } else {
    if (reg_value != ADXL355_GET_RESET_VAL(ADXL355_DEVID_AD)) {
      ret = ADXL355_WRONG_DEVICE_AD;
      goto error_dev;
    }
  }
  
  comm_result = read_device_data(ADXL355_ADDR(ADXL355_DEVID_MST), ADXL355_GET_TRANSF_LEN(ADXL355_DEVID_MST), &reg_value);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
    goto error_com;
  } else {
    if (reg_value != ADXL355_GET_RESET_VAL(ADXL355_DEVID_MST)) {
      ret = ADXL355_WRONG_DEVICE_MST;
      goto error_dev;
    }
  }
  
  comm_result = read_device_data(ADXL355_ADDR(ADXL355_PARTID), ADXL355_GET_TRANSF_LEN(ADXL355_PARTID), &reg_value);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
    goto error_com;
  } else {
    if (reg_value != ADXL355_GET_RESET_VAL(ADXL355_PARTID)) {
      ret = ADXL355_WRONG_DEVICE_PARTID;
      goto error_dev;
    }
  }
  
  comm_result = read_device_data(ADXL355_ADDR(ADXL355_SHADOW_REGISTER_BASE_ADDR), ADXL355_GET_TRANSF_LEN(ADXL355_SHADOW_REGISTER_BASE_ADDR), &shadow_reg_val[0]);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
    goto error_com;
  }
  
  // Default range is set
  device->range = ADXL355_GET_RESET_VAL(ADXL355_RANGE) & ADXL355_RANGE_FIELD_MSK;

	// Default value for FIFO SAMPLES
	device->fifo_samples = ADXL355_GET_RESET_VAL(ADXL355_FIFO_SAMPLES);

	// Default value for POWER_CTL
	device->op_mode = ADXL355_GET_RESET_VAL(ADXL355_POWER_CTL);

	// Default activity count value
	device->act_cnt = ADXL355_GET_RESET_VAL(ADXL355_ACT_CNT);
	device_handle = *device;
  
  return ret;
  
  error_com:
            return ret;
  
  error_dev:
            return ret;  
} // END OF 'adxl355_init'	

extern void adxl355_update_device(adxl355_device_t *device) {
  device_handle = *device;  
} // END OF 'adxl355_update_device'	 

extern adxl355_error_t adxl355_set_op_mode(adxl355_opmode_t op_mode) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  
  comm_result = write_device_data(ADXL355_ADDR(ADXL355_POWER_CTL), ADXL355_GET_TRANSF_LEN(ADXL355_POWER_CTL), &op_mode);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
  }  else {
    device_handle.op_mode = op_mode;    
  }
  return ret;
} // END OF 'adxl355_set_op_mode'	

extern adxl355_error_t adxl355_get_op_mode(adxl355_opmode_t *op_mode) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  
  comm_result = read_device_data(ADXL355_ADDR(ADXL355_POWER_CTL), ADXL355_GET_TRANSF_LEN(ADXL355_POWER_CTL), op_mode);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
  }  else {
    device_handle.op_mode = *op_mode;    
  }
  return ret;
} // END OF 'adxl355_get_op_mode'	

extern adxl355_error_t adxl355_softreset(void) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  
  uint8_t register_values[5];
	uint8_t nb_of_retries = 255;
	uint8_t data = ADXL355_RESET_CODE;
	adxl355_status_register_flags_t flags;
  
  
  // Perform soft reset
	comm_result = write_device_data(ADXL355_ADDR(ADXL355_RESET),	ADXL355_GET_TRANSF_LEN(ADXL355_RESET), &data);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
    goto error_com;
  } 
  ret = adxl355_get_status_register(&flags);
  // After soft reset, the data in the shadow registers will be valid only after NVM is not busy anymore
  while (flags.fields.NVM_BUSY && nb_of_retries) {
    ret = adxl355_get_status_register(&flags);
    nb_of_retries--;
  }
  if ((nb_of_retries == 0) || (ret != ADXL355_COMMUNICATION_ERROR)) {
    ret = ADXL355_EAGAIN;
    goto error_dev;
  }   
  // Delay is needed between soft reset command and shadow registers reading 
  osDelay(1);
  // Read the shadow registers
  comm_result = read_device_data(ADXL355_ADDR(ADXL355_SHADOW_REGISTER_BASE_ADDR), ADXL355_GET_TRANSF_LEN(ADXL355_SHADOW_REGISTER_BASE_ADDR), &register_values[0]);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
    goto error_com;
  } 
  if (strncmp((const char*)register_values, (const char*)shadow_reg_val, ADXL355_GET_TRANSF_LEN(ADXL355_SHADOW_REGISTER_BASE_ADDR)) != 0) {
    ret = ADXL355_EAGAIN;
  }
  
  return ret;
 
  error_com:
            return ret;
  
  error_dev:
            return ret;
} // END OF 'adxl355_softreset'	

extern adxl355_error_t adxl355_get_status_register(adxl355_status_register_flags_t *flags) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  uint8_t reg_value;

	comm_result = read_device_data(ADXL355_ADDR(ADXL355_STATUS), ADXL355_GET_TRANSF_LEN(ADXL355_STATUS), &reg_value);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
  }  else {
    flags->value = reg_value;    
  }  
  
  return ret;
} // END OF 'adxl355_get_status_register'	

extern adxl355_error_t adxl355_set_selftest(void) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  uint8_t data = ADXL355_SELF_TEST_TRIGGER_VAL;
  
  comm_result = read_device_data(ADXL355_ADDR(ADXL355_SELF_TEST), ADXL355_GET_TRANSF_LEN(ADXL355_SELF_TEST), &data);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
  }  
  return ret;
} // END OF 'adxl355_set_selftest'	

extern adxl355_error_t adxl355_set_range(adxl355_range_t range) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  uint8_t range_reg;
  
  comm_result = read_device_data(ADXL355_ADDR(ADXL355_RANGE), ADXL355_GET_TRANSF_LEN(ADXL355_RANGE), &range_reg);
  
  if (comm_result == true) {
    // Reset range bits
    range_reg &= ~ ADXL355_RANGE_FIELD_MSK;
    // Set only range bits inside range registers
    range_reg |= range &  ADXL355_RANGE_FIELD_MSK;
	  if (write_device_data(ADXL355_ADDR(ADXL355_RANGE), ADXL355_GET_TRANSF_LEN( ADXL355_RANGE), &range_reg) == true) {
      device_handle.range = range;
    } else {
       ret = ADXL355_COMMUNICATION_ERROR; 
     }    
  }  else {
    ret = ADXL355_COMMUNICATION_ERROR;  
  }    
  return ret;
} // END OF 'adxl355_set_range'	

extern adxl355_error_t adxl355_set_odr_lpf(adxl355_odr_lpf_t odr_lpf) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  uint8_t reg_value;
  adxl355_opmode_t current_op_mode;
  
  // The lpf settings cannot be changed when the device is in measurement mode.
	// The lpf settings can be changed only when the device's operation mode is in standby.
  
  current_op_mode = device_handle.op_mode;
  
  switch(current_op_mode) {
	case ADXL355_MEAS_TEMP_ON_DRDY_ON:
	case ADXL355_MEAS_TEMP_OFF_DRDY_ON:
	case ADXL355_MEAS_TEMP_ON_DRDY_OFF:
	case ADXL355_MEAS_TEMP_OFF_DRDY_OFF:
		ret = adxl355_set_op_mode(ADXL355_STDBY_TEMP_ON_DRDY_ON);
		if (ret != ADXL355_OK) {
      return ret;
    }
		break;
	default:
		break;
	}
    
  comm_result = read_device_data(ADXL355_ADDR(ADXL355_FILTER), ADXL355_GET_TRANSF_LEN(ADXL355_FILTER), &reg_value);
  
  if (comm_result == true) {
   reg_value &= ~(ADXL355_ODR_LPF_FIELD_MSK);
   reg_value |= odr_lpf & ADXL355_ODR_LPF_FIELD_MSK;
	 if (write_device_data(ADXL355_ADDR(ADXL355_FILTER), ADXL355_GET_TRANSF_LEN(ADXL355_FILTER), &reg_value) == true) {
     device_handle.odr_lpf = odr_lpf;
     ret = adxl355_set_op_mode(current_op_mode);
    } else {
      ret = ADXL355_COMMUNICATION_ERROR;   
    }    
  } else {
    ret = ADXL355_COMMUNICATION_ERROR;  
  }    
  return ret;
} // END OF 'adxl355_set_odr_lpf'	

extern adxl355_error_t adxl355_set_hpf_corner(adxl355_hpf_corner_t hpf_corner)  {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  uint8_t reg_value;
  adxl355_opmode_t current_op_mode;
  
  // Even though the hpf settings can be changed when the device is in
  // measurement mode, it has been observed that the measurements are not
	// correct when doing so. Because of this, the device will be set in standby
	// mode also when changing the hpf value.
  
  current_op_mode = device_handle.op_mode;
  
  switch(current_op_mode) {
	case ADXL355_MEAS_TEMP_ON_DRDY_ON:
	case ADXL355_MEAS_TEMP_OFF_DRDY_ON:
	case ADXL355_MEAS_TEMP_ON_DRDY_OFF:
	case ADXL355_MEAS_TEMP_OFF_DRDY_OFF:
		ret = adxl355_set_op_mode(ADXL355_STDBY_TEMP_ON_DRDY_ON);
		if (ret != ADXL355_OK) {
      return ret;
    }
		break;
	default:
		break;
	}
    
  comm_result = read_device_data(ADXL355_ADDR(ADXL355_FILTER), ADXL355_GET_TRANSF_LEN(ADXL355_FILTER), &reg_value);
  
  if (comm_result == true) {
   reg_value &= ~(ADXL355_HPF_FIELD_MSK);
   reg_value |= (hpf_corner << 4) & ADXL355_HPF_FIELD_MSK;
	 if (write_device_data(ADXL355_ADDR(ADXL355_FILTER), ADXL355_GET_TRANSF_LEN(ADXL355_FILTER), &reg_value) == true) {
     device_handle.hpf_corner = hpf_corner;
     ret = adxl355_set_op_mode(current_op_mode);
    } else {
      ret = ADXL355_COMMUNICATION_ERROR;
    }    
  } else {
    ret = ADXL355_COMMUNICATION_ERROR;  
  }    
  return ret;
} // END OF 'adxl355_set_hpf_corner'	

extern adxl355_error_t adxl355_set_offset(uint16_t x_offset, uint16_t y_offset, uint16_t z_offset) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  uint8_t data_offset_x[2] = {x_offset >> 8, (uint8_t)x_offset};
	uint8_t data_offset_y[2] = {y_offset >> 8, (uint8_t)y_offset};
	uint8_t data_offset_z[2] = {z_offset >> 8, (uint8_t)z_offset};

	comm_result = write_device_data(ADXL355_ADDR(ADXL355_OFFSET_X), ADXL355_GET_TRANSF_LEN(ADXL355_OFFSET_X),	(uint8_t*)data_offset_x);
	if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
		return ret;
  }

	comm_result = write_device_data(ADXL355_ADDR(ADXL355_OFFSET_Y),	ADXL355_GET_TRANSF_LEN(ADXL355_OFFSET_Y), (uint8_t*)data_offset_y);
	if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
		return ret;
  }

	comm_result = write_device_data(ADXL355_ADDR(ADXL355_OFFSET_Z), ADXL355_GET_TRANSF_LEN(ADXL355_OFFSET_Z), (uint8_t*)data_offset_z);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
		return ret;
  }
	device_handle.x_offset = x_offset;
	device_handle.y_offset = y_offset;
	device_handle.z_offset = z_offset;
  
  return ret;
} // END OF 'adxl355_set_offset'	 

extern adxl355_error_t adxl355_get_raw_xyz(uint32_t *raw_x, uint32_t *raw_y, uint32_t *raw_z) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  uint8_t array_raw_x[ADXL355_GET_TRANSF_LEN(ADXL355_XDATA)] = {0};
	uint8_t array_raw_y[ADXL355_GET_TRANSF_LEN(ADXL355_YDATA)] = {0};
	uint8_t array_raw_z[ADXL355_GET_TRANSF_LEN(ADXL355_ZDATA)] = {0};

	comm_result = read_device_data(ADXL355_ADDR(ADXL355_XDATA), ADXL355_GET_TRANSF_LEN(ADXL355_XDATA),array_raw_x);
	if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
		return ret;
  }
	
  *raw_x = accel_array_conv(array_raw_x);
	comm_result = read_device_data(ADXL355_ADDR(ADXL355_YDATA), ADXL355_GET_TRANSF_LEN(ADXL355_YDATA),array_raw_y);
	if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
		return ret;
  }
  
	*raw_y = accel_array_conv(array_raw_y);
	comm_result = read_device_data(ADXL355_ADDR(ADXL355_ZDATA), ADXL355_GET_TRANSF_LEN(ADXL355_ZDATA), array_raw_z);
	if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
		return ret;
  }
  
	*raw_z = accel_array_conv(array_raw_z);  
  return ret;
} // END OF 'adxl355_get_raw_xyz'	 

extern adxl355_error_t adxl355_get_xyz(adxl355_frac_representation_t *x, adxl355_frac_representation_t *y, adxl355_frac_representation_t *z) {
  adxl355_error_t ret = ADXL355_OK;
  uint32_t raw_accel_x;
	uint32_t raw_accel_y;
	uint32_t raw_accel_z;

	ret = adxl355_get_raw_xyz(&raw_accel_x, &raw_accel_y, &raw_accel_z);

	x->integer = div_s64_rem(accel_conv(raw_accel_x), ADXL355_ACC_SCALE_FACTOR_DIV, &(x->fractional));
	y->integer = div_s64_rem(accel_conv(raw_accel_y), ADXL355_ACC_SCALE_FACTOR_DIV, &(y->fractional));
	z->integer = div_s64_rem(accel_conv(raw_accel_z), ADXL355_ACC_SCALE_FACTOR_DIV, &(z->fractional));  
  return ret;
} // END OF 'adxl355_get_xyz'	 

extern adxl355_error_t adxl355_get_raw_temp(uint16_t *raw_temp) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;  
  uint8_t raw_data[2];


	comm_result = read_device_data(ADXL355_ADDR(ADXL355_TEMP), ADXL355_GET_TRANSF_LEN(ADXL355_TEMP), raw_data);
	if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR; 
  } else {
    // raw_data[0] bits [7-4]: reserved
		// raw_data[0] bits [3-0]: DATA bits [11: 8]
		// raw_data[1] bits [7-0]: DATA bits [ 7: 0]
		*raw_temp = ((raw_data[0] & MISC_GENMASK(3, 0)) << 8) | raw_data[1];
  }
  return ret;
} // END OF 'adxl355_get_raw_temp'	 

extern adxl355_error_t adxl355_get_temp(adxl355_frac_representation_t *temp) {
  adxl355_error_t ret = ADXL355_OK;
  uint16_t raw_temp;
	int32_t divisor;

	ret = adxl355_get_raw_temp(&raw_temp);

	switch(device_handle.dev_type) {
	case ID_ADXL355:
		divisor = ADXL355_TEMP_SCALE_FACTOR*ADXL355_TEMP_SCALE_FACTOR_DIV;
		break;
	default:
    ret = ADXL355_EAGAIN;
		break;
	}

	temp->integer = div_s64_rem(temp_conv(raw_temp), divisor, &(temp->fractional));
  
  return ret;
} // END OF 'adxl355_get_temp'	 

extern adxl355_error_t adxl355_get_nb_fifo_entries(uint8_t *reg_value) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  
  comm_result = read_device_data(ADXL355_ADDR(ADXL355_FIFO_ENTRIES), ADXL355_GET_TRANSF_LEN(ADXL355_FIFO_ENTRIES), reg_value);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
  }
  return ret;
} // END OF 'adxl355_get_nb_fifo_entries'	 

extern adxl355_error_t adxl355_set_nb_fifo_entries(uint8_t reg_value) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  
  if (reg_value > ADXL355_MAX_FIFO_SAMPLES_VAL)
		ret = ADXL355_OUT_OF_RANGE;

	comm_result = write_device_data(ADXL355_ADDR(ADXL355_FIFO_SAMPLES), ADXL355_GET_TRANSF_LEN(ADXL355_FIFO_SAMPLES), &reg_value);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
  } else {
		device_handle.fifo_samples = reg_value;
  }
  
  return ret;
} // END OF 'adxl355_set_nb_fifo_entries'	 

extern adxl355_error_t adxl355_get_raw_fifo_data(uint8_t *fifo_entries, uint32_t *raw_x, uint32_t *raw_y, uint32_t *raw_z) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  
  ret = adxl355_get_nb_fifo_entries(fifo_entries);
	if (ret != ADXL355_OK) {
		return ret;
  }

	if (*fifo_entries > 0) {
    comm_result = read_device_data(ADXL355_ADDR(ADXL355_FIFO_DATA), *fifo_entries * 3, device_handle.comm_buff);
		if (comm_result == false) {
      ret = ADXL355_COMMUNICATION_ERROR;
    } else {
      for (uint16_t idx = 0; idx < *fifo_entries * 3; idx = idx + 9) {
        if (((device_handle.comm_buff[idx+2] & 1) == 1) && ((device_handle.comm_buff[idx+2] & 2) == 0)) {
          // This is x-axis
          // Process data
          raw_x[idx/9] = accel_array_conv(&device_handle.comm_buff[idx]);
          raw_y[idx/9] = accel_array_conv(&device_handle.comm_buff[idx+3]);
          raw_z[idx/9] = accel_array_conv(&device_handle.comm_buff[idx+6]);
        }
      }
    }
  }
	return ret;
} // END OF 'adxl355_get_raw_fifo_data'	 

extern adxl355_error_t adxl355_get_fifo_data(uint8_t *fifo_entries, adxl355_frac_representation_t *x, adxl355_frac_representation_t *y, adxl355_frac_representation_t *z) {
  adxl355_error_t ret = ADXL355_OK;
  uint32_t raw_x[32];
	uint32_t raw_y[32];
	uint32_t raw_z[32];

	ret = adxl355_get_raw_fifo_data(fifo_entries, raw_x, raw_y, raw_z);
	if (ret)
		return ret;

	if (*fifo_entries > 0) {
		for (uint8_t idx = 0; idx < *fifo_entries/3; idx++) {
			x[idx].integer = div_s64_rem(accel_conv(raw_x[idx]),ADXL355_ACC_SCALE_FACTOR_DIV, &(x[idx].fractional));
			y[idx].integer = div_s64_rem(accel_conv(raw_y[idx]),ADXL355_ACC_SCALE_FACTOR_DIV, &(y[idx].fractional));
			z[idx].integer = div_s64_rem(accel_conv(raw_z[idx]),ADXL355_ACC_SCALE_FACTOR_DIV, &(z[idx].fractional));
		}
	}  
  return ret;
} // END OF 'adxl355_get_fifo_data'	 

extern adxl355_error_t adxl355_config_act_enab(adxl355_act_enab_flags_t act_config) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  uint8_t reg_val = act_config.value;

	comm_result = write_device_data(ADXL355_ADDR(ADXL355_ACT_EN), ADXL355_GET_TRANSF_LEN(ADXL355_ACT_EN), &reg_val);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
  } else {
    device_handle.act_en = act_config;
  }
  return ret;
} // END OF 'adxl355_config_act_enab'	 

extern adxl355_error_t adxl355_config_act_thres(uint16_t act_thr) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;  
  uint8_t data[2] = {act_thr >> 8, (uint8_t)act_thr};

	comm_result = write_device_data(ADXL355_ADDR(ADXL355_ACT_THRESH), ADXL355_GET_TRANSF_LEN(ADXL355_ACT_THRESH), (uint8_t*)data);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
  } else {
    device_handle.act_thr = act_thr;
  } 
  return ret;
} // END OF 'adxl355_config_act_thres'	 

extern adxl355_error_t adxl355_set_act_cnt(uint8_t act_cnt) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  
  comm_result = write_device_data(ADXL355_ADDR(ADXL355_ACT_CNT), ADXL355_GET_TRANSF_LEN(ADXL355_ACT_CNT), &act_cnt);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
  } else {
    device_handle.act_cnt = act_cnt;
  } 
  return ret;
} // END OF 'adxl355_set_act_cnt'	 

extern adxl355_error_t adxl355_config_int_pins(adxl355_irq_mask_t int_conf) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;
  uint8_t reg_val = int_conf.value;

	comm_result = write_device_data(ADXL355_ADDR(ADXL355_INT_MAP), ADXL355_GET_TRANSF_LEN(ADXL355_INT_MAP), &reg_val);
  if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
  }
  return ret;
} // END OF 'adxl355_config_int_pins'	 

extern adxl355_error_t adxl355_set_irq_polarity(adxl355_irq_polarity_t int_pol) {
  adxl355_error_t ret = ADXL355_OK;
  bool comm_result;  
  uint8_t reg_value;

	comm_result = read_device_data(ADXL355_ADDR(ADXL355_RANGE), ADXL355_GET_TRANSF_LEN(ADXL355_RANGE), &reg_value);
	if (comm_result == false) {
    ret = ADXL355_COMMUNICATION_ERROR;
  } else {
    reg_value &= ~(ADXL355_IRQ_POL_FIELD_MSK);
    reg_value |= ((int_pol) << 6) & ADXL355_IRQ_POL_FIELD_MSK;
    ret = write_device_data(ADXL355_ADDR(ADXL355_RANGE), ADXL355_GET_TRANSF_LEN(ADXL355_RANGE), &reg_value);
    if (comm_result == false) {
      ret = ADXL355_COMMUNICATION_ERROR;
    } 
  }
  return ret;
} // END OF 'adxl355_set_irq_polarity'	 

extern void adxl355_read_to_buffer() {
  //uint8_t txDummy[ADXL355_FRAMESIZE] = {0};
  adxl355_error_t error;
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint16_t temp16;
  uint32_t temp;
  uint8_t fifo_no;
  
  fifo_no = 1;
  error = adxl355_get_raw_fifo_data(&fifo_no, &x, &y, &z);
  error = adxl355_get_raw_temp(&temp16);
  temp = (uint32_t) temp16;
  
  //lwrb_write(&ADXL355_buffer, &x, sizeof(x));
  //lwrb_write(&ADXL355_buffer, &y, sizeof(y));
  //lwrb_write(&ADXL355_buffer, &z, sizeof(z));
  //lwrb_write(&ADXL355_buffer, &temp, sizeof(temp));      
  //
} // END OF 'adxl355_read_to_buffer'	 

/* END OF FILE */
