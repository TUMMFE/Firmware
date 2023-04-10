/**
 * \file            main.c
 * \brief           Main program body
 */
 
 //---- Include Section ---------------------------------------------------------
#include "main.h"
#include "cmsis_os2.h"	
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_it.h"
#include "bsp.h"
#include "sound.h"
#include "adxl355.h"
#include "sg_lib_conf.h"
#include "sg_io.h"
#include "RTE_Components.h"

#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>                     
#include <stdio.h>


//---- Define Section ----------------------------------------------------------
#define SYSCONFIG_MSGQUEUE_OBJECTS                16      // number of Message Queue Objects
#define DATA_MSGQUEUE_OBJECTS                     16      // number of Message Queue Objects
#define ERROR_CODE_WRONG_CONFIG_DATA              0x0A

     


//---- Enumeration/Typedef Section ---------------------------------------------
typedef enum {
  ACOUSTIC_STIMULATION_MODE,
  EXTERNAL_TRIGGER_BEEP_ON,
  EXTERNAL_TRIGGER_BEEP_OFF,
  FREE_RUNNING_MODE,
  GET_SAMPLE_MODE_NO_EXTI,
  GET_SAMPLE_MODE_WITH_EXTI
} SysConfig_Modes;

typedef enum {
  START,
  STOP,
  PAUSE,
  RESUME,
} SysConfig_State;

typedef struct {
  SOUND_StimulationBlock_TypeDef acoustic;
  adxl355_device_t dev;
  SysConfig_Modes mode;
  SysConfig_State state;
  uint16_t numberOfSamplesAcoustic;
  uint16_t numberOfSamplesTrigger;
  uint16_t numberOfSamplesFreeMode;
  uint16_t sample_counter;
  uint16_t event_counter;
  uint32_t trigger_pol; 
  uint8_t error_code;
  bool config_changed;
  bool mode_changed;
  bool state_changed;
  bool beep;
  bool isRunning;
} SystemConfig_TypeDef;

typedef struct {
  uint8_t error_code;
  uint8_t adxl355_status;
  uint8_t mode;
  uint8_t state;
  uint16_t sample_no;
  uint16_t event_id;
  uint32_t raw_x;
  uint32_t raw_y;
  uint32_t raw_z;
  uint16_t raw_temp;  
} SystemData_TypeDef;

  

//---- Global Section ----------------------------------------------------------
extern uint32_t os_time;      /**< defined in RTOS */

SystemConfig_TypeDef SystemConfiguration;

uint64_t app_main_stk[APP_MAIN_STK_SZ / 8];
osThreadId_t main_id;


const osThreadAttr_t app_main_attr = {
  .stack_mem  = &app_main_stk[0],
  .stack_size = sizeof(app_main_stk)
};


//---- Prototype Section -------------------------------------------------------
static void StandardConfiguration(void);
static void ChangeConfiguration(void);
static void ChangeOperationMode(void);
static void ChangeState(void);
static void USBTransmitData(SystemData_TypeDef *data);


//---- Private Function Section ------------------------------------------------

static void StandardConfiguration(void) {
  SystemConfiguration.dev.dev_type    = ID_ADXL355;
  SystemConfiguration.dev.comm_type   = ADXL355_SPI_COMM;
  SystemConfiguration.dev.op_mode     = ADXL355_STDBY_TEMP_ON_DRDY_ON;
	SystemConfiguration.dev.odr_lpf     = ADXL355_ODR_1000HZ;
	SystemConfiguration.dev.hpf_corner  = ADXL355_HPF_OFF;
	SystemConfiguration.dev.range       = ADXL355_RANGE_8G;
	SystemConfiguration.dev.x_offset    = 0;
	SystemConfiguration.dev.y_offset    = 0;
	SystemConfiguration.dev.z_offset    = 0;
  SystemConfiguration.dev.act_cnt     = 0;
  
  SystemConfiguration.sample_counter = 0;
  SystemConfiguration.event_counter = 0;
  SystemConfiguration.numberOfSamplesAcoustic = 10;
  SystemConfiguration.numberOfSamplesTrigger = 20;
  
  SystemConfiguration.mode = FREE_RUNNING_MODE;
  SystemConfiguration.state = STOP;
  SystemConfiguration.trigger_pol = 0;
  SystemConfiguration.beep = false;
  
  SystemConfiguration.acoustic.fstim = 250;
  SystemConfiguration.acoustic.blockduration = 600;
  SystemConfiguration.acoustic.stimulusduration = 300;
  
  SystemConfiguration.isRunning = false;
  
  SOUND_Init();
  
  SystemConfiguration.config_changed = true;
  SystemConfiguration.mode_changed = true;
  SystemConfiguration.state_changed = true;
 
  ChangeConfiguration();    
} // END OF 'StandardConfiguration'	

static void ChangeConfiguration(void) {
  adxl355_error_t                   error;
  adxl355_device_t                  dev;
  SOUND_StimulationBlock_TypeDef  sound;

  
  if (SystemConfiguration.config_changed == true)  {
    //there is change in the configuration
    
    //reconfiguration of ADXL355 parameters
    dev = SystemConfiguration.dev;
    error = adxl355_set_range(dev.range);
    error = adxl355_set_odr_lpf(dev.odr_lpf);
    error = adxl355_set_hpf_corner(dev.hpf_corner);
    error = adxl355_set_offset(dev.x_offset, dev.y_offset, dev.z_offset);
    error = adxl355_set_act_cnt(dev.act_cnt);
    if (SystemConfiguration.dev.act_cnt > 0) {
      dev.act_en.fields.ACT_X = 1;
      dev.act_en.fields.ACT_Y = 1;
      dev.act_en.fields.ACT_Z = 1;
      error = adxl355_config_act_enab(dev.act_en);
    }
    error = adxl355_config_act_thres(dev.act_thr);
    SystemConfiguration.dev = dev;
    SystemConfiguration.error_code = error;
    
    //reconfiguration of acoustic parameters
    sound = SystemConfiguration.acoustic;
    SOUND_Configuration(sound);
    
    adxl355_update_device(&dev);
        
    //there are no more changes
    SystemConfiguration.config_changed = false;
  }
} // END OF 'ChangeConfiguration'	

static void USBTransmitData(SystemData_TypeDef *data) {
  uint8_t USB_TxBuffer[COM_TX_LENGTH] = {0};
  
  if (data->error_code != 0) {
    USB_TxBuffer[0] = COM_NACK;
  } else {
    USB_TxBuffer[0] = COM_ACK;
  }
  USB_TxBuffer[1] = data->error_code;
  USB_TxBuffer[2] = data->mode;
  USB_TxBuffer[3] = data->state;
      
  USB_TxBuffer[4] = (uint8_t) (data->sample_no >> 8);
  USB_TxBuffer[5] = (uint8_t) (data->sample_no);
      
  USB_TxBuffer[6] = (uint8_t) (data->event_id >> 8);
  USB_TxBuffer[7] = (uint8_t) (data->event_id);
  
  USB_TxBuffer[8] = (uint8_t) (data->raw_temp >> 8);
  USB_TxBuffer[9] = (uint8_t) (data->raw_temp);
        
  USB_TxBuffer[10] = (uint8_t) (data->raw_x >> 24);
  USB_TxBuffer[11] = (uint8_t) (data->raw_x >> 16);
  USB_TxBuffer[12] = (uint8_t) (data->raw_x >> 8);
  USB_TxBuffer[13] = (uint8_t) (data->raw_x);
        
  USB_TxBuffer[14] = (uint8_t) (data->raw_y >> 24);
  USB_TxBuffer[15] = (uint8_t) (data->raw_y >> 16);
  USB_TxBuffer[16] = (uint8_t) (data->raw_y >> 8);
  USB_TxBuffer[17] = (uint8_t) (data->raw_y);
       
  USB_TxBuffer[18] = (uint8_t) (data->raw_z >> 24);
  USB_TxBuffer[19] = (uint8_t) (data->raw_z >> 16);
  USB_TxBuffer[20] = (uint8_t) (data->raw_z >> 8);
  USB_TxBuffer[21] = (uint8_t) (data->raw_z);
        
  
     
  CDC_Transmit_FS(USB_TxBuffer, COM_TX_LENGTH);    
} // END OF 'USBTransmitData'
  
static void ChangeOperationMode(void) {  
  if (SystemConfiguration.mode_changed == true)  {
    switch (SystemConfiguration.mode) {
           case ACOUSTIC_STIMULATION_MODE:                
             SG_IO_IRQDetach(TRIGGER_IN_Pin);              
             break;
           case EXTERNAL_TRIGGER_BEEP_ON:     
             SG_IO_IRQAttach(TRIGGER_IN_GPIO_Port, TRIGGER_IN_Pin, SystemConfiguration.trigger_pol);
             SystemConfiguration.beep = true;
             break;
           case EXTERNAL_TRIGGER_BEEP_OFF:
             SG_IO_IRQAttach(TRIGGER_IN_GPIO_Port, TRIGGER_IN_Pin, SystemConfiguration.trigger_pol);
             SystemConfiguration.beep = false;
             break;
           case FREE_RUNNING_MODE:
             break;
           case GET_SAMPLE_MODE_NO_EXTI:
             SG_IO_IRQDetach(TRIGGER_IN_Pin);
             break;
           case GET_SAMPLE_MODE_WITH_EXTI:
             SG_IO_IRQAttach(TRIGGER_IN_GPIO_Port, TRIGGER_IN_Pin, SystemConfiguration.trigger_pol);
             break;
           default:
             SystemConfiguration.error_code = ERROR_CODE_WRONG_CONFIG_DATA;
             break;
         } //switch
    SystemConfiguration.mode_changed = false;      
  }   
} // END OF 'ChangeOperationMode'	

static void ChangeState(void) { 
  adxl355_device_t dev;
  if (SystemConfiguration.state_changed == true)  {
    switch (SystemConfiguration.state) {
            case START:
              SystemConfiguration.sample_counter = 0;
              SystemConfiguration.event_counter = 0;
              SystemConfiguration.dev.op_mode = ADXL355_MEAS_TEMP_ON_DRDY_ON; 
              SG_IO_IRQAttach(DRDY_IN_GPIO_Port, DRDY_Pin, SG_IO_RisingEdge);
              SystemConfiguration.isRunning = true;
              if (SystemConfiguration.mode == ACOUSTIC_STIMULATION_MODE) {
                SOUND_Run();
              }
              if (((SystemConfiguration.mode == GET_SAMPLE_MODE_NO_EXTI) || (SystemConfiguration.mode == GET_SAMPLE_MODE_WITH_EXTI)) && (SystemConfiguration.beep == true)) {                               
                SOUND_ReleaseStimulus();
              }
              break;
            case STOP:              
              SG_IO_IRQDetach(DRDY_Pin); 
              SystemConfiguration.sample_counter = 0;
              SystemConfiguration.event_counter = 0;
              SystemConfiguration.isRunning = false;
              if (SystemConfiguration.mode == ACOUSTIC_STIMULATION_MODE) {
                SOUND_Cancel();
              }
              SystemConfiguration.sample_counter = 0;
              SystemConfiguration.event_counter = 0;
              SystemConfiguration.dev.op_mode = ADXL355_STDBY_TEMP_ON_DRDY_ON;  
              break;
            case PAUSE:
              SystemConfiguration.dev.op_mode = ADXL355_STDBY_TEMP_ON_DRDY_ON; 
              SG_IO_IRQDetach(DRDY_Pin);
              SystemConfiguration.isRunning = false;
              if (SystemConfiguration.mode == ACOUSTIC_STIMULATION_MODE) {
                SOUND_Pause();
              }
              break;
            case RESUME:
              SystemConfiguration.dev.op_mode = ADXL355_MEAS_TEMP_ON_DRDY_ON;  
              SG_IO_IRQAttach(DRDY_IN_GPIO_Port, DRDY_Pin, SG_IO_RisingEdge);
              SystemConfiguration.isRunning = true;
              if (SystemConfiguration.mode == ACOUSTIC_STIMULATION_MODE) {
                SOUND_Resume();
              }
              if (((SystemConfiguration.mode == GET_SAMPLE_MODE_NO_EXTI) || (SystemConfiguration.mode == GET_SAMPLE_MODE_WITH_EXTI)) && (SystemConfiguration.beep == true)) {                               
                SOUND_ReleaseStimulus();
              }
              break;
            default:
              SystemConfiguration.error_code = ERROR_CODE_WRONG_CONFIG_DATA;
              break;            
          } //switch
    dev = SystemConfiguration.dev;
    adxl355_update_device(&dev);
    SystemConfiguration.error_code = adxl355_set_op_mode(dev.op_mode);
    SystemConfiguration.state_changed = false;
  }
} // END OF 'ChangeState'



/******************************************************************************/
/*  			                Application main thread                             */ 
/******************************************************************************/
__NO_RETURN static void app_main(void *argument) {	
  (void)argument;
  uint8_t USB_TxBuffer[COM_TX_LENGTH] = {0};
  uint32_t flags; 
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint16_t temp;
  uint8_t fifo_no;

  adxl355_error_t                   error;
  adxl355_device_t                  dev;
  adxl355_irq_mask_t                adxl_irq_flags;
  SystemData_TypeDef              data;
    
  /* on board LEDs */
  SG_IO_Init(LED0_GPIO_Port, LED0_Pin, SG_IO_Output, SG_IO_PushPull, SG_IO_NoPullUpOrDown, SG_IO_Low);
  
   
  /* chip select */
  SG_IO_Init(CS_GPIO_Port, CS_Pin, SG_IO_Output, SG_IO_PushPull, SG_IO_NoPullUpOrDown, SG_IO_Low);
  SG_IO_SetPinHigh(CS_GPIO_Port, CS_Pin); 
  
  //turn the LED on 
  SG_IO_SetPinLow(LED0_GPIO_Port, LED0_Pin);
  
  SG_IO_Init(DRDY_IN_GPIO_Port, DRDY_Pin, SG_IO_Input, SG_IO_PushPull, SG_IO_NoPullUpOrDown, SG_IO_Low);
  SG_IO_Init(INT1_IN_GPIO_Port, INT1_Pin, SG_IO_Input, SG_IO_PushPull, SG_IO_NoPullUpOrDown, SG_IO_Low);
  SG_IO_Init(INT2_IN_GPIO_Port, INT2_Pin, SG_IO_Input, SG_IO_PushPull, SG_IO_NoPullUpOrDown, SG_IO_Low);
  
  SG_IO_Init(TRIGGER_IN_GPIO_Port, TRIGGER_IN_Pin, SG_IO_Input, SG_IO_PushPull, SG_IO_NoPullUpOrDown, SG_IO_Low);
  SG_IO_SetPinHigh(CS_GPIO_Port, CS_Pin);    //deselect the SPI slave device
  MX_USB_DEVICE_Init();		//init the USB device  
  
  //configuration of ADXL355
  //SG_IO_SetPinLow(CS_GPIO_Port, CS_Pin);    //select the SPI slave device
  dev = SystemConfiguration.dev;
  SystemConfiguration.error_code = adxl355_init(&dev);
  
  adxl_irq_flags.fields.RDY_EN1 = 1;
  SystemConfiguration.error_code = adxl355_config_int_pins(adxl_irq_flags);
  SystemConfiguration.error_code = adxl355_set_irq_polarity(ADXL355_INT_ACTIVE_HIGH);
  
  StandardConfiguration();
    
	while (1) {	
    //main programm loop
    
    //1. check if some system state, system mode or system configuration has changed
    flags = osThreadFlagsWait(FLAG_SYSTEM_CONFIG_CHANGED, osFlagsNoClear, 0);
    if (flags == FLAG_SYSTEM_CONFIG_CHANGED) {
      //some changed from the host needs to be processed
      if (SystemConfiguration.config_changed == true) {
        ChangeConfiguration();
      } 
      if (SystemConfiguration.mode_changed == true) {
        ChangeOperationMode();
      } 
      if (SystemConfiguration.state_changed == true) {
        ChangeState();
      }
      osThreadFlagsClear(FLAG_SYSTEM_CONFIG_CHANGED);
    } // FLAG_SYSTEM_CONFIG_CHANGED
    
    //2. different run time modes
    if (SystemConfiguration.isRunning == true) {
      //2a. free running mode
      dev = SystemConfiguration.dev;
      if (SystemConfiguration.mode == FREE_RUNNING_MODE) {
        ////no need to wait for some signals from other threads, just react on DRDY
        flags = osThreadFlagsWait(FLAG_EXTI_DRDY, osFlagsWaitAny, osWaitForever);
        if (flags == FLAG_EXTI_DRDY) {
          fifo_no = 3;
          //error = adxl355_get_raw_fifo_data(&fifo_no, &x, &y, &z);
          error = adxl355_get_raw_temp(&temp);
          error = adxl355_get_raw_xyz(&x, &y, &z);
          
          data.error_code = error;     
          data.sample_no = 1;
          data.event_id = 1;
          data.state = SystemConfiguration.state;
          data.mode = SystemConfiguration.mode;
          data.raw_x = x;
          data.raw_y = y;
          data.raw_z = z;
          data.raw_temp = temp;   
          USBTransmitData(&data);  
        }
      } else if (SystemConfiguration.mode == ACOUSTIC_STIMULATION_MODE) {
        //2b. acoustic stimulation mode
        flags = osEventFlagsWait(evt_id, FLAG_START_DAQ|FLAG_EXTI_DRDY, osFlagsNoClear, 0);
        if (flags == FLAG_START_DAQ) {           
          flags = osThreadFlagsWait(FLAG_EXTI_DRDY, osFlagsWaitAny, osWaitForever);
          if ((flags == FLAG_EXTI_DRDY) && (SystemConfiguration.sample_counter < SystemConfiguration.numberOfSamplesAcoustic)) {                       
            fifo_no = 3;
            //error = adxl355_get_raw_fifo_data(&fifo_no, &x, &y, &z);
            error = adxl355_get_raw_temp(&temp);
            error = adxl355_get_raw_xyz(&x, &y, &z);
            data.error_code = error;     
            data.sample_no = SystemConfiguration.sample_counter;
            data.event_id = SystemConfiguration.event_counter;
            data.state = SystemConfiguration.state;
            data.mode = SystemConfiguration.mode;
            data.raw_x = x;
            data.raw_y = y;
            data.raw_z = z;
            data.raw_temp = temp;   
            USBTransmitData(&data);
            SystemConfiguration.sample_counter++;
            osEventFlagsClear(evt_id, FLAG_EXTI_DRDY);
          } else if (SystemConfiguration.sample_counter == SystemConfiguration.numberOfSamplesAcoustic) {
            SystemConfiguration.event_counter++;  
            SystemConfiguration.sample_counter = 0;   
            osEventFlagsClear(evt_id, FLAG_START_DAQ);
          }
        }      
      } else if (SystemConfiguration.mode == EXTERNAL_TRIGGER_BEEP_ON) {
      } else if (SystemConfiguration.mode == EXTERNAL_TRIGGER_BEEP_OFF) {
      } else if (SystemConfiguration.mode == GET_SAMPLE_MODE_WITH_EXTI) {
      } else if (SystemConfiguration.mode == GET_SAMPLE_MODE_NO_EXTI) {         
          flags = osThreadFlagsWait(FLAG_EXTI_DRDY, osFlagsWaitAny, osWaitForever);
          if ((flags == FLAG_EXTI_DRDY) && (SystemConfiguration.sample_counter < SystemConfiguration.numberOfSamplesFreeMode)) {  
            
            fifo_no = 3;
            //error = adxl355_get_raw_fifo_data(&fifo_no, &x, &y, &z);
            error = adxl355_get_raw_temp(&temp);
            error = adxl355_get_raw_xyz(&x, &y, &z);
            data.error_code = error;     
            data.sample_no = SystemConfiguration.sample_counter;
            data.event_id = SystemConfiguration.event_counter;
            data.state = SystemConfiguration.state;
            data.mode = SystemConfiguration.mode;
            data.raw_x = x;
            data.raw_y = y;
            data.raw_z = z;
            data.raw_temp = temp;   
            USBTransmitData(&data);
            SystemConfiguration.sample_counter++;
          } else if (SystemConfiguration.sample_counter == SystemConfiguration.numberOfSamplesFreeMode) {
            SystemConfiguration.event_counter++;  
            SystemConfiguration.sample_counter = 0; 
            SystemConfiguration.mode = PAUSE;
            SystemConfiguration.mode_changed = true; 
            ChangeOperationMode();
          }
      }
        
    } // SystemConfiguration.isRunning == true
  }  //while (1)
} // END OF 'app_main'	


/**
 * Main function: Application Entry Point
 */
int main(void) {
  HAL_Init();             //Init HAL
	
  if (SG_RCC_Init() != SG_RCC_Ok) {
    Error_Handler();
  }
  SystemCoreClockUpdate();
  
		
	osKernelInitialize();   //Start RTX Kernel
  
  
	main_id = osThreadNew(app_main, NULL, &app_main_attr); // Create application main thread
  if (main_id == NULL) {
    Error_Handler();
  }
  
	osKernelStart();        //Start OS scheduler
} // END OF 'main'


/**
  * This Callback is called by incoming Data via USB
  */
void CDC_Receive_FS_Callback(uint8_t* Buf, uint32_t *Len) {
  uint8_t Cmd;
    
  if (*Len >= COM_RX_LENGTH) {
    /* the received message has at least the "correct" length */    
    Cmd = Buf[0];
  } else {
    /* a command of wrong size was received, thus we leave the function without any action */
    return;
  }  
  SystemConfiguration.config_changed = false;
  SystemConfiguration.mode_changed = false;
  SystemConfiguration.state_changed = false;
  switch (Cmd) {
        case COM_CMD_SET_ACOUSTIC_PARAM:
          SystemConfiguration.config_changed = true;
          SystemConfiguration.numberOfSamplesAcoustic =           (Buf[1] << 8) | (Buf[2] << 0);
          SystemConfiguration.acoustic.fstim =            (Buf[3] << 8) | (Buf[4] << 0); 
          SystemConfiguration.acoustic.blockduration =    (Buf[5] << 8) | (Buf[6] << 0); 
          SystemConfiguration.acoustic.stimulusduration = (Buf[7] << 8) | (Buf[8] << 0);         
          break;
        case COM_CMD_SET_TRIGGER_PARAM:
          SystemConfiguration.config_changed = true;          
          SystemConfiguration.numberOfSamplesTrigger =           (Buf[1] << 8) | (Buf[2] << 0);
          if (Buf[3] == COM_VAL_IT_FALLING) {
            SystemConfiguration.trigger_pol = GPIO_MODE_IT_FALLING;
          } else if (Buf[3] == COM_VAL_IT_BOTH) {
            SystemConfiguration.trigger_pol = GPIO_MODE_IT_RISING_FALLING;
          } else {
            SystemConfiguration.trigger_pol = GPIO_MODE_IT_RISING;
          }
          break;
        case COM_CMD_SET_ADXL355_PARAM:
          SystemConfiguration.config_changed = true; 
          if ((Buf[1] >= MIX_INT_OF_HPF) && (Buf[1] <= MAX_INT_OF_HPF)) {
            SystemConfiguration.dev.hpf_corner = Buf[1];
          }
          if ((Buf[2] >= MIX_INT_OF_ODR_LPF) && (Buf[2] <= MAX_INT_OF_ODR_LPF)) {
            SystemConfiguration.dev.odr_lpf = Buf[2];
          }
          if ((Buf[3] >= MIX_INT_OF_RANGE) && (Buf[3] <= MAX_INT_OF_RANGE)) {
            SystemConfiguration.dev.range = Buf[3];
          }
          SystemConfiguration.dev.act_cnt = Buf[4];     
          break;
        case COM_CMD_SET_ADXL355_OFFSET:
          SystemConfiguration.config_changed = true;
          SystemConfiguration.dev.x_offset =  (Buf[1] << 8) | (Buf[2] << 0);
          SystemConfiguration.dev.y_offset =  (Buf[3] << 8) | (Buf[4] << 0); 
          SystemConfiguration.dev.z_offset =  (Buf[5] << 8) | (Buf[6] << 0); 
          SystemConfiguration.dev.act_thr =   (Buf[7] << 8) | (Buf[8] << 0);
          break;
        case COM_CMD_SET_STATE_MODE:            
          if ((Buf[1] >= MIN_INT_OF_MODES) && (Buf[1] <= MAX_INT_OF_MODES)) {
            SystemConfiguration.mode = Buf[1];
            SystemConfiguration.mode_changed = true;
          }
          if ((Buf[2] >= MIN_INT_OF_STATES) && (Buf[2] <= MAX_INT_OF_STATES)) {
            SystemConfiguration.state = Buf[2];
            SystemConfiguration.state_changed = true;
          }
          break;
        case COM_CMD_GET_SAMPLES:          
          SystemConfiguration.numberOfSamplesFreeMode = (Buf[1] << 8) | (Buf[2] << 0);
          if (Buf[4] == 0x00) {
            SystemConfiguration.mode = GET_SAMPLE_MODE_NO_EXTI;
          } else {
            SystemConfiguration.mode = GET_SAMPLE_MODE_WITH_EXTI;
          }
          if (Buf[3] == 0x00) {
            SystemConfiguration.beep = false; 
          } else {
            SystemConfiguration.beep = true;
          }            
          SystemConfiguration.state = RESUME;  //start
          SystemConfiguration.config_changed = true;
          SystemConfiguration.state_changed = true;
          SystemConfiguration.mode_changed = true;     
          break;  
        default:
          break;
  }      
  if ((SystemConfiguration.config_changed == true) || (SystemConfiguration.state_changed == true) || (SystemConfiguration.mode_changed == true)) {
    osThreadFlagsSet(main_id, FLAG_SYSTEM_CONFIG_CHANGED);
  }
} // END OF 'CDC_Receive_FS_Callback'  


/**
  * Override default HAL_GetTick function
  */
uint32_t HAL_GetTick (void) {
  static uint32_t ticks = 0U;
         uint32_t i;

  if (osKernelGetState () == osKernelRunning) {
    return ((uint32_t)osKernelGetTickCount ());
  }

  /* If Kernel is not running wait approximately 1 ms then increment 
     and return auxiliary tick counter value */
  for (i = (SystemCoreClock >> 14U); i > 0U; i--) {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  }
  return ++ticks;
} // END OF 'HAL_GetTick'


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  NVIC_SystemReset();
  while (1) {
  }
} // END OF 'Error_Handler'

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}  // END OF 'assert_failed'
#endif /* USE_FULL_ASSERT */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
