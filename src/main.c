
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint16_t gLastError;

union powerstep01_Init_u initDeviceParametersDevice =
{
  /* common parameters */
  .cm.cp.cmVmSelection = POWERSTEP01_CM_VM_CURRENT, // enum powerstep01_CmVm_t
  582, // Acceleration rate in step/s2, range 14.55 to 59590 steps/s^2
  582, // Deceleration rate in step/s2, range 14.55 to 59590 steps/s^2
  488, // Maximum speed in step/s, range 15.25 to 15610 steps/s
  0, // Minimum speed in step/s, range 0 to 976.3 steps/s
  POWERSTEP01_LSPD_OPT_OFF, // Low speed optimization bit, enum powerstep01_LspdOpt_t
  244.16, // Full step speed in step/s, range 7.63 to 15625 steps/s
  POWERSTEP01_BOOST_MODE_OFF, // Boost of the amplitude square wave, enum powerstep01_BoostMode_t
  281.25, // Overcurrent threshold settings via enum powerstep01_OcdTh_t
  STEP_MODE_1_16, // Step mode settings via enum motorStepMode_t
  POWERSTEP01_SYNC_SEL_DISABLED, // Synch. Mode settings via enum powerstep01_SyncSel_t
  (POWERSTEP01_ALARM_EN_OVERCURRENT|
   POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN|
   POWERSTEP01_ALARM_EN_THERMAL_WARNING|
   POWERSTEP01_ALARM_EN_UVLO|
   POWERSTEP01_ALARM_EN_STALL_DETECTION|
   POWERSTEP01_ALARM_EN_SW_TURN_ON|
   POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD), // Alarm settings via bitmap enum powerstep01_AlarmEn_t
  POWERSTEP01_IGATE_64mA, // Gate sink/source current via enum powerstep01_Igate_t 
  POWERSTEP01_TBOOST_0ns, // Duration of the overboost phase during gate turn-off via enum powerstep01_Tboost_t
  POWERSTEP01_TCC_500ns, // Controlled current time via enum powerstep01_Tcc_t
  POWERSTEP01_WD_EN_DISABLE, // External clock watchdog, enum powerstep01_WdEn_t  
  POWERSTEP01_TBLANK_375ns, // Duration of the blanking time via enum powerstep01_TBlank_t
  POWERSTEP01_TDT_125ns, // Duration of the dead time via enum powerstep01_Tdt_t
  /* current mode parameters */
  328.12, // Hold torque in mV, range from 7.8mV to 1000 mV
  328.12, // Running torque in mV, range from 7.8mV to 1000 mV 
  328.12, // Acceleration torque in mV, range from 7.8mV to 1000 mV
  328.12, // Deceleration torque in mV, range from 7.8mV to 1000 mV
  POWERSTEP01_TOFF_FAST_8us, //Maximum fast decay time , enum powerstep01_ToffFast_t 
  POWERSTEP01_FAST_STEP_12us, //Maximum fall step time , enum powerstep01_FastStep_t 
  3.0, // Minimum on-time in us, range 0.5us to 64us
  21.0, // Minimum off-time in us, range 0.5us to 64us 
  POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ, // Clock setting , enum powerstep01_ConfigOscMgmt_t
  POWERSTEP01_CONFIG_SW_HARD_STOP, // External switch hard stop interrupt mode, enum powerstep01_ConfigSwMode_t
  POWERSTEP01_CONFIG_TQ_REG_TVAL_USED, // External torque regulation enabling , enum powerstep01_ConfigEnTqReg_t
  POWERSTEP01_CONFIG_VS_COMP_DISABLE, // Motor Supply Voltage Compensation enabling , enum powerstep01_ConfigEnVscomp_t 
  POWERSTEP01_CONFIG_OC_SD_DISABLE, // Over current shutwdown enabling, enum powerstep01_ConfigOcSd_t
  POWERSTEP01_CONFIG_UVLOVAL_LOW, // UVLO Threshold via powerstep01_ConfigUvLoVal_t
  POWERSTEP01_CONFIG_VCCVAL_15V, // VCC Val, enum powerstep01_ConfigVccVal_t
  POWERSTEP01_CONFIG_TSW_048us, // Switching period, enum powerstep01_ConfigTsw_t
  POWERSTEP01_CONFIG_PRED_DISABLE, // Predictive current enabling , enum powerstep01_ConfigPredEn_t
};

/* Private function prototypes -----------------------------------------------*/
static void MyBusyInterruptHandler(void);
static void MyFlagInterruptHandler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  int32_t pos;
  uint32_t myMaxSpeed;
  uint32_t myMinSpeed;
  uint16_t myAcceleration;
  uint16_t myDeceleration;
  uint32_t readData;

  struct motor_type{
	  uint32_t step_per_cycle = 200;
	  double gearbox_ratio = 6.5637;
  } motor_nema;

  /* STM32F4xx HAL library initialization */
  HAL_Init();
  
    /* Configure the system clock */
    SystemClock_Config();

  	/* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    
    char *msg = "Powerstep01 running test\n\r";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);


//----- Init of the Powerstep01 library 
  /* Set the Powerstep01 library to use 3 devices */
  BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, 3);
  /* When BSP_MotorControl_Init is called with NULL pointer,                  */
  /* the Powerstep01 registers are set with the predefined values from file   */
  /* powerstep01_target_config.h, otherwise the registers are set using the   */
  /* powerstep01_Init_u relevant union of structure values.                   */
  /* The first call to BSP_MotorControl_Init initializes the first device     */
  /* whose Id is 0.                                                           */
  /* The nth call to BSP_MotorControl_Init initializes the nth device         */
  /* whose Id is n-1.                                                         */
  /* Uncomment the 3 calls to BSP_MotorControl_Init below to initialize the   */
  /* devices with the union declared in the the main.c file and comment the   */
  /* 3 subsequent calls having the NULL pointer                               */

  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, &initDeviceParametersDevice);
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, &initDeviceParametersDevice);
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, &initDeviceParametersDevice);

//  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, NULL);
//  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, NULL);
//  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, NULL);
  
  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Attach the function MyBusyInterruptHandler (defined below) to the busy interrupt */
  BSP_MotorControl_AttachBusyInterrupt(MyBusyInterruptHandler);
  
  /* Attach the function Error_Handler (defined below) to the error Handler*/
  BSP_MotorControl_AttachErrorHandler(MyErrorHandler);

  /* Set Current position to be the home position for device 0 */
  BSP_MotorControl_CmdResetPos(0);

  /* Set Current position to be the home position for device 1 */
  BSP_MotorControl_CmdResetPos(1);

  /* Set Current position to be the home position for device 2 */
  BSP_MotorControl_CmdResetPos(2);

  /* Request device 0 to Goto position 3200 */
  BSP_MotorControl_GoTo(0,3200);  

  /* Wait for device 0 ends moving */  
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(0);

  /* If the read position of device 0 is 3200 */
  /* Request device 1 to go to the same position */
  if (pos == 3200)
  {
    /* Set current position of device 0 to be its mark position*/
    BSP_MotorControl_SetMark(0); 
    
    /* Request device 1 to Go to the same position  */ 
    BSP_MotorControl_GoTo(1,pos); 

    /* Wait for  device 1 ends moving */  
    BSP_MotorControl_WaitWhileActive(1);
  }
  
  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(1);

  /* If the read position of device 1 is 3200 */
  /* Request device 2 to go to the same position */
  if (pos == 3200)
  {
    /* Request device 2 to Go to the same position  */ 
    BSP_MotorControl_GoTo(2,pos); 
  
    /* Wait for device 2 ends moving */  
    BSP_MotorControl_WaitWhileActive(2);
  }

  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(2);

  /* Wait for 1s */
  HAL_Delay(1000);
  
  if (pos == 3200)
  {
    /* Request all devices to go home */
    BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_HOME,0);
    BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_HOME,0);
    BSP_MotorControl_QueueCommands(2,POWERSTEP01_GO_HOME,0);
    BSP_MotorControl_SendQueuedCommands();

    /* Wait for all device ends moving */ 
    BSP_MotorControl_WaitForAllDevicesNotBusy();
  }
  
  /* Wait for 1s */
  HAL_Delay(1000);
  
  /* Request device 0 to Goto position -3200 */ 
  BSP_MotorControl_GoTo(0,-3200);  

  /* Wait for device 0 ends moving */  
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(0);
    
  /* If the read position of device 0 is -3200 */
  /* Request device 1 to go to the same position */
  if (pos == -3200)
  {
    /* Request device 1 to go to the same position  */ 
    BSP_MotorControl_GoTo(1,pos); 

    /* Wait for  device 1 ends moving */  
    BSP_MotorControl_WaitWhileActive(1);
  }
  
  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(1);

  /* If the read position of device 1 is -3200 */
  /* Request device 2 to go to the same position */
  if (pos == -3200)
  {
    /* Request device 2 to Go to the same position  */ 
    BSP_MotorControl_GoTo(2,pos); 
  
    /* Wait for device 2 ends moving */  
    BSP_MotorControl_WaitWhileActive(2);
  }

  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(2);

  /* Wait for 1s */
  HAL_Delay(1000);
  
  if (pos == -3200)
  {
    /* Set current position of device 2 to be its mark position*/
    BSP_MotorControl_SetMark(2); 

    /* Request all devices to go home */
    BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_HOME,0);
    BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_HOME,0);
    BSP_MotorControl_QueueCommands(2,POWERSTEP01_GO_HOME,0);
    BSP_MotorControl_SendQueuedCommands();
    
    /* Wait for all device ends moving */ 
    BSP_MotorControl_WaitForAllDevicesNotBusy();
  }

  /* Wait for 1s */
  HAL_Delay(1000);
  
  /* Request device 0 and device 2 to go their mark position */
  BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_MARK,0);
  BSP_MotorControl_QueueCommands(1,POWERSTEP01_NOP,0);
  BSP_MotorControl_QueueCommands(2,POWERSTEP01_GO_MARK,0);
  BSP_MotorControl_SendQueuedCommands();
 
  /* Wait for device 0 and 2 ends moving */ 
  BSP_MotorControl_WaitForAllDevicesNotBusy();
  
  /* Wait for 1s */
  HAL_Delay(1000);

  /* Request device 0 to run in FORWARD direction at 400 steps/s*/
  BSP_MotorControl_CmdRun(0, FORWARD, Speed_Steps_to_Par(400));

  /* Wait for device 0 reaches the speed of 400 steps/s */
  do
  {
    readData = BSP_MotorControl_CmdGetParam(0, POWERSTEP01_SPEED);
  }while (readData != Speed_Steps_to_Par(400));

  /* Request device 0 to run in FORWARD direction at 300 steps/s*/
  BSP_MotorControl_CmdRun(1,FORWARD, Speed_Steps_to_Par(300));

/* Wait for device 1 reaches the speed of 300 steps/s */
    do
  {
    readData = BSP_MotorControl_CmdGetParam(1, POWERSTEP01_SPEED);
  }while (readData != Speed_Steps_to_Par(300));

  /* Request device 2 to run in FORWARD direction at 200 steps/s*/
  BSP_MotorControl_CmdRun(2,FORWARD, Speed_Steps_to_Par(200));


/* Wait for device 2 reaches the speed of 200 steps/s */
  do
  {
    readData = BSP_MotorControl_CmdGetParam(2, POWERSTEP01_SPEED);
  }while (readData != Speed_Steps_to_Par(200));

  /* Wait for 3s */
  HAL_Delay(3000);

  /* Request device 1 to make a soft stop */
  BSP_MotorControl_CmdSoftStop(1);
  
  /* Wait for device 1 end moving */
  BSP_MotorControl_WaitWhileActive(1);  

  /* Request device 0 and 2 to make a hard stop */
  BSP_MotorControl_QueueCommands(0,POWERSTEP01_HARD_STOP,0);
  BSP_MotorControl_QueueCommands(1,POWERSTEP01_NOP,0);
  BSP_MotorControl_QueueCommands(2,POWERSTEP01_HARD_STOP,0);
  BSP_MotorControl_SendQueuedCommands();

  /* Wait for both devices end moving */  
  BSP_MotorControl_WaitForAllDevicesNotBusy();  

  /* Request all devices to go home */
  BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_HOME,0);
  BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_HOME,0);
  BSP_MotorControl_QueueCommands(2,POWERSTEP01_GO_HOME,0);
  BSP_MotorControl_SendQueuedCommands();
    
  /* Wait for all device ends moving */ 
  BSP_MotorControl_WaitForAllDevicesNotBusy();

  /* Get acceleration, deceleration, Maxspeed and MinSpeed of device 0*/
  myMaxSpeed= BSP_MotorControl_CmdGetParam(0, POWERSTEP01_MAX_SPEED);
  myAcceleration = BSP_MotorControl_CmdGetParam(0, POWERSTEP01_ACC);
  myDeceleration = BSP_MotorControl_CmdGetParam(0, POWERSTEP01_DEC);
  myMinSpeed  = BSP_MotorControl_CmdGetParam(0, POWERSTEP01_MIN_SPEED); 
  
  /* Select 1/16 microstepping mode for device 0 */
  BSP_MotorControl_SelectStepMode(0, STEP_MODE_1_16);
  
  /* Select 1/8 microstepping mode for device 1 */
  BSP_MotorControl_SelectStepMode(1, STEP_MODE_1_8);
  
  /* Set speed and acceleration of device 1 */
  /* Do not scale with microstepping mode */
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_ACC, myAcceleration);
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_DEC, myDeceleration);
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_MIN_SPEED, myMinSpeed);
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_MAX_SPEED, myMaxSpeed);
  
  /* Select ful step mode for device 2 */
  BSP_MotorControl_SelectStepMode(2, STEP_MODE_FULL);

  /* Set speed and acceleration of device 2 */
  /* Do not scale with microstepping mode */
  BSP_MotorControl_CmdSetParam(2, POWERSTEP01_ACC, myAcceleration);
  BSP_MotorControl_CmdSetParam(2, POWERSTEP01_DEC, myDeceleration);
  BSP_MotorControl_CmdSetParam(2, POWERSTEP01_MIN_SPEED, myMinSpeed);
  BSP_MotorControl_CmdSetParam(2, POWERSTEP01_MAX_SPEED, myMaxSpeed);
 
  /* Infinite loop */
  while(1)
  {
    /* device 0 is using 1/16 microstepping mode */ 
    /* device 1 is using 1/8 microstepping mode */
    /* device 2 is using full step mode */
    /* position is in microsteps */
    BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_TO,-3200);
    BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_TO,1600);
    BSP_MotorControl_QueueCommands(2,POWERSTEP01_GO_TO,-200);
    BSP_MotorControl_SendQueuedCommands();
    
    /* Wait for all device ends moving */ 
    BSP_MotorControl_WaitForAllDevicesNotBusy();

    BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_TO,3200);
    BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_TO,-1600);
    BSP_MotorControl_QueueCommands(2,POWERSTEP01_GO_TO,200);
    BSP_MotorControl_SendQueuedCommands();
    
    /* Wait for all device ends moving */ 
    BSP_MotorControl_WaitForAllDevicesNotBusy();
  }

}

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  char *msg;
  /* Get the value of the status register via the command GET_STATUS */
  uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);

  /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & POWERSTEP01_STATUS_HIZ) == POWERSTEP01_STATUS_HIZ)
  {
    // HIZ state
  }

  /* Check BUSY flag: if not set, a command is under execution */
  if ((statusRegister & POWERSTEP01_STATUS_BUSY) == 0)
  {
    // BUSY
  }

  /* Check SW_F flag: if not set, the SW input is opened */
  if ((statusRegister & POWERSTEP01_STATUS_SW_F ) == 0)
  {
     // SW OPEN
	 *msg = "SW_F flag not set: Switch input open.\n\r";
	 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }
  else
  {
    // SW CLOSED
	  *msg = "SW_F flag not set: Switch input closed.\n\r";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }  
  /* Check SW_EN bit */
  if ((statusRegister & POWERSTEP01_STATUS_SW_EVN) == POWERSTEP01_STATUS_SW_EVN)
  {
    // switch turn_on event
  }  
  /* Check direction bit */
  if ((statusRegister & POWERSTEP01_STATUS_DIR) == 0)
  {
    // BACKWARD
  }
  else  
  {
    // FORWARD 
  }
  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_STOPPED )
  {
       // MOTOR STOPPED
  }
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_ACCELERATION )
  {
           // MOTOR ACCELERATION
  }  
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_DECELERATION )
  {
           // MOTOR DECELERATION
  }
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_CONST_SPD )
  {
       // MOTOR RUNNING AT CONSTANT SPEED
  }  

  /* Check Command Error flag: if set, the command received by SPI can't be performed */
  /* This often occures when a command is sent to the Powerstep01 */
  /* while it is in HIZ state or it the sent command does not exist*/
  if ((statusRegister & POWERSTEP01_STATUS_CMD_ERROR) == POWERSTEP01_STATUS_CMD_ERROR)
  {
       // Command Error
  }  

  /* Check Step mode clock flag: if set, the device is working in step clock mode */
  if ((statusRegister & POWERSTEP01_STATUS_STCK_MOD) == POWERSTEP01_STATUS_STCK_MOD)
  {
     //Step clock mode enabled
  }  

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & POWERSTEP01_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out 
	 *msg = "UVLO flag not set: undervoltage lock-out.\n\r";
	 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

  }  

  /* Check UVLO ADC flag: if not set, there is an ADC undervoltage lock-out */
  if ((statusRegister & POWERSTEP01_STATUS_UVLO_ADC) == 0)
  {
     //ADC undervoltage lock-out 
  } 
  
  /* Check thermal STATUS flags: if  set, the thermal status is not normal */
  if ((statusRegister & POWERSTEP01_STATUS_TH_STATUS) != 0)
  {
    //thermal status: 1: Warning, 2: Bridge shutdown, 3: Device shutdown
	  switch(POWERSTEP01_STATUS_TH_STATUS){
	  case 1:
		  *msg = "Thermal status 1: Warning.\n\r";
		  break;
	  case 2:
		  *msg = "Thermal status 2: Bridge shutdown.\n\r";
		  break;
	  case 3:
		  *msg = "Thermal status 3: Device shutdown.\n\r";
		  break;
	  default:
		  *msg = "Thermal status: undefined code.\n\r";
		  break;
	  }
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

  }    

  /* Check OCD flag: if not set, there is an overcurrent detection */
  if ((statusRegister & POWERSTEP01_STATUS_OCD) == 0)
  {
    //overcurrent detection
	  *msg = "OCD flag: overcurrent detected.\n\r";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }      

  /* Check STALL_A flag: if not set, there is a Stall condition on bridge A */
  if ((statusRegister & POWERSTEP01_STATUS_STALL_A) == 0)
  {
	  //overcurrent detection
	  *msg = "STALL_A flag: Bridge A stalled.\n\r";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }    

  /* Check STALL_B flag: if not set, there is a Stall condition on bridge B */
  if ((statusRegister & POWERSTEP01_STATUS_STALL_B) == 0)
  {
	  //overcurrent detection
	  *msg = "STALL_B flag: Bridge B stalled.\n\r";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }      

}

/**
  * @brief  This function is the User handler for the busy interrupt
  * @param  None
  * @retval None
  */
void MyBusyInterruptHandler(void)
{

   if (BSP_MotorControl_CheckBusyHw())
   {
      /* Busy pin is low, so at list one Powerstep01 chip is busy */
     /* To be customized (for example Switch on a LED) */
   }
   else
   {
     /* To be customized (for example Switch off a LED) */
   }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param[in] error Number of the error
  * @retval None
  */
void MyErrorHandler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;
  
  /* Infinite loop */
  while(1)
  {
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param[in] error Number of the error
  * @retval None
  */
void Error_Handler()
{

  /* Infinite loop */
  while(1)
  {
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
