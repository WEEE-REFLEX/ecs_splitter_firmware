#define VOLTAGE_MODE

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint16_t gLastError;

/* Private function prototypes -----------------------------------------------*/
static void MyBusyInterruptHandler(void);
static void MyFlagInterruptHandler(void);

/* Private functions ---------------------------------------------------------*/
#define PIGNON_MODULUS 1
#define PIGNON_TEETH 20
#define MOTOR_STEP_ANGLE 0.34736842105263157895
#define SPLITTER_NUM 3
int microstepping;
int min_lin_distance = 25;


	int to_step(int linear_pos, int motor_id){
		return (int)(PIGNON_MODULUS*PIGNON_TEETH/2/(linear_pos-motor_id*min_lin_distance)/MOTOR_STEP_ANGLE*microstepping);
	}

	int to_linear(int abs_pos, int motor_id){
		return (int)(PIGNON_MODULUS*PIGNON_TEETH/2/abs_pos/MOTOR_STEP_ANGLE*microstepping + motor_id*min_lin_distance);
	}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	switch (motorStepMode){
		case STEP_MODE_FULL: microstepping = 1; break;
		case STEP_MODE_HALF: microstepping = 2; break;
		case STEP_MODE_1_4   : microstepping = 4; break;
		case STEP_MODE_1_8   : microstepping = 8; break;
		case STEP_MODE_1_16  : microstepping = 16; break;
		case STEP_MODE_1_32  : microstepping = 32; break;
		case STEP_MODE_1_64  : microstepping = 64; break;
		case STEP_MODE_1_128 : microstepping = 128; break;
		case STEP_MODE_1_256 : microstepping = 256; break;
		case STEP_MODE_UNKNOW: microstepping = 1; break;
		case STEP_MODE_WAVE  : microstepping = 1; break;
	}




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
  BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, SPLITTER_NUM);
  /* When BSP_MotorControl_Init is called with NULL pointer,                  */
  /* the Powerstep01 registers are set with the predefined values from file   */
  /* powerstep01_target_config.h, otherwise the registers are set using the   */
  /* powerstep01_Init_u relevant union of structure values.                   */
  /* The first call to BSP_MotorControl_Init initializes the first device     */
  /* whose Id is 0.                                                           */
  /* The nth call to BSP_MotorControl_Init initializes the nth device         */
  /* whose Id is n-1.                                                         */

  for (int mot_sel = 0; mot_sel<SPLITTER_NUM; ++mot_sel){
	  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, &initDeviceParametersDevice);
  }
  
  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Attach the function MyBusyInterruptHandler (defined below) to the busy interrupt */
  BSP_MotorControl_AttachBusyInterrupt(MyBusyInterruptHandler);
  
  /* Attach the function Error_Handler (defined below) to the error Handler*/
  BSP_MotorControl_AttachErrorHandler(MyErrorHandler);

  /* Set Current position to be the home position for device 0 */
  for (int mot_sel = 0; mot_sel<SPLITTER_NUM; ++mot_sel){
	  BSP_MotorControl_CmdGoUntil(mot_sel,RESET,BACKWARD,70000);
	  BSP_MotorControl_WaitWhileActive(mot_sel);
  }

  /* Set Current position to be the home position for device 0 */
  BSP_MotorControl_CmdResetPos(0);

  /* Set Current position to be the home position for device 1 */
  BSP_MotorControl_CmdResetPos(1);

  /* Set Current position to be the home position for device 2 */
  BSP_MotorControl_CmdResetPos(2);

  HAL_Delay(1000);


  /* Infinite loop */
  while(1)
  {
  }

}

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  char* msg;
  for(int mot_sel = 0; mot_sel<SPLITTER_NUM; ++mot_sel){

  /* Get the value of the status register via the command GET_STATUS */
  /* this will release the stop flag (red letd)*/
  uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(mot_sel);


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
	 msg = (char*) "SW_F flag not set: Switch input open.\n\r";
	 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }
  else
  {
    // SW CLOSED
	  msg = (char*) "SW_F flag not set: Switch input closed.\n\r";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }  
  /* Check SW_EN bit */
  if ((statusRegister & POWERSTEP01_STATUS_SW_EVN) == POWERSTEP01_STATUS_SW_EVN)
  {
	 // button hit
	 msg = (char*) "Reset button hit\n\r";
	 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
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
	  msg = (char*)"UVLO flag not set: undervoltage lock-out.\n\r";
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
		  msg = (char*)"Thermal status 1: Warning.\n\r";
		  break;
	  case 2:
		  msg = (char*)"Thermal status 2: Bridge shutdown.\n\r";
		  break;
	  case 3:
		  msg = (char*)"Thermal status 3: Device shutdown.\n\r";
		  break;
	  default:
		  msg = (char*)"Thermal status: undefined code.\n\r";
		  break;
	  }
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

  }    

  /* Check OCD flag: if not set, there is an overcurrent detection */
  if ((statusRegister & POWERSTEP01_STATUS_OCD) == 0)
  {
    //overcurrent detection
	  msg = (char*)"OCD flag: overcurrent detected.\n\r";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }      

  /* Check STALL_A flag: if not set, there is a Stall condition on bridge A */
  if ((statusRegister & POWERSTEP01_STATUS_STALL_A) == 0)
  {
	  //overcurrent detection
	  msg = (char*)"STALL_A flag: Bridge A stalled.\n\r";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }    

  /* Check STALL_B flag: if not set, there is a Stall condition on bridge B */
  if ((statusRegister & POWERSTEP01_STATUS_STALL_B) == 0)
  {
	  //overcurrent detection
	  msg = (char*)"STALL_B flag: Bridge B stalled.\n\r";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }

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
