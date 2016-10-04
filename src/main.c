#define VOLTAGE_MODE

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PIGNON_MODULUS 1
#define PIGNON_TEETH 20
#define MOTOR_STEP_ANGLE 0.34736842105263157895
#define SPLITTER_NUM 3
#define BUFFER_LEN_RX 15
#define BUFFER_LEN_TX 50
#define SPLITTER_NUM 3
#define M_PI 3.14159265358979323846
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint16_t gLastError;
int microstepping;
int min_lin_distance = 25;

// for communication
extern UART_HandleTypeDef huart2;
uint8_t input_string[BUFFER_LEN_RX];
uint8_t input_string_len = 0;
uint8_t input_char;
int splitter_pos_x[SPLITTER_NUM] = { 100, 200, 300 };
int splitter_pos_y[SPLITTER_NUM] = { 0, 0, 0 };
char to_be_sent[BUFFER_LEN_TX];
unsigned int to_be_sent_length = 0;
bool always_return_ACK = true;
int moving_motor;

/* Private function prototypes -----------------------------------------------*/
static void MyBusyInterruptHandler(void);
static void MyFlagInterruptHandler(void);
static void MyMessageErrorHandler(uint16_t);

/* Private functions ---------------------------------------------------------*/

int to_step(int linear_pos, int motor_id){
	return (int)(-2*180*(linear_pos-motor_id*min_lin_distance)*microstepping/PIGNON_MODULUS/PIGNON_TEETH/MOTOR_STEP_ANGLE/M_PI);
}

int to_linear(int abs_pos, int motor_id){
	return (int)(-abs_pos*PIGNON_MODULUS*PIGNON_TEETH*MOTOR_STEP_ANGLE*M_PI/microstepping/2/180 + motor_id*min_lin_distance);
}

int get_motor_id(char mot_letter) {
	switch (mot_letter) {
	case 'A':
		return 0;
	case 'B':
		return 1;
	case 'C':
		return 2;
	default:
		MyMessageErrorHandler(3);
		return -1;
	}
}

unsigned char get_motor_letter(char mot_id) {
	switch (mot_id) {
	case 0:
		return 'A';
	case 1:
		return 'B';
	case 2:
		return 'C';
	default:
		MyMessageErrorHandler(3);
		return 0;
	}
}

typedef enum {
	ACK,
	RETURN_POSX,
	RETURN_POSY,
	RETURN_STOP,
	RETURN_HALT,
	OVERCURRENT,
	AT_HOME,
	UNREACHABLE
} message_computer_t ;

typedef enum {
	UPDATE_POS,
	HALT,
	STOP
} message_shield_t ;


void sendMessageToPC(message_computer_t message_type, int motor_id) {

	moving_motor = motor_id;
	char mot_let = 0;
	switch(message_type){
		case ACK:         to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "ACK\n"); break;
		case RETURN_POSX:
			mot_let = (char)get_motor_letter(motor_id);
			to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "RX%c%d\n", mot_let, splitter_pos_x[motor_id] );
			break;
		case RETURN_POSY:
			mot_let = (char)get_motor_letter(motor_id);
			to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "RY%c%d\n", mot_let, splitter_pos_y[motor_id] );
			break;
		case RETURN_STOP: to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "IS%c\n", mot_let); break;
		case RETURN_HALT: to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "IH%c\n", mot_let); break;
		case OVERCURRENT: to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "IO%c\n", mot_let); break;
		case AT_HOME: 	  to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "IR%c\n", mot_let); break;
		case UNREACHABLE: to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "EU%c\n", mot_let); break;
		default: MyMessageErrorHandler(4); break;
	}

	if (to_be_sent_length >=BUFFER_LEN_TX){
		MyMessageErrorHandler(4);
		return;
	}

	if (HAL_UART_Transmit_IT(&huart2, (uint8_t*)to_be_sent, to_be_sent_length) != HAL_OK) {
		MyMessageErrorHandler(1);
		return;
	}
}

void sendMessageEasy(char* buffered_data, unsigned int data_length) {

	if (HAL_UART_Transmit_IT(&huart2, (uint8_t*)buffered_data, data_length) != HAL_OK) {
		MyMessageErrorHandler(1);
	}
}

void sendMessageToMotorshield(message_shield_t message_type, int motor_id) {
//	char* msg = "shield updated\n";
//	sendMessageEasy(msg,strlen(msg));

	switch(message_type){
	case UPDATE_POS:
		BSP_MotorControl_GoTo(motor_id,to_step(splitter_pos_x[motor_id],motor_id));
		break;
	case HALT: break;
	case STOP: break;
	default:
		MyMessageErrorHandler(2);
	}


}


void parseCommand(const uint8_t* buffered_data, int data_length) { // can be used global variables without passing no arguments
	if (data_length < 3)
		MyMessageErrorHandler(2);

	if (strncmp((char*)buffered_data, "SSS", 3) == 0) {
		for(int mot_sel = 0; mot_sel<SPLITTER_NUM; ++mot_sel)
			sendMessageToMotorshield(HALT,mot_sel);
	}
	else
	if (strncmp((char*)buffered_data, "HHH", 3) == 0) {
		for(int mot_sel = 0; mot_sel<SPLITTER_NUM; ++mot_sel)
			sendMessageToMotorshield(HALT,mot_sel); //TODO
	}
	else
	if (strncmp((char*)buffered_data, "HBT", 3) == 0) {
		always_return_ACK = true;
	}
	else
	if (strncmp((char*)buffered_data, "HBF", 3) == 0) {
		always_return_ACK = false;
	}
	else{
		switch (buffered_data[0]) {
		case 'P': // set position P..
			switch (buffered_data[1]) {
			case 'X':  // set position PX.
				splitter_pos_x[get_motor_id(buffered_data[2])] = strtol((char*)&buffered_data[3],NULL,10);
				sendMessageToMotorshield(UPDATE_POS,get_motor_id(buffered_data[2]));
				break;
			case 'Y': // set position PY.
				splitter_pos_y[get_motor_id(buffered_data[2])] = strtol((char*)&buffered_data[3],NULL,10);
				sendMessageToMotorshield(UPDATE_POS,get_motor_id(buffered_data[2]));
				break;
			default:
				MyMessageErrorHandler(2);
				return;
			}
			break;
		case 'Q': // dispatch position informations Q..
			switch (buffered_data[1]) {
			case 'X': // Q<X><SS>
				sendMessageToPC(RETURN_POSX, get_motor_id(buffered_data[2])); return;
			case 'Y': // Q<Y><SS>
				sendMessageToPC(RETURN_POSY, get_motor_id(buffered_data[2])); return;
			case 'Q': // QQ.
				for (int mot_sel = 0; mot_sel<SPLITTER_NUM; ++mot_sel){

					if (buffered_data[2] == 'X' || buffered_data[2] == 'Q') // QQ<X|Q>
						sendMessageToPC(RETURN_POSX, mot_sel);

					if (buffered_data[2] == 'Y' || buffered_data[2] == 'Q') // QQ<Y|Q>
						sendMessageToPC(RETURN_POSY, mot_sel);
				}
				break;
			default:
				MyMessageErrorHandler(2);
				break;
			}
			break;
		default:
			MyMessageErrorHandler(2);
			break;
		}
	}
	if (always_return_ACK)
		sendMessageToPC(ACK,-1);

}

// the HAL driver calls its weakly declared functions only after receiveing huart->RxXferCount==0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (input_string_len >= BUFFER_LEN_RX || input_char == '\n') {
		input_string[input_string_len] = 0; // terminate string with null char
		parseCommand(input_string, input_string_len);
		input_string_len = 0;
	} else {
		input_string[input_string_len] = input_char;
		++input_string_len;
	}

	// reset the receiving buffer
	HAL_UART_Receive_IT(&huart2, &input_char, 1);

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
	HAL_UART_Receive_IT(&huart2, &input_char, 1);

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

	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
	BSP_MotorControl_AttachBusyInterrupt(MyBusyInterruptHandler);
	BSP_MotorControl_AttachErrorHandler(MyErrorHandler);

	  /* Set home position */
	  for (int mot_sel = 0; mot_sel<SPLITTER_NUM; ++mot_sel){
		  BSP_MotorControl_CmdGoUntil(mot_sel,RESET,FORWARD,70000);
		  BSP_MotorControl_WaitWhileActive(mot_sel);
	  }

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

		char temp[40];
		snprintf(temp, 40, "Interrupt flag MOT %d", mot_sel );
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

		/* Get the value of the status register via the command GET_STATUS */
		/* this will release the stop flag (red led)*/
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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	switch(huart->ErrorCode){
	case HAL_UART_ERROR_PE :
		break; /*!< Parity error        */
	case HAL_UART_ERROR_NE :
		break; /*!< Noise error         */
	case HAL_UART_ERROR_FE :
		break; /*!< Frame error         */
	case HAL_UART_ERROR_ORE:
		break; /*!< Overrun error       */
	case HAL_UART_ERROR_DMA:
		break; /*!< DMA error       */
	}
}

void MyMessageErrorHandler(uint16_t error_id){
	switch (error_id){
		case 0:
			to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "SC: micro error\n"); break;
		case 1:
			to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "SC: UART async TX error\n"); break;
		case 2:
			to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "SC: parsing-input error\n"); break;
		case 3:
			to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "SC: asked for non-existent motor\n"); break;
		case 4:
			to_be_sent_length = snprintf(to_be_sent, BUFFER_LEN_TX, "SC: TX string too long\n"); break;
		}

		if (to_be_sent_length >=BUFFER_LEN_TX && error_id!=4){
			MyMessageErrorHandler(4);
			return;
		}

		if (HAL_UART_Transmit(&huart2, (uint8_t*)to_be_sent, to_be_sent_length, HAL_UART_STATE_TIMEOUT) != HAL_OK) {
			MyMessageErrorHandler(1);
			return;
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
