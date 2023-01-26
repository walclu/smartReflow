/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdarg.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
#include "math.h"

#include "MAX31855.h"
#include "pid.h"
#include "nextion.h"
#include "reflow_temps_lut.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct _rfo_t {

	UART_HandleTypeDef* uart;
	uint8_t rcv[20];
	uint8_t trm[20];

	SPI_HandleTypeDef* spi;

	uint16_t idx;
	bool print;

	TIM_HandleTypeDef* buzPwmTim;
	TIM_HandleTypeDef* buzElapsTim;

	TIM_HandleTypeDef* ssrTim;

	uint8_t ssr1Ch;
	uint8_t ssr2Ch;

	uint8_t numBeeps;
	uint8_t cntBeeps;

	uint8_t currentReflowState;
	float targetTemp;
	uint32_t elaps;

	bool dataRdy;
	uint8_t packetId;
	float temp;
	float dTemp;

	uint16_t dutyCycle;
	uint16_t pwmFreq;

	uint8_t currentPage;

};
typedef struct _rfo_t rfo_t;

struct _rfo_paste_t {
	float PREHEAT_START_TIME;
	float PREHEAT_DURATION;
	float PREHEAT_START_TEMP;
	float PREHEAT_END_TEMP;

	float SOAK_DURATION;
	float SOAK_START_TEMP;
	float SOAK_END_TEMP;
	float SOAK_START_TIME;

	float RAMP_DURATION;
	float RAMP_START_TEMP;
	float RAMP_END_TEMP;
	float RAMP_START_TIME;


	float REFLOW_DURATION;
	float REFLOW_START_TEMP;
	float REFLOW_END_TEMP;
	float REFLOW_START_TIME;


	float COOL_DURATION;
	float COOL_END_TEMP;
	float COOL_START_TIME;

	float PREHEAT_TEMP_INCREASE;
	float SOAK_TEMP_INCREASE;
	float RAMP_TEMP_INCREASE;
	float REFLOW_TEMP_INCREASE;
	float COOL_TEMP_DECREASE;

};
typedef struct _rfo_paste_t rfo_paste_t;




char *rfoCmdLut[] = {"pwm -s --duty", "pwm -s --freq", "temp -m --cont",
		"start -p --nonleaded", "stop" ,"calibrate", "help"
};


struct _rfo_fsm_t {
	bool connected;
	bool streaming;
	uint8_t state;
};
typedef struct _rfo_fsm_t rfo_fsm_t;

enum _rfoStates {
	RFO_IDLE,
	RFO_PROCESS_CMD,
	RFO_PWM_SET_FREQ,
	RFO_PWM_SET_DUTY,
	RFO_GET_TEMP,
	RFO_START,
	RFO_STOP,
	RFO_TRANSMIT,
	RFO_CALIBRATE,
	RFO_RESET,
	RFO_HELP,
	RFO_STATES,
};

enum _reflowStates_t
{
	REFLOW_STATE_IDLE,
	REFLOW_STATE_DEBUG,
	REFLOW_STATE_PREPARE,
	REFLOW_STATE_PREHEAT,
	REFLOW_STATE_SOAK,
	REFLOW_STATE_RAMP,
	REFLOW_STATE_REFLOW,
	REFLOW_STATE_COOL,
	REFLOW_STATE_COMPLETE,
	REFLOW_STATE_TOO_HOT,
	REFLOW_STATES
};

enum _dp_page_ids {
	DP_LOADING,
	DP_MAIN,
	DP_SETTINGS,
	DP_PAGES
};


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCU_FREQ_KHZ 64000
#define RFO_DEFAULT_DUTY 50
#define RFO_DEFAULT_PWM_FREQ_KHZ 10

#define DEBUG 0



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define _RFO_INIT(_dev, _uartHandle, _spiHandle, _buzPwmTim, _buzElapsTim, _ssrTim, _dutyCycle, _pwmFreq) \
		do{ \
			(_dev)-> uart =  _uartHandle; \
			(_dev)-> spi = _spiHandle; \
			(_dev)-> buzPwmTim = _buzPwmTim; \
			(_dev)-> buzElapsTim = _buzElapsTim; \
			(_dev)-> ssrTim = _ssrTim; \
			(_dev)-> numBeeps = 0; \
			(_dev)-> cntBeeps = 0; \
			(_dev)-> idx = 0; \
			(_dev)-> currentReflowState = REFLOW_STATE_IDLE; \
			(_dev)-> elaps = 0; \
			(_dev)-> targetTemp = 0; \
			(_dev)-> dataRdy = false; \
			(_dev)-> packetId = 0; \
			(_dev)-> temp = 0; \
			(_dev)-> dutyCycle = _dutyCycle; \
			(_dev)-> pwmFreq = _pwmFreq; \
			(_dev)-> currentPage = DP_LOADING; \
		} while(0)

#define _RFO_FSM_INIT(_fsm) \
		do{ \
			(_fsm)->state = RFO_IDLE; \
			(_fsm)->connected = false; \
			(_fsm)->streaming = false; \
		} while(0)


#define _RFO_PERIPHERAL_INIT() \
		do{ \
			HAL_UART_Receive_IT(&hlpuart1, dev->rcv, 1);\
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);\
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);\
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);\
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);\
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);\
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);\
		} while(0)


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
rfo_t rfoDevObj;
rfo_t* dev = &rfoDevObj;

rfo_fsm_t rfoFsmObj;
rfo_fsm_t* fsm = & rfoFsmObj;

rfo_paste_t configObj;
rfo_paste_t* config = &configObj;


max31855_t max81855Obj;
max31855_t* tempSens = &max81855Obj;

PIDController pidObj;

PIDController* pid = &pidObj;

nextion_t dpObj;

nextion_t* dp = &dpObj;

/*
Lead-Free Reflow Curve
======================
Temperature (Degree Celcius)
245-|                                               x  x
    |                                            x   |     x
    |                                         x      |        x
    |                                      x         |           x
130-|                                   x            |              x
    |                              x    |            |              |   x
    |                         x         |            |              |       x
    |                    x              |            |              |
 90-|               x                   |            |              |
    |             x |                   |            |              |
    |           x   |                   |            |              |
    |         x     |                   |            |              |
    |       x       |                   |            |              |
    |     x         |                   |            |              |
    |   x           |                   |            |              |
30 -| x             |                   |            |              |
    |  <  90 s  >|<          90 s      >| <   30   > |
    | Preheat Stage |   Soaking Stage   | Ramp Stage |              | Cool
 0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ |_ _ _ _ _ _ _ |_ _ _ _ _
                                                               Time (Seconds)
 */

const float kP 					=		50;//39;
const float kI 					=		.5f;//1;
const float kD 					=		35;//31;
const float tau			    	=		0.2f;
const float limMin 				=   	0;
const float limMax 				=   	256;
const float limMinInt 			=   	-60.0f;
const float limMaxInt 			=   	60.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void clear_buffer(uint8_t* buf, uint32_t len);
void beep_IT(rfo_t *dev, uint8_t numBeeps);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void clear_buffer(uint8_t* buf, uint32_t len)
{
	for(int z = 0; z < len; z++)
	{
		buf[z] = '\0';
	}
}

void beep_IT(rfo_t* dev, uint8_t numBeeps) {
	dev->numBeeps = numBeeps;
	dev->cntBeeps = 0;
	__HAL_TIM_SET_COMPARE(dev->buzPwmTim, TIM_CHANNEL_1, 2);
	__HAL_TIM_CLEAR_FLAG(dev->buzElapsTim, TIM_SR_UIF);
	HAL_TIM_Base_Start_IT(dev->buzElapsTim);

}

void assign_reflow_temps_from_lut(rfo_paste_t* config, const float* lut) {
	config->PREHEAT_START_TIME 	= lut[0];
	config->PREHEAT_DURATION	= lut[1];
	config->PREHEAT_START_TEMP	= lut[2];
	config->PREHEAT_END_TEMP	= lut[3];

	config->SOAK_START_TIME		= lut[4];
	config->SOAK_DURATION		= lut[5];
	config->SOAK_START_TEMP		= lut[6];
	config->SOAK_END_TEMP		= lut[7];

	config->RAMP_START_TIME		= lut[8];
	config->RAMP_DURATION		= lut[9];
	config->RAMP_START_TEMP		= lut[10];
	config->RAMP_END_TEMP		= lut[11];

	config->REFLOW_START_TIME	= lut[12];
	config->REFLOW_DURATION		= lut[13];
	config->REFLOW_START_TEMP	= lut[14];
	config->REFLOW_END_TEMP		= lut[15];

	config->COOL_START_TIME		= lut[16];
	config->COOL_DURATION		= lut[17];
	config->COOL_END_TEMP		= lut[18];

	config->PREHEAT_TEMP_INCREASE	= lut[19];
	config->SOAK_TEMP_INCREASE		= lut[20];
	config->RAMP_TEMP_INCREASE		= lut[21];
	config->REFLOW_TEMP_INCREASE	= lut[22];
	config->COOL_TEMP_DECREASE		= lut[23];
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_LPUART1_UART_Init();
	MX_SPI1_Init();
	MX_TIM16_Init();
	MX_TIM2_Init();
	MX_TIM17_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	_RFO_INIT(dev, &hlpuart1, &hspi1 ,&htim2, &htim17, &htim2, RFO_DEFAULT_DUTY, RFO_DEFAULT_PWM_FREQ_KHZ);
	_RFO_FSM_INIT(fsm);

	//assign_reflow_temps_from_lut(config, TS391LT50);
	assign_reflow_temps_from_lut(config, SMD291AX50T3);

	HAL_UART_Receive(&huart1, dp->rcv, 50, 1000); // Resets UART buffers, so isr is not fired on boot
	HAL_UART_Receive_IT(&huart1, dp->rcv, 4);


	nextion_init(dp, &huart1);
	char buf[100];

	dp->write_id_val(dp, "j0", 0);

	_RFO_PERIPHERAL_INIT();

	MAX31855_init(tempSens, NSS_GPIO_Port, NSS_Pin, &hspi1);

	PIDController_Init(pid, kP, kI, kD, SAMPLE_TIME, tau, limMin, limMax, limMinInt, limMaxInt);

	dp->write_id_val(dp, "j0", 100);
	HAL_Delay(1000);

	dp->write(dp, "page main");
	dev->currentPage = DP_MAIN;
	nextion_print_page(dp);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		switch(fsm->state)
		{

		case RFO_IDLE:
			break;

		case RFO_PROCESS_CMD:
			if(dev->rcv[0] == '\r') {
				HAL_UART_Transmit(dev->uart, (uint8_t*)"\r\n", 2 , 100);
				buf[dev->idx] = '\0';
				for(int i = 0; i < 7; i++) {
					if(!strcmp(buf, rfoCmdLut[i]))
					{
						switch(i)
						{
						case 0:
							fsm->state = RFO_PWM_SET_DUTY;
							break;
						case 1:
							fsm->state = RFO_PWM_SET_FREQ;
							break;
						case 2:
							fsm->state = RFO_GET_TEMP;
							beep_IT(dev, 2);
							break;
						case 3:
							beep_IT(dev, 3);
							fsm->state = RFO_START;
							dp->write(dp, "page reflow");
							dp->write_id_str(dp, "conState", "Connected");
							dp->write_id_str(dp, "reflowState", "Prepare");
							dp->write_id_val(dp, "reflowProgress", 0);
							dev->currentReflowState = REFLOW_STATE_PREPARE;
							HAL_TIM_Base_Start_IT(&htim16);
							break;
						case 4:
							fsm->state = RFO_STOP;
							break;
						case 5:
							fsm->state = RFO_CALIBRATE;
							break;
						case 6:
							fsm->state = RFO_HELP;
							break;
						default:
							break;
						}
						dev->idx = 0;
						break;
					}

				}

				if(fsm->state == RFO_PROCESS_CMD) {

					fsm->state = RFO_IDLE;

				}
				dev->idx = 0;
				clear_buffer((uint8_t*)buf, 100);

			}


			else if(dev->rcv[0] == '\177') {
				if(dev->idx > 0)
				{
					buf[dev->idx-1] = '\0';
					dev->idx -= 1;
					fsm->state = RFO_IDLE;
				}
			}

			else {
				buf[dev->idx] = dev->rcv[0];
				dev->idx += 1;
				clear_buffer((dev->rcv), 20);
				fsm->state = RFO_IDLE;
			}

			break;


		case RFO_PWM_SET_DUTY:
			__NOP();
			break;

		case RFO_PWM_SET_FREQ:
			__NOP();
			break;

		case RFO_GET_TEMP:
			__NOP();
			HAL_TIM_Base_Start_IT(&htim16);
			fsm->state = RFO_TRANSMIT;
			break;

		case RFO_START:
			__NOP();
			if(dev->dataRdy) {
				dev->dataRdy = false;

				PIDController_Update(pid, dev->targetTemp, dev->temp);

				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pid->out);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pid->out);

				sprintf(buf, "%.2f\n", dev->temp);
				HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);

				sprintf(buf, "%.2f\n", dev->targetTemp);
				HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);

				sprintf(buf, "%.2f\n", pid->out);
				HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);

			}
			break;

		case RFO_TRANSMIT:
			if(dev->dataRdy) {
				dev->dataRdy = false;
				sprintf(buf, "Temp is: %.2f\r\n", dev->temp);
				HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);
			}
			break;

		case RFO_STOP:
			break;

		case RFO_HELP:
			__NOP();
			sprintf(buf, "Help Page for RFO\r\n");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);
			sprintf(buf, "Commands: ");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);
			sprintf(buf, "pwm -<OPTION> --<INSTRUCTION>\r\n");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);
			sprintf(buf, "          Options: -s\r\n");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);
			sprintf(buf, "          Instructions: --duty, --freq");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);

			sprintf(buf, "\r\n");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);

			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);
			sprintf(buf, "          temp -<OPTION> --<INSTRUCTION>\r\n");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);
			sprintf(buf, "          Options: -m\r\n");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);
			sprintf(buf, "          Instructions: --cont, --single\r\n");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);

			sprintf(buf, "\r\n");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);

			sprintf(buf, "          start -<OPTION> --<INSTRUCTION>\r\n");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);
			sprintf(buf, "          Options: -p\r\n");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);
			sprintf(buf, "          Instructions: --nonleaded, --leaded\r\n");
			HAL_UART_Transmit(dev->uart, (uint8_t*)buf, strlen(buf), 100);


			fsm->state = RFO_IDLE;
			break;

		}

	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
			|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
	PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
	PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN Smps */

	/* USER CODE END Smps */
}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void)
{

	/* USER CODE BEGIN LPUART1_Init 0 */

	/* USER CODE END LPUART1_Init 0 */

	/* USER CODE BEGIN LPUART1_Init 1 */

	/* USER CODE END LPUART1_Init 1 */
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 115200;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN LPUART1_Init 2 */

	/* USER CODE END LPUART1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 250;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 256;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 8000-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void)
{

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 64000;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 500;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void)
{

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 64000-1;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 50;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */

	/* USER CODE END TIM17_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : NSS_Pin */
	GPIO_InitStruct.Pin = NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_GREEN_Pin */
	GPIO_InitStruct.Pin = LED_GREEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	dp->trmRdy = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == LPUART1){
		if(HAL_UART_Transmit(dev->uart, dev->rcv, 1, 1000) == HAL_OK) {
			fsm->state = RFO_PROCESS_CMD;
			HAL_UART_Receive_IT(dev->uart, dev->rcv, 1);
		}
	}
	else if(huart->Instance == USART1)  {

		nextion_decode_response(dp);

		switch(dp->response.page) {
		case DP_MAIN:  // page num main
			switch(dp->response.id) {
			case 2: // id of start button
				beep_IT(dev,3);
				dp->write(dp, "page reflow");
				dp->write_id_str(dp, "conState", "Not connected");
				dp->write_id_str(dp, "reflowState", "Prepare");
				dp->write_id_val(dp, "reflowProgress", 0);
				fsm->state = RFO_START;
				dev->currentReflowState = REFLOW_STATE_PREPARE;
				dev->print = true;
				HAL_TIM_Base_Start_IT(&htim16);
				break;
			case 3: // id of settings button
				beep_IT(dev,1);
				dp->write(dp, "page setting");
				dev->currentPage = DP_SETTINGS;
				break;
			}
			break;

			case DP_SETTINGS: // page num settings
				switch(dp->response.id) {
				case 2: // id of back arrow button
					dp->write(dp, "page main");
					dev->currentPage = DP_MAIN;
					nextion_print_page(dp);
					break;
				}
				break;

				case 3:
					switch(dp->response.id) {
					case 15: // id of abort button
						beep_IT(dev,3);
						HAL_TIM_Base_Stop_IT(&htim16);
						dev->currentReflowState = REFLOW_STATE_IDLE;
						fsm->state = RFO_IDLE;
						dev->elaps = 0;

						dp->write(dp, "page main");
						dev->currentPage = DP_MAIN;
						nextion_print_page(dp);
						break;
					}
					break;
		}

		HAL_UART_Receive_IT(&huart1, dp->rcv, 4);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM17) {

		if(dev->cntBeeps == 2*dev->numBeeps-1) {
			__HAL_TIM_SET_COMPARE(dev->buzPwmTim, TIM_CHANNEL_1, 0);
			HAL_TIM_Base_Stop_IT(dev->buzElapsTim);
			dev->cntBeeps = 0;
		}

		if(dev->cntBeeps % 2) {
			__HAL_TIM_SET_COMPARE(dev->buzPwmTim, TIM_CHANNEL_1, 2);
		}
		else {
			__HAL_TIM_SET_COMPARE(dev->buzPwmTim, TIM_CHANNEL_1, 0);
		}

		dev->cntBeeps += 1;



	}
	else if(htim->Instance == TIM16) {

		MAX31855_read_celsius(tempSens);
		dev->elaps += 1;
		dev->temp = tempSens->temp;


		switch(dev->currentReflowState){

		case REFLOW_STATE_DEBUG:
			break;

		case REFLOW_STATE_IDLE:
			break;

		case REFLOW_STATE_PREPARE:
			dev->targetTemp = 256;

			if(dev->temp > config->PREHEAT_START_TEMP) {
				beep_IT(dev,2);
				dev->currentReflowState = REFLOW_STATE_PREHEAT;
				dev->elaps = 0;
			}
			break;

		case REFLOW_STATE_PREHEAT:
			dp->write_id_str(dp, "reflowState", "PREHEAT");
			if(!(dev->elaps % 4) || dev->elaps == 1) {
				dev->targetTemp = (float)config->PREHEAT_START_TEMP + (config->PREHEAT_TEMP_INCREASE*SAMPLE_TIME)*dev->elaps;
			}

			if(dev->elaps == (float)config->PREHEAT_DURATION/SAMPLE_TIME) {
				beep_IT(dev,2);
				dev->currentReflowState = REFLOW_STATE_SOAK;
				dev->elaps = 0;
			}
			break;

		case REFLOW_STATE_SOAK:
			dp->write_id_str(dp, "reflowState", "SOAK");
			if(!(dev->elaps % 4) || dev->elaps == 1) {
				dev->targetTemp = (float)config->SOAK_START_TEMP + (config->SOAK_TEMP_INCREASE*SAMPLE_TIME)*dev->elaps;
			}
			if(dev->elaps == (float)config->SOAK_DURATION/SAMPLE_TIME) {
				beep_IT(dev,2);
				dev->currentReflowState = REFLOW_STATE_RAMP;
				dev->elaps = 0;
				pid->Kp = 50;
			}
			break;

		case REFLOW_STATE_RAMP:
			dp->write_id_str(dp, "reflowState", "RAMP");
			if(!(dev->elaps % 4) || dev->elaps == 1) {
				dev->targetTemp = (float)config->RAMP_START_TEMP + (config->RAMP_TEMP_INCREASE*SAMPLE_TIME)*dev->elaps;
			}
			if(dev->elaps == (float)config->RAMP_DURATION/SAMPLE_TIME) {
				beep_IT(dev,2);
				dev->currentReflowState = REFLOW_STATE_REFLOW;
				dev->elaps = 0;
			}
			break;

		case REFLOW_STATE_REFLOW:
			dp->write_id_str(dp, "reflowState", "REFLOW");
			if(!(dev->elaps % 4) || dev->elaps == 1) {
				dev->targetTemp = config->REFLOW_START_TEMP + (config->REFLOW_TEMP_INCREASE*SAMPLE_TIME)*dev->elaps;
			}
			if(dev->elaps == (float)config->REFLOW_DURATION/SAMPLE_TIME) {
				beep_IT(dev,2);
				dev->currentReflowState = REFLOW_STATE_COOL;
				dev->elaps = 0;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);\
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);\
			}
			break;
		case REFLOW_STATE_COOL:
			dp->write_id_str(dp, "reflowState", "IDLE");
			if(!(dev->elaps % 4) || dev->elaps == 1) {
				dev->targetTemp = config->REFLOW_END_TEMP - (config->COOL_TEMP_DECREASE*SAMPLE_TIME)*dev->elaps;
			}
			if(dev->elaps == (float)config->COOL_DURATION/SAMPLE_TIME) {
				beep_IT(dev,5);
				dev->currentReflowState = REFLOW_STATE_IDLE;
				dev->elaps = 0;
				HAL_TIM_Base_Stop_IT(&htim16);
				dp->write(dp, "page main");
				dev->currentPage = DP_MAIN;
				nextion_print_page(dp);
			}
			break;


		}
		if(dev->currentReflowState != REFLOW_STATE_IDLE) {
			dp->write_id_float(dp, "targetTemp", dev->targetTemp);
			dp->write_id_float(dp, "currentTemp", dev->temp);
			dp->write_id_float(dp, "tempDiff", dev->targetTemp-dev->temp);
			dp->write_id_float(dp, "pidVal", pid->out);
		}

		dev->dataRdy = true;

	}

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

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
}
#endif /* USE_FULL_ASSERT */
