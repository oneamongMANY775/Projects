/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"
#include "LTC6812.h"
#include <math.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
CAN_HandleTypeDef hcan1;
SD_HandleTypeDef hsd;



TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define TOTAL_IC                   10 // Replace 12 with the actual number of LTC6812 ICs in the daisy chain
#define TS_ON_SIGNAL               GPIO_PIN_8
#define STM_AIR_CTRL               GPIO_PIN_7
#define STM_MODE_CTRL              GPIO_PIN_5
#define STM_MODE_BALANCING         GPIO_PIN_5//cell balancing
#define DISCHARGE_ENABLE           0
#define CHARGE_ENABLE              1
#define SELECTED_MODULE            0       // Choose an appropriate module index
#define ADC_RESOLUTION             4096.0 // 12-bit ADC resolution (2^12 = 4096)
#define ADC_REF_VOLTAGE            3.3    // Reference voltage in volts
#define V_OFFSET                   2.5    // Offset voltage at 0A
#define G_CHANNEL_1                0.0267 // Sensitivity for Channel 1 in V/A
#define G_CHANNEL_2                0.004  // Sensitivity for Channel 2 in V/A
//balancing
//  #define DEL_THRESHOLD              0.1f

cell_asic bms_ic[TOTAL_IC];	//the cell_asic struct objects
uint32_t k=1;
uint32_t v=0;
uint32_t m;



//LTC CONFIGURATION VARIABLES
bool REFON = true; //!< Reference Powered Up Bit (true means Vref remains powered on between conversions)
bool ADCOPT = true; //!< ADC Mode option bit	(true chooses the second set of ADC frequencies)
bool gpioBits_a[5] = {true,true,true,true,true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5 (false -> pull-down on)
bool dccBits_a[10] = {false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12 (all false -> no discharge enabled)
bool dctoBits[4] = {false, false, false, false}; //!< Discharge time value // Dcto 0,1,2,3	(all false -> discharge timer disabled)
uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
uint8_t ADC_DCP = DCP_ENABLED; //!< Discharge Permitted
uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; //!< Channel Selection for ADC conversion
uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;  //!< Channel Selection for ADC conversion
uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;  //!< Channel Selection for ADC conversion
uint8_t SEL_ALL_REG = REG_ALL; //!< Register Selection
uint8_t SEL_REG_A = REG_1; //!< Register Selection
uint8_t SEL_REG_B = REG_2; //!< Register Selection
uint8_t error; //!<error flag for isospi failures
bool FDRF = false; //!< Force Digital Redundancy Failure Bit
bool DTMEN = false; //!< Enable Discharge Timer Monitor
bool PSBits[2] = {false, false}; //!< Digital Redundancy Path Selection//ps-0,1 rxBuf



//CAN VARIABLES
static CAN_TxHeaderTypeDef myTxHeader;
static CAN_RxHeaderTypeDef myRxHeader;
uint8_t RxData[8];
uint16_t shifted_data;
int datacheck = 0;
uint8_t MODE_CONTROL = CHARGE_ENABLE;
uint8_t TxData[8] = {0};  // CAN data array
uint32_t TxMailbox;
bool x=false;


//ERROR VARIABLES
uint8_t error = 0,auxError = 0;	//hold if an error has occured while reading cell voltage and aux voltage values
int NV [TOTAL_IC][10];	//total number of voltage measurement errors per IC per cell
int NT [TOTAL_IC][5];	//total number of temperature measurement errors per IC per cell
int NPEC_V = 0;	//number of communication errors in cell voltage measurement
int NPEC_T = 0;	//number of communication errors in temperature voltage measurement
float Idc;	//the Idc (tractive system current)
double testTemperatures[TOTAL_IC][4];	//the temperatures for error checking (converted aux voltages)
uint16_t faultCode;

//received variables
uint8_t rx_data[8];	//holds the received CAN data
uint8_t reset = 0;	//0-> nothing happens, 1-> system reset
float OV = 4.15; //overvoltage	4.24
float UV = 3.0;	//undervoltage	3.3 undervoltage 2.5
uint16_t OT = 60;	//overtemperature	70
uint16_t OC = 400;	//overcurrent 400
uint8_t N_Error = 1;	//number of allowed errors (experimental values)
uint8_t BMS_State_Out=0;
float totalCellVoltage = 0.0;  // Variable to store the sum of all cell voltages

// Declare missing variables at the start of the file with the others
uint8_t V_PackCAN; // For CAN voltage representation
uint8_t CurrentCAN; // For CAN current representation
uint8_t VminCAN; // For minimum voltage representation in CAN
uint8_t TmaxCAN; // For maximum temperature representation in CAN
uint8_t s; // Some status or state representation, can be used for SOC
float OCV_SOC = 0.8; // Assuming a value for SOC (can be calculated or measured)
uint8_t TSMS_Stat = 0; // TSMS status (1 for active, 0 for inactive)
uint8_t bms_stat = 0; // BMS status for fault/error handling
uint16_t FL = 0; // Fault number, to be adjusted based on fault detected
uint8_t isospi_flag = 0; // ISO-SPI communication flag

//ADC VARIABLES
uint16_t adcVal_10Hz[5]; //Array that holds the ADC values (10Hz)
uint16_t TSMS=0;
float   Current,C_O,C_U;
uint16_t fc=0;

//TESTING VARIABLES
float voltages[TOTAL_IC][10];	//holds the converted voltages of each cell for 1 BMS slave (testing)
float temperatures[TOTAL_IC][5];
float Module_Voltage[5], IC_Voltage[10], Vmin_IC[10], V_Pack = 0, Vmax_Pack, Vmin_Pack, Vmax_IC[10], delta;
float Vbus;
uint16_t cell_voltages[110];
//cell balancing
//bool balancing_on[110];
int count=0;
//Thunderblade's data
uint8_t module[5] = {22, 22, 22, 22, 22}; // Cell Module configuration for battery pack of thunderblade 5.0
uint8_t ic[10] = {11,11,11,11,11,11,11,11,11,11}; // IC configuration for battery pack of thunderblade 5.0


//TIMING VARIABLES
uint16_t time = 0, time2 = 0, time3 = 0, time4 = 0, flag = 0, frequency = 0, counter = 0;	//timing and checking variables
uint16_t timeIT10 = 0, timeIT100 = 0, timeIT1K = 0, tim6counter = 0, tim7counter = 0, tim9counter = 0, tim11counter = 0;
uint32_t u=0, w=0, z=0;//counters to check how many times each timer is triggere
uint32_t a=0, b=0, c=0;//counters to check how many times each timer is triggere
uint32_t Error=0;
uint32_t can_error;

//FLAGS
bool flag10Hz = false, flag100Hz = false;	//triggred byt TIM2 and TIM3 respectively
bool flagStartCVConversion = false, flagReadCVConversion = false, flagStartAuxConversion = false, flagReadAuxConversion = false;	//triggered by TIM6, TIM7, TIM9, TIM11 respectively every 12.5ms
bool flagErrorCheck = false;	//triggered by TIM11 (every 50ms)
bool precharge = false;
bool AIR_ON_SIG = false;
bool AIR_ON_Flag = false;
bool DISCHARGE = false;
//balancing
bool balancing = true;  // Balancing state flag
uint8_t AIR_Status = 0;
double Tmax_Pack;

//UART
char uart_rx_buffer[256];
volatile uint8_t uart_rx_index = 0;
volatile uint8_t uart_rx_flag = 0;


//Data logging
#define LOG_BUFFER_SIZE 2048  // Buffer size for logging data
char logBuffer[LOG_BUFFER_SIZE];     // Primary buffer for logging data
char tempBuffer[LOG_BUFFER_SIZE];    // Temporary buffer for writing
uint16_t logIndex = 0;               // Write index for logBuffer
bool writePending = false;           // Flag to trigger SD write
bool bufferSwapFlag = false;         // Flag for swapping buffers




//SOC estimation
#define BATTERY_CAPACITY_AH 100.0   // Battery rated capacity in Ah
#define TIMER_INTERVAL_S    1.0     // Timer interval for SOC updates (1 second)
#define CURRENT_SCALING     1.0     // Scaling factor for current, if needed

float SOC = 100.0;                  // Initial SOC in %
float totalCharge = 0.0;            // Accumulated charge in Ah
float previousSOC = 100.0;          // To track previous SOC value



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void LTC6812_Init(void);
void CanFilterConfig(void);
void errorCounterInit(void);	//A nested for-loop that initializes the error counting variables to 0
void errorCheck(void);	//Checks for errors
void tempConvert(void);
void measure_and_error(void);
void state_change(void);
void pre_charge(void);
void MeasureVoltageParam(void);
void Discharge_Mode(void);
void Charge_Mode(void);
uint16_t generateFaultCode(int icIndex, int cellIndex, const char* errorType);
void DiagnosticMode(uint16_t fault);
void initializeCSVFile(void);
void updateCSVData(void);
void generateUniqueFilename(void);
void run_mux_diagnostic(uint8_t total_ic, cell_asic *ic);
void process_uart_command(char *command);
void SendPackVoltageAndAirStatus(void);
void ChargerControl_Init(void);
void SendChargerControl(uint8_t control);
float gaussian(float cell_voltage, float a , float b, float c);
//void balance(void);//cell balancing
//float SOC();et



// Global SD Card Variables
char* FileName = NULL;
int lenF = 0;
char buffer[256];  // Buffer for logging data
extern FRESULT fresult;
int maxFileNumber = 0;

//error checks
uint8_t mux_error;



/*************SOC Calculation Variables**************/
//float a[4] = {0.9336,0.6840,-269.3,269.5};
//float b[4] = {2.952,3.539,3.634,3.634};
//float c[4] = {0.4791,0.3512,0.06836,0.06839};
float soc;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_SDIO_SD_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  //initializeCSVFile();
   //START CAN
     	CanFilterConfig();	//configures the filter for the CAN communication
		HAL_CAN_Start(&hcan1);	//starts CAN module
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	//enables the message-pending-callback for FIFO 0

		//initialize LTC/SPI
         LTC6812_Init();	//initializes the LTC and SPI communication


		//START TIMERS
		HAL_TIM_Base_Start_IT(&htim2);	//10Hz
		HAL_TIM_Base_Start_IT(&htim3);	//100Hz
		//HAL_TIM_Base_Start_IT(&htim8); // Correct way to enable interrupts
		HAL_TIM_Base_Start_IT(&htim6);	//5Hz
		HAL_TIM_Base_Start_IT(&htim10);//1Khz

		//START ADCs
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcVal_10Hz, 5);	//ADC1 is triggered by TIM2 (10Hz)



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (MODE_CONTROL == CHARGE_ENABLE) Charge_Mode();

    if (BMS_State_Out) {
      HAL_GPIO_WritePin(GPIOA,STM_MODE_CTRL,SET);
    }
    else
    {
    	HAL_GPIO_WritePin(GPIOA,STM_MODE_CTRL,RESET);
        for (int i = 0; i < TOTAL_IC; i++) {
            for (int j = 0; j < ic[i]; j++) {
                // Check Overvoltage/Undervoltage
                if (bms_ic[i].cells.c_codes[j] * 0.0001 > OV || bms_ic[i].cells.c_codes[j] * 0.0001 < UV) {
                    NV[i][j]++;
                    uint16_t faultCode = generateFaultCode(i, j, "VOLTAGE"); // Generate fault code
                    if(faultCode>0)
                    	fc=faultCode;
                    DiagnosticMode(faultCode); // Call diagnostic mode
                } else {
                    NV[i][j] = 0;
                }
            }
        }

        // Check Overtemperature
        tempConvert();
        for (int i = 0; i < TOTAL_IC; i++) {
            for (int j = 1; j < 4 ; j++) {
                if (testTemperatures[i][j] > OT /*|| isnan(testTemperatures[i][j])*/) {
                    NT[i][j]++;
                    uint16_t faultCode = generateFaultCode(i, j, "TEMPERATURE"); // Fault code for temperature
                    DiagnosticMode(faultCode); // Call diagnostic mode
                } else {
                    NT[i][j] = 0;
                }
            }
    }
   }
    if (uart_rx_flag) {
           process_uart_command(uart_rx_buffer);
           uart_rx_flag = 0;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 16;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1279;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 14999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1279;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 14999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 6400;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 2559;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 7499;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 128;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 2559;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 7499;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|LTC6812_CS_Pin|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 LTC6812_CS_Pin PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|LTC6812_CS_Pin|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Configures the CAN communication filter

void CanFilterConfig(){
	CAN_FilterTypeDef filterConfig;

	filterConfig.FilterBank = 0;
	filterConfig.FilterActivation = ENABLE;
	filterConfig.FilterFIFOAssignment = 0;
	filterConfig.FilterIdHigh = 0x0000; //ID you want to allow to pass and shift it left by 5
	filterConfig.FilterIdLow = 0x0000;
	filterConfig.FilterMaskIdHigh = 0x0000; //if(Mask & ID == ID) --> Allow this packet to pass
	filterConfig.FilterMaskIdLow = 0x0000;
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	filterConfig.SlaveStartFilterBank = 14;
	if(HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}
void Discharge_Mode(){
	
    measure_and_error();
    AIR_ON_Flag=true;
    HAL_GPIO_WritePin(GPIOB,STM_AIR_CTRL,RESET);
    HAL_GPIO_WritePin(GPIOB,TS_ON_SIGNAL,RESET);
    precharge=false;
    while(TSMS>2000)
    {
    HAL_GPIO_WritePin(GPIOB,TS_ON_SIGNAL,SET);
    pre_charge();
    measure_and_error();
    }

}

//Initializes the LTC's registers and the SPI communication
void LTC6812_Init(){

	LTC6812_Initialize();	//Initializes the SPI communication at 1MHz
	LTC6812_init_cfg(TOTAL_IC, bms_ic);	//Initializes the confiugration registers to all 0s
	//This for loop initializes the configuration register variables
	for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++)
  {
    LTC6812_set_cfgr(current_ic,bms_ic,REFON,ADCOPT,gpioBits_a,dccBits_a,dctoBits,UV,OV);
    LTC6812_set_cfgr_vref2(TOTAL_IC,bms_ic,true);  }
	LTC6812_reset_crc_count(TOTAL_IC,bms_ic);	//sets the CRC count to 0
	LTC6812_init_reg_limits(TOTAL_IC, bms_ic);	//Initializes the LTC's register limits for LTC6811 (because the generic LTC681x libraries can also be used for LTC6813 and others)
	wakeup_sleep(TOTAL_IC);
	LTC6812_wrcfg(TOTAL_IC,bms_ic);	//writes the configuration variables in the configuration registers via SPI
}

//Initializes the error counting arrays
void errorCounterInit(){
	for (int i = 0; i < TOTAL_IC; i++){
    for (int j = 0; j < 10; j++){
      NV[i][j] = 0;
      if (j < 5)
        NT[i][j] = 0;
    }
  }
}
/*void errorCheck() {
   if(BMS_State_Out == 0 && k==1)
	BMS_State_Out = 1;

    for (int i = 0; i < TOTAL_IC; i++)
       {
        for (int j = 0; j < ic[i]; j++)
        {
            // Check Overvoltage/Undervoltage
            if (bms_ic[i].cells.c_codes[j] * 0.0001 > OV || bms_ic[i].cells.c_codes[j] * 0.0001 < UV)
            {
            	BMS_State_Out = 0;
            	NV[i][j]++;
             	v=1;
             	m++;
            } else
            {
                NV[i][j] = 0;
            }
        }
        if (BMS_State_Out == 0) break;
        else
        	v=0;
    }

    // Check Overtemperature
    if(v==1)
    	return;
    else
    	v=0;
    m++;

    tempConvert();

    for (int i = 0; i < TOTAL_IC; i++) {
        for (int j = 1; j < 4; j++) {
            if (testTemperatures[i][j] > OT || isnan(testTemperatures[i][j]) || testTemperatures[i][j]<0) {
                NT[i][j]++;
                BMS_State_Out = 0;
                k=0;
                break;
            } else {
                NT[i][j] = 0;
                k=1;
            }
        }
    }

    // Implement the BMS state change logic

    	/*for (int i = 0; i < TOTAL_IC; i++)
    	  {
            /*for (int j = 0; j < 12; j++)
            {
              if ( NV[i][j] >= N_Error)
              {
               BMS_State_Out = 0;
              }
    		 }
           }*/
    		/*for (int i = 0; i < TOTAL_IC; i++){
    				for (int j = 1; j < 4; j++){
    					if (NT[i][j] >= N_Error){
    						BMS_State_Out = 0;
    					}
    				}
          }
    			/*if ( (NPEC_V  > N_Error * 5) || (NPEC_T > N_Error * 10) ){
    				BMS_State_Out = 0;
          }
    	if(Tmax_Pack>50.0)
    		BMS_State_Out = 0;*/

    	/*if(adcVal_10Hz[4]<500)
    		BMS_State_Out = 0;
    	}*/
void errorCheck() {
    if (BMS_State_Out == 0 && k == 1)
        BMS_State_Out = 1;

    // Check overvoltage/undervoltage errors
    for (int i = 0; i < TOTAL_IC; i++) {
        for (int j = 0; j < ic[i]; j++) {
            // Check Overvoltage/Undervoltage
            if (bms_ic[i].cells.c_codes[j] * 0.0001 > OV || bms_ic[i].cells.c_codes[j] * 0.0001 < UV) {
                BMS_State_Out = 0;
                NV[i][j]++;
            } else {
                NV[i][j] = 0;
            }
        }
        if (BMS_State_Out == 0) {
            break;  // Exit outer loop if error is found
        }
    }

    // Convert temperature and check for overtemperature
    tempConvert();

    for (int i = 0; i < TOTAL_IC; i++) {
        for (int j = 1; j < 4; j++) {
            if (testTemperatures[i][j] > OT /*|| isnan(testTemperatures[i][j]) || testTemperatures[i][j] < 0*/) {
                NT[i][j]++;
                BMS_State_Out = 0;
                k = 0;
                break;  // Exit inner loop if error is found
            } else {
                NT[i][j] = 0;
                k = 1;
            }
        }
        if (BMS_State_Out == 0) {
            break;  // Exit outer loop if error is found
        }
    }

    // Final ADC check (uncommented to work as expected)
  //  if (adcVal_10Hz[0] < 1800) {
    //    BMS_State_Out = 0;
    //}
}

uint16_t generateFaultCode(int icIndex, int cellIndex, const char* errorType) {

    if (strcmp(errorType, "VOLTAGE") == 0) {
    	uint8_t l=0;
    	for(uint8_t i=0;i<icIndex;i++)
    	{
    		l=ic[icIndex]+l;
    	}
        faultCode = 31 + l + cellIndex; // Assign unique fault code for voltage
    }
    else if (strcmp(errorType, "TEMPERATURE") == 0) {
    	uint8_t l=0;
    	    	for(uint8_t i=0;i<icIndex;i++)
    	    	{
    	    		l=ic[icIndex]+l;
    	    	}
        faultCode = 0 + l + cellIndex; // Assign unique fault code for temperature
    }
    // Add other error types and fault code generation as needed.
    return faultCode;
}


void pre_charge() {

  if (AIR_ON_SIG && AIR_ON_Flag) {
    HAL_GPIO_WritePin(GPIOB,STM_AIR_CTRL,SET);
    DISCHARGE = true;
    AIR_Status=1;
    AIR_ON_Flag=0;
    precharge=true;
  }

}

void MeasureVoltageParam() {
    uint16_t Vmin_IC[10];
    uint16_t Vmax_IC[10];
    Vmax_Pack = bms_ic[0].cells.c_codes[0];
    Vmin_Pack = bms_ic[0].cells.c_codes[0];

    // Initialize IC Voltage
    for (int i = 0; i < TOTAL_IC; i++) {
        IC_Voltage[i] = 0;
    }

    // Measure voltage parameters across all ICs
    for (int cic = 0; cic < TOTAL_IC; cic++) {
        Vmin_IC[cic] = bms_ic[cic].cells.c_codes[0];
        Vmax_IC[cic] = bms_ic[cic].cells.c_codes[0];
        for (int i = 0; i < ic[cic]; i++) {
            cell_voltages[i] = bms_ic[cic].cells.c_codes[i];
            IC_Voltage[cic] += (float)bms_ic[cic].cells.c_codes[i] * 0.0001;

            // Update min and max for this IC
            if (Vmin_IC[cic] > bms_ic[cic].cells.c_codes[i]) {
                Vmin_IC[cic] = bms_ic[cic].cells.c_codes[i];
            }
            if (Vmax_IC[cic] < bms_ic[cic].cells.c_codes[i]) {
                Vmax_IC[cic] = bms_ic[cic].cells.c_codes[i];
            }
        }

        // Update overall min and max for the pack
        if (Vmax_Pack < Vmax_IC[cic]) {
            Vmax_Pack = Vmax_IC[cic];
        }
        if (Vmin_Pack > Vmin_IC[cic]) {
            Vmin_Pack = Vmin_IC[cic];
        }
    }

    // Calculate delta (difference between max and min pack voltage)
    delta = ((float)Vmax_Pack - (float)Vmin_Pack) * 0.0001;

    // Calculate module voltages
    for (int i = 0; i < ((TOTAL_IC / 2) + 1); i++) {
        Module_Voltage[i] = IC_Voltage[2 * i] + IC_Voltage[(2 * i) + 1];
    }

    // Calculate total pack voltage
    V_Pack = 0;
    for (int i = 0; i < (TOTAL_IC / 2); i++) {
        V_Pack += Module_Voltage[i];
    }

    // Transmit Pack Voltage and AIR Status over CAN
    if (MODE_CONTROL == DISCHARGE_ENABLE){
     uint8_t TxData[8] = {0};  // CAN data array

    // Configure the CAN header
    myTxHeader.StdId = 0x101;     // Unique ID for Pack Voltage and AIR status
    myTxHeader.DLC = 8;           // Data Length set to 3 bytes (for V_Pack and AIR_Status)
    myTxHeader.IDE = CAN_ID_STD;
    myTxHeader.RTR = CAN_RTR_DATA;

    // Pack voltage (V_Pack) and AIR status (AIR_Status) into TxData
    // Pack voltage (V_Pack) and AIR status (AIR_Status) into TxData
    uint16_t voltage_scaled = (uint16_t)(V_Pack * 100);  // Scale V_Pack for transmission (2 decimal places)
    TxData[0] = (voltage_scaled >> 8) & 0xFF;            // Higher byte of V_Pack
    TxData[1] = voltage_scaled & 0xFF;                   // Lower byte of V_Pack
    uint16_t Tmax_scaled = (uint16_t)(Tmax_Pack * 100);
    TxData[2] = (Tmax_scaled >> 8) & 0xFF;
    TxData[3] = Tmax_scaled & 0xFF;
    uint16_t Current_scaled = (uint16_t)(Current * 100);
    TxData[4] = (Current_scaled >> 8) & 0xFF;
    TxData[5] =  Current_scaled & 0xFF;
    if(AIR_Status==0 || TSMS<2000)
    {
    AIR_Status=0;
    TxData[6] = 2;
    }                              // AIR status as a single byte
    else if(AIR_Status==1 && TSMS >2000)
    TxData[6] = 1;
    TxData[7] = faultCode;
    // Transmit the data over CAN
    if (HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, TxData, &TxMailbox) != HAL_OK) {
    	can_error = HAL_CAN_GetError(&hcan1);
        HAL_CAN_Stop(&hcan1);  // Handle transmission error if needed
        HAL_CAN_Start(&hcan1);
        v++;
    }
    }

    //Collect data for logging
    uint32_t timestamp = HAL_GetTick();         // Timestamp in ms
    float totalVoltage = V_Pack;                // Total Pack Voltage
    float maxTemp = Tmax_Pack;                  // Max Temperature
    float current = Current;                    // Current value
    uint8_t canStatus = AIR_Status;             // AIR status
    float soc = SOC;                            // SOC value

    // Print current values for debugging
    printf("V_Pack = %.2f, Tmax_Pack = %.2f, Current = %.2f, SOC = %.2f\n", V_Pack, Tmax_Pack, Current, SOC);

    // Check buffer size and remaining space
    printf("Current logBuffer size: %d bytes\n", logIndex);
    printf("Remaining space in buffer: %d bytes\n", LOG_BUFFER_SIZE - logIndex);

    // Append data to the buffer
    int len = snprintf(logBuffer + logIndex, LOG_BUFFER_SIZE - logIndex,
                       "%lu,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                       timestamp, totalVoltage, current, maxTemp, canStatus, soc,
                       Module_Voltage[0], Module_Voltage[1], Module_Voltage[2],
                       Module_Voltage[3], Module_Voltage[4], Module_Voltage[5]);

    // Update buffer index
    logIndex += len;

    // Check if buffer overflow is imminent (reserve space for writing)
    if (logIndex >= LOG_BUFFER_SIZE - 100) {
        writePending = true;  // Set the flag to trigger SD write
    }

    // If data is ready to be written to SD card
    if (writePending) {
        // Mount SD card
        fresult = Mount_SD("/");

        if (fresult == FR_OK) {
            // Write the buffered data to the SD card
            if (Update_File("BMS.TXT", logBuffer) == FR_OK) {
                printf("Data written successfully.\n");
            } else {
                printf("Failed to write data to file.\n");
            }
        } else {
            printf("Failed to mount SD card for writing.\n");
        }

        // Unmount SD card after writing
        Unmount_SD("/");

        // Reset the writePending flag after writing
        writePending = false;
    }

}

void tempConvert() {
    // Initialize Tmax_Pack to a very low value
    Tmax_Pack = -1000.0;
        for (int cic = 0; cic < TOTAL_IC; cic++) {
          for (int i = 1; i < 4	; i++) {
            	double resistance = ((((double)bms_ic[cic].aux.a_codes[i]/(30000-(double)bms_ic[cic].aux.a_codes[i])) * 10000.0));
                testTemperatures[cic][i] = 1.0 / (0.003354016 + 0.000256524 * log(resistance / 10000.0) +0.00000260597 * log(pow((resistance / 10000.0), 2)) + 0.0000000632926 * log(pow((resistance / 10000.0) , 3))) - 273.15;

           if (testTemperatures[cic][i] > Tmax_Pack) Tmax_Pack = testTemperatures[cic][i];
            }
          }
        }


void measure_and_error()
    {
		  		if(flagStartCVConversion){
		  			wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  			LTC6812_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL); //should take 1.1 ms for 27KHz mode and 1.3 ms for 14KHz mode
		  			flagStartCVConversion = false;
		  		}
		  		//TIM7 (3Hz)
                    if(flagReadCVConversion){
		  			wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  			error = LTC6812_rdcv(CELL_CH_ALL, TOTAL_IC, bms_ic);
		  			flagReadCVConversion = false;
		  			MeasureVoltageParam();

		  		}
		  		//TIM9 (5Hz)
		  		if(flagStartAuxConversion){
		  			wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  			LTC6812_adax(MD_7KHZ_3KHZ, AUX_CH_ALL); //should take 1.1 ms for 27KHz mode and 1.3 ms for 14KHz mode
		  			flagStartAuxConversion = false;

		  			}
		  		//TIM11 (5Hz)
		  		if(flagReadAuxConversion){
		  			wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  			auxError = LTC6812_rdaux(REG_ALL, TOTAL_IC, bms_ic);
		  			flagReadAuxConversion = false;

 		}
	//Checks if there is an error every 50ms (TIM11)
		  		if(flagErrorCheck){
		  			errorCheck();
		  			flagErrorCheck = false;
		  			}


		  		if(BMS_State_Out)
		  		{
		  			HAL_GPIO_WritePin(GPIOA,STM_MODE_CTRL,SET);
		  		}
		  		else
		  			{
		  			HAL_GPIO_WritePin(GPIOA,STM_MODE_CTRL,RESET);
		  			//SendChargerControl(1);
		  			}

	}

void DiagnosticMode(uint16_t fault) {
    if (fault >= 0 && fault <= 29) {
        printf("Over Temperature Detected in Module %d : Sensor %d\n", (fault / 6) + 1, (fault % 6) + 1);
    }

    // Overvoltage Faults (31–140)
    if (fault >= 31 && fault <= 140) {
        uint16_t index = fault - 31;   // Normalize to 0–109
        int module = (index / 22) + 1; // Each module has 22 cells
        int cell   = (index % 22) + 1; // Cell number inside module
        printf("Cell Overvoltage Detected in Module %d : Cell %d\n", module, cell);
    }

    // Undervoltage Faults (141–250)
    else if (fault >= 141 && fault <= 250) {
        uint16_t index = fault - 141;  // Normalize to 0–109
        int module = (index / 22) + 1; // Each module has 22 cells
        int cell   = (index % 22) + 1; // Cell number inside module
        printf("Cell Undervoltage Detected in Module %d : Cell %d\n", module, cell);
    }
    if (fault == 361) {
        printf("Current Sensor SCS implausibility detected\n");
    }
    if (fault >= 362 && fault <= 391) {
        printf("Temperature Sensor open wire Detected in Module %d : Sensor %d\n", ((fault - 362) / 6) + 1, ((fault - 362) % 6) + 1);
    }
    // Add more fault handling cases as per your system design.
}

//Processing UART
void process_uart_command(char *command) {
    if (strncmp(command, "OV=", 3) == 0) {
        OV = atof(&command[3]);
        printf("OV set to %.2f\n", OV);
    } else if (strncmp(command, "UV=", 3) == 0) {
        UV = atof(&command[3]);
        printf("UV set to %.2f\n", UV);
    } else if (strncmp(command, "OT=", 3) == 0) {
        OT = (uint16_t)atoi(&command[3]);
        printf("OT set to %d\n", OT);
    } else if (strncmp(command, "OC=", 3) == 0) {
        OC = (uint16_t)atoi(&command[3]);
        printf("OC set to %d\n", OC);
    } else if (strncmp(command, "MODE=", 5) == 0) {
        MODE_CONTROL = (uint8_t)atoi(&command[5]);
        printf("MODE_CONTROL set to %d\n", MODE_CONTROL);
    } else {
        printf("Unknown command: %s\n", command);
    }
}

/*void balance() {
    // Define delta threshold (this can be adjusted as needed)
    float delta_threshold = 0.1f;  // Example threshold of 50mV

    // Reset balancing states for all cells
    for (int cic = 0; cic < 5; cic++) {
        for (int i = 0; i < 11; i++) {
            balancing_on[i + (cic * 11)] = false;
        }
    }

    count = 1;  // Start with odd-indexed cells
    tim11counter= __HAL_TIM_GET_COUNTER(&htim11);  // Start the timer using TIM11 counter

    GPIO_Init();  // Initialize GPIO for mode control and discharge

    // Start the balancing process by enabling mode control
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);  // DISable balancing

    // Balancing loop continues while voltage delta is above threshold
    while (balancing) {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);  // Enable balancing

        // Iterate through each module and each cell within the module
        for (int cic = 0; cic < 5; cic++) {
            for (int i = count; i < 11; i = i + 2) {  // Alternating even and odd cells
                int cell_index = i + (cic * 11);  // Calculate global cell index

                // Calculate delta (voltage difference) for balancing decision
                float delta = cell_voltages[cell_index] - Vmin_Pack;  // Example delta calculation

                // Check if delta exceeds threshold, and if the cell needs to be balanced (discharged)
                if (delta > delta_threshold && !balancing_on[cell_index]) {  // If voltage is above Vmin_Pack and delta exceeds threshold
                    // Discharge the cell
                    wakeup_sleep(TOTAL_IC);
                    LTC6812_set_discharge(i + 1, TOTAL_IC, bms_ic);
                    LTC6812_wrcfg(TOTAL_IC, bms_ic);
                    LTC6812_wrcfgb(TOTAL_IC,bms_ic);
                    wakeup_idle(TOTAL_IC);
                    balancing_on[cell_index] = true;  // Mark this cell as balanced
                }
                // If delta falls below the threshold, stop discharging
                else if (delta < 0 && balancing_on[cell_index]) {
                    // Stop discharging the cell
                    wakeup_sleep(TOTAL_IC);
                    LTC6812_clear_dischargePin(i + 1, TOTAL_IC, bms_ic);
                    LTC6812_wrcfg(TOTAL_IC, bms_ic);
                    LTC6812_wrcfgb(TOTAL_IC,bms_ic);
                    wakeup_idle(TOTAL_IC);
                    balancing_on[cell_index] = false;  // Mark this cell as not balanced
                }
            }
        }

        // Periodic display of voltage, current, and temperature (for monitoring purposes)
        DisplayBalancingVoltage();
        MeasureCurrent();
        MeasureTemp();

        // Print elapsed time to the serial monitor
        uint32_t elapsed_time = __HAL_TIM_GET_COUNTER(&htim11) - tim11counter;
        printf(" Time elapsed: ");
        printf(elapsed_time);

        // Every 30 seconds, alternate between even and odd cells
        if (elapsed_time >= 30000) {
            count ^= 1;  // Toggle between even and odd cells
            tim11counter = __HAL_TIM_GET_COUNTER(&htim11);  // Reset the timer using TIM11 counter

            // Reconfigure LTC6812 for any necessary changes
            wakeup_sleep(TOTAL_IC);
            LTC6812_clear_discharge(TOTAL_IC, bms_ic);
            LTC6812_wrcfg(TOTAL_IC, bms_ic);
            LTC6812_wrcfgb(TOTAL_IC, bms_ic);
            wakeup_idle(TOTAL_IC);
            MeasureVoltageParam();  // Re-measure voltages

            // Reset balancing states for all cells after each 30-second period
            for (int cic = 0; cic < 5; cic++) {
                for (int i = 0; i < 22; i++) {
                    balancing_on[i + (22 * cic)] = false;
                }
            }
        }

        // Allow the user to stop the process via serial input
        /*if (Serial.available()) {
            char* input = read_string();
            if (!strcmp(input, "q")) {
                balancing = false;  // Stop balancing process
            }
        }
    }

    // Reset all cells' balancing state at the end
    for (int cic = 0; cic <5; cic++) {
        for (int i = 0; i < 22; i++) {
            balancing_on[i + (22 * cic)] = false;
        }
    }

    balancing = false;  // Final stop of the balancing process
}*/
// CSV File Initialization and Header Setup
void initializeCSVFile(void) {
    Mount_SD("/");

    if (fresult == FR_OK) {
        printf("SD card mounted successfully.\n");

        // Create or open file and add header if it's a new file
        if (Create_File("BMS.TXT") == FR_OK) {
            sprintf(buffer, "Timestamp,Total_V,Current,Max_Temp,AIR_Status,SOC,"
                            "Module1_V,Module2_V,Module3_V,Module4_V,Module5_V,Module6_V\n");
            Update_File("BMS.TXT", buffer);
        } else {
            printf("Failed to create log file.\n");
        }

        Unmount_SD("/");
    } else {
        printf("SD card mount failed.\n");
    }
}

//Charging code
void ChargerControl_Init() {
    // Initialize CAN message for sending max allowable charging limits
    myTxHeader.ExtId = 0x1806e5f4;
    myTxHeader.IDE = CAN_ID_EXT;
    myTxHeader.RTR = CAN_RTR_DATA;
    myTxHeader.DLC = 8;
}

void SendChargerControl(uint8_t control) {
    // Prepare data for Message 1 to control charger
    uint8_t TxData[8] = {0};
    TxData[0] = 0x11;//(voltage >> 8) & 0xFF;     // Max Voltage High Byte
    TxData[1] = 0xf8;//voltage & 0xFF;             // Max Voltage Low Byte
    TxData[2] = 0;//(current >> 8) & 0xFF;      // Max Current High Byte
    TxData[3] = 20;//current & 0xFF;             // Max Current Low Byte
    TxData[4] = control;                    // Control: 0 = Start, 1 = Stop
    TxData[5] = 0;//current & 0xFF;
    TxData[6] = 0;//current & 0xFF;
    TxData[7] = 0;//current & 0xFF;
    // Send the message on CAN bus
    if(HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, TxData, &TxMailbox)!=HAL_OK)
    {
    Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // CAN RX interrupt handler
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &myRxHeader, RxData);



    if (myRxHeader.StdId == 0x18FF50E5) {  // Check if Message 2 is received
        uint16_t chargerVoltage = (RxData[0] << 8) | RxData[1];
        uint16_t chargerCurrent = (RxData[2] << 8) | RxData[3];
        uint8_t statusFlags = RxData[4];

        // Update BMS based on charger status
        if (statusFlags & 0x01) {
            printf("Hardware failure detected.\n");
            //SendChargerControl(voltage, current, 1); // Stop charging
        }
        if (statusFlags & 0x02) {
            printf("Over temperature protection active.\n");
            //SendChargerControl(voltage, current, 1); // Stop charging
        }
        if (statusFlags & 0x04) {
            printf("Input voltage error detected.\n");
            //SendChargerControl(voltage, current, 1); // Stop charging
        }
        if (statusFlags & 0x08) {
            printf("Communication timeout.\n");
            //SendChargerControl(voltage, current, 1); // Stop charging
        }
    }

	if(myRxHeader.StdId == 0x181)
	{
		  if(RxData[0]==0xeb){
		  shifted_data = RxData[1] | (RxData[2]<<8);
		  Vbus=(shifted_data * 1.0) / 31.585;
		  		if(Vbus>(0.9*V_Pack)){
		  			AIR_ON_SIG=1;
		  		}
		  		else
		  			AIR_ON_SIG=0;
	  }
	}


	if(myRxHeader.StdId == 0x100)
		{
			  if(RxData[0]==10){

               x=true;
		  }
		}
}

void Charge_Mode() {
    // Example voltage and current limits
	   ChargerControl_Init();
       measure_and_error();
    // Send the control message
       HAL_GPIO_WritePin(GPIOB,STM_AIR_CTRL,RESET);
       HAL_GPIO_WritePin(GPIOB,TS_ON_SIGNAL,RESET);

       while(TSMS>2000)
       {
       HAL_GPIO_WritePin(GPIOB,TS_ON_SIGNAL,SET);
       measure_and_error();
       HAL_GPIO_WritePin(GPIOB,STM_AIR_CTRL,SET);
       // Delay to match the cycle time
       }
       //Balance function call
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc == &hadc1) {
	  float Vout_1  = ((adcVal_10Hz[4]*ADC_REF_VOLTAGE) / ADC_RESOLUTION)*(5.0/3.3); // Reading current ADC value
	  C_U = ( Vout_1-V_OFFSET ) / G_CHANNEL_1;
	  float Vout_2  = ((adcVal_10Hz[0]*ADC_REF_VOLTAGE) / ADC_RESOLUTION)*(5.0/3.3); // Reading current ADC value
	  C_O = (Vout_2-V_OFFSET) / G_CHANNEL_2;
	  if(C_O>70)
		  Current=C_O;
	  else
		  Current=C_U;

	  TSMS=adcVal_10Hz[3];
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcVal_10Hz, 5);
      }
}

//When half the timer's period is complete we send the messages in the CAN bus
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	//10Hz CAN transmit
	 if(htim->Instance == TIM2)
	 {
		flag10Hz = true;
        /*float deltaCharge = (Current * TIMER_INTERVAL_S) / 3600.0; // Convert current to Ah over 1 second
        totalCharge -= deltaCharge;  // Accumulate charge
        SOC = previousSOC - (totalCharge / BATTERY_CAPACITY_AH) * 100.0;

        // Bound SOC between 0% and 100%
        if (SOC > 100.0) SOC = 100.0;
        if (SOC < 0.0) SOC = 0.0;

        previousSOC = SOC;*/
		 if (MODE_CONTROL == CHARGE_ENABLE && TSMS>2000)
			SendChargerControl(0);
		 else if (MODE_CONTROL == CHARGE_ENABLE && TSMS<100)
			SendChargerControl(1);
		if(timeIT10 ++ > 65535) timeIT10 = 0;

	 }

	 //TIM6 starts cell voltage conversion
	else if(htim->Instance == TIM6){
		HAL_TIM_Base_Stop_IT(&htim6);	//stops this timer
		__HAL_TIM_CLEAR_IT(&htim6 ,TIM_IT_UPDATE);	//clears the IT flag
		__HAL_TIM_SET_COUNTER(&htim6, 0);	//resets the timer's counter to 0
		flagStartCVConversion = true;	//enables the cell voltages conversion code block
		if(tim6counter ++ > 65535) tim6counter = 0;
		HAL_TIM_Base_Start_IT(&htim7);	//starts the next timer (TIM7)


	}//TIM7 reads cell voltages
	else if(htim->Instance == TIM7){
		HAL_TIM_Base_Stop_IT(&htim7);	//stops this timer
		__HAL_TIM_CLEAR_IT(&htim7 ,TIM_IT_UPDATE);	//clears the IT flag
		__HAL_TIM_SET_COUNTER(&htim7, 0);	//resets the timer's counter to 0
		flagReadCVConversion = true;	//enables the read cell voltages code block
		if(tim7counter ++ > 65535) tim7counter = 0;
		HAL_TIM_Base_Start_IT(&htim9);	//starts the next timer (TIM9)
		HAL_TIM_Base_Start_IT(&htim6);

	}//TIM9 starts aux voltage conversion
	else if(htim->Instance == TIM9){
		HAL_TIM_Base_Stop_IT(&htim9);	//stops this timer
		__HAL_TIM_CLEAR_IT(&htim9 ,TIM_IT_UPDATE);	//clears the IT flag
		__HAL_TIM_SET_COUNTER(&htim9, 0);	//resets the timer's counter to 0
		flagStartAuxConversion = true;	//enables the aux voltages conversion code block
		if(tim9counter ++ > 65535) tim9counter = 0;
		HAL_TIM_Base_Start_IT(&htim11);	//starts the next timer (TIM11)
		a++;

	}
	 //TIM11 reads aux voltage conversion
	else if(htim->Instance == TIM11){
		HAL_TIM_Base_Stop_IT(&htim11);	//stops this timer
		__HAL_TIM_CLEAR_IT(&htim11 ,TIM_IT_UPDATE);	//clears the IT flag
		__HAL_TIM_SET_COUNTER(&htim11, 0);	//resets the timer's counter to 0
		flagReadAuxConversion = true;	//enables the read aux voltages code block
		flagErrorCheck = true;	//enables the error chack and state change code block

		//Code for logging data
		if(tim11counter ++ > 65535) tim11counter = 0;
		HAL_TIM_Base_Start_IT(&htim6);	//starts the next timer (TIM6)
	}
	    // Logging at 1kHz using TIM8
	    else if (htim->Instance == TIM8) {
	       if (writePending || logIndex > 0) {   // Write only if required
	            // Swap buffer to avoid blocking logging
	            memcpy(tempBuffer, logBuffer, logIndex);
	            memset(logBuffer, 0, LOG_BUFFER_SIZE);  // Clear active buffer
	            logIndex = 0;                          // Reset index
	            bufferSwapFlag = true;                 // Mark buffer swapped
	            writePending = false;

	            // Write data to SD card
	            Mount_SD("/");
	            if (fresult == FR_OK) {
	                Update_File("BMS.TXT", tempBuffer);  // Write buffered data
	            }
	            Unmount_SD("/");
	        }
	    }

}
 // Function to run the MUX diagnostic and handle results
	 void run_mux_diagnostic(uint8_t total_ic, cell_asic *ic) {
	     // Start MUX Diagnostic
	     LTC6812_diagn();

	     // Read back diagnostic results
	      mux_error = LTC6812_rdcomm(total_ic, ic);

	     // Interpret Results
	     if (mux_error != 0) {
	        // printf("MUX diagnostic failed with error code: %d\n", mux_error);
	         // Additional handling, such as identifying faulty ICs or cells, can be added here
	     } else {
	         //printf("MUX diagnostic passed, all channels functioning correctly.\n");
	     }
	 }

//UART necessary details
int __io_putchar(int ch)
{
    // Transmit one character over UART
    HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart4) {
        // Check if Enter is pressed to end command
        if (uart_rx_buffer[uart_rx_index - 1] == '\n') {
            uart_rx_flag = 1; // Command ready to process
            uart_rx_buffer[uart_rx_index - 1] = '\0'; // Null-terminate string
            uart_rx_index = 0; // Reset index for next command
        } else {
            // Store character in buffer
            uart_rx_index++;
            if (uart_rx_index >= 256) {
                uart_rx_index = 256 - 1; // Prevent overflow
            }
        }

        // Re-enable interrupt for next character
        HAL_UART_Receive_IT(&huart4, (uint8_t *)&uart_rx_buffer[uart_rx_index], 1);
    }
}

/*float gaussian(float cell_voltage, float a , float b, float c){
  return a*exp(-1*((cell_voltage-b)/c)*((cell_voltage-b)/c));

}
float SOC(){
   OCV_SOC = 1.0-(0.5*(gaussian(Vmin_Pack*0.0001 , a[11] , b[0], c[0]) + gaussian(Vmin_Pack*0.0001, a[1],b[1],c[1]) + gaussian(Vmin_Pack*0.0001,a[2],b[2],c[2]) + gaussian(Vmin_Pack*0.0001,a[3],b[3],c[3])));
  COULOMB_SOC  = (COULOMB_SOC - (Current*0.2/(16.8*3600)));
  return ((OCV_SOC + COULOMB_SOC)*0.5);
}*/


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
  while (1){
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
