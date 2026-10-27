/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "LCD.h"
#include "main.h"
#include "fatfs.h"
#include "sd_functions.h"
#include <inttypes.h> // Required for PRI macros
#include "LCD.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

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

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

#define QUEUE_SIZE 64

// CAN data decoding constants from provided DBC file
#define TIME_SCALE 0.001
#define TIME_OFFSET 1577840400

#define POS_SCALE 0.000001
#define LAT_OFFSET -90
#define LONG_OFFSET -180

#define SPEED_SCALE 0.001

#define ACCEL_SCALE 0.125
#define ACCEL_OFFSET -64
#define ANGULAR_SCALE 0.25
#define ANGULAR_OFFSET -256

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const float R_FIXED = 10000.0;
const float R0      = 5000.0;    // from the table at 25 °C
const float T0      = 298.15;
const float BETA    = 3960.0;    // fit for ~25–85 °C
const float ADC_MAX = 4095.0;    // 1023 for 10-bit



typedef struct {
    uint8_t is_extended;
    uint32_t id;
    uint8_t data[8];
} CAN_Message;

volatile CAN_Message can_queue[QUEUE_SIZE];
volatile int write_index = 0;
volatile int read_index = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  //printf("MAYBE???!\n");
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
    // copy data into queue
	  //printf("We are reading a message!\n");
    memcpy((void*)can_queue[write_index].data, rx_data, rx_header.DLC);

    // check CAN ID format
    if (rx_header.IDE == CAN_ID_EXT) {
      can_queue[write_index].id = rx_header.ExtId;
      can_queue[write_index].is_extended = 1;
    }
    else {
      can_queue[write_index].id = rx_header.StdId;
      can_queue[write_index].is_extended = 0;
    }

    // increment ptr
    write_index = (write_index + 1) % QUEUE_SIZE;
  }
}

// CAN data decoder helper function
uint64_t extract_bits(uint64_t raw_data, int start, int length) {
  // edge case: length = 0
  if (length <= 0) return 0;

  // edge case: define behavior for lengths of 64 or greater
  if (length >= 64) return raw_data >> start;

  // create bit mask with 64-bit 1
  uint64_t mask = (1ULL << length) - 1;
  return (raw_data >> start) & mask;
}

int32_t extract_signed32(uint64_t raw_data, int start)
{
    uint32_t value = (uint32_t)extract_bits(raw_data, start, 32);

    if (value & 0x80000000U) {
        return -1 - (int32_t)(~value);
    }

    return (int32_t)value;
}

static float extract_float32(uint64_t raw_data, int start) {
  uint32_t bits = (uint32_t)extract_bits(raw_data, start, 32);
  float f;
  memcpy(&f, &bits, sizeof f);
  return f;
}

// CAN data decoder for GPS/IMU module
double decode_physical_value(uint64_t raw_data, int start, int length, double offset, double scale) {
  uint64_t raw_value = extract_bits(raw_data, start, length);
  return offset + scale * (double) raw_value;
}

uint32_t Read_ADC_Value(ADC_HandleTypeDef* hadc, bool convert)
{
	//range is 4095 -- 370
    uint32_t adc_val = 0;

    // Start the ADC peripheral conversion
    HAL_ADC_Start(hadc);

    // Wait for conversion to complete (with a 10ms timeout)
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
    {
        // Read the converted value from the data register
        adc_val = HAL_ADC_GetValue(hadc);
    }

    // Stop conversion to save power / clean up state
    HAL_ADC_Stop(hadc);
    if(convert){
    // 1. Constrain the input to your known physical sensor limits
	if (adc_val > 4095) adc_val = 4095;
	if (adc_val < 370)  adc_val = 370;

	// 2. Map the constrained value to a 0 - 100% range
	// Formula: (raw - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
	// Here: (raw_adc - 4095) * (100 - 0) / (370 - 4095) + 0

	uint32_t throttle_percent = (4095 - adc_val) * 100 / (4095 - 370);

		return throttle_percent;
    }else{
    	return adc_val;
    }
}

float readTempC(int adc) {
  float r = R_FIXED * (ADC_MAX - adc)/adc;   // thermistor to ground
  float invT = 1.0 / T0 + log(r / R0) / BETA;
  float tempC = 1.0 / invT - 273.15;
  return tempC * 9.0 / 5.0 + 32.0;

}


//void log_imu_data_to_sd(double accel[3], double angular[3]) {
//  // mount sd card
//  sd_mount();
//
//  // write data to sd card
//  // acceleration x,y,z and angular rate x,y,z
//  char write_buf[64];
//  snprintf(write_buf, sizeof(write_buf), "%f,%f,%f,%f,%f,%f\n", accel[0], accel[1], accel[2], angular[0], angular[1], angular[2]);
//
//  if (sd_append_file("imu_log.csv", write_buf) == FR_OK) {
//    printf("data successfully logged!\n");
//  }
//  else {
//    printf("data logging error\n");
//  }
//
//  // unmount sd card
//  sd_unmount();
//}




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
  MX_CAN1_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  LCD_Handle lcd;
  LCD_Init(&lcd, &hspi3, GPIOB, GPIO_PIN_4, GPIOB, GPIO_PIN_6);
  LCD_Clear(&lcd);
  LCD_PrintStrAt(&lcd, 0,0, "Hello FREDDY :)1");
  HAL_Delay(500);
  LCD_PrintStrAt(&lcd,0,2, "speed:");
  //LCD_DisplaySpeed(&lcd, 12, true);

  //cpp_main();

  HAL_SPI_Transmit(&hspi1, 00000000, 1, 10);
  setvbuf(stdout, NULL, _IONBF, 0); // DISMISS BUFFERING: forces printf to send data immediately

  // 1. Configure the CAN Filter to accept all messages
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT; // enables extended 29-bit ids

  // allow all incoming messages to pass through
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;

  // enable FIFO 0
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK)
  {
    Error_Handler();
  }

  // 2. Start the CAN peripheral
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  // 3. Activate the notification
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  // local variables
  uint32_t trip_distance; // distance traveled since last reset
  uint32_t total_distance; // distance traveled in total

  // sd card init + startup operations
//  sd_init();
//  HAL_Delay(100);
//  sd_mount();
//  sd_list_files();
//  sd_unmount();
printf("TEST HELLO1");


  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //go into an interrutp that triggeres every 0.5 second via semopher give and take
	  uint32_t a = Read_ADC_Value(&hadc1, false);
	  float temp = readTempC(a);
	  printf("thermistor value: %f\n", temp);
	  char str_buffer[6]; // Safe buffer size for 4 digits + null terminator

	  // Convert integer to string format safely
	  snprintf(str_buffer, sizeof(str_buffer), "%lu", (unsigned long)a);
//	  LCD_PrintLinePadded(&lcd, 2, str_buffer);
	  //HAL_Delay(500);
	  uint8_t time_valid = 0;
	  uint8_t pos_valid = 0;
	  uint8_t dist_valid = 0;
	  uint8_t speed_valid = 0;
	  uint8_t imu_valid = 0;
//	  printf("testing read_idx: %d ,write index: %d \n", read_index, write_index);
	      if (read_index != write_index) {
	        // pull next CAN data frame off the queue
	        uint8_t is_ext = can_queue[read_index].is_extended;
	        uint32_t msg_id = can_queue[read_index].id;
	        //uint64_t raw_data = *(uint64_t*)can_queue[read_index].data;
	        uint64_t raw_data;

	        memcpy(&raw_data, (const void*)can_queue[read_index].data, 8);
	        if(msg_id != 9){
	        	//printf("is_ext %d\n", is_ext);
				// printf("message id %d\n", msg_id);
				 //printf("1raw data %llu\n", (unsigned long long)raw_data);
	        }

	        // process data based on CAN message ID

	        if (is_ext) {
	        //values outputted are *1000 so theyre not floats
	          uint32_t base_id = msg_id & 0xFFFFFFC0;
	          uint8_t node_id = msg_id & 0x3F;
	          switch(base_id) {
	            case 117815424: { // PERIODIC_FRAME_2
	              float raw_angle = extract_float32(raw_data, 0);
	              float angle = extract_float32(raw_data, 32);
	              //outputting values that
	              if(node_id != 1){
	            	  printf("Node %u | F2 | RawAngle: %ld mrot | Angle: %ld mrot\n", (unsigned)node_id, (long)(raw_angle * 1000.0f), (long)(angle * 1000.0f));
	              }
	              break;

	            }

	          case 117815488: { // PERIODIC_FRAME_3
	            uint32_t timestamp = (uint32_t)extract_bits(raw_data, 0, 32);
	            float position     = extract_float32(raw_data, 32);
	            if(node_id != 1){
	            	printf("Node %u | F3 | t=%lu ms | Position: %ld mrot\n", (unsigned)node_id, (unsigned long)timestamp, (long)(position * 1000.0f));
	            }
	            break;
	          }
	          case 117815552: { // PERIODIC_FRAME_4
	            uint32_t timestamp = (uint32_t)extract_bits(raw_data, 0, 32);
	            float velocity     = extract_float32(raw_data, 32);
	            if(node_id != 1){
	            	printf("Node %u | F4 | t=%lu ms | Velocity: %ld mRPM\n", (unsigned)node_id, (unsigned long)timestamp, (long)(velocity * 1000.0f));
	            }
	            break;
	          }

	          break;
	          }



	        }
	        else {
	          // process standard IDs (GPS, IMU)
	          switch(msg_id) {
	            // case 0x001: // GPS status - i don't think we really care about this
	            //   fix_type = extract_bits(raw_data, 0, 3);
	            //   break;
	            case 0x002: // GPS time
	              // there is also a time confirmed signal for more accurate time
	              // but time_valid is faster and should be ok
	              time_valid = extract_bits(raw_data, 0, 1);
	              printf("time: %d \n", time_valid);
	              break;
	            case 0x003: // GPS position
	              pos_valid = extract_bits(raw_data, 0, 1);
	              printf("pos: %d\n", pos_valid);
	              break;
	            case 0x006: // GPS distance
	              dist_valid = extract_bits(raw_data, 0, 1);
	              printf("dist: %d\n", dist_valid);
	              break;
	            case 0x007: // GPS speed
	              speed_valid = extract_bits(raw_data, 0, 1);
	              printf("speed: %d\n", speed_valid);
	              break;
	            case 0x009: // IMU data
	              imu_valid = extract_bits(raw_data, 0, 1);
	              //printf("imu: %d", imu_valid);
	              break;
	          }
	        }

	        // data processing based on the message ID and if the data is valid
	        if (time_valid == 1) {
	          time_t now_gmt = (uint64_t) decode_physical_value(raw_data, 8, 40, TIME_OFFSET, TIME_SCALE);
	          struct tm ts;
	          char buf[80];

	          // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
	          ts = *localtime(&now_gmt);
	          strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z\n", &ts);

	          printf("TIME: %s\n", buf);
	        }
	        else if (pos_valid == 1) {
	          double latitude = decode_physical_value(raw_data, 1, 28, LAT_OFFSET, POS_SCALE); // degrees
	          double longitude = decode_physical_value(raw_data, 29, 29, LONG_OFFSET, POS_SCALE);

	          printf("GPS position (deg, deg): %f, %f\n", latitude, longitude);
	        }
	        else if (dist_valid == 1) {
	          trip_distance = extract_bits(raw_data, 1, 22); // meters
	          total_distance = extract_bits(raw_data, 42, 22); // kilometers

	          // printf("trip distance (m): %ld\n", trip_distance);
//	           printf("total distance (km): %ld\n", total_distance);
	        }
	        else if (speed_valid == 1) {
	          double vehicle_speed = decode_physical_value(raw_data, 1, 20, 0, SPEED_SCALE); // meters/s

//	           printf("vehicle speed (m/s): %f\n", vehicle_speed);
	           char str[64];
			   int precision = 4; // This can be a dynamic variable changing at runtime

			   // The '*' matches the 'precision' argument, and 'f' specifies the double
			   snprintf(str, sizeof(str), "%.*f", precision, vehicle_speed);

			   //LCD_Clear(&lcd);
			  LCD_PrintStrAt(&lcd, 1, 6, str);
	        }
	        else if (imu_valid) {
	          double accel[3], angular[3];
	          accel[0] = decode_physical_value(raw_data, 1, 10, ACCEL_OFFSET, ACCEL_SCALE);
	          accel[1] = decode_physical_value(raw_data, 11, 10, ACCEL_OFFSET, ACCEL_SCALE);
	          accel[2] = decode_physical_value(raw_data, 21, 10, ACCEL_OFFSET, ACCEL_SCALE);
	          angular[0] = decode_physical_value(raw_data, 31, 11, ANGULAR_OFFSET, ANGULAR_SCALE);
	          angular[1] = decode_physical_value(raw_data, 42, 11, ANGULAR_OFFSET, ANGULAR_SCALE);
	          angular[2] = decode_physical_value(raw_data, 53, 11, ANGULAR_OFFSET, ANGULAR_SCALE);

	          //printf("logging IMU data to SD Card...\n");
	          //log_imu_data_to_sd(accel, angular);
	          // printf("acceleration (m/s^2): (%f, %f, %f)\n", accel_x, accel_y, accel_z);
	          // printf("angular rate (deg/s): (%f, %f, %f)\n\n", angular_x, angular_y, angular_z);
	        }

	        HAL_Delay(500);

	        // move ptr to next item in queue
	        read_index = (read_index + 1) % QUEUE_SIZE;
	      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
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
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
