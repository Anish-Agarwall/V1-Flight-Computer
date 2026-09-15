/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/**
 *  THIS IS BASED ON THE V1 REV A DESIGN.
 *  SUBJECT TO CHANGE BASED ON REVISIONS.
 *  FUTURE DEVS, PLEASE UPDATE THIS.
 *  I HAVE THIS HERE TO HELP WITH THE CODING PREOCESS
 *
 * I2C1: BMI330, BMP388, LIS3MDL
 * SPI1: E22-900MM22S (dwonlik)
 * SPI2: W25Q128JVS Flash
 * SPI3: 1040310811 SD Card
 * UART1: NEO-M9N
 *
 *  Common Pins:
 * PA0: Voltage ADC
 * PA1: Main detect
 * PA2: Drogue Detect
 * PA4: Heartbeat LED
 * PA15: PWM Buzzer
 * PB0: Main Control
 * PB1: Drogue Control
 */
















/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "circular_buffer.h"

#include "bmi088.h"
#include "bmp388.h"
#include "lis3mdl.h"
#include "NEO-M9N.h"
#include "w25q128jvs.h"
#include "1040310811.h"
#include "E22_900MM22S.h"







/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


// PINS
#define HEARTBEAT_Pin GPIO_PIN_4
#define HEARTBEAT_GPIO_Port GPIOA
#define MAIN_FIRE_Pin GPIO_PIN_0
#define MAIN_FIRE_GPIO_Port GPIOB
#define DROGUE_FIRE_Pin GPIO_PIN_1
#define DROGUE_FIRE_GPIO_Port GPIOB

// BUZZER (PA15)
#define BUZZER_PERIOD 999
#define BUZZER_DUTY 500 // half


#define AIRBRAKE_TIM_PRESCALER 71
#define AIRBRAKE_TIM_PERIOD 19999
#define AIRBRAKE_RETRACTED_US 500 // pulse width in microseconds // was 1000
#define AIRBRAKE_DEPLOYED_US 2000 // was 2706


//#define AIRBRAKE_DEPLOY_ALT_M 300.0f
#define AIRBRAKE_DEPLOY_ALT_M 1679.1f

static float curr_deployment_level = 0.0f;


// 1500 ft AGL = 457.2 m
// main fires on descent below this only after drogie
#define MAIN_DEPLOY_ALT_M 457.2f




// SD BUFFER DEFINES
#define SD_WRITE_BUFFER_SIZE 512
char sd_text_buffer[SD_WRITE_BUFFER_SIZE];
uint16_t buffer_index = 0;



Packet_t telemetry_pkt;



// LOOP MININGS in MS
#define IMU_UPDATE 10
#define BARO_UPDATE 50
#define CONTINUITY_UPDATE_MS 5000
#define HEARTBEAT_UPDATE 500
#define TELEMETRY_UPDATE 200


// how often we write the FIFO buffers to sd and clagsh
#define _FLUSH_MS 500

// ADC for battery
#define ADC_VREF_MV 3300
#define ADC_MAX_COUNTS 4095
// 20k top / 2.5k bottom
// scale = (20 + 2.5) / 2.5 = 9
#define VDIV_SCALE 9.0f
// Continuity Check
#define CONTINUITY_OPEN_MV 400



// sd stuff (double check this)
#define SD_LOG_START_SECTOR 2048u
#define CSV_LINE_BUF 320


// AI, DOUBLE CHECK THIS TO MAKE SURE IT MAKES SENSE
// Flash logging — raw binary record (20 bytes each)
// layout: uint32 time_ms | float alt | float pressure | float temp | float bat_v | uint8 stage | 3-byte pad
#define FLASH_RECORD_SIZE   32u
#define FLASH_MAX_ADDR      0xFFFFFF    // 16 MB (W25Q128)



// default flight config

// launch detection
// 100 ft
#define DEF_LAUNCH_ALT_M 30.48f
// min 2g
#define DEF_LAUNCH_G_M 2.0f

// deployment altitude (lowkey dont use lol, just here to essentially hard start it)
// both are disabled for now, but just here for testing down the road
#define DEF_MAIN_ALT_M 0.0f
#define DEF_DROGUE_ALT_M 0.0f

// pyro channel
// currently set to 5 seconds, change later on.
#define DEF_MAIN_FIRE_MS 5000
#define DEF_DROGUE_FIRE_MS 2000

// hard set timers (use for testing)
#define DEF_MAIN_TIMER_MS 0
#define DEF_DROGUE_TIMER_MS 0

// apogee settins
// do not fire main a apogee, fire only drogue
#define DEF_MAIN_ON_APOGEE 0
#define DEF_DROGUE_ON_APOGEE 1

// if altitude has dropped this many meters below the max, here, we reachd apogee
#define APOGEE_DROP_M           10.0f

// used in loop to see how many samples to confirm dropping
#define APOGEE_CONFIRM_SAMPLES  5


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */



FATFS fs;
FIL logFile;
uint8_t sd_mounted = 0;




// Sensors
BMI088_Data_t imu;
BMP388_Calib_t baro;
LIS3MDL_Data_t mag;

// live sensor data values
// g
float ax = 0;
float ay = 0;
float az = 0;
// deg/s
float gx = 0;
float gy = 0;
float gz = 0;
// pa
float pressure = 0;
// celcisu
float temperature = 0;
// m
float altitude = 0;

// ADC detect
// mv
// 0 = connected & 500 = open
uint16_t main_detect = 0;
uint16_t drogue_detect = 0;
// 12V rail check
float bat_voltage = 0;


// ****************************************************** PUT GPS STUFF HER

extern NEO_M9N_Data_t gps_data;





// hard set info
// all definitions above, not typoing this again lol
float cfg_launch_alt_m = DEF_LAUNCH_ALT_M;
float cfg_launch_g_m = DEF_LAUNCH_G_M;

float cfg_main_alt_m = DEF_MAIN_ALT_M;
float cfg_drogue_alt_m = DEF_DROGUE_ALT_M;

int32_t cfg_main_fire_ms = DEF_MAIN_FIRE_MS;
int32_t cfg_drogue_fire_ms = DEF_DROGUE_FIRE_MS;

int32_t cfg_main_timer_ms = DEF_MAIN_TIMER_MS;
int32_t cfg_drogue_timer_ms= DEF_DROGUE_TIMER_MS;

uint8_t cfg_main_on_apogee = DEF_MAIN_ON_APOGEE;
uint8_t cfg_drogue_on_apogee = DEF_DROGUE_ON_APOGEE;

// flight state
FlightStage_t stage = NOT_LAUNCHED;
uint32_t launch_time_ms = 0;
uint32_t main_fired_ms = 0;
uint32_t drogue_fired_ms = 0;
float ground_alt_abs = 0.0;	// absolute baro altitude at pad
float peak_alt_agl = 0.0f; // highest AGL seen so far


// airbrake state flag
static uint8_t airbrake_deployed = 0;
static uint8_t activate_airbrakes = 0;



// circ bif
// going to be used for state detection
CircBuf_t  recent_alt;


// fifo buffers
// log time
LogU32Buf_t log_time;

LogU32Buf_t log_servo;
LogFloatBuf_t log_airbrakes_level;
LogFloatBuf_t log_filtered_alt;
LogFloatBuf_t log_filtered_vel;

// pressure, temp, altitude (bmp)
LogFloatBuf_t log_pres;
LogFloatBuf_t log_temp;
LogFloatBuf_t log_alt;

// g's on 3 axis (bmi)
LogFloatBuf_t log_ax;
LogFloatBuf_t log_ay;
LogFloatBuf_t log_az;

// deg/s on 3 axis (bmi)
LogFloatBuf_t log_gx;
LogFloatBuf_t log_gy;
LogFloatBuf_t log_gz;

// voltage detection
LogFloatBuf_t log_main_mv;
LogFloatBuf_t log_drogue_mv;
LogFloatBuf_t log_bat_v;

// mag
LogFloatBuf_t mag_x_buf;
LogFloatBuf_t mag_y_buf;
LogFloatBuf_t mag_z_buf;
LogFloatBuf_t mag_temp_buf;

// gps
//LogU32Buf_t gps_year;
//LogU32Buf_t gps_month;
//LogU32Buf_t gps_day;
//LogU32Buf_t gps_hour;
//LogU32Buf_t gps_min;
//LogU32Buf_t gps_sec;
LogFloatBuf_t gps_longitude;
LogFloatBuf_t gps_latitude;
LogFloatBuf_t gps_speed;
LogFloatBuf_t gps_heading_deg;

// imu
LogFloatBuf_t imu_acc_x;
LogFloatBuf_t imu_acc_y;
LogFloatBuf_t imu_acc_z;

LogFloatBuf_t imu_gyro_x;
LogFloatBuf_t imu_gyro_y;
LogFloatBuf_t imu_gyro_z;




// stage
LogStageBuf_t log_stage;

// timestamp
LogU32Buf_t log_unix;

// sd stuff (double check this)
static uint32_t sd_sector = SD_LOG_START_SECTOR;
static uint8_t  sd_available = 0;
static uint8_t  sd_buf[512];
static uint16_t sd_buf_pos = 0;

// flash pointer (double check this)
static uint32_t flash_addr = 0x000000;
static uint8_t  flash_available = 0;


// timing travkers for the loop
static uint32_t t_imu = 0;
static uint32_t t_baro = 0;
static uint32_t t_cont = 0;
static uint32_t t_hb = 0;
static uint32_t t_flush = 0;
static uint32_t t_telem = 0;


static int pre_log_ctr = 0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);

static void MX_SPI3_Init_Fast(void);

static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

// bring up code
void Bringup_I2C(void);
void Bringup_SPI_Flash(void);
void Bringup_SPI_SD(void);
void Bringup_SPI_DL(void);
void Bringup_UART_GPS(void);


// init
static void Init_Sensors(void);
static void Init_SD(void);

static void Init_Buffers(void);

static void Init_Flash(void);

// config stuff
static void Load_Config(void);
static void Parse_Config_Line(const char *line);



// adc for main and drogue node reads
static uint16_t ADC_Read_mV(uint32_t channel);

// buzzer
static void Buzzer_On(void);
static void Buzzer_Off(void);
static void Buzzer_Beep(uint32_t ms);


// airbreka helpers
static void Airbrake_Retract(void);
static void Airbrake_Deploy(void);

// Pyro helpers
static void Fire_Drogue(void);
static void Fire_Main(void);
static void Pyro_Safe(void);




// loop task funcs
static void Task_IMU(void);
static void Task_Baro(void);
//static void Task_Mag(void);
static void Task_Continuity(void);
static void Task_Heartbeat(void);
static void Task_GPS(void);
static void Task_Telemetry(void);
static void Task_FlightState(void);
static void Task_Pyro(void);
static void Task_PyroChecks(void);
static void Task_Logging(void);
static void Task_LogFlush(void);
static void Task_SD_Write(void);
static void Task_Airbrakes(void);


// sd/flash helpers
static void SD_Flush(void);
static void Flash_Write_Record_Raw(uint32_t ts, float alt, float prs, float tmp, float bv, uint8_t st);


// telem
static void Telem_Send(void);



// math funcs (mayber)
static float Altitude_From_Pressure(float pa);



// misc (idk)


// altitude and velocity filtering
uint8_t filter_init = 0;
float filtered_alt = 0;
float filtered_vel = 0;
const float alt_alpha = 0.5;
const float vel_alpha = 0.85;

uint32_t now;









BYTE work[4096];


// drogue and main fired flags
int drogue_fired_flag = 0;
int main_fired_flag = 0;




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

	char log_buffer[100];
	UINT byteswritten;




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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_FATFS_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);


//  FRESULT res;
//  res = f_mount(&fs, "", 1);
//
//  // Initialize FatFS and Mount SD Card
//  if (f_mount(&fs, "", 1) == FR_OK) {
//      // Try to open/create the file.
//      // If FA_OPEN_APPEND gives an error, use (FA_OPEN_ALWAYS | FA_WRITE)
//      if (f_open(&logFile, "LOG.CSV", FA_OPEN_APPEND | FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
//          sd_mounted = 1;
//          // Write CSV Header
//          f_printf(&logFile, "Time_ms,Alt_m,Pres_Pa,Temp_C,Bat_V,Stage\n");
//          f_sync(&logFile); // Force write to card
//      }
//  }

  HAL_Delay(1000);  // let card power rail settle

  __HAL_TIM_SET_AUTORELOAD(&htim3, AIRBRAKE_TIM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, AIRBRAKE_RETRACTED_US);
  htim3.Instance->EGR = TIM_EGR_UG;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);


  extern FATFS USERFatFS;
  extern char USERPath[4];
  FRESULT res;

  res = f_mount(&USERFatFS, USERPath, 1);
  MX_SPI3_Init_Fast();


  if (res == FR_NO_FILESYSTEM) {
      // Card has no FAT — format it (only happens once on a blank card)
      //BYTE work[4096];
	  //     JUST CHANGED THE FM_FAT32 TO THE ORRED VERSION
      res = f_mkfs(USERPath, FM_FAT32 | FM_SFD, 0, work, sizeof(work));
      if (res == FR_OK) {
          f_mount(NULL, USERPath, 0);
          res = f_mount(&USERFatFS, USERPath, 1);
      }
  }

  if (res == FR_OK) {
	  //FRESULT temporary = f_open(&logFile, "0:LOG.CSV",
      //        FA_OPEN_APPEND | FA_WRITE | FA_CREATE_ALWAYS);

	  char filename[16];
	  int file_idx = 0;
	  FRESULT res_stat;
	  FILINFO fno;

	  do {
		  sprintf(filename, "0:LOG_%d.CSV", file_idx++);
		  res_stat = f_stat(filename, &fno);
	  } while (res_stat == FR_OK && file_idx < 1000);


      FRESULT temporary = f_open(&logFile, filename, FA_CREATE_ALWAYS | FA_WRITE);



      if (temporary == FR_OK) {
          //f_lseek(&logFile, f_size(&logFile));
          sd_available = 1;
          sd_mounted   = 1;
          const char *header = "Time_ms,Alt_m,Pres_Pa,Temp_C,Bat_V,Main_mV,Drogue_mV,Stage,Servo,Airbrakes_Level,Filtered_Alt,Filtered_Vel,Mag_x,Mag_y,Mag_z,Mag_temp,"
        		  "GPS_longitude,GPS_latitude,GPS_speed,GPS_heading_deg\r\n";
          UINT bytes_written = 0;
          //temporary = f_printf(&logFile,
          //    "Time_ms,Alt_m,Pres_Pa,Temp_C,Bat_V,Main_mV,Drogue_mV,Stage\r\n");
          //temporary = f_sync(&logFile);

          temporary = f_write(&logFile, header, strlen(header), &bytes_written);

          // temporary = f_close(&logFile);

          f_sync(&logFile);


      }
  }



//	if (res == FR_OK) {
//		if (f_open(&logFile, "LOG.CSV", FA_OPEN_ALWAYS | FA_WRITE) == FR_OK) {
//			f_lseek(&logFile, f_size(&logFile));  // append to end
//			sd_mounted = 1;
//			sd_available = 1;
//			f_printf(&logFile, "Time_ms,Alt_m,Pres_Pa,Temp_C,Bat_V,Stage\n");
//			f_sync(&logFile);
//		}
//	}




  // pyro and airbrakes off
  Pyro_Safe();
  Airbrake_Retract();


  // init bufgf
  Init_Buffers();

  // init sensors
  Init_Sensors();

  // init sd and flahs
  //Init_SD();
  Init_Flash();




  // bmp needs to wait until it chills out ngl
  HAL_Delay(500);
  BMP388_ReadData(&hi2c1, &baro);
  ground_alt_abs = baro.altitude;

  // ready beeps, 3 short beeps for all good
  for (int i = 0; i < 3; i++) {
	  Buzzer_Beep(80);
	  HAL_Delay(80);
  }

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);


  static uint16_t sync_counter = 0; // Initialize counter outside the loop if not using 'static'

// airbrake testing code
//  while (1) {
//	  Airbrake_Deploy();
//	  HAL_Delay(5000);
//	  Airbrake_Retract();
//	  HAL_Delay(5000);
//  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


//
//	sprintf(log_buffer, "%lu, %.2f, %.2f\r\n", HAL_GetTick(), altitude, pressure);
//
//	if (f_write(&logFile, log_buffer, strlen(log_buffer), &byteswritten) == FR_OK)
//	{
//		f_sync(&logFile);
//		sync_counter++;
//	}
//
//	HAL_Delay(10);

	// BOARD BRING UP CODE
//	Bringup_I2C();
	//	Bringup_SPI_Flash();
	//Bringup_SPI_SD();
//		Bringup_SPI_DL();
//	Bringup_UART_GPS();
	//	HAL_Delay(500);
	//
	//
	//	Init_Sensors();
	//	HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
	//
	//	Task_Baro();



	now = HAL_GetTick();

	// servo PWM test
//	while (1) {
//		Airbrake_Deploy();
//		HAL_Delay(1000);
//		Airbrake_Retract();
//		HAL_Delay(1000);
//	}

	Task_GPS(); // It needs to be called frequently so the rx buffer doesn't get full, but maybe move this



	if ((drogue_fired_flag) && ((now - drogue_fired_ms) > 1000)) {
		HAL_GPIO_WritePin(DROGUE_FIRE_GPIO_Port, DROGUE_FIRE_Pin, GPIO_PIN_RESET);
		drogue_fired_flag = 0;
	}

	if ((main_fired_flag) && ((now - main_fired_ms) > 1000)) {
		HAL_GPIO_WritePin(MAIN_FIRE_GPIO_Port, MAIN_FIRE_Pin, GPIO_PIN_RESET);
		main_fired_flag = 0;
	}


	// flight stuff
	if ((now - t_baro) >= BARO_UPDATE) {
		t_baro = now;
		Task_Baro();
		Task_IMU();
		Task_FlightState();
		Task_Pyro();
		Task_Logging();				// MOVE DATA RETRIEVAL AND LOGGING TO AN INTERRUPT, AND DO SD WRITES IN MAIN WHILE LOOP


	}

	// FIXME: I removed this because we aren't connecting charges
//	// contuintuy check
//	if ((now - t_cont) >= CONTINUITY_UPDATE_MS) {
//		t_cont = now;
//		Task_Continuity();
//	}

	// heartbeat
	if ((now - t_hb) >= HEARTBEAT_UPDATE) {
		t_hb = now;
		Task_Heartbeat();
	}

	// FIXME
	// using t_telem, TELEMETRY_UPDATE, and Task_Telemetry() that Anish defined
//	if ((now - t_telem) >= TELEMETRY_UPDATE) {
//		t_telem = now;
//		Task_Telemetry();
//	}

	Task_Telemetry();


	Task_SD_Write();


	if (stage == LANDED && sd_mounted) {
		//Task_LogFlush();
		Task_SD_Write();

		f_close(&logFile);

		sd_mounted = 0;
	}

	if ((stage == LAUNCHED) && ((now - launch_time_ms) >= 7000)) {
		activate_airbrakes = 1;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}


/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init_Fast(void)
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
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  //htim2.Init.Prescaler = 0;

  htim2.Init.Prescaler = 71;

  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim2.Init.Period = 4294967295;

  htim2.Init.Period = BUZZER_PERIOD;

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
  sConfigOC.Pulse = 0;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  //htim3.Init.Prescaler = 0;

  htim3.Init.Prescaler = AIRBRAKE_TIM_PRESCALER;

  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim3.Init.Period = 65535;

  htim3.Init.Period = AIRBRAKE_TIM_PERIOD;

  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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

  /*Configure GPIO pin Output Level */


  //HAL_GPIO_WritePin(GPIOC, TELEM_CS_Pin|TXEN_Pin|RXEN_Pin|TELEM_CLK_Pin |SD_CS_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, TELEM_CS_Pin|TXEN_Pin|RXEN_Pin|TELEM_CLK_Pin|SD_CS_Pin, GPIO_PIN_RESET);

  	// added by user
  	HAL_GPIO_WritePin(GPIOC, TELEM_CS_Pin, GPIO_PIN_SET);   // radio CS idle high
  	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET); // cd CS idle high


  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(Heartbeat_LED_GPIO_Port, Heartbeat_LED_Pin, GPIO_PIN_RESET);

  	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET); // flash CS idle high


  		// added by user
  		// ensure pyro is low at boot
  		HAL_GPIO_WritePin(MAIN_FIRE_GPIO_Port,   MAIN_FIRE_Pin,   GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(DROGUE_FIRE_GPIO_Port, DROGUE_FIRE_Pin, GPIO_PIN_RESET);

  		// added by user
  		// Heartbeat LED off
  		HAL_GPIO_WritePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TELEM_CS_Pin TXEN_Pin RXEN_Pin TELEM_CLK_Pin
                           SD_CS_Pin */

  //GPIO_InitStruct.Pin = TELEM_CS_Pin|TXEN_Pin|RXEN_Pin|TELEM_CLK_Pin |SD_CS_Pin;

  GPIO_InitStruct.Pin = TELEM_CS_Pin|TXEN_Pin|RXEN_Pin|TELEM_CLK_Pin|SD_CS_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TELEM_BUSY_Pin */
  GPIO_InitStruct.Pin = TELEM_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TELEM_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Heartbeat_LED_Pin */
  GPIO_InitStruct.Pin = Heartbeat_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Heartbeat_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  // Hearbeat LED (PA4)
  GPIO_InitStruct.Pin = HEARTBEAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEARTBEAT_GPIO_Port, &GPIO_InitStruct);

  // Main fire (PB0)
  GPIO_InitStruct.Pin = MAIN_FIRE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAIN_FIRE_GPIO_Port, &GPIO_InitStruct);

  // Drogue fire (PB1)
  GPIO_InitStruct.Pin = DROGUE_FIRE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DROGUE_FIRE_GPIO_Port, &GPIO_InitStruct);


  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



// BOARD BRING UP CODE
void Bringup_I2C(){

	int BMI088Aaddress = 0x18; // 0x18
	int BMI088Gaddress = 0x68; // 0x68

//	int BMI330address = 0x68;
	int LS3MDLaddress = 0x1C;
	int BMP388address = 0x76;

	int BMI088DummyReg = 0x00;
//	int BMI330DummyReg = 0x00;
	int LS3MDLDummyReg = 0x0F;
	int BMP388DummyReg = 0x00;

	uint8_t BMI088regDataA; 				// expect 0x1E
	HAL_StatusTypeDef BMI088statusA; 		//expect HAL_OK
	uint8_t BMI088regDataG; 				// expect 0x0F
	HAL_StatusTypeDef BMI088statusG; 		//expect HAL_OK


//	uint8_t BMI330regData;					// expect 0x23
//	HAL_StatusTypeDef BMI330status;			// expect HAL_OK

	uint8_t BMP388regData; 					// expect 0x50
	HAL_StatusTypeDef BMP388status; 		//expect HAL_OK

	uint8_t LS3MDLregData; 					// expect b00111101
	HAL_StatusTypeDef LS3MDLstatus; 		//expect HAL_OK


//	 BMI088 (ACC)
	BMI088statusA = HAL_I2C_Mem_Read(&hi2c1, (BMI088Aaddress << 1), BMI088DummyReg, I2C_MEMADD_SIZE_8BIT, &BMI088regDataA, 1, 100);
//	 BMI088 (GYRO)
	BMI088statusG = HAL_I2C_Mem_Read(&hi2c1, (BMI088Gaddress << 1), BMI088DummyReg, I2C_MEMADD_SIZE_8BIT, &BMI088regDataG, 1, 100);

	HAL_Delay(10);

//	HAL_StatusTypeDef tempStatus = HAL_I2C_IsDeviceReady(&hi2c1, 0xD0, 3, 100); // extra BMI330 test

	//BMI330
//	BMI330status = HAL_I2C_Mem_Read(&hi2c1, (BMI330address << 1), BMI330DummyReg, I2C_MEMADD_SIZE_8BIT, &BMI330regData, 1, 100);
	// LS3MDL
	LS3MDLstatus = HAL_I2C_Mem_Read(&hi2c1, (LS3MDLaddress << 1), LS3MDLDummyReg, I2C_MEMADD_SIZE_8BIT, &LS3MDLregData, 1, 100);
	// BMP388
	BMP388status = HAL_I2C_Mem_Read(&hi2c1, (BMP388address << 1), BMP388DummyReg, I2C_MEMADD_SIZE_8BIT, &BMP388regData, 1, 100);




	// I2C bus scan
	int tempArr[32];
	for (int i = 0; i < 32; i++) {
		tempArr[i] = 0;
	}
	int arrCount = 0;
	for(uint8_t i = 1; i < 128; i++) {
	    // Shift the 7-bit address left by 1 for the HAL
	    HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 2, 10);
	    if(result == HAL_OK) {
	    	tempArr[arrCount] = i;
	    	arrCount++;
	    }
	}

	int nothing = 0;


	// 500ms Delay
	//HAL_Delay(500);
}


void Bringup_SPI_Flash(){
	// W25Q64JV

	uint8_t flash_tx[4] = {0x9F, 0x00, 0x00, 0x00};
	uint8_t flash_rx[4] = {0};


	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET); 	// pull low
	HAL_SPI_TransmitReceive(&hspi2, flash_tx, flash_rx, 4, 100);			// data transfer haha
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);		// pull high

	// expect: [1] = 0xEF, [2] = 0x40, [3] 0x17
	// NOTE: I could be wrong about [2] and [3], lowkey guessed because data sheet confuded me :p
	// Also double checck which mode we need to be in when we actually test, might be fine (just leaving for future me)
}

void Bringup_SPI_SD(){
	// 104031
	// Need to 'wake up' the SD Card
	uint8_t dummy = 0xFF;
	uint8_t rx_dump;


	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET); // ENSURE THE CS LINE IS HIGH OR ELSE BAD THINGS HAPPEN
	for (int i = 0; i < 80; i++){
		HAL_SPI_TransmitReceive(&hspi3, &dummy, &rx_dump, 1, 10);
	}
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

	uint8_t sd_cmd0[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
	uint8_t sd_resp = 0xFF;

	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi3, sd_cmd0, &rx_dump, 6, 100);

	// keep readnig unril we get a response
	for (int i = 0; i < 10; i++){
		HAL_SPI_Receive(&hspi3, &sd_resp, 1, 100);
		if (sd_resp != 0xFF) break;
	}

	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

	// expect to get sd_resp == 0x01
}

void Bringup_SPI_DL(){
	// E22-900MM22S
	uint8_t dl_tx = 0xC0;
	uint8_t dl_rx = 0x00;

	// wait for radio to not be busy
	while (HAL_GPIO_ReadPin(TELEM_BUSY_GPIO_Port, TELEM_BUSY_Pin) == GPIO_PIN_SET){}

	HAL_GPIO_WritePin(GPIOC, TELEM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, &dl_tx, &dl_rx, 1, 100);

	HAL_SPI_Receive(&hspi1, &dl_rx, 1, 100);
	HAL_GPIO_WritePin(GPIOC, TELEM_CS_Pin, GPIO_PIN_SET);

	// expect dl_rx to NOT be 0x00/0xFF
	// idk what to get, but it shouldnt bt that lol. look more into this future me :p
}

void Bringup_UART_GPS(){
	// NEO-M9N-00B

	uint8_t gps_rx[128];
	HAL_StatusTypeDef GPS_Status = HAL_UART_Receive(&huart6, gps_rx, 128, 1000);
	int nothing = 0;
}



// ********************************** INIT HELPERS ************************************************************

static void Init_Buffers(void) {
    CB_Init(&recent_alt);

    LU32_Init(&log_time);

    LFB_Init(&log_pres);
    LFB_Init(&log_temp);
    LFB_Init(&log_alt);

    LFB_Init(&log_ax);
    LFB_Init(&log_ay);
    LFB_Init(&log_az);
    LFB_Init(&log_gx);
    LFB_Init(&log_gy);
    LFB_Init(&log_gz);

    LFB_Init(&log_main_mv);
    LFB_Init(&log_drogue_mv);
    LFB_Init(&log_bat_v);
    LSB_Init(&log_stage);

    LU32_Init(&log_servo);
    LFB_Init(&log_airbrakes_level);
    LFB_Init(&log_filtered_alt);
    LFB_Init(&log_filtered_vel);

    // mag inits
    LFB_Init(&mag_x_buf);
    LFB_Init(&mag_y_buf);
    LFB_Init(&mag_z_buf);
    LFB_Init(&mag_temp_buf);

    // GPS inits
//    LU32_Init(&gps_year);
//    LU32_Init(&gps_month);
//    LU32_Init(&gps_day);
//    LU32_Init(&gps_hour);
//    LU32_Init(&gps_min);
//    LU32_Init(&gps_sec);

    LFB_Init(&gps_longitude);
    LFB_Init(&gps_latitude);

    LFB_Init(&gps_speed);
    LFB_Init(&gps_heading_deg);
    LU32_Init(&log_unix);
}












static void Init_Sensors(void) {

	// for now we have beeps, we can also add heartbeat LED ontop for better debugging
	// just put seomthing down fast for now becasue time crunch
	// also we can just use the debugger :p


    // IMU
//    if (BMI330_Init(&hi2c1) != HAL_OK) {
//        // 3 beeps fast
//        for (int i = 0; i < 3; i++) {
//        	Buzzer_Beep(50);
//        	HAL_Delay(80);
//        }
//    }

    // )PRES
	// this get destroyed on cold starts
	/*
    if (BMPP388_Init(&hi2c1, &baro) != HAL_OK) {
    	// 2 beeps slow
        for (int i = 0; i < 2; i++) {
        	Buzzer_Beep(50);
        	HAL_Delay(200);
        }
    }
    */

	// better for cold starts, still testing it.
	// Presure
	uint8_t bmp_ok = 0;
	for (int attempt = 0; attempt < 5; attempt++) {
		if (BMPP388_Init(&hi2c1, &baro) == HAL_OK) {
			bmp_ok = 1;
			break;
		}
		HAL_Delay(50);
	}
	if (!bmp_ok) {
		// 2 slow beeps, BMP failed after all retries
		for (int i = 0; i < 2; i++) {
			Buzzer_Beep(50);
			HAL_Delay(200);
		}
	}

     //MAG
    if (LIS3MDL_Init(&hi2c1) != HAL_OK) {
    	// 1 beep slower ;))
        Buzzer_Beep(50);
        HAL_Delay(400);
        Buzzer_Beep(50);
    }


    // FLASH
    if (W25Q128JVS_Init(&hspi2) != HAL_OK) {
//    	Buzzer_Beep(200);
    	HAL_Delay(200);
//    	Buzzer_Beep(200);
    	flash_available = 0;
    } else{
        flash_available = 1;
    }

    // GPS
    if (NEO_Init(&huart6) != HAL_OK){
    	// 1 beep slower ;))
    	Buzzer_Beep(50);
    	HAL_Delay(400);
    	Buzzer_Beep(50);
    }

    // DownLink
    if (E22_900MM22S_Init_900MHz(&hspi1) != HAL_OK) {
    	// 1 beep slower ;))
		Buzzer_Beep(50);
		HAL_Delay(400);
		Buzzer_Beep(50);
    }

    if (BMI088_Init(&hi2c1) != HAL_OK) {
    	// 1 beep slower ;))
		Buzzer_Beep(50);
		HAL_Delay(400);
		Buzzer_Beep(50);
    }

//    if (BMI088_Verify_Config(&hi2c1) != HAL_OK) {
//    	Buzzer_Beep(50);
//    }
}

static void Init_SD(void) {
	// wake up the SD card with 80 dummy clocks
	uint8_t dummy  = 0xFF;
	uint8_t rx_dump;

	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
	for (int i = 0; i < 80; i++) {
		HAL_SPI_TransmitReceive(&hspi3, &dummy, &rx_dump, 1, 10);
	}

	if (SD_Card_Init(&hspi3) == HAL_OK) {
		sd_available = 1;
		sd_sector = SD_LOG_START_SECTOR;
		sd_buf_pos = 0;

		// writses CSV header into the first line of the buffer
		sd_buf_pos = (uint16_t)snprintf((char *)sd_buf, sizeof(sd_buf), "time_ms,altitude_m,pressure_pa,temp_c,bat_v,main_mv,drogue_mv,stage\r\n");

		// pad to 512 bytes and flush
		memset(sd_buf + sd_buf_pos, 0xFF, 512 - sd_buf_pos);
		SD_Write_Data(&hspi3, sd_sector++, sd_buf, 1);
		sd_buf_pos = 0;
		memset(sd_buf, 0xFF, sizeof(sd_buf));
	} else {
		sd_available = 0;
		// lonh ahh beep to help me
		Buzzer_Beep(500);
	}

	if (f_mount(&fs, "", 1) == FR_OK) {
		// Open or Create flight_log.csv in Append mode
		if (f_open(&logFile, "LOG.CSV", FA_OPEN_APPEND | FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
			sd_mounted = 1;
			// Write CSV Header
			f_printf(&logFile, "Time_ms,Alt_m,Pres_Pa,Temp_C,Bat_V,Stage\n");
			f_sync(&logFile); // Ensure header is saved
		}
	}

}

static void Init_Flash(void) {
    if (flash_available) {
        // erase the whole chip at startup so we have a clean log
    	// takes too long on cold starts, just dont for now, simply reset the address.
        //Chip_Erase(&hspi2);
        flash_addr = 0x000000;
    }
}



// prob gonna need this with load config itself
static void Parse_Config_Line(const char *line) {
	// TODO: THIS TOO
}


static void Load_Config(void) {
	// TODO: THIS BRAHHHH
}



//***************************************************************** PYRO HELPERS *********************************************

// ensrues it is low when starting up, alkready did this somewhere else, but might as well be double safe.
static void Pyro_Safe(void) {
    HAL_GPIO_WritePin(MAIN_FIRE_GPIO_Port, MAIN_FIRE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DROGUE_FIRE_GPIO_Port, DROGUE_FIRE_Pin, GPIO_PIN_RESET);
}

// sets drogue off
static void Fire_Drogue(void) {
    HAL_GPIO_WritePin(DROGUE_FIRE_GPIO_Port, DROGUE_FIRE_Pin, GPIO_PIN_SET);
    drogue_fired_ms = HAL_GetTick();
    drogue_fired_flag = 1;
}

//sets main off
static void Fire_Main(void) {
    HAL_GPIO_WritePin(MAIN_FIRE_GPIO_Port, MAIN_FIRE_Pin, GPIO_PIN_SET);
    main_fired_ms = HAL_GetTick();
    main_fired_flag = 1;
}


// ********************************************************************* AIRBRAKES ***********************************************

static void Airbrake_Retract(void) {
//    __HAL_TIM_SET_AUTORELOAD(&htim3, AIRBRAKE_TIM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, AIRBRAKE_RETRACTED_US);

    // THE FIX: Force an Update Generation to latch the shadow registers immediately
//    htim3.Instance->EGR = TIM_EGR_UG;
//    HAL_StatusTypeDef temp = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    airbrake_deployed = 0;
}

static void Airbrake_Deploy(void) {
//    __HAL_TIM_SET_AUTORELOAD(&htim3, AIRBRAKE_TIM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, AIRBRAKE_DEPLOYED_US);

    // THE FIX: Force an Update Generation to latch the shadow registers immediately
//	htim3.Instance->EGR = TIM_EGR_UG;

//    HAL_StatusTypeDef temp = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    airbrake_deployed = 1;
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	int reandom = 0;
}





// ************************************************************** ADC/BUZZERS*****************************************

// Single-channel software-triggered ADC read; returns millivolts
static uint16_t ADC_Read_mV(uint32_t channel) {
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel = channel;
    cfg.Rank = 1;
    cfg.SamplingTime = ADC_SAMPLETIME_56CYCLES;  // i want it to eb long asf ngl, might be overkill, look more into this
    HAL_ADC_ConfigChannel(&hadc1, &cfg);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return (uint16_t)((raw * ADC_VREF_MV) / ADC_MAX_COUNTS);
}

static void Buzzer_On(void) {
    __HAL_TIM_SET_AUTORELOAD(&htim2, BUZZER_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, BUZZER_DUTY);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

static void Buzzer_Off(void) {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

static void Buzzer_Beep(uint32_t ms) {
    Buzzer_On();
    HAL_Delay(ms);
    Buzzer_Off();
}




// ******************************************************************* LOOP TASK FUNCTIONS ******************************************



// get bmi088 data
static void Task_IMU(void){
	BMI088_ReadData(&hi2c1, &imu);

	ax = imu.acc_x;
	ay = imu.acc_y;
	az = imu.acc_z;

	gx = imu.gyro_x;
	gy = imu.gyro_y;
	gz = imu.gyro_z;

	// chat told me to also put mag here and not use an indpendent task func (double check this)
	LIS3MDL_ReadData(&hi2c1, &mag);

}


static void Alt_Filtering(float new_alt) {
	if (!filter_init) {
		filtered_alt = new_alt;
		filter_init = 1;
		return;
	}

	float temp_alt = filtered_alt * alt_alpha + (new_alt) * (1 - alt_alpha);
	float temp_vel = (temp_alt - filtered_alt) * 20;
	filtered_vel = filtered_vel * vel_alpha + (temp_vel) * (1 - vel_alpha);
	filtered_alt = temp_alt;
	return;
}

// get bmp388 data
static void Task_Baro(void){
	BMP388_ReadData(&hi2c1, &baro);

	// pressur and temp
	pressure = baro.pressure;
	temperature = baro.temp;

	// get altitude in respect to the ground
	// planning on getting a ground altitue from the main loop, basciallt update this value when we actually detect a launch
	float curr_alt = baro.altitude -ground_alt_abs;

	// yoinked spike filter from prev flight computer
	// change if needed
	if (stage == NOT_LAUNCHED && CB_Length(&recent_alt) > 10){
		float prev = CB_Get(&recent_alt, 0);

		// clamp spikes
		if (curr_alt < prev - 0.25f){
			curr_alt = prev - 0.25f;
		}

		if (curr_alt > prev + 80.0f){
			curr_alt = prev + 10.0f;
		}
	}

	// store the final processed altitude
	altitude = curr_alt;
	CB_Push(&recent_alt, altitude);

	// need to update peaks altitude
	if (stage == LAUNCHED && altitude > peak_alt_agl) {
		peak_alt_agl = altitude;
	}

	// filtering for PD loop
	Alt_Filtering(altitude);

}

// told we wont need this (double check tho later on)
//static void Task_Mag(void){
//
//}

// checks the continuity of both main and drogie and battery voltage
static void Task_Continuity(void){

	// ok so,
	// double check these ADC channels when actualy testing.

	// PA0 (bat voltage)
	uint16_t raw_mv = ADC_Read_mV(ADC_CHANNEL_0);
	bat_voltage = ((float)raw_mv * VDIV_SCALE) / 1000.0f;

	// PA1 (main detect)
	main_detect = ADC_Read_mV(ADC_CHANNEL_1);

	// PA2 (drogue detect)
	drogue_detect = ADC_Read_mV(ADC_CHANNEL_2);

	// when wating for laucnh, beep it
	if (stage == NOT_LAUNCHED){
		uint8_t main_open = (main_detect > CONTINUITY_OPEN_MV);
		uint8_t drogue_open = (drogue_detect > CONTINUITY_OPEN_MV);

		if (!main_open && !drogue_open) {
			// both good, med beep
			Buzzer_Beep(250);
		} else {
			if (main_open) {
				// main open, 2 short beep
				Buzzer_Beep(25);
				HAL_Delay(100);
				Buzzer_Beep(25);
				HAL_Delay(100);
			}

			HAL_Delay(1000);

			if (drogue_open) {
				// drogue open, 3 short beeps
				Buzzer_Beep(25);
				HAL_Delay(100);
				Buzzer_Beep(25);
				HAL_Delay(100);
				Buzzer_Beep(25);
			}
		}
	}

}

// simply toggler led pin
static void Task_Heartbeat(void){
    HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
}

static void Task_GPS(void){
	GPS_Process_Stream();
}

static uint8_t telem_tx_pending = 0;

static void Task_Telemetry(void){

	// NEW START
	uint32_t telemNow = HAL_GetTick();

	// if a TX is in flight, try to finish it first
	if (telem_tx_pending) {
		if (E22_900MM22S_Transmit_Finish(&hspi1) == HAL_OK) {
			telem_tx_pending = 0;
		}
		return; // return either way so we don't start a new TX this time
	}

	// only start a new TX on the schedule
	if ((telemNow - t_telem) < TELEMETRY_UPDATE) return;
	t_telem = telemNow;
//	NEW END
//
//	// populate the packet
	telemetry_pkt.gps_longitude = gps_data.longitude;
	telemetry_pkt.gps_latitude = gps_data.latitude;
	telemetry_pkt.gps_altitude = gps_data.altitude; // this is currently calculated from the pressure sensor
	telemetry_pkt.gps_heading = gps_data.heading_deg;
	telemetry_pkt.gps_unix_timestamp = gps_data.unix_timestamp;
	telemetry_pkt.gps_SIV = gps_data.siv;
	telemetry_pkt.imu_ax = ax;
	telemetry_pkt.imu_ay = ay;
	telemetry_pkt.imu_az = az;
	telemetry_pkt.imu_pitch = gx;
	telemetry_pkt.imu_yaw = gy;
	telemetry_pkt.imu_roll = gz;
	telemetry_pkt.barometric_pressure = (int)pressure;
	telemetry_pkt.state = (short)stage;

//	// convert to string
	char tx_buffer[FULL_PACKET_SIZE_BYTES];
	packet_to_string(&telemetry_pkt, tx_buffer);

//		NEW START
	if (E22_900MM22S_Transmit_Start(&hspi1, (uint8_t *)tx_buffer, FULL_PACKET_SIZE_BYTES) == HAL_OK) {
		telem_tx_pending = 1;
	}
	// NEW END



//	// send via LoRa module
//	HAL_StatusTypeDef LoRaStatus = E22_900MM22S_Transmit(&hspi1, (uint8_t *)tx_buffer, FULL_PACKET_SIZE_BYTES);
//
//	int ohhhhnothin = 0;



}


static void Task_FlightState(void){
	uint32_t state_now = HAL_GetTick();

	    switch (stage) {
	        case NOT_LAUNCHED:
	        	// launch detected, went above thredhold
	        	// can add imu on top later on
	            if (altitude >= cfg_launch_alt_m) {
	                stage         = LAUNCHED;
	                launch_time_ms = state_now;
	                peak_alt_agl  = altitude;
	                activate_airbrakes = 0;
	            }
	            break;

	        case LAUNCHED:
	        	// ap[ogee detected if recent (based on apogree confiemed camples loop) samples are below recent
	            if (CB_Length(&recent_alt) >= APOGEE_CONFIRM_SAMPLES) {
	                uint8_t all_descending = 1;
	                for (uint16_t i = 0; i < APOGEE_CONFIRM_SAMPLES; i++) {
	                    if (CB_Get(&recent_alt, i) > (peak_alt_agl - APOGEE_DROP_M)) {
	                        all_descending = 0;
	                        break;
	                    }
	                }
	                if (all_descending) {
	                    stage = POST_APOGEE;
	                }
	            }

	            if (activate_airbrakes) {
	            	Task_Airbrakes();
	            }

	            // airbrake will deploy at set altitude on the way up
//	            if (AIRBRAKE_DEPLOY_ALT_M > 0.0f && altitude >= AIRBRAKE_DEPLOY_ALT_M && !airbrake_deployed) {
//	                Airbrake_Deploy();
//	            }
	            break;

	        // -------------------------------------------------------
	        case POST_APOGEE:
	            // fire drogue at apofee
	            if (cfg_drogue_on_apogee && drogue_fired_ms == 0) {
	                stage = DROGUE_F;
	            }

	            // HERE FOR TESTING ONLY DO NO ACTUALLY USE THIS
	            // FIRES MAIN AT APOGEE IF NEED BE
	            if (cfg_main_on_apogee && main_fired_ms == 0) {
	                stage = MAIN_F;
	            }

	            // retract airbrakes post
	            if (airbrake_deployed) {
	                Airbrake_Retract();
	            }
	            break;

	        // -------------------------------------------------------
	        case DROGUE_F:
	        	// fires drogie and will move onto firing main soon after
//	            if (drogue_fired_ms > 0 && (now - drogue_fired_ms) >= (uint32_t)cfg_drogue_fire_ms) {
//	                HAL_GPIO_WritePin(DROGUE_FIRE_GPIO_Port, DROGUE_FIRE_Pin, GPIO_PIN_RESET);
//	                stage = MAIN_F;
//	            }
//	            break;



				// cut drogue channel once its burn time is up
				if (drogue_fired_ms > 0 && (now - drogue_fired_ms) >= (uint32_t)cfg_drogue_fire_ms) {
					HAL_GPIO_WritePin(DROGUE_FIRE_GPIO_Port, DROGUE_FIRE_Pin, GPIO_PIN_RESET);
				}

				// will wait until we are below 1500 ft to fire main.
				if (drogue_fired_ms > 0 && altitude <= MAIN_DEPLOY_ALT_M) {
					stage = MAIN_F;
				}
				break;


	        // -------------------------------------------------------
	        case MAIN_F:
	        	// fire main and move onto landed (just recording stage)
	            if (main_fired_ms > 0 && (now - main_fired_ms) >= (uint32_t)cfg_main_fire_ms) {
	                HAL_GPIO_WritePin(MAIN_FIRE_GPIO_Port, MAIN_FIRE_Pin, GPIO_PIN_RESET);
	                stage = LANDED;
	            }
	            break;

	        // -------------------------------------------------------
	        case LANDED:
	            // nothing, ust keep logging and heartbeat
	            break;
	    }



}

// does the firing
static void Task_Pyro(void){
	if (stage == DROGUE_F && drogue_fired_ms == 0) {
		Fire_Drogue();
	}

	if (stage == MAIN_F && main_fired_ms == 0) {
		Fire_Main();
	}
}

static void Task_PyroChecks(void){

}

// logs datat in the loop
static void Task_Logging(void){
    //uint32_t logTime = HAL_GetTick();

    LU32_Push(&log_time, now);
    LFB_Push(&log_alt, altitude);
    LFB_Push(&log_pres, pressure);
    LFB_Push(&log_temp, temperature);
    LFB_Push(&log_bat_v, bat_voltage);
    LFB_Push(&log_main_mv, (float)main_detect);
    LFB_Push(&log_drogue_mv, (float)drogue_detect);
    LSB_Push(&log_stage, stage);

    LU32_Push(&log_servo, airbrake_deployed);
    LFB_Push(&log_airbrakes_level, curr_deployment_level);
    LFB_Push(&log_filtered_alt, filtered_alt);
    LFB_Push(&log_filtered_vel, filtered_vel);

    // mag logging
    LFB_Push(&mag_x_buf, mag.mag_x);
    LFB_Push(&mag_y_buf, mag.mag_y);
    LFB_Push(&mag_z_buf, mag.mag_z);
    LFB_Push(&mag_temp_buf, mag.temp);

    // gps logging
//    LU32_Push(&gps_year, gps_data.year);
//    LU32_Push(&gps_month, gps_data.month);
//    LU32_Push(&gps_day, gps_data.day);
//    LU32_Push(&gps_hour, gps_data.hour);
//    LU32_Push(&gps_min, gps_data.min);
//    LU32_Push(&gps_sec, gps_data.sec);

    LFB_Push(&gps_longitude, gps_data.longitude);
    LFB_Push(&gps_latitude, gps_data.latitude);

    LFB_Push(&gps_speed, gps_data.speed_m_s);
    LFB_Push(&gps_heading_deg, gps_data.heading_deg);

    LU32_Push(&log_unix, gps_data.unix_timestamp);

    //FIXME: consider adding the long michael time



}



static void Task_SD_Write(void) {

	// check if there's data in the FIFOs
	while (LU32_Available(&log_time) > 0) {
		// pop data off the buffers
		uint32_t t = LU32_Pop(&log_time);
		float alt = LFB_Pop(&log_alt);
		float pres = LFB_Pop(&log_pres);
		float temp = LFB_Pop(&log_temp);
		float bat_v = LFB_Pop(&log_bat_v);
		float main_mv = LFB_Pop(&log_main_mv);
		float drogue_mv = LFB_Pop(&log_drogue_mv);
		FlightStage_t s = LSB_Pop(&log_stage);
		uint32_t serv = LU32_Pop(&log_servo);

		// mag
		float m_x = LFB_Pop(&mag_x_buf);
		float m_y = LFB_Pop(&mag_y_buf);
		float m_z = LFB_Pop(&mag_z_buf);
		float m_temp = LFB_Pop(&mag_temp_buf);

		// gps
//		uint32_t t_year = LU32_Pop(&gps_year);
//		uint32_t t_month = LU32_Pop(&gps_month);
//		uint32_t t_day = LU32_Pop(&gps_day);
//		uint32_t t_hour = LU32_Pop(&gps_hour);
//		uint32_t t_min = LU32_Pop(&gps_min);
//		uint32_t t_sec = LU32_Pop(&gps_sec);

		float t_longitude = LFB_Pop(&gps_longitude);
		float t_latitude = LFB_Pop(&gps_latitude);

		float t_speed = LFB_Pop(&gps_speed);
		float t_heading_deg = LFB_Pop(&gps_heading_deg);


		char temp_str[16];
		char serv_str[16];
		switch (s) {

		case NOT_LAUNCHED:
			strcpy(temp_str, "NOT_LAUNCHED");
			break;
		case LAUNCHED:
			strcpy(temp_str, "LAUNCHED");
			break;
		case DROGUE_F:
			strcpy(temp_str, "DROGUE_F");
			break;
		case MAIN_F:
			strcpy(temp_str, "MAIN_F");
			break;
		case POST_APOGEE:
			strcpy(temp_str, "POST_APOGEE");
			break;
		case LANDED:
			strcpy(temp_str, "LANDED");
			break;
		}

		switch (serv) {
		case 0:
			strcpy(serv_str, "RETRACTED");
			break;
		case 1:
			strcpy(serv_str, "DEPLOYED");
			break;
		}

		float curr_airbrakes = LFB_Pop(&log_airbrakes_level);
		float curr_filtered_alt = LFB_Pop(&log_filtered_alt);
		float curr_filtered_vel = LFB_Pop(&log_filtered_vel);

		uint32_t curr_unix = LU32_Pop(&log_unix);

		char row_str[CSV_LINE_BUF];
		int len = snprintf(row_str, sizeof(row_str), "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu\r\n",
				t, alt, pres, temp, bat_v, main_mv, drogue_mv, temp_str, serv_str, curr_airbrakes, curr_filtered_alt, curr_filtered_vel, m_x, m_y, m_z, m_temp, t_longitude, t_latitude, t_speed, t_heading_deg, curr_unix);

		const char *header = "Time_ms,Alt_m,Pres_Pa,Temp_C,Bat_V,Main_mV,Drogue_mV,Stage,Servo,Airbrakes_Level,Filtered_Alt,Filtered_Vel,Mag_x,Mag_y,Mag_z,Mag_temp,"
		        		  "GPS_longitude,GPS_latitude,GPS_speed,GPS_heading_deg,Unix_Timestamp\r\n";
		if (len <= 0) continue;
		// if this new row fits in our buffer, add it
		if ((buffer_index + len) < SD_WRITE_BUFFER_SIZE) {
			memcpy(&sd_text_buffer[buffer_index], row_str, len);
			buffer_index += len;
		} else {	// if it doesn't, write buffer to SD card then add the row
			UINT bytes_written;
			f_write(&logFile, sd_text_buffer, buffer_index, &bytes_written);
			f_sync(&logFile);

			// clear the buffer, and add this row to it
			buffer_index = 0;
			memcpy(&sd_text_buffer[buffer_index], row_str, len);
			buffer_index += len;
		}



	}
}

static void Task_Airbrakes(void) {

	// tuning
	static const float H_TARGET = 3048.0f;
	static const float KP = 5.15f;
	static const float KD = 0.075f;
	static const float MAX_RATE = 3.0f;
	static const float G = 9.80665f;

	// pd state
	static float prev_error = 0.0f;
	static uint32_t prev_time_ms = 0;
	static float prev_level = 0.0f;
	static uint8_t initialized = 0;

	uint32_t airbrake_now = HAL_GetTick();

	if (airbrake_now - launch_time_ms < 7000) return;

	if (filtered_vel <= 0.0f) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, AIRBRAKE_RETRACTED_US);
		airbrake_deployed = 0;
		prev_level = 0;
		initialized = 0;
		return;
	}

	// apogee prediction under quadratic drag
	float projected_apogee = filtered_alt + (filtered_vel * filtered_vel) / (2.0f * G);

	// error: negative = going too high, positive = going too low
	float error = H_TARGET - projected_apogee;

	if (!initialized) {
		prev_error = error;
		prev_time_ms = airbrake_now;
		initialized = 1;
		return;
	}

	// dt
	float dt = (float)(airbrake_now - prev_time_ms) / 1000.0f;
	if (dt <= 0.0f) dt = 1e-6f;

	// derivative of error
	float d_error = (error - prev_error) / dt;

	// raw pd output
	// negated so that overshoot = (error < 0) and undershoot = (error > 0)
	float raw_deployment = -(KP * error + KD * d_error);

	// rate limiter
	float max_delta = MAX_RATE * dt;
	float deployment = raw_deployment;
	if (deployment > prev_level + max_delta) deployment = prev_level + max_delta;
	if (deployment < prev_level - max_delta) deployment = prev_level - max_delta;

	// physical clamp [0, 1]
	if (deployment < 0.0f) deployment = 0.0f;
	if (deployment > 1.0f) deployment = 1.0f;

	// update state
	prev_error = error;
	prev_time_ms = airbrake_now;
	prev_level = deployment;

	// drive servo (map [0, 1] to [RETRACTED_US, DEPLOYED_US]
	uint32_t pulse = (uint32_t)(AIRBRAKE_RETRACTED_US + deployment * (float)(AIRBRAKE_DEPLOYED_US - AIRBRAKE_RETRACTED_US));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);

	// update existing airbrake_deployed flag for logging
	airbrake_deployed = (deployment > 0.1f) ? 1 : 0;
	curr_deployment_level = deployment;

}







// ***************************************************************************** AI USE PAST THIS POINT, THIS STUFF IS FOR THE LOGGING/CD CARD STUFF THAT I DONT
// KNOW NAYTHING ABOUT. THIS NEEDS TO BE DOUBLE CHECKED **********************************************************************************************************




static char  sd_line_buf[2048];
static uint16_t sd_line_pos = 0;

static void SD_Flush(void) {
	if (!sd_available || sd_line_pos == 0) return;
	UINT bw;
	f_write(&logFile, sd_line_buf, sd_line_pos, &bw);
	f_sync(&logFile);   // sync only when flushing the batch, not per-record
	sd_line_pos = 0;
}


// Drain the FIFO log buffers to SD and Flash
static void Task_LogFlush(void) {

	if (!sd_available) return;

	    while (LU32_Available(&log_time)) {
	        uint32_t      ts  = LU32_Pop(&log_time);
	        float         alt = LFB_Pop(&log_alt);
	        float         prs = LFB_Pop(&log_pres);
	        float         tmp = LFB_Pop(&log_temp);
	        float         bv  = LFB_Pop(&log_bat_v);
	        float         mmv = LFB_Pop(&log_main_mv);
	        float         dmv = LFB_Pop(&log_drogue_mv);
	        FlightStage_t st  = LSB_Pop(&log_stage);

	        char line[CSV_LINE_BUF];
	        int len = snprintf(line, sizeof(line),
	            "%lu,%d.%02d,%d.%01d,%d.%02d,%d.%03d,%d,%d,%d\r\n",
	            (unsigned long)ts,
	            (int)alt,  (int)(fabsf(alt) * 100) % 100,
	            (int)prs,  (int)(fabsf(prs) * 10)  % 10,
	            (int)tmp,  (int)(fabsf(tmp) * 100) % 100,
	            (int)bv,   (int)(fabsf(bv)  * 1000) % 1000,
	            (int)mmv,
	            (int)dmv,
	            (int)st);

	        if (len > 0 && (sd_line_pos + (uint16_t)len) < sizeof(sd_line_buf)) {
	            memcpy(sd_line_buf + sd_line_pos, line, (size_t)len);
	            sd_line_pos += (uint16_t)len;
	        }

	        // Write to flash as binary record (this path is fine — separate SPI bus)
	        if (flash_available) {
	            Flash_Write_Record_Raw(ts, alt, prs, tmp, bv, (uint8_t)st);
	        }
	    }

	    // Flush the batch to SD every LOG_FLUSH_MS (called from the timer in the loop)
	    SD_Flush();
}





// Binary record layout (24 bytes):
//   [0-3]   uint32  time_ms
//   [4-7]   float   altitude_m
//   [8-11]  float   pressure_pa
//   [12-15] float   temp_c
//   [16-19] float   bat_v
//   [20]    uint8   stage
//   [21-23] pad 0x00
static void Flash_Write_Record_Raw(uint32_t ts, float alt, float prs, float tmp, float bv, uint8_t st) {
    if (!flash_available) return;
    if (flash_addr + FLASH_RECORD_SIZE > FLASH_MAX_ADDR) return;

    uint8_t rec[FLASH_RECORD_SIZE];
    memset(rec, 0x00, FLASH_RECORD_SIZE);   // zero the whole record including padding
    memcpy(rec + 0,  &ts,  4);
    memcpy(rec + 4,  &alt, 4);
    memcpy(rec + 8,  &prs, 4);
    memcpy(rec + 12, &tmp, 4);
    memcpy(rec + 16, &bv,  4);
    rec[20] = st;
    // rec[21..31] are already 0x00 from memset

    Flash_Write(&hspi2, flash_addr, rec, FLASH_RECORD_SIZE);
    flash_addr += FLASH_RECORD_SIZE;
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
