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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "octospi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "../ICM42688P/icm42688p.h"
#include "hmc5883l.h"
#include "../LED/MAIN_BOARD_RGB/ws2812.h"
#include "targets.h"
#include "../BUZZER/buzzer.h"
#include "../EEPROM/eeprom.h"
#include "../PID/Double Loop PID/pid_controller.h"
#include "../RECEIVER/FS-iA6B/FS-iA6B.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* *********** USART6 printf function code ************ */

int _write(int file, char* p, int len)
{
    for (int i = 0; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART6)) {
            if (LL_USART_IsActiveFlag_ORE(USART6) || LL_USART_IsActiveFlag_FE(USART6)) {
                LL_USART_ClearFlag_ORE(USART6);
                LL_USART_ClearFlag_FE(USART6);
                return -1; // Indicate error
            }
        }
        LL_USART_TransmitData8(USART6, *(p + i));
    }
    while (!LL_USART_IsActiveFlag_TC(USART6)) {}
    return len;
}

/* *********** USART6 printf function code ************ */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern SPI_HandleTypeDef hspi3;
ICM42688P_HandleTypeDef icm;
ICM42688P_Data_t icmData;

extern I2C_HandleTypeDef hi2c1;  // Ensure hi2c1 is initialized by CubeMX
HMC5883L_Data_t magData;

volatile uint8_t sensors_ready = 0;
extern TIM_HandleTypeDef htim3;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
unsigned char failsafe_flag = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern uint8_t uart6_rx_flag;
extern uint8_t uart6_rx_data;
extern uint8_t uart4_rx_flag;
extern uint8_t uart4_rx_data;
extern uint8_t ibus_rx_buf[32];
extern uint8_t ibus_rx_cplt_flag;
extern uint8_t uart7_rx_data;
uint8_t telemetry_tx_buf[20];

extern DMA_HandleTypeDef WS2812_DMA; // DMA handle
extern volatile uint8_t data_sent_flag; // Flag from ws2812.c

void eeprom_startup(void);

float eeprom_pid_read[3];
float eeprom_gyro_read[3];
float eeprom_accel_read[3];
float eeprom_mag_read[3];
DualPID_t eeprom_roll_pid_read;
DualPID_t eeprom_pitch_pid_read;
PID_t eeprom_yaw_rate_pid_read;

AircraftLights_t aircraft_lights;

RuntimeDualPID_t roll_pid;
RuntimeDualPID_t pitch_pid;
RuntimePID_t yaw_rate_pid;

HAL_StatusTypeDef status;
float pressure, temperature, altitude;
float ground_pressure = 101325.0f;

unsigned char motor_arming_flag = 0;
unsigned short iBus_SwA_Prev = 0;
unsigned short iBus_rx_cnt = 0;
uint16_t ccr[4];
unsigned short ccr1, ccr2, ccr3, ccr4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

int Is_iBus_Throttle_Min(void);
int Is_iBus_Throttle_Armed(void);
void ESC_Calibration(void);
int Is_iBus_Received(void);

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART6_UART_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_OCTOSPI1_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_TIM5_Init();
  MX_UART7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  StartupTone();

  LL_USART_EnableIT_RXNE(USART6);
  LL_USART_EnableIT_RXNE_RXFNE(UART4);

  HAL_Delay(500);

  eeprom_startup();

  // Initialize PID controllers
  pid_init(&roll_pid, &pitch_pid, &yaw_rate_pid,
		  &eeprom_roll_pid_read, &eeprom_pitch_pid_read, &eeprom_yaw_rate_pid_read);

  /* *********** ESC Startup Calibration ************ */
  HAL_Delay(3000);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  /* *********** ESC Startup Calibration END ************ */

  /* *********** iBus Calibration Check ************ */
  while (Is_iBus_Received() == 0) {
	  Buzzer_On(3000);
	  HAL_Delay(200);
	  Buzzer_Off();
	  HAL_Delay(200);
  }
  if (iBus.SwC == 2000) {
	  Buzzer_On(1500);
	  HAL_Delay(200);
	  Buzzer_On(2000);
	  HAL_Delay(200);
	  Buzzer_On(1500);
	  HAL_Delay(200);
	  Buzzer_On(2000);
	  HAL_Delay(200);
	  Buzzer_Off();
	  ESC_Calibration();
	  while (iBus.SwC != 1000) {
		  Is_iBus_Received();
		  Buzzer_On(1500);
		  HAL_Delay(200);
		  Buzzer_On(2000);
		  HAL_Delay(200);
		  Buzzer_Off();
	  }
  }
  /* *********** iBus Calibration Check END ************ */

  /* *********** iBus Throttle Check ************ */
  while (Is_iBus_Throttle_Min() == 0 || iBus.SwA == 2000) {
	  Buzzer_On(343);
	  HAL_Delay(70);
	  Buzzer_Off();
	  HAL_Delay(70);
  }
  Buzzer_On(1092);
  HAL_Delay(100);
  Buzzer_On(592);
  HAL_Delay(100);
  Buzzer_On(292);
  HAL_Delay(100);
  Buzzer_Off();
  /* *********** iBus Throttle Check END ************ */

//  LL_TIM_EnableCounter(TIM7);
//  LL_TIM_EnableIT_UPDATE(TIM7);

  /* === Initialize sensors here === */

  HMC5883L_Init();
  uint8_t hmc_id = HMC5883L_ReadReg(HMC5883L_ID_A);
  printf("HMC5883L ID: %c\n", hmc_id);

  WS2812_Init(&htim3);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void send_deftask(void)
{
	printf("Hello from deftask\r\n");
//	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
}

void send_task2(void)
{
//	HAL_GPIO_TogglePin(GPIOx, GPIO_Pin)
}

void send_task3(void)
{
	printf("Hello from Task3\r\n");
}

void run_imu(void) {

	if (!sensors_ready) return;

    // Read sensor data directly
    ICM42688P_ReadData(&icm, &icmData);

    // Print data (or send to other tasks / filters)
    printf("Accel: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f | Temp: %.2f\r\n",
           icmData.accel[0], icmData.accel[1], icmData.accel[2],
           icmData.gyro[0], icmData.gyro[1], icmData.gyro[2],
           icmData.temperature);
}


void run_mag(void)
{

	if (!sensors_ready) return;

    HMC5883L_ProcessMagData();   // Reads raw data into hmc_data
    Compass_Data_t *compass = HMC5883L_GetCompassData();

    printf("Mag: %.2f %.2f %.2f | Heading: %.2f°\r\n",
           hmc_data.mag_x, hmc_data.mag_y, hmc_data.mag_z,
           compass->heading);
}

void sensor_init(void){
    // === Initialize ICM42688P ===
	ICM42688P_Init(&icm, &hspi3);
	printf("ICM42688P initialization called.\r\n");

    // === Initialize HMC5883L ===
    HMC5883L_Init();
    uint8_t hmc_id = HMC5883L_ReadReg(HMC5883L_ID_A);
    printf("HMC5883L ID: %c\r\n", hmc_id);

    WS2812_Init(&WS2812_TIMER);

    printf("Sensor initialization complete. Deleting InitTask...\r\n");

    sensors_ready = 1;

    system_startup();

}

void system_startup(void) {

	while (Is_iBus_Throttle_Armed() == 0) {
//		calibration_task();
//		if (is_cmd_mode()) {
//			continue;
//		}

	}

	/* *********** iBus Throttle Check ************ */
	while (Is_iBus_Throttle_Min() == 0 || iBus.SwA != 2000) {
		Buzzer_On(1200);
		HAL_Delay(300);
		Buzzer_Off();
		HAL_Delay(70);
	}
	Buzzer_On(1092);
	HAL_Delay(100);
	Buzzer_On(592);
	HAL_Delay(100);
	Buzzer_On(292);
	HAL_Delay(100);
	Buzzer_Off();
	/* *********** iBus Throttle Check END ************ */

}

void eeprom_startup(void){

	if (EEPROM_Init() != W25Qxx_OK) {
		printf("EEPROM Init Failed\r\n");
		while(1)
		{
			Buzzer_On(500);
			HAL_Delay(200);
			Buzzer_Off();
			HAL_Delay(200);
			Buzzer_On(500);
			HAL_Delay(200);
			Buzzer_Off();
			HAL_Delay(200);
		}
		Error_Handler();
	}

	DroneConfig_t config;

	printf("================= Connecting to EEPROM =================\n"
			"\r\n");

	if (EEPROM_ReadConfig(&config) == W25Qxx_OK) {
		printf("Config Loaded: Flight Mode %d, PID P: %.2f\r\n",
				config.flight_mode, config.pid[0]);
	} else {
		printf("No valid config found, loading defaults\r\n");
		DroneConfig_t default_config = {
				.accel_cal = {0.030256f, -0.026638f, -0.122559f},
				.gyro_cal = {0.016646f, 0.173536f, -0.355303f},
				.mag_cal = {1.50f, -1.50f, 0.50f},
				.pid = {1.0f, 0.1f, 0.5f},
				.flight_mode = 0,
				.gps_lat = 0.0f,
				.gps_lon = 0.0f,
				.gps_alt = 0.0f,
				.roll_pid = {
						.out = {0.62f, 0.01f, 0.10f},
						.in = {0.30f, 0.07f, 0.010f}
				},
				.pitch_pid = {
						.out = {0.62f, 0.01f, 0.10f},
						.in = {0.30f, 0.07f, 0.010f}
				},
				.yaw_rate_pid = {0.2f, 0.01f, 0.003f},
				.lights = {
						.rgb = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}},
						.mode = 0
				},
				.crc = 0
		};
		default_config.crc = CalculateCRC32((uint8_t*)&default_config, sizeof(DroneConfig_t) - sizeof(uint32_t));
		if (EEPROM_WriteConfig(&default_config) != W25Qxx_OK) {
			printf("Failed to write default config\r\n");
			Error_Handler();
		}
		printf("Default config written and verified\r\n");
	}

	if (EEPROM_GetPID(eeprom_pid_read) == W25Qxx_OK) {
		printf("EEPROM PID read: P=%.2f, I=%.2f, D=%.2f\r\n",
				eeprom_pid_read[0], eeprom_pid_read[1], eeprom_pid_read[2]);
	} else {
		printf("Failed to read PID\r\n");
	}

	if (EEPROM_GetGyroCalibration(eeprom_gyro_read) == W25Qxx_OK) {
		printf("EEPROM Gyro read: X=%.2f, Y=%.2f, Z=%.2f\r\n",
				eeprom_gyro_read[0], eeprom_gyro_read[1], eeprom_gyro_read[2]);
	} else {
		printf("Failed to read EEPROM Gyro Data\r\n");
	}

	if (EEPROM_GetAccelCalibration(eeprom_accel_read) == W25Qxx_OK) {
		printf("EEPROM Accel read: X=%.2f, Y=%.2f, Z=%.2f\r\n",
				eeprom_accel_read[0], eeprom_accel_read[1], eeprom_accel_read[2]);
	} else {
		printf("Failed to read EEPROM Accel Data\r\n");
	}

	if (EEPROM_GetMagCalibration(eeprom_mag_read) == W25Qxx_OK) {
		printf("EEPROM Mag read: X=%.2f, Y=%.2f, Z=%.2f\r\n",
				eeprom_mag_read[0], eeprom_mag_read[1], eeprom_mag_read[2]);
	} else {
		printf("Failed to read EEPROM Mag Data\r\n");
	}

	if (EEPROM_GetRollPID(&eeprom_roll_pid_read) == W25Qxx_OK) {
		printf("Roll PID: Out P=%.3f, I=%.3f, D=%.3f, In P=%.3f, I=%.3f, D=%.3f\r\n",
				eeprom_roll_pid_read.out.kp, eeprom_roll_pid_read.out.ki, eeprom_roll_pid_read.out.kd,
				eeprom_roll_pid_read.in.kp, eeprom_roll_pid_read.in.ki, eeprom_roll_pid_read.in.kd);
	} else {
		printf("Failed to read Roll PID\r\n");
	}

	if (EEPROM_GetPitchPID(&eeprom_pitch_pid_read) == W25Qxx_OK) {
		printf("Pitch PID: Out P=%.3f, I=%.3f, D=%.3f, In P=%.3f, I=%.3f, D=%.3f\r\n",
				eeprom_pitch_pid_read.out.kp, eeprom_pitch_pid_read.out.ki, eeprom_pitch_pid_read.out.kd,
				eeprom_pitch_pid_read.in.kp, eeprom_pitch_pid_read.in.ki, eeprom_pitch_pid_read.in.kd);
	} else {
		printf("Failed to read Pitch PID\r\n");
	}

	if (EEPROM_GetYawRatePID(&eeprom_yaw_rate_pid_read) == W25Qxx_OK) {
		printf("Yaw Rate PID: P=%.3f, I=%.3f, D=%.3f\r\n",
				eeprom_yaw_rate_pid_read.kp, eeprom_yaw_rate_pid_read.ki, eeprom_yaw_rate_pid_read.kd);
	} else {
		printf("Failed to read Yaw Rate PID\r\n");
	}

	if (EEPROM_GetAircraftLights(&aircraft_lights) == W25Qxx_OK) {
		printf("Lights: LED1(R=%d,G=%d,B=%d), LED2(R=%d,G=%d,B=%d), LED3(R=%d,G=%d,B=%d), LED4(R=%d,G=%d,B=%d), Mode=%d\r\n",
				aircraft_lights.rgb[0][0], aircraft_lights.rgb[0][1], aircraft_lights.rgb[0][2],
				aircraft_lights.rgb[1][0], aircraft_lights.rgb[1][1], aircraft_lights.rgb[1][2],
				aircraft_lights.rgb[2][0], aircraft_lights.rgb[2][1], aircraft_lights.rgb[2][2],
				aircraft_lights.rgb[3][0], aircraft_lights.rgb[3][1], aircraft_lights.rgb[3][2],
				aircraft_lights.mode);
	} else {
		printf("Failed to read Aircraft Lights\r\n");
	}

	printf("\r\n"
			"================= EEPROM Data Fetched =================\n"
			"\r\n");

}

int Is_iBus_Throttle_Min(void) {
    if (ibus_rx_cplt_flag == 1) {
        ibus_rx_cplt_flag = 0;
        if (iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1) {
            iBus_Parsing(&ibus_rx_buf[0], &iBus);
            if (iBus.LV < 1010)
                return 1;
        }
    }
    return 0;
}

void mixer_run(void) {

	// Motor mixing
	if (motor_arming_flag == 1 && failsafe_flag == 0) {
//		motor_mixing(roll_out, pitch_out, yaw_out, throttle, ccr);

		ccr1 = 1050 + ((iBus.LV - 1000) * 950) / 1000;
		ccr2 = 1050 + ((iBus.LV - 1000) * 950) / 1000;
		ccr3 = 1050 + ((iBus.LV - 1000) * 950) / 1000;
		ccr4 = 1050 + ((iBus.LV - 1000) * 950) / 1000;

		TIM5->CCR1 = ccr1;
		TIM5->CCR2 = ccr2;
		TIM5->CCR3 = ccr3;
		TIM5->CCR4 = ccr4;
	} else {
		TIM5->CCR1 = 1000;
		TIM5->CCR2 = 1000;
		TIM5->CCR3 = 1000;
		TIM5->CCR4 = 1000;
	}

	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);



	if (iBus.SwA == 2000 && iBus_SwA_Prev != 2000) {
		if (iBus.LV < 1010) {
			motor_arming_flag = 1;
		}
	}
	iBus_SwA_Prev = iBus.SwA;

	if (iBus.SwA != 2000) {
		motor_arming_flag = 0;
	}

	if (ibus_rx_cplt_flag == 1) {
		ibus_rx_cplt_flag = 0;
		if (iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1) {
			//			main_led(0, 0, 255, 0, 0.3);
			//			main_led(0, 0, 255, 0, 0.0);
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			iBus_rx_cnt++;
			if (iBus_isActiveFailsafe(&iBus) == 1) {
				failsafe_flag = 1;
				Buzzer_On(292);
				HAL_Delay(50);
				Buzzer_Off();
			} else {
				failsafe_flag = 0;
				Buzzer_Off();
			}
		}
	}

	if (failsafe_flag == 1 || failsafe_flag == 2) {
		Buzzer_On(292);
		HAL_Delay(50);
		Buzzer_Off();
	}

}

void FSiA6B_Print(void) {
	printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
			iBus.RH, iBus.RV, iBus.LV, iBus.LH, iBus.SwA, iBus.SwB, iBus.SwC, iBus.SwD, iBus.VrA, iBus.VrB);
}

void FailSafe_1000Hz(void) {

	if (iBus_rx_cnt == 0) {
		failsafe_flag = 2;
	}
	iBus_rx_cnt = 0;
}

int Is_iBus_Throttle_Armed(void) {
    if (ibus_rx_cplt_flag == 1) {
        ibus_rx_cplt_flag = 0;
        if (iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1) {
            iBus_Parsing(&ibus_rx_buf[0], &iBus);
            if (iBus.SwA >= 1900)
                return 1;
        }
    }
    return 0;
}

void ESC_Calibration(void) {
    TIM5->CCR1 = 2000;
    TIM5->CCR2 = 2000;
    TIM5->CCR3 = 2000;
    TIM5->CCR4 = 2000;
    main_led(0, 0, 255, 0, 0.1, 1);
    HAL_Delay(7000);
    TIM5->CCR1 = 1000;
    TIM5->CCR2 = 1000;
    TIM5->CCR3 = 1000;
    TIM5->CCR4 = 1000;
    main_led(0, 0, 255, 0, 0.1, 1);
    HAL_Delay(8000);
    main_led(0, 0, 255, 0, 0.1, 1);
}

int Is_iBus_Received(void) {
    if (ibus_rx_cplt_flag == 1) {
        ibus_rx_cplt_flag = 0;
        if (iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1) {
            iBus_Parsing(&ibus_rx_buf[0], &iBus);
            return 1;
        }
    }
    return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART7) {
        HAL_UART_Receive_IT(&huart7, &uart7_rx_data, 1);
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
