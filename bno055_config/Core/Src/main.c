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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BNO055.h"
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

/* USER CODE BEGIN PV */
BNO055_t bno;

uint8_t sensor_found = 0;
volatile uint8_t calibration_mode = 0;
uint32_t calibration_start_time = 0;
calibration_data_t new_calib;

typedef enum {
    CALIB_STATE_IDLE = 0,
    CALIB_STATE_FLAT_UP,
    CALIB_STATE_FLAT_DOWN,
    CALIB_STATE_SIDE_LEFT,
    CALIB_STATE_SIDE_RIGHT,
    CALIB_STATE_FRONT_UP,
    CALIB_STATE_BACK_DOWN,
    CALIB_STATE_FIGURE_8,
    CALIB_STATE_COMPLETE,
    CALIB_STATE_FAILED
} calibration_state_t;

typedef struct {
    calibration_state_t state;
    uint32_t state_timer;
    uint32_t hold_time;
    uint8_t accel_done;
    uint8_t gyro_done;
    uint8_t mag_done;
    uint8_t retry_count;
} calibration_process_t;

calibration_process_t calib_process = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Start_Calibration(void) {
    calibration_mode = 1;
    calibration_start_time = HAL_GetTick();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    calib_process.state = CALIB_STATE_FLAT_UP;
    calib_process.state_timer = HAL_GetTick();
    calib_process.hold_time = 3000;
    calib_process.accel_done = 0;
    calib_process.gyro_done = 0;
    calib_process.mag_done = 0;
    calib_process.retry_count = 0;
}

void Update_LED_Pattern(uint8_t pattern) {
    static uint32_t led_timer = 0;
    static uint8_t led_count = 0;

    if (HAL_GetTick() - led_timer > 200) {
        led_timer = HAL_GetTick();
        if (led_count < pattern * 2) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            led_count++;
        } else if (led_count > pattern * 2 + 4) {
            led_count = 0;
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            led_count++;
        }
    }
}

void Process_Calibration(void) {
    static uint32_t last_check = 0;

    if (HAL_GetTick() - last_check < 100)
        return;
    last_check = HAL_GetTick();

    BNO055_GetCalibrationStatus(&bno);

    if (bno.calib_status.gyro == 3 && !calib_process.gyro_done) {
        calib_process.gyro_done = 1;
    }

    if (bno.calib_status.accel == 3 && !calib_process.accel_done) {
        calib_process.accel_done = 1;
        calib_process.state = CALIB_STATE_FIGURE_8;
        calib_process.state_timer = HAL_GetTick();
    }

    if (bno.calib_status.mag == 3 && !calib_process.mag_done) {
        calib_process.mag_done = 1;
    }

    switch (calib_process.state) {
        case CALIB_STATE_FLAT_UP:
            Update_LED_Pattern(1);
            if (HAL_GetTick() - calib_process.state_timer > calib_process.hold_time) {
                calib_process.state = CALIB_STATE_FLAT_DOWN;
                calib_process.state_timer = HAL_GetTick();
            }
            break;

        case CALIB_STATE_FLAT_DOWN:
            Update_LED_Pattern(2);
            if (HAL_GetTick() - calib_process.state_timer > calib_process.hold_time) {
                calib_process.state = CALIB_STATE_SIDE_LEFT;
                calib_process.state_timer = HAL_GetTick();
            }
            break;

        case CALIB_STATE_SIDE_LEFT:
            Update_LED_Pattern(3);
            if (HAL_GetTick() - calib_process.state_timer > calib_process.hold_time) {
                calib_process.state = CALIB_STATE_SIDE_RIGHT;
                calib_process.state_timer = HAL_GetTick();
            }
            break;

        case CALIB_STATE_SIDE_RIGHT:
            Update_LED_Pattern(4);
            if (HAL_GetTick() - calib_process.state_timer > calib_process.hold_time) {
                calib_process.state = CALIB_STATE_FRONT_UP;
                calib_process.state_timer = HAL_GetTick();
            }
            break;

        case CALIB_STATE_FRONT_UP:
            Update_LED_Pattern(5);
            if (HAL_GetTick() - calib_process.state_timer > calib_process.hold_time) {
                calib_process.state = CALIB_STATE_BACK_DOWN;
                calib_process.state_timer = HAL_GetTick();
            }
            break;

        case CALIB_STATE_BACK_DOWN:
            Update_LED_Pattern(6);
            if (HAL_GetTick() - calib_process.state_timer > calib_process.hold_time) {
                if (bno.calib_status.accel < 3) {
                    calib_process.retry_count++;
                    if (calib_process.retry_count < 3) {
                        calib_process.state = CALIB_STATE_FLAT_UP;
                        calib_process.state_timer = HAL_GetTick();
                    } else {
                        calib_process.state = CALIB_STATE_FIGURE_8;
                        calib_process.state_timer = HAL_GetTick();
                    }
                } else {
                    calib_process.state = CALIB_STATE_FIGURE_8;
                    calib_process.state_timer = HAL_GetTick();
                }
            }
            break;

        case CALIB_STATE_FIGURE_8:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

            if (bno.calib_status.mag == 3 || HAL_GetTick() - calib_process.state_timer > 30000) {
                if (bno.calib_status.system == 3 && bno.calib_status.gyro == 3
                    && bno.calib_status.accel == 3 && bno.calib_status.mag == 3) {
                    calib_process.state = CALIB_STATE_COMPLETE;
                } else if (HAL_GetTick() - calibration_start_time > 60000) {
                    calib_process.state = CALIB_STATE_FAILED;
                }
            }
            break;

        case CALIB_STATE_COMPLETE:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            BNO055_GetCalibration(&bno, &new_calib);
            calibration_mode = 0;
            calib_process.state = CALIB_STATE_IDLE;
            break;

        case CALIB_STATE_FAILED:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            calibration_mode = 0;
            calib_process.state = CALIB_STATE_IDLE;
            break;

        case CALIB_STATE_IDLE:
        default:
            break;
    }

    if (calibration_mode && HAL_GetTick() - calibration_start_time > 120000) {
        calib_process.state = CALIB_STATE_FAILED;
    }
}

uint8_t Get_Calibration_State(void) {
    return calib_process.state;
}

void Get_Calibration_Info(uint8_t *state, uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
    *state = calib_process.state;
    *sys = bno.calib_status.system;
    *gyro = bno.calib_status.gyro;
    *accel = bno.calib_status.accel;
    *mag = bno.calib_status.mag;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_I2C1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	if (BNO055_Init(&bno, &hi2c1, 0) != HAL_OK) {
			Error_Handler();
		}

	BNO055_SetAxisRemap(&bno, AXIS_REMAP_P1, AXIS_REMAP_SIGN_P1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		static uint32_t last_update = 0;
		if (HAL_GetTick() - last_update > 100) {
			last_update = HAL_GetTick();
			BNO055_Update(&bno);
		}

		if (calibration_mode) {
			Process_Calibration();
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		if (calibration_mode == 0) {
			Start_Calibration();
		}
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		BNO055_ProcessDMA(&bno);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
//		if (bno.dma_ready) {
//			BNO055_UpdateDMA(&bno);
//		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
