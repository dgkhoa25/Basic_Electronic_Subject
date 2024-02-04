/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hx711.h"
#include "Servo.h"
#include "Button.h"
#include "LiquidCrystal_I2C.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	init_state,
	cat_state,
	xiMang_state,
	nuoc_state,
	da_state,
	finish_state
} system_state_t;

typedef enum
{
	init_st,
	start_st,
	stop_st,
	mac75_st,
	mac200_st
} options_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define MAC 75
#define XIMANG75_SETVAL 		(100U)
#define XIMANG75_SETTIMES  	(5U)
#define NUOC75_SETVAL  			(120U)
#define NUOC75_SETTIMES  		(3U)
#define CAT75_SETVAL  			(201U)
#define CAT75_SETTIMES  		(3U)

// Define MAC 200
#define XIMANG200_SETVAL 		(100U)
#define XIMANG200_SETTIMES 	(5U)
#define NUOC200_SETVAL 			(144U)
#define NUOC200_SETTIMES 		(2U)
#define CAT200_SETVAL 			(252U)
#define CAT200_SETTIMES 		(5U)
#define DA200_SETVAL 				(515U)
#define DA200_SETTIMES			(4U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static system_state_t sys_state;
static options_state_t op_state;

// Khai bao bien LCD 16x2 + I2C
static LiquidCrystal_I2C lcd;

// Khai bien doi tuong(objects) cho he thong
hx711_t loadcell1, loadcell2, loadcell3, loadcell4;
static servo_typedef_t servo1, servo2, servo3, servo4;
static Button_Typdef cb1, cb2, cb3,cb4, btn_start, btn_stop, btn_mac75, btn_mac200;

// Khoi tao bien de dem so lan can
static uint8_t ui8_cat75Count = 0;
static uint8_t ui8_xiMang75Count = 0;
static uint8_t ui8_nuoc75Count = 0;

static uint8_t ui8_cat200Count = 0;
static uint8_t ui8_xiMang200Count = 0;
static uint8_t ui8_nuoc200Count = 0;
static uint8_t ui8_da200Count = 0;

extern volatile uint8_t ui8_start_var;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Khai bao nguyen mau ham dieu khien bang tai
static void conveyor_start(void);
static void conveyor_stop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void	btn_pressing_callback(Button_Typdef *ButtonX)
{
	switch (ButtonX->GPIO_Pin)
	{
		case GPIO_PIN_2: // CB 1 - Cat
		{
			// MAC 75
			if (sys_state == cat_state && op_state == mac75_st)
			{
				if (ui8_cat75Count == CAT75_SETTIMES)
				{
					ui8_cat75Count = 0;
					lcd_set_cursor(&lcd, 1,11);
					lcd_printf(&lcd, "%d", ui8_cat75Count);
					sys_state = xiMang_state;
					break;
				}
				conveyor_stop();
				servo_open(&servo1);
				ui8_cat75Count++;
				lcd_set_cursor(&lcd, 0, 9);
				lcd_printf(&lcd, "CAT      ");
				lcd_set_cursor(&lcd, 1,11);
				lcd_printf(&lcd, "%d", ui8_cat75Count);
			}
			// MAC 200
			if (sys_state == cat_state && op_state == mac200_st)
			{
				if (ui8_cat200Count == CAT200_SETTIMES)
				{
					ui8_cat200Count = 0;
					lcd_set_cursor(&lcd, 1,11);
					lcd_printf(&lcd, "%d", ui8_cat200Count);
					sys_state = xiMang_state;
					break;
				}
				conveyor_stop();
				servo_open(&servo1);
				ui8_cat200Count++;
				lcd_set_cursor(&lcd, 0, 9);
				lcd_printf(&lcd, "CAT      ");
				lcd_set_cursor(&lcd, 1,11);
				lcd_printf(&lcd, "%d", ui8_cat200Count);
			}
			break;
		}
		case GPIO_PIN_3: // CB 2 - Xi mang
		{
			// MAC 75
			if (sys_state == xiMang_state && op_state == mac75_st)
			{
				if (ui8_xiMang75Count == XIMANG75_SETTIMES)
				{
					ui8_xiMang75Count = 0;
					lcd_set_cursor(&lcd, 1,11);
					lcd_printf(&lcd, "%d", ui8_xiMang75Count);
					sys_state = nuoc_state;
					break;
				}
				conveyor_stop();
				servo_open(&servo2);
				ui8_xiMang75Count++;
				lcd_set_cursor(&lcd, 0, 9);
				lcd_printf(&lcd, "X.MANG");
				lcd_set_cursor(&lcd, 1,11);
				lcd_printf(&lcd, "%d", ui8_xiMang75Count);
			}
			// MAC 200
			if (sys_state == xiMang_state && op_state == mac200_st)
			{
				if (ui8_xiMang200Count == XIMANG200_SETTIMES)
				{
					ui8_xiMang200Count = 0;
					lcd_set_cursor(&lcd, 1,11);
					lcd_printf(&lcd, "%d", ui8_xiMang200Count);
					sys_state = da_state;
					break;
				}
				conveyor_stop();
				servo_open(&servo2);
				ui8_xiMang200Count++;
				lcd_set_cursor(&lcd, 0, 9);
				lcd_printf(&lcd, "X.MANG");
				lcd_set_cursor(&lcd, 1,11);
				lcd_printf(&lcd, "%d", ui8_xiMang200Count);
			}
			break;
		}
		case GPIO_PIN_4: // CB 3 - Da
		{
			// MAC 75
			if (sys_state == da_state && op_state == mac75_st)
			{
				sys_state = nuoc_state;
			}
			// MAC 200
			if (sys_state == da_state && op_state == mac200_st)
			{
				if (ui8_da200Count == DA200_SETTIMES)
				{
					ui8_da200Count = 0;
					lcd_set_cursor(&lcd, 1,11);
					lcd_printf(&lcd, "%d", ui8_da200Count);
					sys_state = nuoc_state;
					break;
				}
				conveyor_stop();
				servo_open(&servo3);
				ui8_da200Count++;
				lcd_set_cursor(&lcd, 0, 9);
				lcd_printf(&lcd, "DA      ");
				lcd_set_cursor(&lcd, 1,11);
				lcd_printf(&lcd, "%d", ui8_da200Count);
			}

			break;
		}
		case GPIO_PIN_5: // CB 4 - Nuoc
		{
			// MAC 75
			if (sys_state == nuoc_state && op_state == mac75_st)
			{
				conveyor_stop();
				servo_open(&servo4);
				ui8_nuoc75Count++;
				lcd_set_cursor(&lcd, 0, 9);
				lcd_printf(&lcd, "NUOC     ");
				lcd_set_cursor(&lcd, 1,11);
				lcd_printf(&lcd, "%d", ui8_nuoc75Count);
				if (ui8_nuoc75Count == NUOC75_SETTIMES)
				{

					break;
				}
			}
			// MAC 200
			if (sys_state == nuoc_state && op_state == mac200_st)
			{
				conveyor_stop();
				servo_open(&servo4);
				ui8_nuoc200Count++;
				lcd_set_cursor(&lcd, 0, 9);
				lcd_printf(&lcd, "NUOC     ");
				lcd_set_cursor(&lcd, 1,11);
				lcd_printf(&lcd, "%d", ui8_nuoc200Count);
				if (ui8_nuoc200Count == NUOC200_SETTIMES)
				{

					break;
				}
			}
			break;
		}
		case GPIO_PIN_13:
		{
			// START
			HAL_Delay(20);
			lcd_clear_display(&lcd);
			lcd_set_cursor(&lcd, 0, 3);
			lcd_printf(&lcd, "Starting....");
			op_state = start_st;
			break;
		}
		case GPIO_PIN_14:
		{
			// STOP
			if (op_state != start_st)
			{
				op_state = stop_st;
				lcd_clear_display(&lcd);
				HAL_Delay(20);
				lcd_set_cursor(&lcd, 0, 1);
				lcd_printf(&lcd, "<STOPED SYSTEM>");
				lcd_set_cursor(&lcd, 1, 0);
				lcd_printf(&lcd, "->Start to Reset");
			}
			break;
		}
		case GPIO_PIN_15:
		{
			// MAC75
			if (op_state != stop_st)
			{
				op_state = mac75_st;
				sys_state = cat_state;
				lcd_clear_display(&lcd);
				HAL_Delay(20);
				lcd_set_cursor(&lcd, 0, 0);
				lcd_printf(&lcd, "MAC 75 |");
				lcd_set_cursor(&lcd, 1, 0);
				lcd_printf(&lcd, "W: ");
				lcd_set_cursor(&lcd, 1, 9);
				lcd_printf(&lcd, "T: ");
			}
			break;
		}
		case GPIO_PIN_12:
		{
			// MAC200
			if (op_state != stop_st)
			{
				op_state = mac200_st;
				sys_state = cat_state;
				lcd_clear_display(&lcd);
				HAL_Delay(20);
				lcd_set_cursor(&lcd, 0, 0);
				lcd_printf(&lcd, "MAC 200 |");
				lcd_set_cursor(&lcd, 1, 0);
				lcd_printf(&lcd, "W: ");
				lcd_set_cursor(&lcd, 1, 9);
				lcd_printf(&lcd, "T: ");
			}
			break;
		}
	}
}

void btn_press_timeout_callback(Button_Typdef *ButtonX)
{
	if (ButtonX->GPIO_Pin == GPIO_PIN_13)
	{
		NVIC_SystemReset();
	}
}


// -------- Ham callback xu li va dieu khien RC Servo state --------
void hx711_callback(hx711_t *hx711)
{
	//   Begin mode MAC75 
	if (op_state == mac75_st && sys_state != finish_state && sys_state != init_state) // MAC 75
	{
		if (hx711->weight >= (CAT75_SETVAL + 2) && hx711 == &loadcell1) // Cat
		{
			lcd_set_cursor(&lcd, 1,3);
			if (hx711->weight > 0)
				lcd_printf(&lcd, "%.0f", hx711->weight - 4);
			servo_close(&servo1);
			conveyor_start();
		}
		else if (hx711->weight == 0 && hx711 == &loadcell1 && ui8_cat75Count != 0)
		{
			lcd_set_cursor(&lcd, 1,3);
			lcd_printf(&lcd, "   ");
		}
		if (hx711->weight >= (XIMANG75_SETVAL + 2)&& hx711 == &loadcell2)	// Xi mang
		{
			lcd_set_cursor(&lcd, 1,3);
			if (hx711->weight > 0)
				lcd_printf(&lcd, "%.0f", hx711->weight - 4);
			servo_close(&servo2);
			conveyor_start();
		}
		else if (hx711->weight == 0 && hx711 == &loadcell2 && ui8_xiMang75Count != 0)
		{
			lcd_set_cursor(&lcd, 1,3);
			lcd_printf(&lcd, "   ");
		}
		if (hx711->weight >= 0 && hx711 == &loadcell3)	// Ða
		{
	
		}
		if (hx711->weight >= (NUOC75_SETVAL + 2) && hx711 == &loadcell4)	// Nuoc
		{
			lcd_set_cursor(&lcd, 1,3);
			if (hx711->weight > 0)
				lcd_printf(&lcd, "%.0f", hx711->weight - 4);
			servo_close(&servo4);
			if (ui8_nuoc75Count == NUOC75_SETTIMES)
			{
				sys_state = finish_state;
				lcd_set_cursor(&lcd, 1, 0);
				lcd_printf(&lcd, "               ");
				lcd_set_cursor(&lcd, 1, 4);
				lcd_printf(&lcd, "FINISHED");
				ui8_nuoc75Count = 0;
			}
			conveyor_start();
		}
		else if (hx711->weight == 0 && hx711 == &loadcell4 && ui8_nuoc75Count != 0)
		{
			lcd_set_cursor(&lcd, 1,3);
			lcd_printf(&lcd, "   ");
		}
	}
	//   End mode MAC75 
	
	//    Mode MAC 200
	else if (op_state == mac200_st && sys_state != finish_state && sys_state != init_state) // MAC 200
	{
		if (hx711->weight >= (CAT200_SETVAL + 2) && hx711 == &loadcell1) // Cat
		{
			lcd_set_cursor(&lcd, 1,3);
			if (hx711->weight > 0)
				lcd_printf(&lcd, "%.0f", hx711->weight - 4);
			servo_close(&servo1);
			conveyor_start();
		}
		else if (hx711->weight == 0 && hx711 == &loadcell1 && ui8_cat200Count != 0)
		{
			lcd_set_cursor(&lcd, 1,3);
			lcd_printf(&lcd, "   ");
		}
		if (hx711->weight >= (XIMANG200_SETVAL + 2)&& hx711 == &loadcell2)	// Xi mang
		{
			lcd_set_cursor(&lcd, 1,3);
			if (hx711->weight > 0)
				lcd_printf(&lcd, "%.0f", hx711->weight - 4);
			servo_close(&servo2);
			conveyor_start();
		}
		else if (hx711->weight == 0 && hx711 == &loadcell2 && ui8_xiMang200Count != 0)
		{
			lcd_set_cursor(&lcd, 1,3);
			lcd_printf(&lcd, "   ");
		}
		if (hx711->weight >= (DA200_SETVAL + 2) && hx711 == &loadcell3)	// Ða
		{
			lcd_set_cursor(&lcd, 1,3);
			if (hx711->weight > 0)
				lcd_printf(&lcd, "%.0f", hx711->weight - 4);
			servo_close(&servo3);
			conveyor_start();
		}
		else if (hx711->weight == 0 && hx711 == &loadcell3 && ui8_da200Count != 0)
		{
			lcd_set_cursor(&lcd, 1,3);
			lcd_printf(&lcd, "   ");
		}
		if (hx711->weight >= (NUOC200_SETVAL + 2) && hx711 == &loadcell4)	// Nuoc
		{
			lcd_set_cursor(&lcd, 1,3);
			if (hx711->weight > 0)
				lcd_printf(&lcd, "%.0f", hx711->weight - 4);
			servo_close(&servo4);
			if (ui8_nuoc200Count == NUOC200_SETTIMES)
			{
				sys_state = finish_state;
				lcd_set_cursor(&lcd, 1, 0);
				lcd_printf(&lcd, "               ");
				lcd_set_cursor(&lcd, 1, 4);
				lcd_printf(&lcd, "FINISHED");
				ui8_nuoc200Count = 0;
			}
			conveyor_start();
		}
		else if (hx711->weight == 0 && hx711 == &loadcell4 && ui8_nuoc200Count != 0)
		{
			lcd_set_cursor(&lcd, 1,3);
			lcd_printf(&lcd, "   ");
		}
	}
		//   End mode MAC 200
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	
	/* Khoi tao LCD 16x2 */
	lcd_init(&lcd, &hi2c2, LCD_ADDR_DEFAULT);
	HAL_Delay(100);
	lcd_set_cursor(&lcd, 0, 2);
	lcd_printf(&lcd, "Press START");
	lcd_set_cursor(&lcd, 1,3);
	lcd_printf(&lcd, "........");
	while(ui8_start_var == 0)
	{
		__NOP;
	}
	op_state = start_st;
	lcd_clear_display(&lcd);
	lcd_set_cursor(&lcd, 0, 1);
	HAL_Delay(10);
	lcd_printf(&lcd, "Cabliration...");
	lcd_set_cursor(&lcd, 1, 1);
	lcd_printf(&lcd, "Initialize....");
	/* Khoi tao trang thai ban dau cua he thong can DL */
	sys_state = init_state;
	
	/* Khoi tao RC Servo */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Bat dau TB khoi chay he thong
	servo_init(&servo1, &htim1, TIM_CHANNEL_1);
	servo_init(&servo2, &htim1, TIM_CHANNEL_2);
	servo_init(&servo3, &htim1, TIM_CHANNEL_3);
	servo_init(&servo4, &htim1, TIM_CHANNEL_4);
	
	// Khoi tao PWM cho dong co bang chuyen
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	
	/* Khoi tao nut nhan */
	button_init(&cb1, GPIOA, GPIO_PIN_2);
	button_init(&cb2, GPIOA, GPIO_PIN_3);
	button_init(&cb3, GPIOA, GPIO_PIN_4);
	button_init(&cb4, GPIOA, GPIO_PIN_5);
	button_init(&btn_start, GPIOB, GPIO_PIN_13);
	button_init(&btn_stop, GPIOB, GPIO_PIN_14);
	button_init(&btn_mac75, GPIOB, GPIO_PIN_15);
	button_init(&btn_mac200, GPIOB, GPIO_PIN_12);
	/* Khoi tao trang thai ban dau RC Servo */
	HAL_Delay(500);
	servo_close(&servo1);
	HAL_Delay(500);
	servo_close(&servo2);
	HAL_Delay(500);
	servo_close(&servo3);
	HAL_Delay(500);
	servo_close(&servo4);
	HAL_Delay(200);
	
	/* Cac ham khoi tao Loadcell va cabliration */
	hx711_init(&loadcell1, CLK1_GPIO_Port, CLK1_Pin, DT1_GPIO_Port, DT1_Pin, &htim4);
	hx711_init(&loadcell2, CLK2_GPIO_Port, CLK2_Pin, DT2_GPIO_Port, DT2_Pin, &htim4);
	hx711_init(&loadcell3, CLK3_GPIO_Port, CLK3_Pin, DT3_GPIO_Port, DT3_Pin, &htim4);
	hx711_init(&loadcell4, CLK4_GPIO_Port, CLK4_Pin, DT4_GPIO_Port, DT4_Pin, &htim4);
	HAL_Delay(200); // Waiting complete cablration
	hx711_coef_set(&loadcell1, 1990); 	// 1kg
	hx711_coef_set(&loadcell2, 2260);		// 1kg
	hx711_coef_set(&loadcell3, 442.5);	// 5kg
	hx711_coef_set(&loadcell4, 1965);		// 1kg 
	HAL_Delay(200); // Waiting completely cablration
  hx711_tare(&loadcell1, 10);
	hx711_tare(&loadcell2, 10);
	hx711_tare(&loadcell3, 10);
	hx711_tare(&loadcell4, 10);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	// Ket thuc TB he thong da chay on dinh
	
	/* Khoi tao trang thai cat cua he thong can DL */
	sys_state = cat_state;
	op_state = init_st;
	conveyor_start();
	HAL_Delay(10);
	lcd_clear_display(&lcd);
	lcd_set_cursor(&lcd, 0, 0);
	lcd_printf(&lcd, "Complete Cablib");
	HAL_Delay(1000);
	lcd_clear_display(&lcd);
	lcd_set_cursor(&lcd, 0, 3);
	lcd_printf(&lcd, "Starting....");
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		// Cac ham polling de kiem tra cac cam bien quang
		button_handle(&cb1);
		button_handle(&cb2);
		button_handle(&cb3);
		button_handle(&cb4);
		// Cac ham polling de kiem tra nut nhan chon che do
		button_handle(&btn_start);
		button_handle(&btn_stop);
		button_handle(&btn_mac75);
		button_handle(&btn_mac200);
		// Cac ham polling de do can nang
		hx711_handle(&loadcell1);	
		hx711_handle(&loadcell2);
		hx711_handle(&loadcell3);
		hx711_handle(&loadcell4);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
void conveyor_start(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	TIM2->CCR1 = 700; // Duty cycle(%) = 70%
}

void conveyor_stop(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	TIM2->CCR1 = 1;
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
