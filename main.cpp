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
#include <iostream>
#include <string>
#include <string.h>
#include <vector>
using namespace std;


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_ADDR (0x27 << 1)       // адрес дисплея, сдвинутый на 1 бит влево (HAL работает с I2C-адресами, сдвинутыми на 1 бит влево)

#define PIN_RS    (1 << 0)         // если на ножке 0, данные воспринимаются как команда, если 1 - как символы для вывода
#define PIN_EN    (1 << 2)         // бит, по изменению сост. которого считывается информация
#define BACKLIGHT (1 << 3)         // управление подсветкой

#define LCD_DELAY_MS 5             // пауза перед высвечиванием символа
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// функция для отправки данных, data - сами данные, flags - 1 (отправка данных) или 0 (отправка команд)
void I2C_send(uint8_t data, uint8_t flags)
{
	HAL_StatusTypeDef res;
	    for(;;) {                                                                     // бесконечный цикл
	        res = HAL_I2C_IsDeviceReady(&hi2c1, LCD_ADDR, 1, HAL_MAX_DELAY);          // проверяем, готово ли устройство по адресу lcd_addr для связи
	        if(res == HAL_OK) break;                                                  // если да, то выходим из бесконечного цикла
	    }

	uint8_t up = data & 0xF0;                 // операция И с 1111 0000, приводит к обнулению последних бит с 0 по 3, остаются биты с 4 по 7
	uint8_t lo = (data << 4) & 0xF0;          // тоже самое, но data сдвигается на 4 бита влево, т.е. в этой
	                                           // переменной остаются  биты с 0 по 3
	uint8_t data_arr[4];
	data_arr[0] = up|flags|BACKLIGHT|PIN_EN;  // 4-7 биты содержат информацию, биты 0-3 конфигурируют работу
	data_arr[1] = up|flags|BACKLIGHT;         // ублирование сигнала, на выводе Е в этот раз 0
	data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
	data_arr[3] = lo|flags|BACKLIGHT;

	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
	HAL_Delay(LCD_DELAY_MS);
}


void LCD_SendString(char *str)
{
    // *char по сути является строкой
	while(*str) {                                   // пока строчка не закончится
		I2C_send((uint8_t)(*str), 1);               // передача первого символа строки
        str++;                                      // сдвиг строки налево на 1 символ
    }
}


//парсер. Пока void, но позже над этим подумаю
vector<string> parser(std::string pre_parser)
{
	std::vector<std::string> after_parser;
	std::vector<std::string> list_command;
	/*не забыть переделать этот блок. Сделать поиск не через предварительно записынные команды,
	 * а именно через парсинг команды, чтобы и не дублировать код, и уменьшить его количество*/
	list_command.push_back("print_str");
	list_command.push_back("print_arr");
	list_command.push_back("sum");
	list_command.push_back("diff");

	//поиск команды в введенной строке (че надо сделать: сложить, вычесть, напечатать)
	for (int i = 0; i < list_command.size(); i++)
	{
		if ( pre_parser.find(list_command[i]) )
		{
			after_parser.push_back(list_command[i]);

			//выборка значений указанных в скобках
				for (int j = 0; j < pre_parser.size(); j++)
				{
					if(pre_parser[j] == '(')
					{
						string source = pre_parser.substr(j+1, pre_parser.size() - (j-1));
						char *s = new char[source.size() + 1];
						strcpy(s, source.c_str());
						char *splited_data = strtok(s, ", ");
						while (splited_data != NULL)
						{
							after_parser.push_back(splited_data);
							splited_data = strtok(NULL, ", ");
						}
						delete[] s;
						break;
					}
				}
			break;
		}
	}

return after_parser;

}

//функция, выводящая на дисплей строку
void print_str(vector<string> parsered_data)
{
    I2C_send(0b00110000,0);   // 8ми битный интерфейс
    I2C_send(0b00000010,0);   // установка курсора в начале строки
    I2C_send(0b00001100,0);   // нормальный режим работы
    I2C_send(0b00000001,0);   // очистка дисплея

    char *s = new char[parsered_data[2].size() + 1];
	strcpy(s, parsered_data[2].c_str());
	//delete[] s;

    I2C_send(0b10000000,0);   // переход на 1 строку, тут не обязателен
    LCD_SendString(s);
}

//функция, выводящая число, записанное в указанной ячейке массива
void print_arr(vector<string> parsered_data)
{

}

//функция, находящая сумму и записывающая ее в массив
int sum(vector<string> parsered_data)
//void sum(int num_cell, double a, double b)
{

}

//функция, находящая разность и записывающая ее в массив
int diff(vector<string> parsered_data)
//void diff(int num_cell, double a, double b)
{

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


    I2C_send(0b00110000,0);   // 8ми битный интерфейс
    I2C_send(0b00000010,0);   // установка курсора в начале строки
    I2C_send(0b00001100,0);   // нормальный режим работы
    I2C_send(0b00000001,0);   // очистка дисплея

    //Текст заставки. Сделано по большей части потому что могу и так как то прикольнее
    I2C_send(0b10000000,0);   // переход на 1 строку, тут не обязателен
    LCD_SendString("Hello Everybody!");
    I2C_send(0b11000000,0);   // переход на 2 строку
    LCD_SendString("Welcome to the");
    I2C_send(0b10010100,0);   // переход на 3 строку
    LCD_SendString("Input - Output sys.");
    I2C_send(0b11010100,0);   // переход на 4 строку
    LCD_SendString("-enter the command-");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    char input_data_read[1];
	char input_data[100];
    int i;
    std::vector<std::string> parsered_data;

    memset(input_data, 0, sizeof(input_data)); //уж лучше путь нули точно во всей строке будут, чем что то не сильно понятное


    while (1)
    {
    	//цикл, осуществляющий запись и вывод данных
    	for(i = 0;; i++)
    	{
    		//если данные отпралены, то они все считаются и запищутся в новую переменную
			if( HAL_UART_Receive(&huart2, (uint8_t *)input_data_read, 1, 10) == HAL_OK )
			{
				input_data[i] = input_data_read[0]; //запись полученных данных в новую переменную
			}
			//вызввается, если считывание данных не прошло
			else
			{
				//если данные завершатся указанным знаком, то откроется режим вывода на дисплей
				if(input_data[i - 1] == ')')
				{
					parsered_data = parser(input_data);
					//пока так. Надо подумать как через switch case сделать в идеале
					print_str(parsered_data);
					if ( parsered_data[0] == "print_arr" ) print_str(parsered_data);
					if ( parsered_data[0] == "sum" ) print_str(parsered_data);
					if ( parsered_data[0] == "diff" ) print_str(parsered_data);


					HAL_UART_Transmit(&huart2, (uint8_t *)input_data, i, 3); //чисто для проверки. Удалить
					/*I2C_send(0b00110000,0);   // 8ми битный интерфейс
					I2C_send(0b00000010,0);   // установка курсора в начале строки
					I2C_send(0b00001100,0);   // нормальный режим работы
					I2C_send(0b00000001,0);   // очистка дисплея

					I2C_send(0b10000000,0);   // переход на 1 строку, тут не обязателен
					LCD_SendString("some string");/*
					I2C_send(0b11000000,0);   // переход на 2 строку
					LCD_SendString((char *)input_data);*/
					memset(input_data, 0, sizeof(input_data));
					/*выше была произведена змена всех символов нулями. Это сделано для двух целей
					 * 1. подмена завершающего символа, чтобы закрыть возможность входа в это условие и надпись осталась
					 * 2. затирание лишней информации поступающей на выход, когда последующая строка меньше предыдущей
					 * возможно потом откажусь от этого решения */
					//input_data[i - 1] = 0; //подмена завершающего символа, чтобы закрыть возможность входа в это условие и надпись осталась
				}
				break;
			}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
