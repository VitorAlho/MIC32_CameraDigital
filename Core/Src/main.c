/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//Bibliotecas do LCD
#include "fonts.h"
#include "tft.h"
#include "user_setting.h"
#include "functions.h"
//Biblioteca de ajuste de funções para o SD Card (criada por terceiro)
#include "fatfs_sd.h"
#include "file.h"
#include <string.h>
#include <stdio.h>
//Biblioteca da câmera OV7670
#include "Camera_OV7670.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define RGB		//Definido em Camera_OV7670.h
//#define W320H240	//Definido em Camera_OV7670.h

#ifdef W320H240
  #define W 320
  #define H 240
#else
  #define W 160
  #define H 120
#endif

typedef struct{
	uint32_t id;
	uint32_t width;
	uint32_t height;
}Display;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

Display displayTFT;

//uint16_t pixelLineVector[W];

//Manipulação de arquivos via SD Card

FATFS fs0, fs1;					/* Work area (filesystem object) for logical drives */

FIL fsrc, fdst;					/* File objects */

BYTE buffer[4096];   			/* File copy buffer */

fileErrorCode fr;						/* FatFs function common result code */

UINT br, bw;					/* File read/write count */

//Transferência de arquivos BMP do SD Card para o LCD

uint32_t first=100;

//Flag do Botão CAPTURAR

uint8_t flagBotaoCapturar;

uint8_t oneTime = 0;

uint8_t ultimaFotoTirada = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint16_t geraRGB565(uint8_t red, uint8_t green, uint8_t blue);
void LCD_TxBMP(unsigned char* data, unsigned int BitmapStart);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void exibirFrameNoDisplay ( void ) {

	if( W==320 ){

		setAddrWindow( 0, 0, W, H );

	}
	else{

		setAddrWindow( 80, 60, W+80-1, H+60-1 );

	}

	inicioDados();  // Habilita o CHIP SELECT do display TFT

	captureImg( displayTFT.width, displayTFT.height, &huart2);

	fimDados();		// Desliga o CHIP SELECT do display TFT

}

uint8_t initSave = 0;

void piscaTelaDoDisplay ( uint8_t color ) {

	fillScreen(color);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	int32_t i;

	displayTFT.width = W;

	displayTFT.height = H;

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  //Sequência de inicialização do LCD

  tft_gpio_init();						//Inicializa os GPIOs do LCD (evita uso do CubeMX)

  HAL_TIM_Base_Start( &htim1 );			//Inicializa o Timer1 (base de tempo de us do LCD)

  displayTFT.id = tft_readID();			//Lê o ID do LCD (poderia ser chamada pela inicialização do LCD)

  HAL_Delay( 100 );

  tft_init ( displayTFT.id );			//Inicializa o LCD de acordo com seu ID

  setRotation( 3 );						//Ajusta a orientação da tela

  fillScreen( BLACK );					//Preenche a tela em uma só cor

  //Mensagem inicial

  HAL_UART_Transmit( &huart2, (uint8_t *)"Este programa recebe os pixels de uma camera OV7670 conectada\r\n", 66, 100 );

  HAL_UART_Transmit( &huart2, (uint8_t *)"nos GPIOs e envia para display LCD TFT                       \r\n", 66, 100 );

  //Plota molduras no LCD

  for( i=0; i<8; i++ )
  {

	  drawRect( i * 20, 					// X
			  	i * 15, 					// Y
			    320 - (2 * ( i * 20 ) ), 	// W
				240 - ( 2 * ( i * 15 ) ), 	// H
				WHITE ); 					// Collor

	  HAL_Delay( 100 );

  }

  //Liga o PWM

  HAL_TIM_PWM_Start( &htim3, TIM_CHANNEL_1 );

  htim3.Instance->CCR1 = 2;	//Define o duty_cycle em aprox. 50%

  HAL_Delay( 1000 );

  //Configura a câmera OV7670

  setup( &hi2c1, &huart2 );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /*
   * Função de teste da biblioteca file.c, testa se o FATFS foi implementado corretamente.
   */

  //testFileLibrary(&huart2);

  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  exibirFrameNoDisplay();

	  if( flagBotaoCapturar == PRESSIONADO ) {

		  HAL_Delay(100);

		  captureAndSaveImg(W, H, &huart2);

		  oneTime = 0;

		  flagBotaoCapturar = SOLTO;

		  piscaTelaDoDisplay(BLACK);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : OV7670_HREF_Pin OV7670_PCLK_Pin OV7670_VSYNC_Pin OV7670_D0_Pin
                           OV7670_D1_Pin OV7670_D2_Pin OV7670_D3_Pin OV7670_D4_Pin
                           OV7670_D5_Pin OV7670_D6_Pin OV7670_D7_Pin */
  GPIO_InitStruct.Pin = OV7670_HREF_Pin|OV7670_PCLK_Pin|OV7670_VSYNC_Pin|OV7670_D0_Pin
                          |OV7670_D1_Pin|OV7670_D2_Pin|OV7670_D3_Pin|OV7670_D4_Pin
                          |OV7670_D5_Pin|OV7670_D6_Pin|OV7670_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA8
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB3 PB4
                           PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CAPTURAR_Pin */
  GPIO_InitStruct.Pin = CAPTURAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CAPTURAR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	uint8_t i = 0;

	if (GPIO_Pin == CAPTURAR_Pin){

		if(HAL_GPIO_ReadPin(CAPTURAR_GPIO_Port, CAPTURAR_Pin) == RESET){

			for(i = 0; i <= 254; i++){

			}

			if(HAL_GPIO_ReadPin(CAPTURAR_GPIO_Port, CAPTURAR_Pin) == RESET){

				flagBotaoCapturar = PRESSIONADO;
			}
		}

	}

}

//Gera uma cor no formato RGB 565

uint16_t geraRGB565( uint8_t red, uint8_t green, uint8_t blue )
{

	//rrrrrggg gggbbbbb

	return ( ( red << 8 ) & 0xF800 ) + ( ( green << 5 ) & 0x07E0 ) + ( ( blue >>3 ) & 0x001F );

}

//Envia uma string como um bitmp para o LCD TFT
//Recebe: String com 512 bytes (1 setor de arquivo) que fazem parte da imagem
//				Flag que sinaliza se é o primeiro setor (onde está o cabeçalho)
void LCD_TxBMP(unsigned char* data, unsigned int BitmapStart)
{
	static unsigned int tamanho = 0;
	static unsigned char sobrou[3] = {0,0,0};
	unsigned int i = 0;
	//unsigned int setor = 512;
	unsigned short int cor;
	//unsigned char tam[11];
	unsigned char altura, x = 0, y = 127, bits_por_pixel;
	static unsigned char largura = 0, erro_bits = 0, bytes_extras = 0, pixels_por_linha = 0;

	//Se é o primeiro setor do arquivo, possui o cabeçalho
	if(BitmapStart)
	{
		BitmapStart = 0;

		//Reseta variável estática
		pixels_por_linha = 0;

		//Pula o cabeçalho
		i = 54;
		//Lê o tamanho da área de dados do arquivo em bytes
		tamanho = data[0x22] + (unsigned int)(data[0x23]<<8) + (unsigned int)(data[0x24]<<16) + (unsigned int)(data[0x25]<<24);
		//Leandro (01/09/2019) - Lê a largura e a altura da imagem para definir o tamanho da janela
		largura = data[18];
		altura = data[22];
		//Configura a janela
		setAddrWindow(x, y-altura+1, x+largura-1, y);
		//Envia para o LCD sinalização de início de envio de dados
		inicioDados();
		//Verifica se existirão bytes extras no arquivo em função da largura da imagem
		//Obervação: Existe uma restrição de que cada linha deva ter N bytes, sendo N um número
		//divisível por 4. Caso contrário, o BMP deve ser preenchido com bytes não válidos. Por
		//exemplo, se a imagem tem 1 x 100 pixels em 24 bits/pixel, o BMP teria 3 bytes válidos em
		//cada linha e mais 1 byte que não tem qualquer significado.
		switch((largura*3)%4)
		{
			case 1: bytes_extras = 3; break;
			case 2: bytes_extras = 2; break;
			case 3: bytes_extras = 1; break;
			default: bytes_extras = 0; break;
		}

		//Lê a quantidade de bits por pixel (neste caso é aceito apenas 24 bits por pixel)
		bits_por_pixel = data[28];
		//Testa a quatidade de bits
		if((bits_por_pixel != 24) || (largura > 128) || (altura > 128))
			erro_bits = 1;
		else
			erro_bits = 0;
	}

	//Se houver erro na quantidade de bits retorna e não envia para o LCD
	if(erro_bits)
	{
		return;
	}

	//Envia os pixels enquanto não acabar o setor ou o Bitmap
	while((i <= (512-3)) && (tamanho >= 3))	//24 bits por pixels
	{
		//Se completou uma linha
		if(pixels_por_linha == largura)
		{
			//Zera o contador
			pixels_por_linha = 0;
			//Verifica se tem bytes nulos para ignorar
			if(bytes_extras >= sobrou[0])
			{
				//Desconta os bytes_extras-sobrou[0] do tamanho do setor
				tamanho -= (bytes_extras-sobrou[0]);
				//Incrementa a posição do byte a ser lido do setor
				i += (bytes_extras-sobrou[0]);
				//Atualiza o valor da sobra
				sobrou[0] = 0;
				//Verifica se não cabe mais nenhum pixel, encerra o loop
				if((i>(512-3)) || (tamanho < 3))
					break;
			}
			else
			{
				//Atualiza o valor da sobra
				sobrou[0] -= bytes_extras;
			}
			//break;
			if(tamanho<3)
				break;
		}

		if(sobrou[0] == 0)			//Tamanho -= 3
		{
			//((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
			//Seguencia BGR (24 bits) --> RGB (565)
			cor = (data[i] >> 3) | ((data[i+1] & 0xFC) << 3) | ((data[i+2] & 0xF8) << 8);
			i += 3;
			tamanho -= 3;
		}
		else if(sobrou[0] == 1)	//Tamanho -= 2
		{
			//Sobrou a cor Azul
			cor = (sobrou[2] >> 3) | ((data[i] & 0xFC) << 3) | ((data[i+1] & 0xF8) << 8);
			i += 2;
			tamanho -= 2;
		}
		else if(sobrou[0] == 2)	//Tamanho -= 1
		{
			//Sobrou a cor Azul e Verde
			cor = (sobrou[1] >> 3) | ((sobrou[2] & 0xFC) << 3) | ((data[i] & 0xF8) << 8);
			i += 1;
			tamanho -= 1;
		}
		else
		{
			i = 512;
			//setor = 0;
			tamanho = 0;
			break;
		}
		//Envia pixel 565 para o LCD
		desenhaPixel(cor);

		sobrou[0] = 0;	//Sobra algum byte apenas no final do setor (i>= 510)

		//Incrementa o número de pixels enviados por linha e testa
		pixels_por_linha++;
	}
	//Se ainda não acabou o arquivo
	if(tamanho >= 3)
	{
		//Salva o número de bytes que sobreram para formar um pixels
		sobrou[0] = 512 - i;
		//Completa os 512 bytes do setor
		tamanho -= sobrou[0];
		//Salva o penúltimo byte
		sobrou[1] = data[510];
		//Salva o último byte
		sobrou[2] = data[511];
	}
	else
	{
		//Envia para o LCD sinalização de fim de envio de dados
		fimDados();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
