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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iis3dwb_reg.h"
#include <stdio.h>
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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/** Please note that is MANDATORY: return 0 -> no Error.**/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);

void transmitSPIManual(uint8_t where_is_reg, uint8_t *rev_val, uint8_t size_);
void receiveSPIManual(uint8_t where_is_reg, uint8_t *rev_val, uint8_t size_);

int32_t iis3dwb_fifo_status_get(stmdev_ctx_t *ctx, iis3dwb_fifo_status_t *val);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void bytecpy(uint8_t *target, const uint8_t *source);

//void iis3dwb_fifo(void);
//stmdev_ctx_t dev_ctx;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BOOT_TIME         10
#define FIFO_WATERMARK    511

int a = 0;
//volatile uint8_t flag_tx = 0;
//volatile uint8_t flag_rx = 0;
#define SENSOR_BUS hspi2

//static int16_t data_raw_acceleration[3];
//static int16_t data_raw_temperature;
//static float acceleration_mg[3];
//static float temperature_degC;
static uint8_t whoamI;
iis3dwb_fifo_status2_t status;
//static uint8_t rst;

//static uint8_t scale_CTRL1 = 0xA8; // Accelerometer full-scale selection +- 4G
//static uint8_t scale_return_CTRL1 = 0x11;


//uint8_t scale_CTRL3 = 0x01; // Accelerometer full-scale selection +- 4G
//static uint8_t scale_return_CTRL3 = 0x11;

uint8_t val_return1 = 0x11;
uint8_t val_return2 = 0x22;

uint8_t transmit_val = 0x11;
uint8_t receive_val = 0x22;



iis3dwb_ctrl1_xl_t ctrl1_xl;
//static uint8_t keep_return = 0x30;

stmdev_ctx_t dev_ctx;
iis3dwb_fifo_status_t fifo_status;

static uint8_t tx_buffer[1000];

static iis3dwb_fifo_out_raw_t fifo_data[FIFO_WATERMARK];
static int16_t *datax;
static int16_t *datay;
static int16_t *dataz;
static int32_t *ts;

uint8_t tx_set_cs = 0;

uint8_t tx_buf_rmt[ 2 ] = {0x00, 0x00};
uint8_t rx_buf_rmt[ 2 ] = {0x0, 0x00};
iis3dwb_ctrl3_c_t ctrl3_c;

//uint8_t buff;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */
	//	stmdev_ctx_t dev_ctx;
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
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */
	//  stmdev_ctx_t dev_ctx;
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &SENSOR_BUS;
	//  dev_ctx.handle = &hspi2;
	//  stmdev_ctx_t dev_ctx;

	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	iis3dwb_device_id_get(&dev_ctx, &whoamI);
	while (whoamI != IIS3DWB_ID)
		{
	  iis3dwb_device_id_get(&dev_ctx, &whoamI);
	  a += 1;
	  HAL_Delay(50);
	}
	/*
	 *  Accelerometer full-scale selection
	 *	A = Enables accelerometer
	 *	8 = Selects accelerometer full-scale [+- 4G]
	 */
	transmit_val = 0xA8;
	transmitSPIManual(IIS3DWB_CTRL1_XL, &transmit_val, 1);

	/* Restore default configuration
	 * - Software reset at ctrl3_c
	 */

	transmit_val = 0x01;
	transmitSPIManual(IIS3DWB_CTRL3_C, &transmit_val, 1);

	/* Enable Block Data Update
	 * Set Block data update
	 */

	transmit_val = 0x44;
	transmitSPIManual(IIS3DWB_CTRL3_C, &transmit_val, 1);

	 /*
	  * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
	  * stored in FIFO) to FIFO_WATERMARK samples
	  * - iis3dwb_fifo_watermark_set
	  */
	transmit_val = (uint8_t)(0x00FFU & FIFO_WATERMARK);
	transmitSPIManual(IIS3DWB_FIFO_CTRL1, &transmit_val, 1);
	/////////////////////////////////////////////////////////
	transmit_val = (uint8_t)((0x0100U & FIFO_WATERMARK) >> 8);
	transmitSPIManual(IIS3DWB_FIFO_CTRL2, &transmit_val, 1);

	/* Set FIFO batch XL ODR to 12.5Hz
	 * - iis3dwb_fifo_xl_batch_set
	 */
	transmit_val = IIS3DWB_XL_BATCHED_AT_26k7Hz;
	transmitSPIManual(IIS3DWB_FIFO_CTRL3, &transmit_val, 1);

	/* Set FIFO mode to Stream mode (aka Continuous Mode) & iis3dwb_fifo_timestamp_batch_set
	 * - iis3dwb_fifo_mode_set
	 * - iis3dwb_fifo_timestamp_batch_set
	 */
	transmit_val = 0x86;
	transmitSPIManual(IIS3DWB_FIFO_CTRL4, &transmit_val, 1);

	/* Set Output Data Rate
	 * - iis3dwb_timestamp_set: Enables timestamp counter
	 */
	transmit_val = 0x20;
	transmitSPIManual(IIS3DWB_CTRL10_C, &transmit_val, 1);

//	receiveSPIManual(IIS3DWB_CTRL3_C, &receive_val, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  uint16_t num = 0, k;
	  /* Read watermark flag
	   * - iis3dwb_fifo_out_multi_raw_get
	   */
//	  iis3dwb_fifo_status_get(&dev_ctx, &fifo_status);
//	  uint8_t combine_val = 0x00;
	  receiveSPIManual(IIS3DWB_CTRL3_C, &receive_val, 1);
//	  combine_val = receive_val;

//	  receiveSPIManual(IIS3DWB_FIFO_STATUS2, &receive_val, 1);
//
//	  receive_val <<= 8;
//	  receive_val |= ( 0x0F &combine_val);
//	  uint8_t buff[2];
////	  iis3dwb_fifo_status2_t status;
//
//	  transmitSPIManual(IIS3DWB_FIFO_STATUS1, (uint8_t *)&buff[0], 2);
//	  bytecpy((uint8_t *)&status, &buff[1]);
//
//	  fifo_status.fifo_bdr = status.counter_bdr_ia;
//	  fifo_status.fifo_ovr = status.fifo_ovr_ia | status.fifo_ovr_latched;
//	  fifo_status.fifo_full = status.fifo_full_ia;
//	  fifo_status.fifo_th = status.fifo_wtm_ia;
//
//	  fifo_status.fifo_level = (uint16_t)buff[1] & 0x03U;
//	  fifo_status.fifo_level = (fifo_status.fifo_level * 256U) + buff[0];


//	  if (fifo_status.fifo_th == 1) {
//		  num = fifo_status.fifo_level;
//
//		 /* read out all FIFO entries in a single read */
//		 iis3dwb_fifo_out_multi_raw_get(&dev_ctx, fifo_data, num);
//
//		 for (k = 0; k < num; k++) {
//			 iis3dwb_fifo_out_raw_t *f_data;
//
//			 f_data = &fifo_data[k];
//			 /* Read FIFO sensor value */
//			 datax = (int16_t *)&f_data->data[0];
//			 datay = (int16_t *)&f_data->data[1];
//			 dataz = (int16_t *)&f_data->data[2];
//			 ts = (int32_t *)&f_data->data[0];
//
//			 switch (f_data->tag >> 3) {
//			 case IIS3DWB_XL_TAG: //2
//				 sprintf((char *)tx_buffer, "%d: ACC [mg]:\t%4.2f\t%4.2f\t%4.2f\r\n",
//						 k,
//						 iis3dwb_from_fs8g_to_mg(*datax),
//						 iis3dwb_from_fs8g_to_mg(*datay),
//						 iis3dwb_from_fs8g_to_mg(*dataz));
//
//				 tx_com(tx_buffer, strlen((char const *)tx_buffer));
//				 break;
//
//			case IIS3DWB_TIMESTAMP_TAG: //4
//				 sprintf((char *)tx_buffer, "%d TIMESTAMP [ms] %d\r\n", (uint16_t)k, (uint16_t)*ts);
//				 tx_com(tx_buffer, strlen((char const *)tx_buffer));
//				 break;
//			default:
//				 break;
//				}
//			 }
//	     sprintf((char *)tx_buffer, "------ \r\n\r\n");
//	     tx_com(tx_buffer, strlen((char const *)tx_buffer));
//		 }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SSC_GPIO_Port, SSC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : SSC_Pin */
  GPIO_InitStruct.Pin = SSC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SSC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	  tx_buf_rmt[ 0 ] = reg;
	  tx_buf_rmt[ 1 ] = *bufp;
	  tx_set_cs = 1;

	  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, RESET);
	  HAL_SPI_Transmit_IT(handle, tx_buf_rmt, len+1);

	  return 0;
}
/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	  tx_buf_rmt[ 0 ] = reg;
	  tx_buf_rmt[ 0 ] |= 0x80;
	  tx_buf_rmt[ 1 ] = 0x00;

	  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit_IT(handle, tx_buf_rmt, 2);
	  HAL_SPI_Receive_IT(handle, bufp, len);

	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  return 0;
}
/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}
/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
//	HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), len);
	HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

///* Main Example --------------------------------------------------------------*/
//void iis3dwb_fifo(void)
//{
////  iis3dwb_fifo_status_t fifo_status;
////  stmdev_ctx_t dev_ctx;
////  uint8_t rst;
//
//  /* Uncomment to configure INT 1 */
//  //iis3dwb_pin_int1_route_t int1_route;
//  /* Uncomment to configure INT 2 */
//  //iis3dwb_pin_int2_route_t int2_route;
//  /* Initialize mems driver interface */
////  dev_ctx.write_reg = platform_write;
////  dev_ctx.read_reg = platform_read;
////  dev_ctx.handle = &SENSOR_BUS;
//  /* Init test platform */
////  platform_init();
//  /* Wait sensor boot time */
////  platform_delay(BOOT_TIME);
//
//  /* Check device ID */
////  iis3dwb_device_id_get(&dev_ctx, &whoamI);
////
////  if (whoamI != IIS3DWB_ID)
////    while (1);
//
//  /* Restore default configuration */
////  iis3dwb_reset_set(&dev_ctx, PROPERTY_ENABLE);
//
//  do {
//    iis3dwb_reset_get(&dev_ctx, &rst);
//  } while (rst);
//
//  /* Enable Block Data Update */
////  iis3dwb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
//  /* Set full scale */
////  iis3dwb_xl_full_scale_set(&dev_ctx, IIS3DWB_8g);
//
//  /*
//   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
//   * stored in FIFO) to FIFO_WATERMARK samples
//   */
//  iis3dwb_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);
//  /* Set FIFO batch XL ODR to 12.5Hz */
//  iis3dwb_fifo_xl_batch_set(&dev_ctx, IIS3DWB_XL_BATCHED_AT_26k7Hz);
//
//  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
//  iis3dwb_fifo_mode_set(&dev_ctx, IIS3DWB_STREAM_MODE);
//
//  /* Set Output Data Rate */
//  iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_26k7Hz);
//  iis3dwb_fifo_timestamp_batch_set(&dev_ctx, IIS3DWB_DEC_8);
//  iis3dwb_timestamp_set(&dev_ctx, PROPERTY_ENABLE);
//
//  /* Wait samples */
//  while (1) {
//    uint16_t num = 0, k;
//    /* Read watermark flag */
//    iis3dwb_fifo_status_get(&dev_ctx, &fifo_status);
//
//    if (fifo_status.fifo_th == 1) {
//      num = fifo_status.fifo_level;
//      sprintf((char *)tx_buffer, "-- FIFO num %d \r\n", num);
//      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//
//      /* read out all FIFO entries in a single read */
//      iis3dwb_fifo_out_multi_raw_get(&dev_ctx, fifo_data, num);
//
//      for (k = 0; k < num; k++) {
//        iis3dwb_fifo_out_raw_t *f_data;
//
//        /* print out first two and last two FIFO entries only */
////        if (k > 7 && k < num - 8)
////          continue;
//
//        f_data = &fifo_data[k];
//
//        /* Read FIFO sensor value */
//        datax = (int16_t *)&f_data->data[0];
//        datay = (int16_t *)&f_data->data[2];
//        dataz = (int16_t *)&f_data->data[4];
//        ts = (int32_t *)&f_data->data[0];
//
//        switch (f_data->tag >> 3) {
//        case IIS3DWB_XL_TAG: //2
//          sprintf((char *)tx_buffer, "%d: ACC [mg]:\t%4.2f\t%4.2f\t%4.2f\r\n",
//                  k,
//                  iis3dwb_from_fs8g_to_mg(*datax),
//                  iis3dwb_from_fs8g_to_mg(*datay),
//                  iis3dwb_from_fs8g_to_mg(*dataz));
//          tx_com(tx_buffer, strlen((char const *)tx_buffer));
//          break;
//        case IIS3DWB_TIMESTAMP_TAG: //4
//          sprintf((char *)tx_buffer, "%d TIMESTAMP [ms] %d\r\n", (uint16_t)k, (uint16_t)*ts);
//          tx_com(tx_buffer, strlen((char const *)tx_buffer));
//          break;
//        default:
////		sprintf((char *)tx_buffer, "Tag %x\r\n", f_data->tag >> 3);
////		tx_com(tx_buffer, strlen((char const *)tx_buffer));
//          break;
//        }
//      }
//
//      sprintf((char *)tx_buffer, "------ \r\n\r\n");
//      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//    }
//  }
//}
//
//void iis3dwb_read_data_polling(void)
//{
////  stmdev_ctx_t dev_ctx;
//  /* Initialize mems driver interface */
////  dev_ctx.write_reg = platform_write;
////  dev_ctx.read_reg = platform_read;
////  dev_ctx.handle = &SENSOR_BUS;
//  /* Wait sensor boot time */
////  platform_delay(BOOT_TIME);
//  /* Check device ID */
////  iis3dwb_device_id_get(&dev_ctx, &whoamI);
//
////  if (whoamI != IIS3DWB_ID)
////    while (1);
//
//  /* Restore default configuration */
////  iis3dwb_reset_set(&dev_ctx, PROPERTY_ENABLE);
//
//  do {
//    iis3dwb_reset_get(&dev_ctx, &rst);
//  } while (rst);
//
//  /* Enable Block Data Update */
////  iis3dwb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
//  /* Set Output Data Rate */
//  iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_26k7Hz);
//  /* Set full scale */
//  iis3dwb_xl_full_scale_set(&dev_ctx, IIS3DWB_2g);
//  /* Configure filtering chain(No aux interface)
//   * Accelerometer low pass filter path
//   */
//  iis3dwb_xl_filt_path_on_out_set(&dev_ctx, IIS3DWB_LP_ODR_DIV_100);
//
//
//  /* Read samples in polling mode (no int) */
//  while (1) {
//    uint8_t reg;
//    /* Read output only if new xl value is available */
//    iis3dwb_xl_flag_data_ready_get(&dev_ctx, &reg);
//
//    if (reg) {
//      /* Read acceleration field data */
//      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
//      iis3dwb_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
//      acceleration_mg[0] =
//        iis3dwb_from_fs2g_to_mg(data_raw_acceleration[0]);
//      acceleration_mg[1] =
//        iis3dwb_from_fs2g_to_mg(data_raw_acceleration[1]);
//      acceleration_mg[2] =
//        iis3dwb_from_fs2g_to_mg(data_raw_acceleration[2]);
////      sprintf((char *)tx_buffer,
////              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
////              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
////      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//
////      sprintf((char *)tx_buffer,
////              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
////              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
////      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//
////      HAL_UART_Transmit(&huart2, tx_buffer, strlen((char const *)tx_buffer), 1000);
//      sprintf((char *)tx_buffer,
//              "%4.2f,%4.2f,%4.2f\r\n",
//              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
//      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//    }
//
//    iis3dwb_temp_flag_data_ready_get(&dev_ctx, &reg);
//
//
//    if (reg) {
//      /* Read temperature data */
//      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
//      iis3dwb_temperature_raw_get(&dev_ctx, &data_raw_temperature);
//      temperature_degC = iis3dwb_from_lsb_to_celsius(data_raw_temperature);
//      sprintf((char *)tx_buffer,
//              "TEMP [degC]:%6.2f\r\n", temperature_degC);
//      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//    }
//  }
//}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{

	if(hspi->Instance == hspi2.Instance)
		 {
			if(tx_set_cs){
				HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
				tx_set_cs = 0;
			}
		 }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{

	if(hspi->Instance == hspi2.Instance)
	 {
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
//		Rxflag = 1;
	 }
}

void transmitSPIManual(uint8_t where_is_reg, uint8_t *rev_val, uint8_t size_){

   iis3dwb_write_reg(&dev_ctx, where_is_reg, rev_val, size_);
}

void receiveSPIManual(uint8_t where_is_reg, uint8_t *rev_val, uint8_t size_){
   iis3dwb_read_reg(&dev_ctx, where_is_reg, rev_val, size_);
}

static void bytecpy(uint8_t *target, const uint8_t *source)
{
  if ((target != NULL) && (source != NULL))
  {
    *target = *source;
  }
}

int32_t iis3dwb_fifo_status_get(stmdev_ctx_t *ctx, iis3dwb_fifo_status_t *val)
{
  uint8_t buff[2];
  iis3dwb_fifo_status2_t status;

  receiveSPIManual(IIS3DWB_FIFO_STATUS1, (uint8_t *)&buff[0], 2);
  bytecpy((uint8_t *)&status, &buff[1]);

  val->fifo_bdr = status.counter_bdr_ia;
  val->fifo_ovr = status.fifo_ovr_ia | status.fifo_ovr_latched;
  val->fifo_full = status.fifo_full_ia;
  val->fifo_th = status.fifo_wtm_ia;

  val->fifo_level = (uint16_t)buff[1] & 0x03U;
  val->fifo_level = (val->fifo_level * 256U) + buff[0];

  return 0;
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
