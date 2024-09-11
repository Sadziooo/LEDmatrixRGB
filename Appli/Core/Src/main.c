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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Matrix dimensions
#define MATRIX_WIDTH  64
#define MATRIX_HEIGHT 32

// Color channel indices
#define RED    0
#define GREEN  1
#define BLUE   2

#define BCM_BITS 8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// lines steering
#define RGB_R1(value)  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, (value) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define RGB_G1(value)  HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, (value) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define RGB_B1(value)  HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, (value) ? GPIO_PIN_SET : GPIO_PIN_RESET)

#define RGB_R2(value)  HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, (value) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define RGB_G2(value)  HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, (value) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define RGB_B2(value)  HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, (value) ? GPIO_PIN_SET : GPIO_PIN_RESET)

#define RGB_A(value)   HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, (value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define RGB_B(value)   HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, (value & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define RGB_C(value)   HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, (value & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define RGB_D(value)   HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, (value & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET)

#define RGB_CLK(value) HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, (value) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define RGB_LAT(value) HAL_GPIO_WritePin(LAT_GPIO_Port, LAT_Pin, (value) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define RGB_OE(value)  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, (value) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

// Image Buffer
uint8_t ImageBuffer[3*MATRIX_HEIGHT*MATRIX_WIDTH]; // 3 colors

uint8_t *Image;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void DWT_Init(void);
void DelayUs(uint32_t us);

void HUB75_Init(void);
void HUB75_SendRowData(void);
void Paint_NewImage(uint8_t image[]);
void DrawPixel(uint16_t Xpoint, uint16_t Ypoint, uint8_t R, uint8_t G, uint8_t B);
void DrawRectangle(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint8_t R, uint8_t G, uint8_t B);

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

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Update SystemCoreClock variable according to RCC registers values. */
  SystemCoreClockUpdate();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  DWT_Init();
  HUB75_Init();

//  DrawPixel(15, 20, 255, 0, 0);
//  DrawPixel(15, 21, 0, 255, 0);
//  DrawPixel(15, 22, 0, 0, 255);
//
//  DrawPixel(60, 10, 1, 1, 1);
//  DrawPixel(61, 10, 127, 127, 127);
//  DrawPixel(62, 10, 255, 255, 255);
//
//  DrawPixel(60, 25, 1, 0, 0);
//  DrawPixel(61, 25, 127, 0, 0);
//  DrawPixel(62, 25, 255, 0, 0);
//
//  DrawPixel(60, 22, 0, 0, 1);
//  DrawPixel(61, 22, 0, 0, 127);
//  DrawPixel(62, 22, 0, 0, 255);
//
//  DrawPixel(60, 19, 0, 1, 0);
//  DrawPixel(61, 19, 0, 127, 0);
//  DrawPixel(62, 19, 0, 255, 0);

//  DrawRectangle(0, 0, 63, 31, 255, 255, 255);
  DrawRectangle(10, 13, 14, 17, 255, 165, 0);
  DrawRectangle(15, 13, 19, 17, 120, 255, 10);
  DrawRectangle(20, 13, 24, 17, 255, 10, 100);
  DrawRectangle(25, 13, 29, 17, 10, 130, 120);
  DrawRectangle(30, 13, 34, 17, 255, 0, 0);
  DrawRectangle(35, 13, 39, 17, 0, 255, 0);
  DrawRectangle(40, 13, 44, 17, 0, 0, 255);
  DrawRectangle(45, 13, 49, 17, 255, 255, 255);
  DrawRectangle(50, 13, 54, 17, 50, 50, 50);
  DrawRectangle(55, 13, 59, 17, 1, 1, 1);


//  DrawRectangle(1, 1, 5, 5, 255, 255, 255);

//  DrawRectangle(1, 30, 5, 31, 255, 255, 255);
//  DrawRectangle(1, 15, 5, 15, 255, 255, 255);

//  DrawRectangle(59, 27, 63, 31, 255, 255, 255);



  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_TogglePin(LD1_GPIO_PORT, LD1_PIN);
//	  for (uint16_t row = 0; row < MATRIX_HEIGHT; row++) {
//		  for (uint16_t col = 0; col < MATRIX_WIDTH; col++) {
//			  DrawPixel(col, row, 255, 255, 255); // White color
//		  }
//
//		  HUB75_SendRowData(); // Refresh display
//		  memset(ImageBuffer, 0, 3*MATRIX_HEIGHT*MATRIX_WIDTH);
//		  HAL_Delay(100);
//	  }

	  HUB75_SendRowData(); // Refresh display
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

void HUB75_Init(void) {
    // Reset all control lines initially
    RGB_OE(0);  // Disable output
    RGB_LAT(0); // Latch low
    RGB_CLK(0); // Clock low

    Paint_NewImage(ImageBuffer);
}

uint16_t base_delay = 5;

void HUB75_SendRowData(void) {
    // Iterate over the bit planes from LSB to MSB
    for (uint8_t bit = 0; bit < BCM_BITS; bit++) {
    	uint32_t delay_time = base_delay * (1 << bit); // Delay time doubles with each bit significance

        for (uint16_t row = 0; row < MATRIX_HEIGHT / 2; row++) {

            RGB_A(row);
			RGB_B(row);
			RGB_C(row);
			RGB_D(row);

            for (uint16_t col = 0; col < MATRIX_WIDTH; col++) {
                uint32_t index_upper = (MATRIX_WIDTH - col - 1) + ((MATRIX_HEIGHT - row - 1) * MATRIX_WIDTH);
                index_upper *= 3; // 3 bytes per pixel (R, G, B)

                uint8_t red1 = ImageBuffer[index_upper];
                uint8_t green1 = ImageBuffer[index_upper + 1];
                uint8_t blue1 = ImageBuffer[index_upper + 2];

                uint32_t index_lower = (MATRIX_WIDTH - col - 1) + ((MATRIX_HEIGHT - (row + 16) - 1) * MATRIX_WIDTH);
                index_lower *= 3;

                uint8_t red2 = ImageBuffer[index_lower];
                uint8_t green2 = ImageBuffer[index_lower + 1];
                uint8_t blue2 = ImageBuffer[index_lower + 2];

                // Set the RGB values based on the current bit plane
                RGB_R1((red1 >> bit) & 0x01);
                RGB_G1((green1 >> bit) & 0x01);
                RGB_B1((blue1 >> bit) & 0x01);

                RGB_R2((red2 >> bit) & 0x01);
                RGB_G2((green2 >> bit) & 0x01);
                RGB_B2((blue2 >> bit) & 0x01);

                RGB_CLK(1);
                RGB_CLK(0);
            }

			RGB_LAT(1);
			RGB_LAT(0);

            RGB_OE(0);
            DelayUs(delay_time); // Delay proportional to the bit weight
            RGB_OE(1);
        }
    }
}

void Paint_NewImage(uint8_t image[])
{
	Image = image;
}

void DrawPixel(uint16_t Xpoint, uint16_t Ypoint, uint8_t R, uint8_t G, uint8_t B)
{
    // Make sure coordinates are within bounds
    if (Xpoint >= MATRIX_WIDTH || Ypoint >= MATRIX_HEIGHT) return;

    uint32_t index = (MATRIX_WIDTH - Xpoint - 1) + ((MATRIX_HEIGHT - Ypoint - 1) * MATRIX_WIDTH);
    index *= 3; // Each pixel has 3 bytes (R, G, B)

    Image[index] = R;       // Red value
    Image[index + 1] = G;   // Green value
    Image[index + 2] = B;   // Blue value
}

void DrawRectangle(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint8_t R, uint8_t G, uint8_t B) {
    // Clamp coordinates to be within the matrix bounds
    if (x_start < 0) x_start = 0;
    if (x_start >= MATRIX_WIDTH) x_start = MATRIX_WIDTH - 1;
    if (x_end < 0) x_end = 0;
    if (x_end >= MATRIX_WIDTH) x_end = MATRIX_WIDTH - 1;
    if (y_start < 0) y_start = 0;
    if (y_start >= MATRIX_HEIGHT) y_start = MATRIX_HEIGHT - 1;
    if (y_end < 0) y_end = 0;
    if (y_end >= MATRIX_HEIGHT) y_end = MATRIX_HEIGHT - 1;

    // Iterate over each pixel in the rectangle area and set its color
    for (uint16_t x = x_start; x <= x_end; x++) {
        for (uint16_t y = y_start; y <= y_end; y++) {
            DrawPixel(x, y, R, G, B);
        }
    }
}


// Function to initialize the DWT cycle counter
void DWT_Init(void) {
    // Enable TRC (Trace Control) for DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Unlock the DWT (Data Watchpoint and Trace) registers
	// On STM32H7, this is necessary to access DWT->CYCCNT
    DWT->LAR = 0xC5ACCE55;

    // Reset the cycle counter
	DWT->CYCCNT = 0;

    // Enable the cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Function to delay in microseconds
void DelayUs(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t usTicks = us * (HAL_RCC_GetHCLKFreq() / 1000000); // Clock cycles per microsecond

    while ((DWT->CYCCNT - startTick) < usTicks);
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
