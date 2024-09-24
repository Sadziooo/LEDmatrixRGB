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
#include "gpdma.h"
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

// Matrix config
#define PANEL_COUNT		4
#define MATRIX_WIDTH  (PANEL_COUNT * 32)
#define MATRIX_HEIGHT 32
#define SCAN_RATE 16  // scanning lines 1/SCAN_RATE

// Color channel indices
#define RED    0
#define GREEN  1
#define BLUE   2

#define BCM_BITS 8
#define scroll_speed 6
#define BASE_DELAY 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// lines steering
#define RGB_GPIO_Port	R1_GPIO_Port
#define ABCD_GPIO_Port	A_GPIO_Port

#define RGB_R1(value)  	(value ? (R1_GPIO_Port->ODR |= R1_Pin) : (R1_GPIO_Port->ODR &= ~R1_Pin))
#define RGB_G1(value)  	(value ? (G1_GPIO_Port->ODR |= G1_Pin) : (G1_GPIO_Port->ODR &= ~G1_Pin))
#define RGB_B1(value)  	(value ? (B1_GPIO_Port->ODR |= B1_Pin) : (B1_GPIO_Port->ODR &= ~B1_Pin))

#define RGB_R2(value)  	(value ? (R2_GPIO_Port->ODR |= R2_Pin) : (R2_GPIO_Port->ODR &= ~R2_Pin))
#define RGB_G2(value)  	(value ? (G2_GPIO_Port->ODR |= G2_Pin) : (G2_GPIO_Port->ODR &= ~G2_Pin))
#define RGB_B2(value)  	(value ? (B2_GPIO_Port->ODR |= B2_Pin) : (B2_GPIO_Port->ODR &= ~B2_Pin))

#define RGB_A(value)   		(value & 0x01 ? (A_GPIO_Port->ODR |= A_Pin) : (A_GPIO_Port->ODR &= ~A_Pin))
#define RGB_B(value)   		(value & 0x02 ? (B_GPIO_Port->ODR |= B_Pin) : (B_GPIO_Port->ODR &= ~B_Pin))
#define RGB_C(value)   	(value & 0x04 ? (C_GPIO_Port->ODR |= C_Pin) : (C_GPIO_Port->ODR &= ~C_Pin))
#define RGB_D(value)   	(value & 0x08 ? (D_GPIO_Port->ODR |= D_Pin) : (D_GPIO_Port->ODR &= ~D_Pin))

#define RGB_CLK(value) 	(value ? (CLK_GPIO_Port->ODR |= CLK_Pin) : (CLK_GPIO_Port->ODR &= ~CLK_Pin))
#define RGB_LAT(value) 	(value ? (LAT_GPIO_Port->ODR |= LAT_Pin) : (LAT_GPIO_Port->ODR &= ~LAT_Pin))
#define RGB_OE(value)  	(value ? (OE_GPIO_Port->ODR |= OE_Pin) : (OE_GPIO_Port->ODR &= ~OE_Pin))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

// Image Buffer
uint8_t ImageBuffer[3 * MATRIX_HEIGHT * MATRIX_WIDTH]; // 3 colors
uint8_t DisplayBuffer[MATRIX_HEIGHT / 2][MATRIX_WIDTH][BCM_BITS][2];

uint8_t *Image;
uint32_t *Display;

const uint8_t Font5x7[][5];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void DWT_Init(void);
void DelayUs(uint32_t us);

void HUB75_Init(void);
void HUB75_SendRowData(void);
void SetRGBPins(uint8_t  rgb_data1, uint8_t rgb_data2);

void PrepareBuffer(void);

void Paint_NewImage(uint8_t image[]);
void DrawPixel(uint16_t Xpoint, uint16_t Ypoint, uint8_t R, uint8_t G, uint8_t B);
void DrawRectangle(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint8_t R, uint8_t G, uint8_t B);
void DrawChar(uint16_t x, uint16_t y, char c, uint8_t R, uint8_t G, uint8_t B);
void DrawString(uint16_t x, uint16_t y, const char *str, uint8_t R, uint8_t G, uint8_t B);
void ScrollHorizontal(int direction);
void ScrollVertical(int direction);

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
  MX_GPDMA1_Init();
  /* USER CODE BEGIN 2 */

  DWT_Init();
  HUB75_Init();

  for (uint16_t oui = 64; oui < MATRIX_WIDTH; oui++) {
	  for (uint16_t iou = 0; iou < MATRIX_HEIGHT; iou+=3) {
		  if (oui % 2 == 0) {
			  DrawRectangle(oui, iou, oui, iou, 255, 0, 0);
			  DrawRectangle(oui, iou + 1, oui, iou + 1, 0, 255, 0);
			  DrawRectangle(oui, iou + 2, oui, iou + 2, 0, 0, 255);
		  } else {
			  DrawRectangle(oui, iou, oui, iou, 10, 0, 0);
			  DrawRectangle(oui, iou + 1, oui, iou + 1, 0, 10, 0);
			  DrawRectangle(oui, iou + 2, oui, iou + 2, 0, 0, 10);
		  }
	  }
  }

//  DrawRectangle(0, 0, 127, 31, 200, 200, 200);
//  DrawRectangle(64, 0, 127, 31, 255, 255, 255);

//  DrawRectangle(64, 14, 127, 17, 255, 255, 255);
//
  DrawRectangle(5, 21, 9, 25, 255, 100, 0);
  DrawRectangle(10, 21, 14, 25, 255, 215, 0);
  DrawRectangle(15, 21, 19, 25, 120, 255, 10);
  DrawRectangle(20, 21, 24, 25, 255, 10, 100);
  DrawRectangle(25, 21, 29, 25, 10, 130, 120);
  DrawRectangle(30, 21, 34, 25, 255, 0, 0);
  DrawRectangle(35, 21, 39, 25, 0, 255, 0);
  DrawRectangle(40, 21, 44, 25, 0, 0, 255);
  DrawRectangle(45, 21, 49, 25, 255, 255, 255);
  DrawRectangle(50, 21, 54, 25, 50, 50, 50);
  DrawRectangle(55, 21, 59, 25, 1, 1, 1);

  DrawString(5, 1, "HELLO", 255, 0, 0);
  DrawString(5, 11, "WORLD", 0, 255, 0);
  DrawString(37, 1, "XD", 0, 0, 255);
  DrawString(37, 11, "XD", 0, 0, 10);
  DrawString(51, 1, "XD", 255, 255, 255);
  DrawString(51, 11, "XD", 10, 10, 10);

  //  DrawString(5, 11, "JAZDA!!!!", 50, 50, 50);

//  DrawRectangle(0, 1, 31, 1, 255, 255, 255);
//  DrawRectangle(0, 3, 31, 3, 255, 0, 0);
//  DrawRectangle(0, 5, 31, 5, 0, 255, 0);
//  DrawRectangle(0, 7, 31, 7, 0, 0, 255);
//
//  DrawString(1, 9, "X", 255, 255, 255);
//  DrawString(9, 9, "X", 100, 100, 100);
//  DrawString(17, 9, "X", 1, 1, 1);

  PrepareBuffer();

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
//	  ScrollVertical(0);
//	  ScrollHorizontal(1);
	  HUB75_SendRowData(); // Refresh display
	  DelayUs(100);
//	  HAL_Delay(scroll_speed);
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

void HUB75_SendRowData(void) {
    for (uint16_t row = 0; row < MATRIX_HEIGHT / 2; row++) {

        RGB_A(row);
        RGB_B(row);
        RGB_C(row);
        if (SCAN_RATE > 8) {
        	RGB_D(row);
        }

        for (uint8_t bit = 0; bit < BCM_BITS; bit++) {

            uint32_t delay_time = BASE_DELAY * (1 << bit);

            for (uint16_t col = 0; col < MATRIX_WIDTH; col++) {

				SetRGBPins(DisplayBuffer[row][col][bit][0], DisplayBuffer[row][col][bit][1]);

                RGB_CLK(1);
                RGB_CLK(0);
            }

            SetRGBPins(0, 0);

            RGB_LAT(1);
            RGB_LAT(0);

            RGB_OE(0);
            DelayUs(delay_time);
            RGB_OE(1);
        }
    }
}

void SetRGBPins(uint8_t rgb_data1, uint8_t  rgb_data2) {
	RGB_GPIO_Port->ODR = (RGB_GPIO_Port->ODR & ~(R1_Pin | G1_Pin | B1_Pin | R2_Pin | G2_Pin | B2_Pin)) |
	                 ((rgb_data1 & 0x01 ? R1_Pin : 0) |
	                  (rgb_data1 & 0x02 ? G1_Pin : 0) |
	                  (rgb_data1 & 0x04 ? B1_Pin : 0) |
	                  (rgb_data2 & 0x01 ? R2_Pin : 0) |
	                  (rgb_data2 & 0x02 ? G2_Pin : 0) |
	                  (rgb_data2 & 0x04 ? B2_Pin : 0));
}

void PrepareBuffer(void) {
    for (uint16_t row = 0; row < MATRIX_HEIGHT / 2; row++) {
    	uint32_t row_upper_base = row * MATRIX_WIDTH * 3;
    	uint32_t row_lower_base = (row + SCAN_RATE) * MATRIX_WIDTH * 3;

        for (uint8_t bit = 0; bit < BCM_BITS; bit++) {
            for (uint16_t col = 0, upper_offset = 0, lower_offset = 0; col < MATRIX_WIDTH; col++, upper_offset += 3, lower_offset += 3) {

                // Extract color components from the image buffer
                uint8_t red1 = ImageBuffer[row_upper_base + upper_offset];
                uint8_t green1 = ImageBuffer[row_upper_base + upper_offset + 1];
                uint8_t blue1 = ImageBuffer[row_upper_base + upper_offset + 2];

                uint8_t red2 = ImageBuffer[row_lower_base + lower_offset];
                uint8_t green2 = ImageBuffer[row_lower_base + lower_offset + 1];
                uint8_t blue2 = ImageBuffer[row_lower_base + lower_offset + 2];

                // Create the RGB data for both upper and lower rows
                uint8_t rgb_data1 = ((red1 >> bit) & 0x01) | ((green1 >> bit) & 0x01) << 1 | ((blue1 >> bit) & 0x01) << 2;
                uint8_t rgb_data2 = ((red2 >> bit) & 0x01) | ((green2 >> bit) & 0x01) << 1 | ((blue2 >> bit) & 0x01) << 2;

                // Store the result into the display buffer
                DisplayBuffer[row][col][bit][0] = rgb_data1; // Upper half
                DisplayBuffer[row][col][bit][1] = rgb_data2; // Lower half
            }
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

void DrawChar(uint16_t x, uint16_t y, char c, uint8_t R, uint8_t G, uint8_t B) {
    if (c < ' ' || c > '~') return; // Unsupported characters

    const uint8_t *char_pattern = Font5x7[c - ' ']; // Get character pattern

    // Loop over each column (5 columns for 5x7 font)
    for (uint8_t col = 0; col < 5; col++) {
        uint8_t column_data = char_pattern[col];

        // Loop over each row (7 rows per column)
        for (uint8_t row = 0; row < 7; row++) {
            if (column_data & (1 << row)) { // If the bit is set, draw pixel
                DrawPixel(x + col, y + row, R, G, B);
            }
        }
    }
}


void DrawString(uint16_t x, uint16_t y, const char *str, uint8_t R, uint8_t G, uint8_t B) {
    while (*str) {
        DrawChar(x, y, *str, R, G, B);  // Draw each character
        x += 6;  // Move to the right by 6 pixels (5 for the character, 1 for spacing)
        str++;   // Move to the next character
    }
}


void ScrollHorizontal(int direction) {
    // Shift left if direction is negative, shift right if direction is positive
    for (uint16_t row = 0; row < MATRIX_HEIGHT; row++) {
        if (direction > 0) {  // Scroll right
            // Store the last pixel's RGB values for the row
            uint8_t lastPixelR = ImageBuffer[(row * MATRIX_WIDTH * 3) + (MATRIX_WIDTH - 1) * 3];
            uint8_t lastPixelG = ImageBuffer[(row * MATRIX_WIDTH * 3) + (MATRIX_WIDTH - 1) * 3 + 1];
            uint8_t lastPixelB = ImageBuffer[(row * MATRIX_WIDTH * 3) + (MATRIX_WIDTH - 1) * 3 + 2];

            // Shift all pixels to the right
            for (uint16_t col = MATRIX_WIDTH - 1; col > 0; col--) {
                ImageBuffer[(row * MATRIX_WIDTH * 3) + col * 3] = ImageBuffer[(row * MATRIX_WIDTH * 3) + (col - 1) * 3];
                ImageBuffer[(row * MATRIX_WIDTH * 3) + col * 3 + 1] = ImageBuffer[(row * MATRIX_WIDTH * 3) + (col - 1) * 3 + 1];
                ImageBuffer[(row * MATRIX_WIDTH * 3) + col * 3 + 2] = ImageBuffer[(row * MATRIX_WIDTH * 3) + (col - 1) * 3 + 2];
            }

            // Move the last pixel to the first column (wraparound)
            ImageBuffer[(row * MATRIX_WIDTH * 3)] = lastPixelR;
            ImageBuffer[(row * MATRIX_WIDTH * 3) + 1] = lastPixelG;
            ImageBuffer[(row * MATRIX_WIDTH * 3) + 2] = lastPixelB;
        }
        else if (direction < 0) {  // Scroll left
            // Store the first pixel's RGB values
            uint8_t firstPixelR = ImageBuffer[(row * MATRIX_WIDTH * 3)];
            uint8_t firstPixelG = ImageBuffer[(row * MATRIX_WIDTH * 3) + 1];
            uint8_t firstPixelB = ImageBuffer[(row * MATRIX_WIDTH * 3) + 2];

            // Shift all pixels to the left
            for (uint16_t col = 0; col < MATRIX_WIDTH - 1; col++) {
                ImageBuffer[(row * MATRIX_WIDTH * 3) + col * 3] = ImageBuffer[(row * MATRIX_WIDTH * 3) + (col + 1) * 3];
                ImageBuffer[(row * MATRIX_WIDTH * 3) + col * 3 + 1] = ImageBuffer[(row * MATRIX_WIDTH * 3) + (col + 1) * 3 + 1];
                ImageBuffer[(row * MATRIX_WIDTH * 3) + col * 3 + 2] = ImageBuffer[(row * MATRIX_WIDTH * 3) + (col + 1) * 3 + 2];
            }

            // Move the first pixel to the last column (wraparound)
            ImageBuffer[(row * MATRIX_WIDTH * 3) + (MATRIX_WIDTH - 1) * 3] = firstPixelR;
            ImageBuffer[(row * MATRIX_WIDTH * 3) + (MATRIX_WIDTH - 1) * 3 + 1] = firstPixelG;
            ImageBuffer[(row * MATRIX_WIDTH * 3) + (MATRIX_WIDTH - 1) * 3 + 2] = firstPixelB;
        }
    }

    // After scrolling, update the display buffer
    PrepareBuffer();
}


void ScrollVertical(int direction) {
    if (direction == 1) { // Down
        uint8_t temp[MATRIX_WIDTH * 3]; // Store the last row for wrapping

        // Store last row
        memcpy(temp, &ImageBuffer[(MATRIX_HEIGHT - 1) * MATRIX_WIDTH * 3], MATRIX_WIDTH * 3);

        // Shift all rows down
        for (int y = MATRIX_HEIGHT - 1; y > 0; y--) {
            memcpy(&ImageBuffer[y * MATRIX_WIDTH * 3], &ImageBuffer[(y - 1) * MATRIX_WIDTH * 3], MATRIX_WIDTH * 3);
        }

        // Wrap the stored last row to the first
        memcpy(&ImageBuffer[0], temp, MATRIX_WIDTH * 3);
    } else { // Up
        uint8_t temp[MATRIX_WIDTH * 3]; // Store the first row for wrapping

        // Store first row
        memcpy(temp, &ImageBuffer[0], MATRIX_WIDTH * 3);

        // Shift all rows up
        for (int y = 0; y < MATRIX_HEIGHT - 1; y++) {
            memcpy(&ImageBuffer[y * MATRIX_WIDTH * 3], &ImageBuffer[(y + 1) * MATRIX_WIDTH * 3], MATRIX_WIDTH * 3);
        }

        // Wrap the stored first row to the last
        memcpy(&ImageBuffer[(MATRIX_HEIGHT - 1) * MATRIX_WIDTH * 3], temp, MATRIX_WIDTH * 3);
    }

    PrepareBuffer(); // Rebuild the display buffer with the new image data
}


const uint8_t Font5x7[][5] = {
    // Space
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ' '

    // ! - Exclamation mark
    {0x00, 0x00, 0x5F, 0x00, 0x00},

    // " - Double quotes
    {0x00, 0x03, 0x00, 0x03, 0x00},

    // # - Hash
    {0x14, 0x7F, 0x14, 0x7F, 0x14},

    // $ - Dollar sign
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},

    // % - Percent
    {0x23, 0x13, 0x08, 0x64, 0x62},

    // & - Ampersand
    {0x36, 0x49, 0x55, 0x22, 0x50},

    // ' - Single quote
    {0x00, 0x05, 0x03, 0x00, 0x00},

    // ( - Left parenthesis
    {0x00, 0x1C, 0x36, 0x41, 0x00},

    // ) - Right parenthesis
    {0x00, 0x41, 0x36, 0x1C, 0x00},

    // * - Asterisk
    {0x00, 0x36, 0x1C, 0x36, 0x00},

    // + - Plus
    {0x00, 0x08, 0x1C, 0x08, 0x00},

    // , - Comma
    {0x00, 0x80, 0x60, 0x00, 0x00},

    // - - Hyphen
    {0x00, 0x08, 0x08, 0x08, 0x00},

    // . - Period
    {0x00, 0x60, 0x60, 0x00, 0x00},

    // / - Slash
    {0x00, 0x06, 0x18, 0x60, 0x00},

    // 0 - Zero
    {0x3E, 0x51, 0x49, 0x45, 0x3E},

    // 1 - One
    {0x00, 0x42, 0x7F, 0x40, 0x00},

    // 2 - Two
    {0x42, 0x61, 0x51, 0x49, 0x46},

    // 3 - Three
    {0x21, 0x41, 0x45, 0x4B, 0x31},

    // 4 - Four
    {0x18, 0x14, 0x12, 0x7F, 0x10},

    // 5 - Five
    {0x27, 0x45, 0x45, 0x45, 0x39},

    // 6 - Six
    {0x3C, 0x4A, 0x49, 0x49, 0x30},

    // 7 - Seven
    {0x01, 0x71, 0x09, 0x07, 0x00},

    // 8 - Eight
    {0x36, 0x49, 0x49, 0x49, 0x36},

    // 9 - Nine
    {0x06, 0x49, 0x49, 0x29, 0x1E},

    // : - Colon
    {0x00, 0x36, 0x00, 0x36, 0x00},

    // ; - Semicolon
    {0x00, 0x36, 0x00, 0x36, 0x80},

    // < - Less than
    {0x00, 0x08, 0x14, 0x22, 0x00},

    // = - Equal
    {0x00, 0x14, 0x14, 0x14, 0x00},

    // > - Greater than
    {0x00, 0x22, 0x14, 0x08, 0x00},

    // ? - Question mark
    {0x02, 0x01, 0x51, 0x09, 0x06},

    // @ - At sign
    {0x32, 0x49, 0x79, 0x41, 0x3E},

    // A - A
    {0x7E, 0x11, 0x11, 0x11, 0x7E},

    // B - B
    {0x7F, 0x49, 0x49, 0x49, 0x36},

    // C - C
    {0x3E, 0x41, 0x41, 0x41, 0x22},

    // D - D
    {0x7F, 0x41, 0x41, 0x22, 0x1C},

    // E - E
    {0x7F, 0x49, 0x49, 0x49, 0x41},

    // F - F
    {0x7F, 0x09, 0x09, 0x09, 0x01},

    // G - G
    {0x3E, 0x41, 0x51, 0x51, 0x32},

    // H - H
    {0x7F, 0x08, 0x08, 0x08, 0x7F},

    // I - I
    {0x00, 0x41, 0x7F, 0x41, 0x00},

    // J - J
    {0x20, 0x40, 0x41, 0x3F, 0x01},

    // K - K
    {0x7F, 0x08, 0x14, 0x22, 0x41},

    // L - L
    {0x7F, 0x40, 0x40, 0x40, 0x40},

    // M - M
	{0x7F, 0x06, 0x0C, 0x06, 0x7F},

    // N - N
    {0x7F, 0x30, 0x0C, 0x03, 0x7F},

    // O - O
    {0x3E, 0x41, 0x41, 0x41, 0x3E},

    // P - P
    {0x7F, 0x09, 0x09, 0x09, 0x06},

    // Q - Q
    {0x3E, 0x41, 0x51, 0x21, 0x5E},

    // R - R
    {0x7F, 0x09, 0x19, 0x29, 0x46},

    // S - S
    {0x26, 0x49, 0x49, 0x49, 0x32},

    // T - T
    {0x01, 0x01, 0x7F, 0x01, 0x01},

    // U - U
    {0x3F, 0x40, 0x40, 0x40, 0x3F},

    // V - V
    {0x1F, 0x20, 0x40, 0x20, 0x1F},

    // W - W
    {0x3F, 0x40, 0x38, 0x40, 0x3F},

    // X - X
    {0x63, 0x14, 0x08, 0x14, 0x63},

    // Y - Y
    {0x03, 0x04, 0x78, 0x04, 0x03},

    // Z - Z
    {0x61, 0x51, 0x49, 0x45, 0x43},

    // [ - Left bracket
    {0x00, 0x7F, 0x41, 0x41, 0x00},

    // \ - Backslash
    {0x00, 0x41, 0x22, 0x1C, 0x00},

    // ] - Right bracket
    {0x00, 0x41, 0x41, 0x7F, 0x00},

    // ^ - Caret
    {0x04, 0x02, 0x01, 0x02, 0x04},

    // _ - Underscore
    {0x80, 0x80, 0x80, 0x80, 0x80},

    // ` - Grave accent
    {0x00, 0x01, 0x02, 0x04, 0x00},

    // a - a
    {0x20, 0x54, 0x54, 0x54, 0x78},

    // b - b
    {0x7F, 0x44, 0x44, 0x44, 0x38},

    // c - c
    {0x38, 0x44, 0x44, 0x44, 0x20},

    // d - d
    {0x38, 0x44, 0x44, 0x44, 0x7F},

    // e - e
    {0x38, 0x54, 0x54, 0x54, 0x18},

    // f - f
    {0x08, 0x7E, 0x09, 0x01, 0x02},

    // g - g
    {0x0C, 0x52, 0x52, 0x52, 0x3E},

    // h - h
    {0x7F, 0x08, 0x04, 0x04, 0x78},

    // i - i
    {0x00, 0x44, 0x7D, 0x40, 0x00},

    // j - j
    {0x20, 0x40, 0x44, 0x3D, 0x00},

    // k - k
    {0x7F, 0x08, 0x14, 0x22, 0x41},

    // l - l
    {0x00, 0x41, 0x7F, 0x40, 0x00},

    // m - m
    {0x7C, 0x10, 0x0C, 0x10, 0x7C},

    // n - n
    {0x7C, 0x10, 0x0C, 0x04, 0x78},

    // o - o
    {0x38, 0x44, 0x44, 0x44, 0x38},

    // p - p
    {0x7C, 0x14, 0x14, 0x14, 0x08},

    // q - q
    {0x08, 0x14, 0x14, 0x14, 0x7C},

    // r - r
    {0x7C, 0x10, 0x08, 0x04, 0x08},

    // s - s
    {0x48, 0x54, 0x54, 0x54, 0x24},

    // t - t
    {0x04, 0x3F, 0x44, 0x40, 0x20},

    // u - u
    {0x7C, 0x40, 0x40, 0x40, 0x7C},

    // v - v
    {0x7C, 0x20, 0x10, 0x08, 0x7C},

    // w - w
    {0x7C, 0x20, 0x18, 0x20, 0x7C},

    // x - x
    {0x44, 0x28, 0x10, 0x28, 0x44},

    // y - y
    {0x4C, 0x30, 0x10, 0x0F, 0x00},

    // z - z
    {0x44, 0x64, 0x54, 0x4C, 0x44},

    // { - Left brace
    {0x00, 0x1C, 0x36, 0x41, 0x00},

    // | - Vertical bar
    {0x00, 0x00, 0x77, 0x00, 0x00},

    // } - Right brace
    {0x00, 0x41, 0x36, 0x1C, 0x00},

    // ~ - Tilde
    {0x00, 0x40, 0x30, 0x00, 0x00},
};



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
