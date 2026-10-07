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

/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <math.h>
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

#define MIC_FRAMES 512
#define FFT_SIZE   256

#define LCD_WIDTH 240
#define LCD_HEIGHT 320
#define SPECTRUM_BARS 16

#define SAMPLE_RATE_HZ 32000.0f

/*
 * Spectrum display scaling.
 */
#define DISPLAY_DB_MIN      (-60.0f)
#define DISPLAY_DB_MAX      (-10.0f)
#define SILENCE_GATE_DB     (-58.0f)

/*
 * Level mode uses true RMS rather than the largest individual sample.
 */
#define LEVEL_SILENCE_DBFS  (-58.0f)

/*
 * Keep the last valid PEAK/LEVEL reading on screen briefly through
 * natural gaps in speech.
 */
#define SIGNAL_HOLD_MS 500U

/*
 * Smooth the FFT power used only for the peak-frequency detector.
 * Larger OLD weight = steadier but slower.
 */
#define FREQ_SMOOTH_OLD 0.72f
#define FREQ_SMOOTH_NEW 0.28f

/*
 * Require the strongest spectral component to be somewhat stronger
 * than the average spectrum before accepting it as a useful peak.
 */
#define PEAK_CONFIDENCE_RATIO 1.8f

#define MIC_CLIP_THRESHOLD 125000U
#define LCD_REFRESH_MS 33U

/* Display modes selected with the blue B1 button. */
#define DISPLAY_MODE_SPECTRUM 0U
#define DISPLAY_MODE_PEAK     1U
#define DISPLAY_MODE_LEVEL    2U
#define DISPLAY_MODE_COUNT    3U

/* Peak-hold behaviour for the spectrum bars. */
#define PEAK_HOLD_BLOCKS      20U
#define PEAK_FALL_PER_BLOCK   0.45f

/* RGB565 colours. */
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_GREEN   0x07E0
#define COLOR_YELLOW  0xFFE0
#define COLOR_RED     0xF800
#define COLOR_GRAY    0x8410

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2S_HandleTypeDef hi2s2;

DMA_HandleTypeDef hdma_spi2_rx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t micBuffer[MIC_FRAMES * 4];

volatile uint8_t micHalfReady = 0;
volatile uint8_t micFullReady = 0;

char micMessage[100];

arm_rfft_fast_instance_f32 fftInstance;

float fftInput[FFT_SIZE];
float fftOutput[FFT_SIZE];
float fftMagnitude[FFT_SIZE / 2];

/*
 * Separate smoothed spectrum used by PEAK FREQUENCY mode.
 * This avoids changing the spectrum-bar behaviour that is already working.
 */
float frequencySmoothPower[FFT_SIZE / 2] = {0};

float hannWindow[FFT_SIZE];

/* Smoothed bar heights produced by the FFT processing. */
float spectrumBarHeight[SPECTRUM_BARS] = {0};

/* Peak-hold marker state. */
float spectrumPeakHold[SPECTRUM_BARS] = {0};
uint8_t spectrumPeakHoldCounter[SPECTRUM_BARS] = {0};

/* What has already been drawn on the LCD. */
uint16_t drawnBarHeight[SPECTRUM_BARS] = {0};
uint16_t drawnPeakHeight[SPECTRUM_BARS] = {0};

/* Audio-level meter drawing state. */
uint16_t drawnLevelMeterWidth = 0;
uint16_t drawnLevelMeterColor = COLOR_GREEN;

/*
 * PEAK FREQUENCY state.
 * This is the strongest FFT component ("dominant frequency"), not a
 * voice-pitch estimator.
 */
uint32_t latestPeakFrequency = 0;
uint32_t lastPeakSignalTime = 0;

/*
 * AUDIO LEVEL state.
 * latestLevelDb is a smoothed RMS level in dBFS.
 */
float latestLevelDb = DISPLAY_DB_MIN;
uint32_t lastLevelSignalTime = 0;

uint8_t latestClipping = 0;

uint8_t displayMode = DISPLAY_MODE_SPECTRUM;

uint32_t lastLcdUpdate = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_DMA_Init(void);

static void MX_SPI1_Init(void);

static void MX_I2S2_Init(void);

static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

static void LCD_WriteCommand(uint8_t cmd)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

static void LCD_WriteData(uint8_t data)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

static void LCD_Init(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(120);

    LCD_WriteCommand(0x01);
    HAL_Delay(120);

    LCD_WriteCommand(0x11);
    HAL_Delay(120);

    LCD_WriteCommand(0x3A);
    LCD_WriteData(0x55);

    LCD_WriteCommand(0x36);
    LCD_WriteData(0x48);

    LCD_WriteCommand(0x29);
    HAL_Delay(20);
}

static void LCD_SetAddressWindow(uint16_t x0,
                                 uint16_t y0,
                                 uint16_t x1,
                                 uint16_t y1)
{
    LCD_WriteCommand(0x2A);
    LCD_WriteData(x0 >> 8);
    LCD_WriteData(x0 & 0xFF);
    LCD_WriteData(x1 >> 8);
    LCD_WriteData(x1 & 0xFF);

    LCD_WriteCommand(0x2B);
    LCD_WriteData(y0 >> 8);
    LCD_WriteData(y0 & 0xFF);
    LCD_WriteData(y1 >> 8);
    LCD_WriteData(y1 & 0xFF);

    LCD_WriteCommand(0x2C);
}

static void LCD_Fill(uint16_t color)
{
    LCD_SetAddressWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    uint8_t data[64];

    for (uint32_t i = 0; i < 32; i++)
    {
        data[i * 2]     = (uint8_t)(color >> 8);
        data[i * 2 + 1] = (uint8_t)(color & 0xFF);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    uint32_t pixelsRemaining = LCD_WIDTH * LCD_HEIGHT;

    while (pixelsRemaining > 0)
    {
        uint16_t pixelsToSend =
            (pixelsRemaining > 32) ? 32 : (uint16_t)pixelsRemaining;

        HAL_SPI_Transmit(
            &hspi1,
            data,
            pixelsToSend * 2,
            HAL_MAX_DELAY
        );

        pixelsRemaining -= pixelsToSend;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

static void LCD_FillRect(uint16_t x,
                         uint16_t y,
                         uint16_t width,
                         uint16_t height,
                         uint16_t color)
{
    if ((width == 0) || (height == 0) ||
        (x >= LCD_WIDTH) || (y >= LCD_HEIGHT))
    {
        return;
    }

    if ((uint32_t)x + width > LCD_WIDTH)
    {
        width = LCD_WIDTH - x;
    }

    if ((uint32_t)y + height > LCD_HEIGHT)
    {
        height = LCD_HEIGHT - y;
    }

    LCD_SetAddressWindow(
        x,
        y,
        x + width - 1,
        y + height - 1
    );

    uint8_t data[64];

    for (uint32_t i = 0; i < 32; i++)
    {
        data[i * 2]     = (uint8_t)(color >> 8);
        data[i * 2 + 1] = (uint8_t)(color & 0xFF);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    uint32_t pixelsRemaining = (uint32_t)width * height;

    while (pixelsRemaining > 0)
    {
        uint16_t pixelsToSend =
            (pixelsRemaining > 32) ? 32 : (uint16_t)pixelsRemaining;

        HAL_SPI_Transmit(
            &hspi1,
            data,
            pixelsToSend * 2,
            HAL_MAX_DELAY
        );

        pixelsRemaining -= pixelsToSend;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

static const uint8_t *FONT_GetGlyph(char c)
{
    switch (c)
    {
        case ' ': { static const uint8_t g[7] = {0, 0, 0, 0, 0, 0, 0}; return g; }
        case '-': { static const uint8_t g[7] = {0, 0, 0, 31, 0, 0, 0}; return g; }
        case '.': { static const uint8_t g[7] = {0, 0, 0, 0, 0, 12, 12}; return g; }
        case '0': { static const uint8_t g[7] = {14, 17, 19, 21, 25, 17, 14}; return g; }
        case '1': { static const uint8_t g[7] = {4, 12, 4, 4, 4, 4, 14}; return g; }
        case '2': { static const uint8_t g[7] = {14, 17, 1, 2, 4, 8, 31}; return g; }
        case '3': { static const uint8_t g[7] = {30, 1, 1, 14, 1, 1, 30}; return g; }
        case '4': { static const uint8_t g[7] = {2, 6, 10, 18, 31, 2, 2}; return g; }
        case '5': { static const uint8_t g[7] = {31, 16, 16, 30, 1, 1, 30}; return g; }
        case '6': { static const uint8_t g[7] = {14, 16, 16, 30, 17, 17, 14}; return g; }
        case '7': { static const uint8_t g[7] = {31, 1, 2, 4, 8, 8, 8}; return g; }
        case '8': { static const uint8_t g[7] = {14, 17, 17, 14, 17, 17, 14}; return g; }
        case '9': { static const uint8_t g[7] = {14, 17, 17, 15, 1, 1, 14}; return g; }
        case 'A': { static const uint8_t g[7] = {14, 17, 17, 31, 17, 17, 17}; return g; }
        case 'B': { static const uint8_t g[7] = {30, 17, 17, 30, 17, 17, 30}; return g; }
        case 'C': { static const uint8_t g[7] = {15, 16, 16, 16, 16, 16, 15}; return g; }
        case 'D': { static const uint8_t g[7] = {30, 17, 17, 17, 17, 17, 30}; return g; }
        case 'E': { static const uint8_t g[7] = {31, 16, 16, 30, 16, 16, 31}; return g; }
        case 'F': { static const uint8_t g[7] = {31, 16, 16, 30, 16, 16, 16}; return g; }
        case 'G': { static const uint8_t g[7] = {15, 16, 16, 23, 17, 17, 15}; return g; }
        case 'H': { static const uint8_t g[7] = {17, 17, 17, 31, 17, 17, 17}; return g; }
        case 'I': { static const uint8_t g[7] = {14, 4, 4, 4, 4, 4, 14}; return g; }
        case 'J': { static const uint8_t g[7] = {7, 2, 2, 2, 18, 18, 12}; return g; }
        case 'K': { static const uint8_t g[7] = {17, 18, 20, 24, 20, 18, 17}; return g; }
        case 'L': { static const uint8_t g[7] = {16, 16, 16, 16, 16, 16, 31}; return g; }
        case 'M': { static const uint8_t g[7] = {17, 27, 21, 21, 17, 17, 17}; return g; }
        case 'N': { static const uint8_t g[7] = {17, 25, 21, 19, 17, 17, 17}; return g; }
        case 'O': { static const uint8_t g[7] = {14, 17, 17, 17, 17, 17, 14}; return g; }
        case 'P': { static const uint8_t g[7] = {30, 17, 17, 30, 16, 16, 16}; return g; }
        case 'Q': { static const uint8_t g[7] = {14, 17, 17, 17, 21, 18, 13}; return g; }
        case 'R': { static const uint8_t g[7] = {30, 17, 17, 30, 20, 18, 17}; return g; }
        case 'S': { static const uint8_t g[7] = {15, 16, 16, 14, 1, 1, 30}; return g; }
        case 'T': { static const uint8_t g[7] = {31, 4, 4, 4, 4, 4, 4}; return g; }
        case 'U': { static const uint8_t g[7] = {17, 17, 17, 17, 17, 17, 14}; return g; }
        case 'V': { static const uint8_t g[7] = {17, 17, 17, 17, 17, 10, 4}; return g; }
        case 'W': { static const uint8_t g[7] = {17, 17, 17, 21, 21, 21, 10}; return g; }
        case 'X': { static const uint8_t g[7] = {17, 17, 10, 4, 10, 17, 17}; return g; }
        case 'Y': { static const uint8_t g[7] = {17, 17, 10, 4, 4, 4, 4}; return g; }
        case 'Z': { static const uint8_t g[7] = {31, 1, 2, 4, 8, 16, 31}; return g; }
        default: { static const uint8_t g[7] = {0, 0, 0, 0, 0, 0, 0}; return g; }
    }
}

static void LCD_DrawChar(uint16_t x,
                         uint16_t y,
                         char c,
                         uint16_t color,
                         uint16_t background,
                         uint8_t scale)
{
    const uint8_t *glyph = FONT_GetGlyph(c);

    /*
     * Draw one complete 5x7 character using ONE LCD window/transfer.
     * The previous version called LCD_FillRect for every tiny font pixel,
     * which made PEAK/LEVEL modes so slow that button presses were missed.
     *
     * Maximum used scale is 3:
     * 5*3 x 7*3 = 315 pixels = 630 bytes.
     */
    uint16_t charWidth  = (uint16_t)(5U * scale);
    uint16_t charHeight = (uint16_t)(7U * scale);

    uint8_t pixelData[630];
    uint32_t pos = 0;

    for (uint8_t row = 0; row < 7; row++)
    {
        for (uint8_t repeatY = 0; repeatY < scale; repeatY++)
        {
            for (uint8_t col = 0; col < 5; col++)
            {
                uint16_t pixelColor =
                    (glyph[row] & (1U << (4U - col))) ?
                    color :
                    background;

                for (uint8_t repeatX = 0; repeatX < scale; repeatX++)
                {
                    pixelData[pos++] = (uint8_t)(pixelColor >> 8);
                    pixelData[pos++] = (uint8_t)(pixelColor & 0xFF);
                }
            }
        }
    }

    LCD_SetAddressWindow(
        x,
        y,
        x + charWidth - 1U,
        y + charHeight - 1U
    );

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    HAL_SPI_Transmit(
        &hspi1,
        pixelData,
        (uint16_t)pos,
        HAL_MAX_DELAY
    );

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

static void LCD_DrawText(uint16_t x,
                         uint16_t y,
                         const char *text,
                         uint16_t color,
                         uint16_t background,
                         uint8_t scale)
{
    while (*text != '\0')
    {
        LCD_DrawChar(
            x,
            y,
            *text,
            color,
            background,
            scale
        );

        x += (uint16_t)(6U * scale);
        text++;
    }
}

static uint16_t LCD_TextWidth(const char *text, uint8_t scale)
{
    uint16_t width = 0;

    while (*text != '\0')
    {
        width += (uint16_t)(6U * scale);
        text++;
    }

    if (width >= scale)
    {
        width -= scale;
    }

    return width;
}

static void LCD_DrawCenteredText(uint16_t y,
                                 const char *text,
                                 uint16_t color,
                                 uint16_t background,
                                 uint8_t scale)
{
    uint16_t width = LCD_TextWidth(text, scale);
    uint16_t x = (width < LCD_WIDTH) ?
                 (LCD_WIDTH - width) / 2U :
                 0U;

    LCD_DrawText(
        x,
        y,
        text,
        color,
        background,
        scale
    );
}

static uint8_t UI_HasRecentPeak(void)
{
    if (lastPeakSignalTime == 0U)
    {
        return 0U;
    }

    return ((HAL_GetTick() - lastPeakSignalTime) <= SIGNAL_HOLD_MS) ? 1U : 0U;
}

static uint8_t UI_HasRecentLevel(void)
{
    if (lastLevelSignalTime == 0U)
    {
        return 0U;
    }

    return ((HAL_GetTick() - lastLevelSignalTime) <= SIGNAL_HOLD_MS) ? 1U : 0U;
}

static void UI_ResetDrawState(void)
{
    for (uint32_t i = 0; i < SPECTRUM_BARS; i++)
    {
        drawnBarHeight[i] = 0;
        drawnPeakHeight[i] = 0;
    }

    drawnLevelMeterWidth = 0;
    drawnLevelMeterColor = COLOR_GREEN;
}

static void UI_DrawStatic(void)
{
    LCD_Fill(COLOR_BLACK);
    UI_ResetDrawState();

    if (displayMode == DISPLAY_MODE_SPECTRUM)
    {
        LCD_DrawCenteredText(
            4,
            "AUDIO SPECTRUM",
            COLOR_WHITE,
            COLOR_BLACK,
            1
        );

        LCD_DrawText(1,   306, "125", COLOR_GRAY, COLOR_BLACK, 1);
        LCD_DrawText(69,  306, "1K",  COLOR_GRAY, COLOR_BLACK, 1);
        LCD_DrawText(132, 306, "4K",  COLOR_GRAY, COLOR_BLACK, 1);
        LCD_DrawText(215, 306, "16K", COLOR_GRAY, COLOR_BLACK, 1);
    }
    else if (displayMode == DISPLAY_MODE_PEAK)
    {
        LCD_DrawCenteredText(
            12,
            "PEAK FREQUENCY",
            COLOR_WHITE,
            COLOR_BLACK,
            1
        );
    }
    else
    {
        LCD_DrawCenteredText(
            12,
            "AUDIO LEVEL",
            COLOR_WHITE,
            COLOR_BLACK,
            1
        );

        LCD_DrawText(
            20,
            236,
            "-60",
            COLOR_GRAY,
            COLOR_BLACK,
            1
        );

        LCD_DrawText(
            198,
            236,
            "-10",
            COLOR_GRAY,
            COLOR_BLACK,
            1
        );
    }
}

static void UI_SetMode(uint8_t newMode)
{
    displayMode = newMode % DISPLAY_MODE_COUNT;
    UI_DrawStatic();
}

static void UI_HandleButton(void)
{
    /*
     * B1 on NUCLEO-F446RE is active LOW.
     *
     * pressLatched guarantees one mode change per physical press.
     * The time guard rejects contact bounce.  This is intentionally simple
     * so the button remains reliable even when an LCD update takes time.
     */
    static uint8_t pressLatched = 0U;
    static uint32_t lastAcceptedPress = 0U;

    GPIO_PinState state =
        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

    uint32_t now = HAL_GetTick();

    if (state == GPIO_PIN_RESET)
    {
        if ((!pressLatched) &&
            ((now - lastAcceptedPress) >= 150U))
        {
            pressLatched = 1U;
            lastAcceptedPress = now;

            UI_SetMode(
                (uint8_t)(
                    (displayMode + 1U) %
                    DISPLAY_MODE_COUNT
                )
            );
        }
    }
    else
    {
        pressLatched = 0U;
    }
}

static void LCD_DrawSpectrumBars(void)
{
    const uint16_t graphBottom = 296;
    const uint16_t maxBarHeight = 265;
    const uint16_t barSlotWidth = LCD_WIDTH / SPECTRUM_BARS;
    const uint16_t barWidth = barSlotWidth - 3;

    for (uint32_t bar = 0; bar < SPECTRUM_BARS; bar++)
    {
        float heightFloat = spectrumBarHeight[bar];
        float peakFloat = spectrumPeakHold[bar];

        if (heightFloat < 0.0f) heightFloat = 0.0f;
        if (heightFloat > maxBarHeight) heightFloat = maxBarHeight;

        if (peakFloat < 0.0f) peakFloat = 0.0f;
        if (peakFloat > maxBarHeight) peakFloat = maxBarHeight;

        uint16_t newHeight = (uint16_t)heightFloat;
        uint16_t newPeakHeight = (uint16_t)peakFloat;
        uint16_t x = (uint16_t)(bar * barSlotWidth + 1U);

        /*
         * Remove the previously drawn yellow peak marker.
         * Restore green if it lies inside the current bar,
         * otherwise restore black.
         */
        if (drawnPeakHeight[bar] > 0U)
        {
            uint16_t oldPeakY =
                graphBottom - drawnPeakHeight[bar];

            uint16_t restoreColor =
                (drawnPeakHeight[bar] <= newHeight) ?
                COLOR_GREEN :
                COLOR_BLACK;

            LCD_FillRect(
                x,
                oldPeakY,
                barWidth,
                2,
                restoreColor
            );
        }

        if (newHeight > drawnBarHeight[bar])
        {
            uint16_t addedHeight =
                newHeight - drawnBarHeight[bar];

            LCD_FillRect(
                x,
                graphBottom - newHeight,
                barWidth,
                addedHeight,
                COLOR_GREEN
            );
        }
        else if (newHeight < drawnBarHeight[bar])
        {
            uint16_t removedHeight =
                drawnBarHeight[bar] - newHeight;

            LCD_FillRect(
                x,
                graphBottom - drawnBarHeight[bar],
                barWidth,
                removedHeight,
                COLOR_BLACK
            );
        }

        if (newPeakHeight > 0U)
        {
            LCD_FillRect(
                x,
                graphBottom - newPeakHeight,
                barWidth,
                2,
                COLOR_YELLOW
            );
        }

        drawnBarHeight[bar] = newHeight;
        drawnPeakHeight[bar] = newPeakHeight;
    }
}

static void UI_DrawPeakFrequency(void)
{
    char text[24];

    /*
     * IMPORTANT:
     * Both the frequency value and SILENCE are always drawn at the same
     * x/y position, with the same font scale, and padded to the same width.
     * Therefore the black background pixels in the new string erase the
     * old string instead of the two messages overlapping.
     */
    if (UI_HasRecentPeak())
    {
        snprintf(
            text,
            sizeof(text),
            "%5lu HZ   ",
            (unsigned long)latestPeakFrequency
        );
    }
    else
    {
        snprintf(
            text,
            sizeof(text),
            " SILENCE   "
        );
    }

    LCD_DrawText(
        24,
        112,
        text,
        UI_HasRecentPeak() ? COLOR_GREEN : COLOR_GRAY,
        COLOR_BLACK,
        3
    );

    if (latestClipping && UI_HasRecentPeak())
    {
        LCD_DrawText(
            90,
            180,
            "CLIP   ",
            COLOR_RED,
            COLOR_BLACK,
            2
        );
    }
    else
    {
        LCD_DrawText(
            90,
            180,
            "       ",
            COLOR_BLACK,
            COLOR_BLACK,
            2
        );
    }
}

static void UI_DrawAudioLevel(void)
{
    char text[24];
    uint8_t recentLevel = UI_HasRecentLevel();

    /*
     * Same position, same scale, fixed-width padded text:
     * this prevents SILENCE and the numeric reading from overlapping.
     */
    if (recentLevel)
    {
        snprintf(
            text,
            sizeof(text),
            "%5.1f DBFS   ",
            latestLevelDb
        );
    }
    else
    {
        snprintf(
            text,
            sizeof(text),
            " SILENCE      "
        );
    }

    LCD_DrawText(
        30,
        100,
        text,
        recentLevel ?
            (latestClipping ? COLOR_RED : COLOR_GREEN) :
            COLOR_GRAY,
        COLOR_BLACK,
        2
    );

    float level = latestLevelDb;

    if (!recentLevel)
    {
        level = DISPLAY_DB_MIN;
    }

    if (level < DISPLAY_DB_MIN) level = DISPLAY_DB_MIN;
    if (level > DISPLAY_DB_MAX) level = DISPLAY_DB_MAX;

    float normalized =
        (level - DISPLAY_DB_MIN) /
        (DISPLAY_DB_MAX - DISPLAY_DB_MIN);

    uint16_t newWidth =
        (uint16_t)(normalized * 200.0f);

    uint16_t newColor =
        latestClipping ? COLOR_RED : COLOR_GREEN;

    /*
     * If the colour changes (for example, clipping turns red),
     * redraw the current filled part once.
     */
    if ((drawnLevelMeterWidth > 0U) &&
        (newColor != drawnLevelMeterColor))
    {
        LCD_FillRect(
            20,
            200,
            drawnLevelMeterWidth,
            22,
            newColor
        );
    }

    if (newWidth > drawnLevelMeterWidth)
    {
        LCD_FillRect(
            20 + drawnLevelMeterWidth,
            200,
            newWidth - drawnLevelMeterWidth,
            22,
            newColor
        );
    }
    else if (newWidth < drawnLevelMeterWidth)
    {
        LCD_FillRect(
            20 + newWidth,
            200,
            drawnLevelMeterWidth - newWidth,
            22,
            COLOR_BLACK
        );
    }

    drawnLevelMeterWidth = newWidth;
    drawnLevelMeterColor = newColor;
}

static void UI_UpdateDisplay(void)
{
    if (displayMode == DISPLAY_MODE_SPECTRUM)
    {
        LCD_DrawSpectrumBars();
    }
    else if (displayMode == DISPLAY_MODE_PEAK)
    {
        UI_DrawPeakFrequency();
    }
    else
    {
        UI_DrawAudioLevel();
    }
}

static int32_t MIC_ConvertSample(uint16_t first, uint16_t second)
{
    uint32_t raw =
        ((uint32_t)first << 16) |
        (uint32_t)second;

    return ((int32_t)raw) >> 14;
}

static void MIC_ProcessBlock(uint16_t *buffer, uint32_t frames)
{
    static const uint16_t bandEdges[SPECTRUM_BARS + 1] =
    {
        1, 2, 3, 4, 5, 6, 8, 10, 13,
        17, 22, 29, 38, 50, 66, 88, 128
    };

    if (frames < FFT_SIZE)
    {
        return;
    }

    /* =========================================================
     * 1. REMOVE MICROPHONE DC OFFSET
     * ========================================================= */
    int64_t leftSum = 0;

    for (uint32_t i = 0; i < FFT_SIZE; i++)
    {
        uint32_t index = i * 4;

        int32_t left =
            MIC_ConvertSample(
                buffer[index],
                buffer[index + 1]
            );

        leftSum += left;
    }

    int32_t leftAverage =
        (int32_t)(leftSum / FFT_SIZE);

    /* =========================================================
     * 2. PREPARE FFT INPUT + CALCULATE TRUE RMS LEVEL
     * ========================================================= */
    uint32_t peakRawLevel = 0;
    uint64_t squareSum = 0;
    uint8_t clipped = 0;

    for (uint32_t i = 0; i < FFT_SIZE; i++)
    {
        uint32_t index = i * 4;

        int32_t left =
            MIC_ConvertSample(
                buffer[index],
                buffer[index + 1]
            );

        int32_t centered =
            left - leftAverage;

        uint32_t absoluteValue =
            (centered < 0) ?
            (uint32_t)(-centered) :
            (uint32_t)centered;

        if (absoluteValue > peakRawLevel)
        {
            peakRawLevel = absoluteValue;
        }

        if (absoluteValue >= MIC_CLIP_THRESHOLD)
        {
            clipped = 1U;
        }

        int64_t centered64 = centered;
        squareSum +=
            (uint64_t)(centered64 * centered64);

        float normalized =
            (float)centered / 131072.0f;

        fftInput[i] =
            normalized * hannWindow[i];
    }

    float rmsCounts =
        sqrtf(
            (float)squareSum /
            (float)FFT_SIZE
        );

    float rmsNormalized =
        rmsCounts / 131072.0f;

    float rmsDb =
        20.0f *
        log10f(
            rmsNormalized +
            0.000000001f
        );

    uint8_t levelPresent =
        (rmsDb >= LEVEL_SILENCE_DBFS) ?
        1U :
        0U;

    /* Smooth AUDIO LEVEL mode itself. */
    static uint8_t levelInitialised = 0U;

    if (levelPresent)
    {
        if (!levelInitialised)
        {
            latestLevelDb = rmsDb;
            levelInitialised = 1U;
        }
        else
        {
            float alpha =
                (rmsDb > latestLevelDb) ?
                0.35f :
                0.12f;

            latestLevelDb +=
                alpha *
                (rmsDb - latestLevelDb);
        }

        lastLevelSignalTime =
            HAL_GetTick();
    }

    latestClipping = clipped;

    /* =========================================================
     * 3. FFT
     * ========================================================= */
    arm_rfft_fast_f32(
        &fftInstance,
        fftInput,
        fftOutput,
        0
    );

    fftMagnitude[0] = 0.0f;

    float totalPower = 0.0f;

    for (uint32_t bin = 1;
         bin < FFT_SIZE / 2;
         bin++)
    {
        float real =
            fftOutput[bin * 2];

        float imag =
            fftOutput[bin * 2 + 1];

        float power =
            (real * real) +
            (imag * imag);

        fftMagnitude[bin] = power;

        /*
         * Separate smoothing only for dominant-frequency detection.
         * Spectrum bars continue using the original instantaneous power.
         */
        frequencySmoothPower[bin] =
            FREQ_SMOOTH_OLD *
            frequencySmoothPower[bin] +
            FREQ_SMOOTH_NEW *
            power;

        totalPower +=
            frequencySmoothPower[bin];
    }

    /* =========================================================
     * 4. STABLE DOMINANT / PEAK FREQUENCY
     * ========================================================= */
    uint32_t dominantBin = 1U;
    float dominantPower =
        frequencySmoothPower[1];

    for (uint32_t bin = 2;
         bin < (FFT_SIZE / 2) - 1U;
         bin++)
    {
        if (frequencySmoothPower[bin] >
            dominantPower)
        {
            dominantPower =
                frequencySmoothPower[bin];

            dominantBin = bin;
        }
    }

    float averagePower =
        totalPower /
        (float)((FFT_SIZE / 2) - 1U);

    uint8_t frequencyReliable =
        levelPresent &&
        (dominantPower >
         (averagePower *
          PEAK_CONFIDENCE_RATIO));

    /*
     * Parabolic interpolation around the dominant FFT bin.
     *
     * A 256-point FFT has 125 Hz raw bin spacing.
     * Interpolation gives a better estimate for a steady tone that lies
     * between two bins.
     */
    float dominantBinFloat =
        (float)dominantBin;

    if (frequencyReliable &&
        (dominantBin > 1U) &&
        (dominantBin <
         (FFT_SIZE / 2) - 1U))
    {
        float leftLog =
            logf(
                frequencySmoothPower[
                    dominantBin - 1U
                ] +
                0.000000001f
            );

        float centreLog =
            logf(
                frequencySmoothPower[
                    dominantBin
                ] +
                0.000000001f
            );

        float rightLog =
            logf(
                frequencySmoothPower[
                    dominantBin + 1U
                ] +
                0.000000001f
            );

        float denominator =
            leftLog -
            (2.0f * centreLog) +
            rightLog;

        if (fabsf(denominator) >
            0.000001f)
        {
            float delta =
                0.5f *
                (leftLog - rightLog) /
                denominator;

            if (delta > 0.5f)
            {
                delta = 0.5f;
            }

            if (delta < -0.5f)
            {
                delta = -0.5f;
            }

            dominantBinFloat +=
                delta;
        }
    }

    if (frequencyReliable)
    {
        float dominantFrequency =
            dominantBinFloat *
            SAMPLE_RATE_HZ /
            (float)FFT_SIZE;

        latestPeakFrequency =
            (uint32_t)(
                dominantFrequency +
                0.5f
            );

        lastPeakSignalTime =
            HAL_GetTick();
    }

    /*
     * Spectrum-bar silence detection remains based on FFT amplitude because
     * that part of the project was already working well.
     */
    float dominantAmplitude =
        sqrtf(dominantPower) *
        (4.0f / (float)FFT_SIZE);

    float spectrumPeakDb =
        20.0f *
        log10f(
            dominantAmplitude +
            0.000000001f
        );

    uint8_t spectrumSignalPresent =
        (spectrumPeakDb >=
         SILENCE_GATE_DB) ?
        1U :
        0U;

    /* =========================================================
     * 5. SPECTRUM BARS + PEAK HOLD
     * ========================================================= */
    const float maxBarHeight =
        265.0f;

    for (uint32_t bar = 0;
         bar < SPECTRUM_BARS;
         bar++)
    {
        float bandPeakPower = 0.0f;

        uint16_t startBin =
            bandEdges[bar];

        uint16_t endBin =
            bandEdges[bar + 1];

        for (uint16_t bin = startBin;
             bin < endBin;
             bin++)
        {
            if (fftMagnitude[bin] >
                bandPeakPower)
            {
                bandPeakPower =
                    fftMagnitude[bin];
            }
        }

        float targetHeight = 0.0f;

        if (spectrumSignalPresent)
        {
            float bandAmplitude =
                sqrtf(bandPeakPower) *
                (4.0f /
                 (float)FFT_SIZE);

            float bandDb =
                20.0f *
                log10f(
                    bandAmplitude +
                    0.000000001f
                );

            if (bandDb >
                DISPLAY_DB_MIN)
            {
                float normalizedHeight =
                    (bandDb -
                     DISPLAY_DB_MIN) /
                    (DISPLAY_DB_MAX -
                     DISPLAY_DB_MIN);

                if (normalizedHeight < 0.0f)
                {
                    normalizedHeight = 0.0f;
                }

                if (normalizedHeight > 1.0f)
                {
                    normalizedHeight = 1.0f;
                }

                targetHeight =
                    normalizedHeight *
                    maxBarHeight;
            }
        }

        float current =
            spectrumBarHeight[bar];

        float smoothing =
            (targetHeight > current) ?
            0.70f :
            0.22f;

        current +=
            smoothing *
            (targetHeight -
             current);

        if (current < 1.0f)
        {
            current = 0.0f;
        }

        spectrumBarHeight[bar] =
            current;

        if (current >=
            spectrumPeakHold[bar])
        {
            spectrumPeakHold[bar] =
                current;

            spectrumPeakHoldCounter[bar] =
                PEAK_HOLD_BLOCKS;
        }
        else if (
            spectrumPeakHoldCounter[bar] >
            0U)
        {
            spectrumPeakHoldCounter[bar]--;
        }
        else
        {
            spectrumPeakHold[bar] -=
                PEAK_FALL_PER_BLOCK;

            if (spectrumPeakHold[bar] <
                current)
            {
                spectrumPeakHold[bar] =
                    current;
            }

            if (spectrumPeakHold[bar] <
                0.0f)
            {
                spectrumPeakHold[bar] =
                    0.0f;
            }
        }
    }

    /* =========================================================
     * 6. UART DIAGNOSTICS
     * ========================================================= */
    static uint32_t printDivider = 0U;
    printDivider++;

    if (printDivider >= 20U)
    {
        printDivider = 0U;

        int length;

        if (levelPresent)
        {
            length = snprintf(
                micMessage,
                sizeof(micMessage),
                "Freq: %lu Hz  RMS: %.1f dBFS  Clip: %s\r\n",
                (unsigned long)
                    latestPeakFrequency,
                latestLevelDb,
                clipped ? "YES" : "NO"
            );
        }
        else
        {
            length = snprintf(
                micMessage,
                sizeof(micMessage),
                "Silence  RMS: %.1f dBFS\r\n",
                rmsDb
            );
        }

        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)micMessage,
            length,
            HAL_MAX_DELAY
        );
    }

    (void)peakRawLevel;
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

  MX_DMA_Init();

  MX_SPI1_Init();

  MX_I2S2_Init();

  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  LCD_Init();
  LCD_Fill(0x0000);

  {
      char message[] = "SYSTEM START\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)message,
          sizeof(message) - 1,
          HAL_MAX_DELAY
      );
  }

  arm_status fftStatus;

  fftStatus = arm_rfft_fast_init_f32(
      &fftInstance,
      FFT_SIZE
  );

  if (fftStatus != ARM_MATH_SUCCESS)
  {
      int length = snprintf(
          micMessage,
          sizeof(micMessage),
          "FFT INIT ERROR, status = %d\r\n",
          (int)fftStatus
      );

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)micMessage,
          length,
          HAL_MAX_DELAY
      );

      LCD_Fill(0xF800);
      Error_Handler();
  }

  /*
   * Pre-calculate Hann window once.
   */
  for (uint32_t i = 0; i < FFT_SIZE; i++)
  {
      hannWindow[i] =
          0.5f -
          0.5f * cosf(
              (2.0f * 3.14159265f * (float)i) /
              (float)(FFT_SIZE - 1)
          );
  }

  {
      char message[] = "FFT + HANN OK\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)message,
          sizeof(message) - 1,
          HAL_MAX_DELAY
      );
  }

  HAL_StatusTypeDef micStartStatus;

  micStartStatus = HAL_I2S_Receive_DMA(
      &hi2s2,
      micBuffer,
      MIC_FRAMES * 2
  );

  if (micStartStatus != HAL_OK)
  {
      char error[] =
          "I2S DMA START ERROR\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)error,
          sizeof(error) - 1,
          HAL_MAX_DELAY
      );

      LCD_Fill(0xF800);
      Error_Handler();
  }

  {
      char message[] = "I2S DMA OK\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)message,
          sizeof(message) - 1,
          HAL_MAX_DELAY
      );
  }

  lastLcdUpdate = HAL_GetTick();

  UI_SetMode(DISPLAY_MODE_SPECTRUM);

/* USER CODE END 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    UI_HandleButton();

    if (micHalfReady)
    {
        micHalfReady = 0;

        MIC_ProcessBlock(
            &micBuffer[0],
            MIC_FRAMES / 2
        );
    }

    if (micFullReady)
    {
        micFullReady = 0;

        MIC_ProcessBlock(
            &micBuffer[(MIC_FRAMES / 2) * 4],
            MIC_FRAMES / 2
        );
    }

    uint32_t now =
        HAL_GetTick();

    if ((now - lastLcdUpdate) >= LCD_REFRESH_MS)
    {
        lastLcdUpdate = now;
        UI_UpdateDisplay();
    }

    /*
     * Poll again after LCD work so a press is less likely to be missed.
     */
    UI_HandleButton();

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

  /** Macro to configure the PLL multiplication factor

  */

  __HAL_RCC_PLL_PLLM_CONFIG(16);

  /** Macro to configure the PLL clock source

  */

  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

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

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)

  {

    Error_Handler();

  }

  /** Initializes the CPU, AHB and APB buses clocks

  */

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK

                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)

  {

    Error_Handler();

  }

}

/**

  * @brief I2S2 Initialization Function

  * @param None

  * @retval None

  */

static void MX_I2S2_Init(void)

{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */

  hi2s2.Instance = SPI2;

  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;

  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;

  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;

  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;

  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;

  hi2s2.Init.CPOL = I2S_CPOL_LOW;

  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;

  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;

  if (HAL_I2S_Init(&hi2s2) != HAL_OK)

  {

    Error_Handler();

  }

  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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

  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

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

  * Enable DMA controller clock

  */

static void MX_DMA_Init(void)

{

  /* DMA controller clock enable */

  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */

  /* DMA1_Stream3_IRQn interrupt configuration */

  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);

  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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

  HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */

  HAL_GPIO_WritePin(GPIOA, DC_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */

  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BL_Pin */

  GPIO_InitStruct.Pin = BL_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(BL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin RST_Pin */

  GPIO_InitStruct.Pin = DC_Pin|RST_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */

  GPIO_InitStruct.Pin = CS_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /*
   * NUCLEO-F446RE blue USER button B1.
   * PC13 is active LOW, so use an input with pull-up.
   */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */

}

/* USER CODE BEGIN 4 */

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2)
    {
        micHalfReady = 1;
    }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2)
    {
        micFullReady = 1;
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
