#pragma once

#include <cstdint>

/**
 * @brief Class for converting RGB565 images to grayscale
 */
class ImageConverter {
public:
    /**
     * @brief Constructor
     */
    ImageConverter() = default;
    
    /**
     * @brief Destructor
     */
    ~ImageConverter() = default;
    
    /**
     * @brief Convert RGB565 pixel to grayscale
     * @param pixel RGB565 pixel value
     * @return Grayscale value (0-255)
     */
    static uint8_t rgb565ToGray(uint16_t pixel);
    
    /**
     * @brief Convert RGB565 buffer to grayscale buffer
     * @param src Source RGB565 buffer
     * @param dst Destination grayscale buffer
     * @param width Image width
     * @param height Image height
     */
    static void convertBufferToGrayscale(const uint8_t* src, uint8_t* dst, int width, int height);
    
private:
    // Private helper methods if needed
};



----------------------------------------------



#include "ImageConverter.hpp"

uint8_t ImageConverter::rgb565ToGray(uint16_t pixel) {
    // Extract RGB components from RGB565
    uint8_t r = (pixel >> 11) & 0x1F;  // 5 bits red
    uint8_t g = (pixel >> 5) & 0x3F;   // 6 bits green
    uint8_t b = pixel & 0x1F;          // 5 bits blue
    
    // Convert to 8-bit values
    uint8_t r8 = (r << 3) | (r >> 2);  // 5 bits to 8 bits
    uint8_t g8 = (g << 2) | (g >> 4);  // 6 bits to 8 bits
    uint8_t b8 = (b << 3) | (b >> 2);  // 5 bits to 8 bits
    
    // Convert to grayscale using standard luminance formula
    return static_cast<uint8_t>((r8 * 30 + g8 * 59 + b8 * 11) / 100);
}

void ImageConverter::convertBufferToGrayscale(const uint8_t* src, uint8_t* dst, int width, int height) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int index = (y * width + x) * 2;
            uint16_t pixel = src[index] | (src[index + 1] << 8);
            
            dst[y * width + x] = rgb565ToGray(pixel);
        }
    }
}

-------------------------------------------
#pragma once

#include "quirc.h"
#include "ImageConverter.hpp"
#include <cstdint>
#include <functional>

/**
 * @brief Callback function type for QR code detection
 * @param qrData Decoded QR code data
 * @param index Index of the QR code (if multiple found)
 */
using QRCodeCallback = std::function<void(const char* qrData, int index)>;

/**
 * @brief Class for scanning QR codes from image buffers
 */
class QRScanner {
public:
    /**
     * @brief Constructor
     * @param width Image width
     * @param height Image height
     */
    QRScanner(int width, int height);
    
    /**
     * @brief Destructor
     */
    ~QRScanner();
    
    /**
     * @brief Scan for QR codes in RGB565 buffer
     * @param buffer RGB565 image buffer
     * @param callback Callback function to handle found QR codes
     * @return Number of QR codes found
     */
    int scanFromRGB565Buffer(const uint8_t* buffer, QRCodeCallback callback = nullptr);
    
    /**
     * @brief Get the last error message
     * @return Error message string
     */
    const char* getLastError() const { return lastError; }
    
    /**
     * @brief Check if scanner is initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const { return (qr != nullptr); }

private:
    struct quirc* qr;           ///< QUIRC decoder instance
    int imageWidth;             ///< Image width
    int imageHeight;            ///< Image height
    const char* lastError;      ///< Last error message
    
    // Disable copy constructor and assignment operator
    QRScanner(const QRScanner&) = delete;
    QRScanner& operator=(const QRScanner&) = delete;
    
    /**
     * @brief Initialize the QUIRC decoder
     * @return true if successful, false otherwise
     */
    bool initializeDecoder();
    
    /**
     * @brief Process found QR codes
     * @param callback Callback function to handle found QR codes
     * @return Number of successfully decoded QR codes
     */
    int processQRCodes(QRCodeCallback callback);
};

-----------------------------
#include "QRScanner.hpp"
#include <cstdio>

QRScanner::QRScanner(int width, int height) 
    : qr(nullptr), imageWidth(width), imageHeight(height), lastError("No error") {
    initializeDecoder();
}

QRScanner::~QRScanner() {
    if (qr) {
        quirc_destroy(qr);
        qr = nullptr;
    }
}

bool QRScanner::initializeDecoder() {
    qr = quirc_new();
    if (!qr) {
        lastError = "Could not allocate QR decoder";
        return false;
    }
    
    if (quirc_resize(qr, imageWidth, imageHeight) < 0) {
        lastError = "Failed to resize QR decoder";
        quirc_destroy(qr);
        qr = nullptr;
        return false;
    }
    
    return true;
}

int QRScanner::scanFromRGB565Buffer(const uint8_t* buffer, QRCodeCallback callback) {
    if (!qr) {
        lastError = "QR decoder not initialized";
        return -1;
    }
    
    if (!buffer) {
        lastError = "Buffer is null";
        return -1;
    }
    
    // Get grayscale buffer from QUIRC
    uint8_t* grayBuffer = quirc_begin(qr, nullptr, nullptr);
    if (!grayBuffer) {
        lastError = "Failed to get grayscale buffer from QUIRC";
        return -1;
    }
    
    // Convert RGB565 to grayscale
    ImageConverter::convertBufferToGrayscale(buffer, grayBuffer, imageWidth, imageHeight);
    
    // End the image input process
    quirc_end(qr);
    
    // Process and decode QR codes
    return processQRCodes(callback);
}

int QRScanner::processQRCodes(QRCodeCallback callback) {
    int count = quirc_count(qr);
    int decodedCount = 0;
    
    printf("QR codes found: %d\r\n", count);
    
    for (int i = 0; i < count; i++) {
        struct quirc_code code;
        struct quirc_data data;
        
        quirc_extract(qr, i, &code);
        int result = quirc_decode(&code, &data);
        
        if (result == QUIRC_SUCCESS) {
            printf("QR DECODED [%d]: %s\r\n", i, data.payload);
            decodedCount++;
            
            // Call callback if provided
            if (callback) {
                callback(reinterpret_cast<const char*>(data.payload), i);
            }
        } else {
            printf("QR decode failed for code %d, error code: %d\r\n", i, result);
        }
    }
    
    return decodedCount;
}
-----------------------------------------------
* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body (C++ version)
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
#include "QRScanner.hpp"
#include "ImageConverter.hpp"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "isp_api.h"
#include "imx335_E27_isp_param_conf.h"
#include <cstdio>
#include <cstring>
#include <memory>
/* USER CODE END Includes */

#include "stm32n6xx_hal_rcc.h"

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
DCMIPP_HandleTypeDef hdcmipp;
LTDC_HandleTypeDef hltdc;
ISP_HandleTypeDef hcamera_isp;

/* USER CODE BEGIN PV */
static volatile uint32_t NbMainFrames = 0;
static IMX335_Object_t IMX335Obj;
static int32_t isp_gain;
static int32_t isp_exposure;

// C++ objects for QR scanning
static std::unique_ptr<QRScanner> qrScanner;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DCMIPP_Init(void);
static void LCD_Init(uint32_t Width, uint32_t Height);

/* USER CODE BEGIN PFP */
static void IMX335_Probe(uint32_t Resolution, uint32_t PixelFormat);
static ISP_StatusTypeDef GetSensorInfoHelper(uint32_t Instance, ISP_SensorInfoTypeDef *SensorInfo);
static ISP_StatusTypeDef SetSensorGainHelper(uint32_t Instance, int32_t Gain);
static ISP_StatusTypeDef GetSensorGainHelper(uint32_t Instance, int32_t *Gain);
static ISP_StatusTypeDef SetSensorExposureHelper(uint32_t Instance, int32_t Exposure);
static ISP_StatusTypeDef GetSensorExposureHelper(uint32_t Instance, int32_t *Exposure);

// C++ QR scanning function
void ScanQRCodeFromBuffer();

// QR code callback function
void onQRCodeFound(const char* qrData, int index);
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
    ISP_AppliHelpersTypeDef appliHelpers = {0};
    /* USER CODE END 1 */

    /* Enable the CPU Cache */
    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();

    /* Enable D-Cache---------------------------------------------------------*/
    SCB_EnableDCache();

    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    printf("Hello World from C++!\r\n");
    
    // Activating the ram
    __HAL_RCC_DCMIPP_CLK_ENABLE();
    __HAL_RCC_AXISRAM5_MEM_IS_CLK_ENABLED();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_RED);
    
    /* UART log */
#if USE_COM_LOG
    COM_InitTypeDef COM_Init;

    /* Initialize COM init structure */
    COM_Init.BaudRate   = 115200;
    COM_Init.WordLength = COM_WORDLENGTH_8B;
    COM_Init.StopBits   = COM_STOPBITS_1;
    COM_Init.Parity     = COM_PARITY_NONE;
    COM_Init.HwFlowCtl  = COM_HWCONTROL_NONE;

    BSP_COM_Init(COM1, &COM_Init);

    if (BSP_COM_SelectLogPort(COM1) != BSP_ERROR_NONE) {
        Error_Handler();
    }
#endif
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_DCMIPP_Init();

    /* Initialize the IMX335 Sensor ----------------------------- */
    IMX335_Probe(IMX335_R2592_1944, IMX335_RAW_RGGB10);

    /* USER CODE BEGIN 2 */
    LCD_Init(FRAME_WIDTH, FRAME_HEIGHT);

    /* Fill init struct with Camera driver helpers */
    appliHelpers.GetSensorInfo = GetSensorInfoHelper;
    appliHelpers.SetSensorGain = SetSensorGainHelper;
    appliHelpers.GetSensorGain = GetSensorGainHelper;
    appliHelpers.SetSensorExposure = SetSensorExposureHelper;
    appliHelpers.GetSensorExposure = GetSensorExposureHelper;

    /* Initialize the Image Signal Processing middleware */
    if (ISP_Init(&hcamera_isp, &hdcmipp, 0, &appliHelpers, ISP_IQParamCacheInit[0]) != ISP_OK) {
        Error_Handler();
    }

    if (HAL_DCMIPP_CSI_PIPE_Start(&hdcmipp, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0, BUFFER_ADDRESS, DCMIPP_MODE_CONTINUOUS) != HAL_OK) {
        Error_Handler();
    }

    /* Start the Image Signal Processing */
    if (ISP_Start(&hcamera_isp) != ISP_OK) {
        Error_Handler();
    }
    
    // Initialize QR Scanner
    qrScanner = std::make_unique<QRScanner>(800, 480);
    if (!qrScanner->isInitialized()) {
        printf("Failed to initialize QR Scanner: %s\r\n", qrScanner->getLastError());
        BSP_LED_On(LED_RED);
    } else {
        printf("QR Scanner initialized successfully\r\n");
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        BSP_LED_Toggle(LED_GREEN);
        printf("Frame count: %lu\r\n", NbMainFrames);

        if (ISP_BackgroundProcess(&hcamera_isp) != ISP_OK) {
            BSP_LED_Toggle(LED_RED);
        }

        if (ISP_BackgroundProcess(&hcamera_isp) != ISP_OK) {
            BSP_LED_Toggle(LED_RED);
        }

        // Scan for QR codes using C++ class
        ScanQRCodeFromBuffer();
    }
    /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief QR code scanning function using C++ classes
 */
void ScanQRCodeFromBuffer()
{
    if (!qrScanner) {
        printf("QR Scanner not initialized\r\n");
        return;
    }
    
    const uint8_t* buffer = reinterpret_cast<const uint8_t*>(BUFFER_ADDRESS);
    
    // Scan for QR codes with callback
    int decodedCount = qrScanner->scanFromRGB565Buffer(buffer, onQRCodeFound);
    
    if (decodedCount < 0) {
        printf("QR scan error: %s\r\n", qrScanner->getLastError());
    }
}

/**
 * @brief Callback function called when QR code is found
 * @param qrData Decoded QR code data
 * @param index Index of the QR code
 */
void onQRCodeFound(const char* qrData, int index)
{
    printf("QR Code %d callback: %s\r\n", index, qrData);
    
    // Add your custom QR code handling logic here
    // For example, you could:
    // - Store the QR code data in a buffer
    // - Send it over UART/USB/Ethernet
    // - Trigger specific actions based on content
    // - Light up different LEDs based on QR code content
    
    BSP_LED_Toggle(LED_GREEN);
}

/**
  * @brief  Register Bus IOs if component ID is OK
  * @retval error status
  */
static void IMX335_Probe(uint32_t Resolution, uint32_t PixelFormat)
{
    IMX335_IO_t IOCtx;
    uint32_t id;

    /* Configure the camera driver */
    IOCtx.Address     = CAMERA_IMX335_ADDRESS;
    IOCtx.Init        = BSP_I2C1_Init;
    IOCtx.DeInit      = BSP_I2C1_DeInit;
    IOCtx.ReadReg     = BSP_I2C1_ReadReg16;
    IOCtx.WriteReg    = BSP_I2C1_WriteReg16;
    IOCtx.GetTick     = BSP_GetTick;

    if (IMX335_RegisterBusIO(&IMX335Obj, &IOCtx) != IMX335_OK) {
        Error_Handler();
    }
    else if (IMX335_ReadID(&IMX335Obj, &id) != IMX335_OK) {
        Error_Handler();
    }
    else {
        if (id != static_cast<uint32_t>(IMX335_CHIP_ID)) {
            Error_Handler();
        }
        else {
            if (IMX335_Init(&IMX335Obj, Resolution, PixelFormat) != IMX335_OK) {
                Error_Handler();
            }
            else if (IMX335_SetFrequency(&IMX335Obj, IMX335_INCK_24MHZ) != IMX335_OK) {
                Error_Handler();
            }
        }
    }
}

/**
  * @brief  ISP Middleware helper. Camera sensor info getter
  * @retval ISP Status
  */
static ISP_StatusTypeDef GetSensorInfoHelper(uint32_t Instance, ISP_SensorInfoTypeDef *SensorInfo)
{
    UNUSED(Instance);
    return static_cast<ISP_StatusTypeDef>(IMX335_GetSensorInfo(&IMX335Obj, reinterpret_cast<IMX335_SensorInfo_t*>(SensorInfo)));
}

/**
  * @brief  ISP Middleware helper. Camera gain setter
  * @retval ISP Status
  */
static ISP_StatusTypeDef SetSensorGainHelper(uint32_t Instance, int32_t Gain)
{
    UNUSED(Instance);
    isp_gain = Gain;
    return static_cast<ISP_StatusTypeDef>(IMX335_SetGain(&IMX335Obj, Gain));
}

/**
  * @brief  ISP Middleware helper. Camera gain getter
  * @retval ISP Status
  */
static ISP_StatusTypeDef GetSensorGainHelper(uint32_t Instance, int32_t *Gain)
{
    UNUSED(Instance);
    *Gain = isp_gain;
    return ISP_OK;
}

/**
  * @brief  ISP Middleware helper. Camera exposure setter
  * @retval ISP Status
  */
static ISP_StatusTypeDef SetSensorExposureHelper(uint32_t Instance, int32_t Exposure)
{
    UNUSED(Instance);
    isp_exposure = Exposure;
    return static_cast<ISP_StatusTypeDef>(IMX335_SetExposure(&IMX335Obj, Exposure));
}

/**
  * @brief  ISP Middleware helper. Camera exposure getter
  * @retval ISP Status
  */
static ISP_StatusTypeDef GetSensorExposureHelper(uint32_t Instance, int32_t *Exposure)
{
    UNUSED(Instance);
    *Exposure = isp_exposure;
    return ISP_OK;
}
/* USER CODE END 4 */

// C functions that need to remain as C functions for HAL callbacks
extern "C" {
    void HAL_DCMIPP_PIPE_FrameEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
    {
        NbMainFrames++;
    }

    /**
     * @brief  Vsync Event callback on pipe
     * @param  hdcmipp DCMIPP device handle
     *         Pipe    Pipe receiving the callback
     * @retval None
     */
    void HAL_DCMIPP_PIPE_VsyncEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
    {
        UNUSED(hdcmipp);
        /* Update the frame counter and call the ISP statistics handler */
        switch (Pipe) {
            case DCMIPP_PIPE0:
                ISP_IncDumpFrameId(&hcamera_isp);
                break;
            case DCMIPP_PIPE1:
                ISP_IncMainFrameId(&hcamera_isp);
                ISP_GatherStatistics(&hcamera_isp);
                break;
            case DCMIPP_PIPE2:
                ISP_IncAncillaryFrameId(&hcamera_isp);
                break;
        }
    }

    /**
      * @brief  This function is executed in case of error occurrence.
      * @retval None
      */
    void Error_Handler(void)
    {
        /* USER CODE BEGIN Error_Handler_Debug */
        /* User can add his own implementation to report the HAL error return state */
        while (1) {
            HAL_Delay(250);
            BSP_LED_Toggle(LED_RED);
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
        /* Infinite loop */
        while (1) {
        }
        /* USER CODE END 6 */
    }
#endif /* USE_FULL_ASSERT */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    /** Configure the System Power Supply */
    if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK) {
        Error_Handler();
    }

    /** Enable HSI */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Get current CPU/System buses clocks configuration and
       if necessary switch to intermediate HSI clock to ensure target clock can be set */
    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
    if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
        (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11)) {
        RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
        RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) {
            Error_Handler();
        }
    }

    /** Initializes the RCC Oscillators according to the specified parameters
       in the RCC_OscInitTypeDef structure. */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
    RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL1.PLLM = 4;
    RCC_OscInitStruct.PLL1.PLLN = 75;
    RCC_OscInitStruct.PLL1.PLLFractional = 0;
    RCC_OscInitStruct.PLL1.PLLP1 = 1;
    RCC_OscInitStruct.PLL1.PLLP2 = 1;
    RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_HCLK
                                | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
                                | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK5
                                | RCC_CLOCKTYPE_PCLK4;
    RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
    RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;
    RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
