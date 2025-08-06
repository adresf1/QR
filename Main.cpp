#include "main.h"
#include "string.h"

#define FRAME_WIDTH  320
#define FRAME_HEIGHT 240
#define FRAME_SIZE   (FRAME_WIDTH * FRAME_HEIGHT)

extern UART_HandleTypeDef huart3;
extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;

uint8_t framebuffer[FRAME_SIZE];
uint8_t rxBuf[100];

void take_snapshot() {
    HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)framebuffer, FRAME_SIZE / 4);
    while (HAL_DCMI_GetState(&hdcmi) != HAL_DCMI_STATE_READY);
}

void send_frame_to_pc() {
    const char* header = "<IMG_START>\n";
    const char* footer = "\n<IMG_END>\n";

    HAL_UART_Transmit(&huart3, (uint8_t*)header, strlen(header), HAL_MAX_DELAY);

    // Send billedet i blokke af fx 1024 bytes
    for (uint32_t i = 0; i < FRAME_SIZE; i += 1024) {
        uint32_t chunk = (FRAME_SIZE - i < 1024) ? FRAME_SIZE - i : 1024;
        HAL_UART_Transmit(&huart3, &framebuffer[i], chunk, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart3, (uint8_t*)footer, strlen(footer), HAL_MAX_DELAY);
}

void receive_qr_result() {
    memset(rxBuf, 0, sizeof(rxBuf));
    HAL_UART_Receive(&huart3, rxBuf, sizeof(rxBuf), HAL_MAX_DELAY);
    printf("QR Result: %s\r\n", rxBuf); // vis QR-data
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART3_UART_Init();  // UART til PC
    MX_DCMI_Init();         // Kamera
    MX_DMA_Init();          // DMA til DCMI

    HAL_Delay(2000); // Giv WSL tid til at starte server

    while (1)
    {
        take_snapshot();
        send_frame_to_pc();
        receive_qr_result();
        HAL_Delay(500); // 2 FPS
    }
}
