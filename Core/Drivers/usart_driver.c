#include "usart_driver.h"
#include <string.h>

#define RING_BUF_SIZE 256
#define DMA_RX_BUF_SIZE 128

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t *dma_rx_buf;
    uint8_t ring_buf[RING_BUF_SIZE];
    uint16_t ring_head;
    uint16_t ring_tail;
} USART_Driver_t;

static uint8_t usart1_dma_rx_buf[DMA_RX_BUF_SIZE] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
static uint8_t usart3_dma_rx_buf[DMA_RX_BUF_SIZE] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));

static USART_Driver_t usart1_drv = {.dma_rx_buf = usart1_dma_rx_buf};
static USART_Driver_t usart3_drv = {.dma_rx_buf = usart3_dma_rx_buf};

static USART_Driver_t* get_drv(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) return &usart1_drv;
    if (huart->Instance == USART3) return &usart3_drv;
    return NULL;
}

void USART_Driver_Init(UART_HandleTypeDef *huart) {
    USART_Driver_t *drv = get_drv(huart);
    if (!drv) return;
    
    drv->huart = huart;
    drv->ring_head = 0;
    drv->ring_tail = 0;
    
    HAL_UARTEx_ReceiveToIdle_DMA(huart, drv->dma_rx_buf, DMA_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // Disable half transfer IT to reduce overhead
}

uint8_t USART_Driver_ReadByte(UART_HandleTypeDef *huart, uint8_t *byte) {
    USART_Driver_t *drv = get_drv(huart);
    if (!drv) return 0;
    
    if (drv->ring_head == drv->ring_tail) return 0;
    *byte = drv->ring_buf[drv->ring_tail];
    drv->ring_tail = (drv->ring_tail + 1) % RING_BUF_SIZE;
    return 1;
}

void USART_Driver_WriteBytes(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len) {
    // Blocking transmission for simplicity in decoupling.
    // If DMA is preferred, we need to ensure the buffer outlives the DMA transfer.
    HAL_UART_Transmit(huart, (uint8_t*)data, len, HAL_MAX_DELAY);
}

void USART_Driver_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    USART_Driver_t *drv = get_drv(huart);
    if (!drv) return;
    
    // Invalidate D-Cache if the DMA buffer is in D1 RAM
    SCB_InvalidateDCache_by_Addr((uint32_t*)drv->dma_rx_buf, DMA_RX_BUF_SIZE);
    
    for (uint16_t i = 0; i < Size; i++) {
        uint16_t next_head = (drv->ring_head + 1) % RING_BUF_SIZE;
        if (next_head != drv->ring_tail) { // Not full
            drv->ring_buf[drv->ring_head] = drv->dma_rx_buf[i];
            drv->ring_head = next_head;
        }
    }
    
    // Restart DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(huart, drv->dma_rx_buf, DMA_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
