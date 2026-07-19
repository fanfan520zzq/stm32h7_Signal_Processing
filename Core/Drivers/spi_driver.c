#include "spi_driver.h"
#include "spi.h"
#include "module_state.h"
#include "timebase_driver.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_spi.h"

#define FPGA_CS_PORT GPIOF
#define FPGA_CS_PIN  LL_GPIO_PIN_9
#define FPGA_CS_HIGH_HOLD_CYCLES 64U

static volatile uint32_t spi_ll_transfer_count = 0U;
static volatile uint32_t spi_ll_timeout_count = 0U;
static volatile uint32_t spi_ll_error_count = 0U;

static void SPI_Driver_WaitCycles(uint32_t cycles) {
    uint32_t start = Timebase_Driver_Now();
    while ((uint32_t)(Timebase_Driver_Now() - start) < cycles) {
        __NOP();
    }
}

void SPI_Driver_Init(void) {
    // CubeMX MX_SPI2_Init() already called in main.c
    // Ensure CS pin (PF9) is high
    Timebase_Driver_Init();
    LL_GPIO_SetOutputPin(FPGA_CS_PORT, FPGA_CS_PIN);
}

int32_t SPI_Driver_AnchorPulse(uint32_t min_low_cycles, SPI_AnchorTiming_t *timing) {
    SPI_AnchorTiming_t local;
    uint32_t primask;

    if (timing == NULL) return ERR_PARAM;
    if (!Timebase_Driver_IsRunning()) return ERR_NOT_READY;
    if (min_low_cycles < 32U) min_low_cycles = 32U;

    primask = __get_PRIMASK();
    __disable_irq();
    local.envelope_start = DWT->CYCCNT;
    LL_GPIO_ResetOutputPin(FPGA_CS_PORT, FPGA_CS_PIN);
    local.cs_low_observed = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - local.cs_low_observed) < min_low_cycles) {
        __NOP();
    }
    local.cs_high_requested = DWT->CYCCNT;
    LL_GPIO_SetOutputPin(FPGA_CS_PORT, FPGA_CS_PIN);
    local.envelope_end = DWT->CYCCNT;
    if (primask == 0U) __enable_irq();

    local.low_cycles = local.cs_high_requested - local.cs_low_observed;
    local.uncertainty_cycles =
        (local.cs_low_observed - local.envelope_start) +
        (local.envelope_end - local.cs_high_requested);
    *timing = local;
    return ERR_OK;
}

static void SPI_Driver_LLClose(void) {
    LL_SPI_ClearFlag_EOT(SPI2);
    LL_SPI_ClearFlag_TXTF(SPI2);
    LL_SPI_Disable(SPI2);
}

int32_t SPI_Driver_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
    uint16_t tx_count = 0U;
    uint16_t rx_count = 0U;
    uint32_t start;
    uint32_t timeout_cycles;

    if (pTxData == NULL || pRxData == NULL || Size == 0U) return ERR_PARAM;
    if (!Timebase_Driver_IsRunning()) return ERR_NOT_READY;

    timeout_cycles = (SystemCoreClock / 1000U) * Timeout;
    if (timeout_cycles == 0U) timeout_cycles = 1U;

    if (LL_SPI_IsEnabled(SPI2)) LL_SPI_Disable(SPI2);
    LL_SPI_ClearFlag_EOT(SPI2);
    LL_SPI_ClearFlag_TXTF(SPI2);
    LL_SPI_ClearFlag_OVR(SPI2);
    LL_SPI_ClearFlag_UDR(SPI2);
    LL_SPI_ClearFlag_MODF(SPI2);
    LL_SPI_SetTransferSize(SPI2, Size);
    LL_SPI_Enable(SPI2);

    start = DWT->CYCCNT;
    /* STM32H7 master mode can clock immediately after CSTART. Preload the TX FIFO
       so cache/interrupt latency cannot create an underrun at the start of a frame. */
    while (tx_count < Size && LL_SPI_IsActiveFlag_TXP(SPI2)) {
        LL_SPI_TransmitData8(SPI2, pTxData[tx_count++]);
    }
    if (tx_count == 0U) {
        SPI_Driver_LLClose();
        spi_ll_timeout_count++;
        return ERR_TIMEOUT;
    }
    LL_GPIO_ResetOutputPin(FPGA_CS_PORT, FPGA_CS_PIN);
    LL_SPI_StartMasterTransfer(SPI2);

    while (tx_count < Size || rx_count < Size) {
        if (tx_count < Size && LL_SPI_IsActiveFlag_TXP(SPI2)) {
            LL_SPI_TransmitData8(SPI2, pTxData[tx_count++]);
        }
        if (rx_count < Size && LL_SPI_IsActiveFlag_RXP(SPI2)) {
            pRxData[rx_count++] = LL_SPI_ReceiveData8(SPI2);
        }
        if ((uint32_t)(DWT->CYCCNT - start) >= timeout_cycles) {
            LL_GPIO_SetOutputPin(FPGA_CS_PORT, FPGA_CS_PIN);
            SPI_Driver_LLClose();
            spi_ll_timeout_count++;
            return ERR_TIMEOUT;
        }
    }

    while (!LL_SPI_IsActiveFlag_EOT(SPI2)) {
        if ((uint32_t)(DWT->CYCCNT - start) >= timeout_cycles) {
            LL_GPIO_SetOutputPin(FPGA_CS_PORT, FPGA_CS_PIN);
            SPI_Driver_LLClose();
            spi_ll_timeout_count++;
            return ERR_TIMEOUT;
        }
    }

    if (LL_SPI_IsActiveFlag_UDR(SPI2) || LL_SPI_IsActiveFlag_OVR(SPI2) ||
        LL_SPI_IsActiveFlag_MODF(SPI2)) {
        LL_GPIO_SetOutputPin(FPGA_CS_PORT, FPGA_CS_PIN);
        SPI_Driver_LLClose();
        LL_SPI_ClearFlag_UDR(SPI2);
        LL_SPI_ClearFlag_OVR(SPI2);
        LL_SPI_ClearFlag_MODF(SPI2);
        spi_ll_error_count++;
        return ERR_HARDWARE;
    }

    LL_GPIO_SetOutputPin(FPGA_CS_PORT, FPGA_CS_PIN);
    SPI_Driver_LLClose();
    spi_ll_transfer_count++;
    return ERR_OK;
}

void SPI_Driver_GetLLStats(uint32_t *transfers, uint32_t *timeouts, uint32_t *errors) {
    if (transfers != NULL) *transfers = spi_ll_transfer_count;
    if (timeouts != NULL) *timeouts = spi_ll_timeout_count;
    if (errors != NULL) *errors = spi_ll_error_count;
}

int32_t SPI_Driver_TransferFrame(const uint8_t tx[4], uint8_t rx[4]) {
    if (tx == NULL || rx == NULL) return ERR_PARAM;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int32_t res = SPI_Driver_TransmitReceive((uint8_t *)tx, rx, 4U, 100U);
    if (primask == 0U) __enable_irq();
    SPI_Driver_WaitCycles(FPGA_CS_HIGH_HOLD_CYCLES);
    return res;
}
