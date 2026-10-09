#include "stn_sniffer.hpp"
#include <Arduino.h>

STNSniffer& STNSniffer::instance() {
    static STNSniffer inst;
    return inst;
}

// Global C-wrapper callbacks for attachInterrupt
static void hsync_isr() {
    STNSniffer::instance().onHsync();
}

static void vsync_isr() {
    STNSniffer::instance().onVsync();
}

void STNSniffer::init() {
    // 1. Enable Peripheral Clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // 2. Configure PE0-PE3 as Data Inputs
    GPIO_InitTypeDef gpio_struct = {0};
    gpio_struct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    gpio_struct.Mode  = GPIO_MODE_INPUT;
    gpio_struct.Pull  = GPIO_NOPULL;
    gpio_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // High-speed config for 8 MHz sampling
    HAL_GPIO_Init(GPIOE, &gpio_struct);

    // 3. Configure Line & Frame Sync via attachInterrupt
    pinMode(PE8, INPUT_PULLDOWN); // LP
    pinMode(PE9, INPUT_PULLDOWN); // YD

    attachInterrupt(digitalPinToInterrupt(PE8), hsync_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(PE9), vsync_isr, RISING);

    // 4. Configure PA6 for 8 MHz TIM3_CH1 clock input
    gpio_struct.Pin       = GPIO_PIN_6;
    gpio_struct.Mode      = GPIO_MODE_AF_PP;
    gpio_struct.Pull      = GPIO_NOPULL;
    gpio_struct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_struct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio_struct);

    // 5. Configure TIM3 Capture for 8 MHz sampling
    htim3_.Instance           = TIM3;
    htim3_.Init.Prescaler     = 0;
    htim3_.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim3_.Init.Period        = 0xFFFF;
    htim3_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&htim3_);

    TIM_IC_InitTypeDef ic_config = {0};
    ic_config.ICPolarity  = TIM_ICPOLARITY_FALLING; // Sample data on falling edge for stability
    ic_config.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic_config.ICPrescaler = TIM_ICPSC_DIV1;
    ic_config.ICFilter    = 0; // Disable filter for 8 MHz signal
    HAL_TIM_IC_ConfigChannel(&htim3_, &ic_config, TIM_CHANNEL_1);

    // 6. Configure DMA2 Stream 4 Channel 5
    hdma_tim3_ch1_.Instance                 = DMA2_Stream4;
    hdma_tim3_ch1_.Init.Channel             = DMA_CHANNEL_5;
    hdma_tim3_ch1_.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_tim3_ch1_.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tim3_ch1_.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tim3_ch1_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tim3_ch1_.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tim3_ch1_.Init.Mode                = DMA_NORMAL;
    hdma_tim3_ch1_.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_tim3_ch1_.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdma_tim3_ch1_);
    __HAL_LINKDMA(&htim3_, hdma[TIM_DMA_ID_CC1], hdma_tim3_ch1_);
}
void STNSniffer::start() {
    current_line_ = 0;
    __HAL_TIM_ENABLE(&htim3_);
}

void STNSniffer::stop() {
    HAL_DMA_Abort(&hdma_tim3_ch1_);
    __HAL_TIM_DISABLE(&htim3_);
}

void STNSniffer::onHsync() {
    HAL_DMA_Abort(&hdma_tim3_ch1_);

    if (current_line_ < TOTAL_SCAN_LINES) {
        uint32_t line_offset = current_line_ * CLOCKS_PER_LINE;
        uint32_t dest_addr   = reinterpret_cast<uint32_t>(&frame_buffer_[active_buffer_][line_offset]);
        uint32_t src_addr    = reinterpret_cast<uint32_t>(&(GPIOE->IDR));

        // Capture 80 XSCL clock nibble transfers for the active line
        HAL_DMA_Start(&hdma_tim3_ch1_, src_addr, dest_addr, CLOCKS_PER_LINE);
        __HAL_TIM_ENABLE_DMA(&htim3_, TIM_DMA_CC1);

        current_line_++;
    }
}

void STNSniffer::onVsync() {
    current_line_ = 0;
    active_buffer_ ^= 1; // Double buffer swap
}
