#include "stn_sniffer.hpp"
#include <Arduino.h>

static STNSniffer* g_sniffer = nullptr;

static void exti_lp_callback(void) {
    if (g_sniffer) g_sniffer->handleHsync();
}

static void exti_yd_callback(void) {
    if (g_sniffer) g_sniffer->handleVsync();
}

void STNSniffer::init() {
    g_sniffer = this;

    // 1. Configure Data Pins (PE0 - PE3) as Inputs
    pinMode(PE0, INPUT);
    pinMode(PE1, INPUT);
    pinMode(PE2, INPUT);
    pinMode(PE3, INPUT);

    // 2. Configure XSCL clock pin on PA8 (TIM1_CH1)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3U << (8 * 2));
    GPIOA->MODER |=  (2U << (8 * 2)); // AF mode
    GPIOA->AFR[1] &= ~(0xF << ((8 - 8) * 4));
    GPIOA->AFR[1] |=  (1U << ((8 - 8) * 4)); // AF1 = TIM1
    GPIOA->OSPEEDR |= (3U << (8 * 2));

    // 3. Configure Sync Pins (LP on PE8, YD on PE9)
    pinMode(PE8, INPUT_PULLUP);
    pinMode(PE9, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PE8), exti_lp_callback, RISING);
    attachInterrupt(digitalPinToInterrupt(PE9), exti_yd_callback, RISING);

    // 4. Initialize TIM1 in External Clock Mode 1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->CR1 = 0;
    TIM1->PSC = 0;
    TIM1->ARR = 0xFFFF;
    TIM1->CCMR1 = (1 << TIM_CCMR1_CC1S_Pos); // CC1 as TI1 input
    TIM1->SMCR  = (5 << TIM_SMCR_TS_Pos) | (7 << TIM_SMCR_SMS_Pos); // TI1FP1, External Clock Mode 1
    TIM1->DIER  = TIM_DIER_CC1DE;          // Enable DMA request on capture event
    TIM1->BDTR |= TIM_BDTR_MOE;            // Advanced timer main output enable
    TIM1->CR1  |= TIM_CR1_CEN;             // Enable timer counter

    // 5. Initialize DMA2 Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Clear frame buffers
    memset(buffer_a_, 0, sizeof(buffer_a_));
    memset(buffer_b_, 0, sizeof(buffer_b_));
}

void STNSniffer::start() {
    current_line_ = 0;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void STNSniffer::stop() {
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    TIM1->CR1 &= ~TIM_CR1_CEN;
}

void STNSniffer::handleHsync() {
    // LP Interrupt (PE8): Setup DMA2 Stream 1 (Channel 6) for the incoming scanline
    if (current_line_ < TOTAL_SCAN_LINES) {
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        while (DMA2_Stream1->CR & DMA_SxCR_EN);

        // Clear interrupt flags for Stream 1
        DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | 
                      DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1;// | DMA_LIFCR_FEIF1;

        DMA2_Stream1->PAR  = (uint32_t)&(GPIOE->IDR);
        DMA2_Stream1->M0AR = (uint32_t)(back_buffer_ + (current_line_ * CLOCKS_PER_LINE));
        DMA2_Stream1->NDTR = CLOCKS_PER_LINE;
        DMA2_Stream1->CR   = (6 << DMA_SxCR_CHSEL_Pos) | // Channel 6 (TIM1_CH1)
                             DMA_SxCR_MINC             | // Memory increment mode
                             (0 << DMA_SxCR_MSIZE_Pos) | // 8-bit memory
                             (0 << DMA_SxCR_PSIZE_Pos) | // 8-bit peripheral
                             (0 << DMA_SxCR_DIR_Pos)   | // Peripheral-to-memory
                             DMA_SxCR_EN;                // Enable DMA Stream

        current_line_++;
    }
}

void STNSniffer::handleVsync() {
    // YD Interrupt (PE9): Reset scanline index and swap ping-pong buffers
    current_line_ = 0;

    volatile uint8_t* temp = front_buffer_;
    front_buffer_ = back_buffer_;
    back_buffer_  = temp;
}
