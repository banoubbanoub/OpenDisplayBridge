#include "stn_sniffer.hpp"
#include <Arduino.h>

static STNSniffer* g_sniffer = nullptr;

// Raw EXTI Vector for PE8 (HSYNC/LP) and PE9 (VSYNC/YD)
extern "C" void EXTI9_5_IRQHandler(void) {
    if (g_sniffer) {
        // Handle HSYNC (LP) on PE8 (EXTI Line 8)
        if (EXTI->PR & (1 << 8)) {
            EXTI->PR = (1 << 8); // Clear pending flag
            g_sniffer->handleHsync();
        }

        // Handle VSYNC (YD) on PE9 (EXTI Line 9)
        if (EXTI->PR & (1 << 9)) {
            EXTI->PR = (1 << 9); // Clear pending flag
            g_sniffer->handleVsync();
        }
    }
}

void STNSniffer::init() {
    g_sniffer = this;

    // 1. Configure Data Pins (PD0 - PD3) as Inputs to avoid PE0-PE3 accelerometer conflict
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~((3U << (0 * 2)) | (3U << (1 * 2)) | (3U << (2 * 2)) | (3U << (3 * 2)));

    // 2. Configure XSCL clock pin on PA8 (TIM1_CH1)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3U << (8 * 2));
    GPIOA->MODER |=  (2U << (8 * 2)); // AF mode
    GPIOA->AFR[1] &= ~(0xF << ((8 - 8) * 4));
    GPIOA->AFR[1] |=  (1U << ((8 - 8) * 4)); // AF1 = TIM1
    GPIOA->OSPEEDR |= (3U << (8 * 2));

    // 3. Configure Sync Pins (LP on PE8, YD on PE9) with Raw EXTI Registers
 
  // 1. Enable GPIOE clock
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

// 2. Configure PE8 and PE9 as Input Mode (MODER = 00)
GPIOE->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)));

// 3. Configure Pull-Up Resistors for PE8 and PE9 (PUPDR = 01 for pull-up)
GPIOE->PUPDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
GPIOE->PUPDR |=  ((1U << (8 * 2)) | (1U << (9 * 2)));

// 4. Set High Speed for responsive pulse capture (OSPEEDR = 11)
GPIOE->OSPEEDR |= ((3U << (8 * 2)) | (3U << (9 * 2)));

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[2] &= ~((0xF << 0) | (0xF << 4));
    SYSCFG->EXTICR[2] |=  ((4 << 0)  | (4 << 4));  // Port E (4) for EXTI8 and EXTI9

    //EXTI->IMR  |= (1 << 8) | (1 << 9);  // Unmask lines 8 and 9
   // EXTI->RTSR |= (1 << 8) | (1 << 9);  // Set rising edge trigger
     EXTI->IMR  |= (1 << 8) | (1 << 9);  // Unmask lines 8 and 9
     EXTI->RTSR |= (1 << 8);             // Rising edge for HSYNC (PE8)
     EXTI->RTSR &= ~(1 << 9);            // Disable rising edge for VSYNC (PE9)
    EXTI->FTSR |=  (1 << 9);            // Enable falling edge for VSYNC (PE9)
    NVIC_SetPriority(EXTI9_5_IRQn, 0);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    // 4. Initialize TIM1 in External Clock Mode 1 with Capture Channel Enabled
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->CR1 = 0;
    TIM1->PSC = 0;
    TIM1->ARR = 0xFFFF;
    TIM1->CCMR1 = (1 << TIM_CCMR1_CC1S_Pos); // CC1 as TI1 input
    TIM1->CCER  |= TIM_CCER_CC1E;            // ENABLE CAPTURE CHANNEL 1
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
    if (current_line_ < TOTAL_SCAN_LINES) {
        // 1. Guard against infinite blocking if previous transfer stalled
        uint32_t timeout = 5000;
        while ((DMA2_Stream1->CR & DMA_SxCR_EN) && --timeout);
        
        if (timeout == 0) {
            // Force disable stream on timeout to recover gracefully
            DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        }

        // 2. Clear interrupt flags for Stream 1
        DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | 
                      DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

        // 3. Re-arm DMA2 Stream 1 for the current scanline
        DMA2_Stream1->PAR  = (uint32_t)&(GPIOD->IDR);
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

/*
void STNSniffer::handleVsync() {
    // YD Interrupt (PE9): Reset scanline index and swap ping-pong buffers
    current_line_ = 0;

    volatile uint8_t* temp = front_buffer_;
    front_buffer_ = back_buffer_;
    back_buffer_  = temp;
}
*/

void STNSniffer::handleVsync() {
    // Ignore stray pulses occurring before 210 lines in a 240-line frame
    if (current_line_ < 210) {
      // Ignore early VSYNC pulses
        return; 
    }

    current_line_ = 0;

    volatile uint8_t* temp = front_buffer_;
    front_buffer_ = back_buffer_;
    back_buffer_  = temp;
}