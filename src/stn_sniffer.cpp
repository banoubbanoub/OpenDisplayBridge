#include "stn_sniffer.hpp"

// Sniff only 1 of every N LCD frames (1 = every frame, ~152 Hz).
#ifndef STN_FRAME_DIV
#define STN_FRAME_DIV 1
#endif

static STNSniffer* g_sniffer = nullptr;

// Clocks at the end of each burst that are not line data; raise it to bring in more pixels on the left.
#ifndef STN_LINE_END_SKIP
#define STN_LINE_END_SKIP 0
#endif

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

    // 1. Configure Data Pins (PD0 - PD3) as Inputs
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
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    GPIOE->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOE->PUPDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOE->PUPDR |=  ((1U << (8 * 2)) | (1U << (9 * 2)));
    GPIOE->OSPEEDR |= ((3U << (8 * 2)) | (3U << (9 * 2)));

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[2] &= ~((0xF << 0) | (0xF << 4));
    SYSCFG->EXTICR[2] |=  ((4 << 0)  | (4 << 4));  // Port E (4) for EXTI8 and EXTI9

    EXTI->IMR  |= (1 << 8) | (1 << 9);  // Unmask lines 8 and 9
    EXTI->RTSR |= (1 << 8);             // Rising edge for HSYNC (PE8)
    EXTI->RTSR &= ~(1 << 9);            // Disable rising edge for VSYNC (PE9)
    EXTI->FTSR |=  (1 << 9);            // Enable falling edge for VSYNC (PE9)
    
    NVIC_SetPriority(EXTI9_5_IRQn, 2); // below VGA TIM4 (0) and DMA1 (1) so sniffing never delays VGA lines
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    // 4. Initialize TIM1 in External Clock Mode 1 with Falling Edge Capture
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->CR1 = 0;
    TIM1->PSC = 0;
    TIM1->ARR = 0xFFFF;
    // IC1F=2: input filter (4 timer clocks, ~24 ns) rejects XSCL ringing. XSCL pulses are only ~60 ns
    // wide, so a longer filter (3 = ~48 ns) can swallow real clocks.
    TIM1->CCMR1 = (1 << TIM_CCMR1_CC1S_Pos) | (2 << TIM_CCMR1_IC1F_Pos);
    
    TIM1->CCER &= ~TIM_CCER_CC1P;            // Rising edge; the filter delay puts the read near the falling edge
    TIM1->CCER |=  TIM_CCER_CC1E;            // Enable capture channel 1

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

    // Circular DMA is started once and never re-armed, so there is no per-line setup to race
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream1->CR & DMA_SxCR_EN);
    DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 |
                  DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

    DMA2_Stream1->PAR  = (uint32_t)&(GPIOD->IDR);
    DMA2_Stream1->M0AR = (uint32_t)ring_;
    DMA2_Stream1->NDTR = RING_SIZE;
    DMA2_Stream1->CR   = (6 << DMA_SxCR_CHSEL_Pos) | // Channel 6 (TIM1_CH1)
                         DMA_SxCR_PL               | // Very high priority
                         DMA_SxCR_CIRC             | // Circular mode
                         DMA_SxCR_MINC             | // Memory increment mode
                         (0 << DMA_SxCR_MSIZE_Pos) | // 8-bit memory
                         (0 << DMA_SxCR_PSIZE_Pos) | // 8-bit peripheral
                         (0 << DMA_SxCR_DIR_Pos)   | // Peripheral-to-memory
                         DMA_SxCR_EN;

    TIM1->CR1 |= TIM_CR1_CEN;
}

void STNSniffer::stop() {
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    TIM1->CR1 &= ~TIM_CR1_CEN;
}

// LP rises after the line's XSCL burst has ended and the next burst starts ~17 us later,
// so the last CLOCKS_PER_LINE samples in the ring are exactly the line just received.
void STNSniffer::handleHsync() {
    const uint16_t mask   = RING_SIZE - 1;
    // LP can rise before the last clocks of the burst, so wait until XSCL is idle to include them
    uint16_t ndtr = DMA2_Stream1->NDTR;
    for (uint8_t n = 0; n < 12; ++n) {
        for (volatile uint8_t d = 0; d < 20; ++d) {}
        const uint16_t now = DMA2_Stream1->NDTR;
        if (now == ndtr) break;
        ndtr = now;
    }
    const uint16_t pos    = (RING_SIZE - ndtr) & mask;
    const uint16_t clocks = (pos - prev_pos_) & mask; // XSCL clocks since the previous LP
    prev_pos_ = pos;
    lp_count_++;

    // Histogram of clocks per line: [0] <80, [1..5] = 80..84, [6] >84
    hist_[clocks < CLOCKS_PER_LINE ? 0 : (clocks <= STORE_CLOCKS ? clocks - CLOCKS_PER_LINE + 1 : 6)]++;

    if (current_line_ >= TOTAL_SCAN_LINES) return;

    if (capturing_) {
        // Bursts shorter or much longer than a line are dropped; a few extra clocks are kept so edge pixels are not lost.
        if (clocks >= CLOCKS_PER_LINE && clocks <= STORE_CLOCKS) {
            const uint16_t src = (pos - STORE_CLOCKS) & mask;
            uint8_t* dst = const_cast<uint8_t*>(const_cast<volatile uint8_t*>(back_buffer_)) + (current_line_ * STORE_CLOCKS);
            const uint8_t* ring = const_cast<const uint8_t*>(const_cast<volatile uint8_t*>(ring_));

            if (src + STORE_CLOCKS <= RING_SIZE) {
                // Contiguous: copy as 32-bit words (Cortex-M4 allows the unaligned loads)
                for (uint16_t i = 0; i < STORE_CLOCKS; i += 4) {
                    uint32_t w;
                    memcpy(&w, ring + src + i, 4);
                    memcpy(dst + i, &w, 4);
                }
            } else {
                for (uint16_t i = 0; i < STORE_CLOCKS; ++i) {
                    dst[i] = ring[(src + i) & mask];
                }
            }
            good_lines_++;
        } else {
            bad_lines_++;
        }
    }

    current_line_++;
}

// YD falls inside the first LP period of a frame, so the second LP of the frame is the first
// one that copies a line (LINE 1).
void STNSniffer::handleVsync() {
    const bool frame_complete = current_line_ >= TOTAL_SCAN_LINES;
    frame_lp_ = lp_count_;
    frame_good_ = good_lines_ - good_at_vsync_;
    good_at_vsync_ = good_lines_;
    lp_count_ = 0;
    current_line_ = 0;

    const bool was_capturing = capturing_;
    if (++frame_count_ >= STN_FRAME_DIV) {
        frame_count_ = 0;
        capturing_ = true;
    } else {
        capturing_ = false;
    }

    // Publish only complete captured frames, and not while the renderer is reading
    if (!frame_complete || !was_capturing || reading_) return;

    volatile uint8_t* temp = front_buffer_;
    front_buffer_ = back_buffer_;
    back_buffer_  = temp;
    frame_seq_++;
}

void STNSniffer::setSamplePhase(bool falling, uint8_t filter) {
    TIM1->CCER &= ~TIM_CCER_CC1E;
    TIM1->CCMR1 = (1 << TIM_CCMR1_CC1S_Pos) | ((filter & 0xF) << TIM_CCMR1_IC1F_Pos);
    if (falling) {
        TIM1->CCER |= TIM_CCER_CC1P;
    } else {
        TIM1->CCER &= ~TIM_CCER_CC1P;
    }
    TIM1->CCER |= TIM_CCER_CC1E;
}

uint32_t STNSniffer::measureScore(uint8_t frames) {
    uint32_t score = 0;
    const uint32_t bad0 = bad_lines_;
    uint32_t seen = frame_seq_;
    bool have_prev = false;

    for (uint8_t n = 0; n <= frames; ++n) {
        const uint32_t t0 = millis();
        while (frame_seq_ == seen) {
            if (millis() - t0 > 200) return 0xFFFFFFFFu; // no frames arriving
        }
        seen = frame_seq_;

        reading_ = true;
        const uint8_t* p = const_cast<const uint8_t*>(const_cast<volatile uint8_t*>(front_buffer_));
        for (uint16_t y = 0; y < TOTAL_SCAN_LINES; ++y) {
            uint32_t h = 2166136261u;
            for (uint16_t i = 0; i < STORE_CLOCKS; ++i) {
                h = (h ^ p[y * STORE_CLOCKS + i]) * 16777619u;
            }
            if (have_prev && h != line_hash_[y]) score++;
            line_hash_[y] = h;
        }
        reading_ = false;
        have_prev = true;
    }

    return score + (bad_lines_ - bad0);
}