#pragma once
#include "stm32f4xx_hal.h"
#include <cstdint>

// Resolution parameters for Samsung SPS-1000 320x240 screen
constexpr uint16_t DISPLAY_WIDTH        = 320;
constexpr uint16_t DISPLAY_HEIGHT       = 240;

// Single-scan outputs 4 bits per XSCL pulse (80 pulses per line)
constexpr uint16_t CLOCKS_PER_LINE      = DISPLAY_WIDTH / 4;  // 80 clock cycles
constexpr uint16_t TOTAL_SCAN_LINES     = DISPLAY_HEIGHT;     // 240 lines sequentially

// Frame buffer sized to store 80 captured raw IDR bytes per line
constexpr uint32_t FRAME_BUFFER_SIZE_BYTES = CLOCKS_PER_LINE * TOTAL_SCAN_LINES; // 19200 bytes

class STNSniffer {
public:
    static STNSniffer& instance();

    void init();
    void start();
    void stop();

    // Interrupt Callbacks
    void onHsync();
    void onVsync();

    const uint8_t* getFrameBuffer() const { return frame_buffer_[active_buffer_]; }

private:
    STNSniffer() = default;

    alignas(4) uint8_t frame_buffer_[2][FRAME_BUFFER_SIZE_BYTES];
    volatile uint8_t active_buffer_ = 0;
    volatile uint16_t current_line_ = 0;

    TIM_HandleTypeDef htim3_;
    DMA_HandleTypeDef hdma_tim3_ch1_;
};