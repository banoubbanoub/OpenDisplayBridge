#pragma once
#include "stm32f4xx_hal.h"
#include <cstdint>

// Resolution parameters for Samsung SPS-1000 320x240 screen
constexpr uint16_t DISPLAY_WIDTH        = 320;
constexpr uint16_t DISPLAY_HEIGHT       = 240;

// Single-scan outputs 4 bits per XSCL pulse (80 pulses per line)
constexpr uint16_t CLOCKS_PER_LINE      = DISPLAY_WIDTH / 4;  // 80 bytes per line
constexpr uint16_t TOTAL_SCAN_LINES     = DISPLAY_HEIGHT;     // 240 lines

class STNSniffer {
public:
    static STNSniffer& instance() {
        static STNSniffer _instance;
        return _instance;
    }

    void init();
    void start();
    void stop();

    // Interrupt service routines (LP on PE8, YD on PE9)
    void handleHsync(); 
    void handleVsync(); 

    // Returns pointer to the stable front buffer for the VGA renderer
    const uint8_t* getFrameBuffer() const { 
        return const_cast<const uint8_t*>(front_buffer_); 
    }

private:
    STNSniffer() = default;
    ~STNSniffer() = default;
    STNSniffer(const STNSniffer&) = delete;
    STNSniffer& operator=(const STNSniffer&) = delete;

    // Ping-pong frame buffers
    uint8_t buffer_a_[CLOCKS_PER_LINE * TOTAL_SCAN_LINES];
    uint8_t buffer_b_[CLOCKS_PER_LINE * TOTAL_SCAN_LINES];

    volatile uint8_t* back_buffer_ = buffer_a_;  // Written by DMA2
    volatile uint8_t* front_buffer_ = buffer_b_; // Read by VGA display engine

    volatile uint16_t current_line_ = 0;
};