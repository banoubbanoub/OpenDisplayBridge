/*
#include <Arduino.h>
#include "stn_sniffer.hpp"
#include "VGA.h"
#include "Engine3D.h"

// Lookup table to expand a 4-bit nibble (UD0-UD3) into an 8-bit double-pixel mask
static const uint8_t NIBBLE_TO_VGA_EXPAND[16] = {
    0x00, // 0000 -> 00000000
    0x03, // 0001 -> 00000011
    0x0C, // 0010 -> 00001100
    0x0F, // 0011 -> 00001111
    0x30, // 0100 -> 00110000
    0x33, // 0101 -> 00110011
    0x3C, // 0110 -> 00111100
    0x3F, // 0111 -> 00111111
    0xC0, // 1000 -> 11000000
    0xC3, // 1001 -> 11000011
    0xCC, // 1010 -> 11001100
    0xCF, // 1011 -> 11001111
    0xF0, // 1100 -> 11110000
    0xF3, // 1101 -> 11110011
    0xFC, // 1110 -> 11111100
    0xFF  // 1111 -> 11111111
};

// Instantiate object without parentheses (Fixes Most Vexing Parse)
Engine3D engine3D;

void render_stn_to_vga(Engine3D& engine) {
    const uint8_t* stn_buffer = STNSniffer::instance().getFrameBuffer();
    uint8_t* vga_back = engine.getBackBuffer();

    if (!stn_buffer || !vga_back) return;

    for (uint16_t stn_y = 0; stn_y < TOTAL_SCAN_LINES; ++stn_y) { // 0 to 239
        uint16_t stn_line_offset = stn_y * CLOCKS_PER_LINE;       // 60 bytes per line
        
        uint16_t vga_y1 = stn_y * 2;
        uint16_t vga_y2 = vga_y1 + 1;
        
        uint32_t vga_offset_line1 = vga_y1 * VGA_BYTES_PER_LINE;  
        uint32_t vga_offset_line2 = vga_y2 * VGA_BYTES_PER_LINE;

        for (uint16_t x_clock = 0; x_clock < CLOCKS_PER_LINE; ++x_clock) { // 0 to 79
            uint8_t raw_byte = stn_buffer[stn_line_offset + x_clock];
            uint8_t nibble   = raw_byte & 0x0F;

            uint8_t vga_expanded_pixels = NIBBLE_TO_VGA_EXPAND[nibble];

            vga_back[vga_offset_line1 + x_clock] = vga_expanded_pixels;
            vga_back[vga_offset_line2 + x_clock] = vga_expanded_pixels;
        }
    }
    engine.display();
}


void render_stn_to_vga_optimized(Engine3D& engine) {
    const uint8_t* stn_buffer = STNSniffer::instance().getFrameBuffer();
    uint8_t* vga_back = engine.getBackBuffer();

    if (!stn_buffer || !vga_back) return;

    // No offsets needed since 320x240 scaled 2x equals 640x480 VGA resolution
    const uint16_t HORIZ_BYTE_OFFSET = 0; 
    const uint16_t VERT_ROW_OFFSET = 0;    

    for (uint16_t stn_y = 0; stn_y < TOTAL_SCAN_LINES; ++stn_y) {
        const uint32_t* stn_line_src = reinterpret_cast<const uint32_t*>(stn_buffer + (stn_y * CLOCKS_PER_LINE));
        
        uint16_t vga_y1 = VERT_ROW_OFFSET + (stn_y * 2);
        uint16_t vga_y2 = vga_y1 + 1;
        
        uint8_t* vga_line1 = vga_back + (vga_y1 * VGA_BYTES_PER_LINE) + HORIZ_BYTE_OFFSET;
        uint8_t* vga_line2 = vga_back + (vga_y2 * VGA_BYTES_PER_LINE) + HORIZ_BYTE_OFFSET;

        // Process 80 bytes per line in chunks of 4 bytes (20 iterations total)
        for (uint16_t i = 0; i < (CLOCKS_PER_LINE / 4); ++i) {
            uint32_t raw_chunk = stn_line_src[i];

            uint32_t expanded_chunk = 
                (static_cast<uint32_t>(NIBBLE_TO_VGA_EXPAND[(raw_chunk >> 0)  & 0x0F]) << 0)  |
                (static_cast<uint32_t>(NIBBLE_TO_VGA_EXPAND[(raw_chunk >> 8)  & 0x0F]) << 8)  |
                (static_cast<uint32_t>(NIBBLE_TO_VGA_EXPAND[(raw_chunk >> 16) & 0x0F]) << 16) |
                (static_cast<uint32_t>(NIBBLE_TO_VGA_EXPAND[(raw_chunk >> 24) & 0x0F]) << 24);

            reinterpret_cast<uint32_t*>(vga_line1)[i] = expanded_chunk;
            reinterpret_cast<uint32_t*>(vga_line2)[i] = expanded_chunk;
        }
    }

    engine.display();
}


void initCycleCounter() {
    // Enable TRC (Trace) in CoreDebug
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Reset and enable Cycle Counter
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void measureRenderTime(Engine3D& engine) {
    uint32_t start_cycles = DWT->CYCCNT;
    
    // Execute the optimized render pipeline
    render_stn_to_vga_optimized(engine);
    
    uint32_t end_cycles = DWT->CYCCNT;
    uint32_t elapsed_cycles = end_cycles - start_cycles;
    
    // Convert to microseconds (STM32F407 running at 168 MHz -> 168 cycles per microsecond)
    float elapsed_us = static_cast<float>(elapsed_cycles) / 168.0f;
    
    Serial.print("Render Time: ");
    Serial.print(elapsed_us);
    Serial.print(" us (Cycles: ");
    Serial.print(elapsed_cycles);
    Serial.println(")");
}

void setup() {
    Serial.begin(115200);

    // 1. Initialize STN Parallel Sniffer
    STNSniffer::instance().init();
    STNSniffer::instance().start();

    // 2. Initialize 3D Engine & VGA Hardware
    engine3D.begin();
    engine3D.setDoubleBuffering(true);

    //initCycleCounter();
}

void loop() {
    // Wait for vertical blanking sync, then render the captured frame
    VGA.waitVBlank();
    //render_stn_to_vga_optimized(engine3D);
    engine3D.clear();
    render_stn_to_vga(engine3D);

   // render_stn_to_vga_optimized(engine3D);
    //measureRenderTime(engine3D);
}
*/






#include <Arduino.h>
#include "stn_sniffer.hpp"
#include "VGA.h"
#include "Engine3D.h"

// Lookup table to expand each 4-bit nibble (UD0-UD3) into an 8-bit double-pixel mask.
// Each bit is duplicated (e.g., bit 0 becomes bits 0 & 1) to perform a 2x horizontal 
// stretch from 320 display pixels to 640 VGA columns.
static const uint8_t NIBBLE_TO_VGA_EXPAND[16] = {
    0x00, // 0000 -> 00000000 (4 black pixels -> 8 black VGA pixels)
    0x03, // 0001 -> 00000011
    0x0C, // 0010 -> 00001100
    0x0F, // 0011 -> 00001111
    0x30, // 0100 -> 00110000
    0x33, // 0101 -> 00110011
    0x3C, // 0110 -> 00111100
    0x3F, // 0111 -> 00111111
    0xC0, // 1000 -> 11000000
    0xC3, // 1001 -> 11000011
    0xCC, // 1010 -> 11001100
    0xCF, // 1011 -> 11001111
    0xF0, // 1100 -> 11110000
    0xF3, // 1101 -> 11110011
    0xFC, // 1110 -> 11111100
    0xFF  // 1111 -> 11111111 (4 white pixels -> 8 white VGA pixels)
};

Engine3D engine3D;

void render_stn_to_vga_optimized(Engine3D& engine) {
    // 1. Fetch pointers to the active STN captured frame buffer and VGA back buffer
    const uint8_t* stn_buffer = STNSniffer::instance().getFrameBuffer();
    uint8_t* vga_back = engine.getBackBuffer();

    if (!stn_buffer || !vga_back) return;

    // 2. Loop through each scanline of the 320x240 display (240 total lines)
    for (uint16_t stn_y = 0; stn_y < TOTAL_SCAN_LINES; ++stn_y) {
        // Point to the beginning of the current 80-byte scanline in the STN buffer,
        // reinterpreting as 32-bit chunks (4 bytes = 16 pixels per chunk)
        const uint32_t* stn_line_src = reinterpret_cast<const uint32_t*>(stn_buffer + (stn_y * CLOCKS_PER_LINE));
        
        // Calculate vertical 2x scaling coordinates for 480-line VGA resolution
        uint16_t vga_y1 = stn_y * 2;
        uint16_t vga_y2 = vga_y1 + 1;
        
        // Get row destination pointers in the VGA back buffer
        uint8_t* vga_line1 = vga_back + (vga_y1 * VGA_BYTES_PER_LINE);
        uint8_t* vga_line2 = vga_back + (vga_y2 * VGA_BYTES_PER_LINE);

        // 3. Process 80 bytes per line in chunks of 4 bytes (20 iterations total)
        for (uint16_t i = 0; i < (CLOCKS_PER_LINE / 4); ++i) {
            // Load 32 bits (4 raw bytes) from the source line
            uint32_t raw_chunk = stn_line_src[i];

            // Apply bitwise NOT (~raw_chunk) to invert video polarity so highlights 
            // (dark selection boxes with light text) render correctly on the monitor
            raw_chunk = ~raw_chunk;

            // Extract each byte, isolate its 4-bit nibble, and expand it using the lookup table.
            // Shift each expanded result into its proper 8-bit position within the 32-bit word.
            uint32_t expanded_chunk = 
                (static_cast<uint32_t>(NIBBLE_TO_VGA_EXPAND[(raw_chunk >> 0)  & 0x0F]) << 0)  |
                (static_cast<uint32_t>(NIBBLE_TO_VGA_EXPAND[(raw_chunk >> 8)  & 0x0F]) << 8)  |
                (static_cast<uint32_t>(NIBBLE_TO_VGA_EXPAND[(raw_chunk >> 16) & 0x0F]) << 16) |
                (static_cast<uint32_t>(NIBBLE_TO_VGA_EXPAND[(raw_chunk >> 24) & 0x0F]) << 24);

            // Write the 32-bit expanded chunk (representing 16 stretched pixels) to 
            // both vertical scanline rows concurrently for 2x vertical scaling
            reinterpret_cast<uint32_t*>(vga_line1)[i] = expanded_chunk;
            reinterpret_cast<uint32_t*>(vga_line2)[i] = expanded_chunk;
        }
    }

    // Push the updated back buffer to the VGA display engine
    engine.display();
}

void setup() {
    Serial.begin(115200);

    // Initialize and start the parallel STN data sniffer (DMA & Timers)
    STNSniffer::instance().init();
    STNSniffer::instance().start();

    // Initialize 3D/VGA engine subsystems with double buffering enabled
    engine3D.begin();
    engine3D.setDoubleBuffering(true);
}

void loop() {
    // Wait for VGA vertical blanking interval to avoid screen tearing, then render
    VGA.waitVBlank();
    render_stn_to_vga_optimized(engine3D);
}