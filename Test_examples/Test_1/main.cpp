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
        uint16_t stn_line_offset = stn_y * CLOCKS_PER_LINE;       // 80 bytes per STN line
        
        // Calculate destination line offsets for 2x vertical scaling
        uint16_t vga_y1 = stn_y * 2;
        uint16_t vga_y2 = vga_y1 + 1;
        
        uint32_t vga_offset_line1 = vga_y1 * VGA_BYTES_PER_LINE;  // 80 bytes/line stride
        uint32_t vga_offset_line2 = vga_y2 * VGA_BYTES_PER_LINE;

        for (uint16_t x_clock = 0; x_clock < CLOCKS_PER_LINE; ++x_clock) {
            // Extract low nibble (UD0-UD3 mapped to PE0-PE3)[cite: 10, 13]
            uint8_t raw_byte = stn_buffer[stn_line_offset + x_clock];
            uint8_t nibble   = raw_byte & 0x0F;

            // Expand 4 pixels into 8 VGA pixels
            uint8_t vga_expanded_pixels = NIBBLE_TO_VGA_EXPAND[nibble];

            // Write to both scanlines
            vga_back[vga_offset_line1 + x_clock] = vga_expanded_pixels;
            vga_back[vga_offset_line2 + x_clock] = vga_expanded_pixels;
        }
    }

    // Flush backbuffer to active VGA VRAM[cite: 16]
    engine.display();
}

void setup() {
    Serial.begin(115200);

    // 1. Initialize STN Parallel Sniffer
    STNSniffer::instance().init();
    STNSniffer::instance().start();

    // 2. Initialize 3D Engine & VGA Hardware
    engine3D.begin();
    engine3D.setDoubleBuffering(true);
}

void loop() {
    // Continuously map incoming STN frames into VGA buffer
    render_stn_to_vga(engine3D);
}