#include <Arduino.h>
#include "stn_sniffer.hpp"
#include "stdio.h"

UART_HandleTypeDef huart2;

static void MX_USART2_UART_Init(void);

  int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  if (len <= 0)
  {
    return 0;
  }

  if (HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY) != HAL_OK)
  {
    return -1;
  }

  return len;
}

// Lookup table to extract 4 individual bit pixels from a 4-bit UD0-UD3 nibble[cite: 10]
static void dump_frame_ascii() {
    const uint8_t* frame = STNSniffer::instance().getFrameBuffer();
    if (!frame) return;

    Serial.println("\n=================== START FRAME DUMP ===================");

    for (uint16_t y = 0; y < TOTAL_SCAN_LINES; y += 2) { // Skip every other line for proper ASCII aspect ratio
        uint16_t line_offset = y * CLOCKS_PER_LINE;       // 80 bytes/clocks per line[cite: 11]

        for (uint16_t x_clock = 0; x_clock < CLOCKS_PER_LINE; ++x_clock) {
            uint8_t raw_byte = frame[line_offset + x_clock];
            uint8_t nibble   = raw_byte & 0x0F; // UD0-UD3 mapped to PE0-PE3[cite: 10, 13]

            // Print '█' for set bits (white pixels) and '.' for clear bits
            Serial.print((nibble & 0x01) ? '#' : ' ');
            Serial.print((nibble & 0x02) ? '#' : ' ');
            Serial.print((nibble & 0x04) ? '#' : ' ');
            Serial.print((nibble & 0x08) ? '#' : ' ');
        }
        Serial.println(); // Next scanline
    }

    Serial.println("==================== END FRAME DUMP ====================\n");
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // Wait for USB Serial Monitor

    Serial.println("Initializing STN Sniffer...");

    // 1. Start STN Sniffer DMA Capture
    STNSniffer::instance().init();
    STNSniffer::instance().start();

    Serial.println("Capture running. Printing ASCII frame dumps...");
}

void loop() {
    // Dump frame buffer every 2 seconds
    dump_frame_ascii();
    delay(2000);
}