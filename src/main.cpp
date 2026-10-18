#include <Arduino.h>
#include "stn_sniffer.hpp"
#include "VGA.h"
#include "Engine3D.h"

Engine3D engine3D;

// Define to run the phase sweep and show the scores for 8 s at boot; otherwise the phase below is used directly.
// #define STN_CALIBRATE
#define STN_DEBUG_OVERLAY

// 0..7 forces that sampling phase (see calibrateSampling).
#define STN_FORCE_PHASE 1

#ifdef STN_CALIBRATE
static int g_picked_phase = 0;

static uint32_t median3(uint32_t a, uint32_t b, uint32_t c) {
    if (a > b) { uint32_t t = a; a = b; b = t; }
    if (b > c) { uint32_t t = b; b = c; c = t; }
    if (a > b) { uint32_t t = a; a = b; b = t; }
    return b;
}
#endif

static constexpr uint16_t OUTPUT_WIDTH = DISPLAY_WIDTH * 5 / 4;
static constexpr uint16_t OUTPUT_HEIGHT = DISPLAY_HEIGHT * 5 / 4;
static constexpr uint16_t X_BYTE_OFF = (VGA_BYTES_PER_LINE - OUTPUT_WIDTH / 8) / 2;
static constexpr uint16_t Y_ROW_OFF = (VGA_HEIGHT - OUTPUT_HEIGHT) / 2 - 8;

void render_stn_to_vga_optimized(Engine3D& engine) {
    const uint8_t* stn_buffer = STNSniffer::instance().getFrameBuffer();
    uint8_t* vga_back = engine.getBackBuffer();

    if (!stn_buffer || !vga_back) {
        STNSniffer::instance().releaseFrameBuffer();
        return;
    }

    memset(vga_back, 0, VGA_VRAM_SIZE);

    for (uint16_t output_y = 0; output_y < OUTPUT_HEIGHT; ++output_y) {
        const uint16_t source_y = output_y * DISPLAY_HEIGHT / OUTPUT_HEIGHT;
        const uint8_t* stn_line_src = stn_buffer + (source_y * STORE_CLOCKS) + (STORE_CLOCKS - CLOCKS_PER_LINE);
        uint8_t* out = vga_back + ((Y_ROW_OFF + output_y) * VGA_BYTES_PER_LINE) + X_BYTE_OFF;

        for (uint16_t output_byte = 0; output_byte < OUTPUT_WIDTH / 8; ++output_byte) {
            uint8_t packed_pixels = 0;
            for (uint8_t bit = 0; bit < 8; ++bit) {
                const uint16_t output_x = output_byte * 8 + bit;
                const uint16_t source_x = output_x * DISPLAY_WIDTH / OUTPUT_WIDTH;
                const uint8_t source_nibble = static_cast<uint8_t>(~stn_line_src[source_x / 4]) & 0x0F;
                const uint8_t source_bit = (source_nibble >> (3 - (source_x & 3))) & 1;
                packed_pixels |= source_bit << (7 - bit);
            }
            *out++ = packed_pixels;
        }
    }

    STNSniffer::instance().releaseFrameBuffer();
    engine.display();
}

#ifdef STN_CALIBRATE
// Tries each XSCL sampling phase on the (static) boot screen and keeps the middle of the cleanest run.
static void calibrateSampling() {
    // Ordered by increasing read delay after the XSCL rising edge (~0, 12, 24, 48, 62, 74, 86, 110 ns)
    static const struct { bool falling; uint8_t filter; } phases[8] = {
        {false, 0}, {false, 1}, {false, 2}, {false, 3},
        {true, 0},  {true, 1},  {true, 2},  {true, 3}
    };

    STNSniffer& s = STNSniffer::instance();
    uint32_t score[8];
    uint32_t best = 0xFFFFFFFFu;

    for (int i = 0; i < 8; ++i) {
        s.setSamplePhase(phases[i].falling, phases[i].filter);
        delay(50);

        // Measure each phase multiple times and keep the median to reject one-off glitches.
        const uint32_t m0 = s.measureScore(24);
        const uint32_t m1 = s.measureScore(24);
        const uint32_t m2 = s.measureScore(24);
        score[i] = median3(m0, m1, m2);

        if (score[i] < best) best = score[i];
        Serial.print("phase ");
        Serial.print(i);
        Serial.print(" score=");
        Serial.println(score[i]);
    }

    uint32_t tol = best / 5; // ~20% above best when scores are large
    if (tol < 8) tol = 8;
    if (tol > 200) tol = 200;
    const uint32_t limit = best + tol;

    bool good[8];
    for (int i = 0; i < 8; ++i) good[i] = score[i] <= limit;

    int bestStart = 0, bestLen = 0;
    for (int i = 0; i < 8; ++i) {
        const int prev = (i + 7) & 7;
        if (!good[i] || good[prev]) continue; // run start

        int len = 0;
        int j = i;
        while (len < 8 && good[j]) {
            ++len;
            j = (j + 1) & 7;
        }
        if (len > bestLen) {
            bestLen = len;
            bestStart = i;
        }
    }

    int pick;
    if (bestLen <= 2) {
        // If only one or two phases pass, pick the true minimum score phase.
        pick = 0;
        uint32_t minScore = score[0];
        for (int i = 1; i < 8; ++i) {
            if (score[i] < minScore) {
                minScore = score[i];
                pick = i;
            }
        }
    } else {
        pick = (bestStart + bestLen / 2) & 7;
    }
    // Scores show data is unstable ~60-90 ns after the XSCL rising edge, so force a phase near the rising edge (middle of the stable window).
#ifdef STN_FORCE_PHASE
    pick = STN_FORCE_PHASE;
#endif
    s.setSamplePhase(phases[pick].falling, phases[pick].filter);
    g_picked_phase = pick;
    Serial.print("picked phase ");
    Serial.println(pick);

#ifdef STN_DEBUG_OVERLAY
    // Show the sweep result on the VGA for 8 s (R/F = XSCL edge, Fn = input filter, < = picked)
    engine3D.clear();
    engine3D.setColor(1);
    char buf[40];
    for (int i = 0; i < 8; ++i) {
        snprintf(buf, sizeof(buf), "%d %s F%d  %lu%s", i, phases[i].falling ? "FALL" : "RISE",
                 phases[i].filter, static_cast<unsigned long>(score[i]), i == pick ? " <" : "");
        engine3D.setPen(20, 20 + i * 18);
        engine3D.drawText(buf, 2);
    }
    engine3D.display();
    delay(8000);
#endif
}
#endif  // STN_CALIBRATE

void setup() {
    Serial.begin(115200);

    STNSniffer::instance().init();
    STNSniffer::instance().start();

    engine3D.begin();
    engine3D.setDoubleBuffering(true);

#ifdef STN_CALIBRATE
    calibrateSampling();
#else
    STNSniffer::instance().setSamplePhase(false, STN_FORCE_PHASE);  // RISE edge, filter 1 (phase 1)
#endif
}

void loop() {
    // Render during active video; display() then copies in the vblank that follows
    VGA.waitActive();
    render_stn_to_vga_optimized(engine3D);

    static uint16_t n = 0;
    if (++n >= 120) {
        n = 0;
        Serial.print("good=");
        Serial.print(STNSniffer::instance().goodLines());
        Serial.print(" bad=");
        Serial.println(STNSniffer::instance().badLines());
    }
}