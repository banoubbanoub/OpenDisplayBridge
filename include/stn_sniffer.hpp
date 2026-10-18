#ifndef STN_SNIFFER_HPP
#define STN_SNIFFER_HPP

#include <Arduino.h>

// Display resolution and buffer sizing constants
constexpr uint16_t DISPLAY_WIDTH = 320;
constexpr uint16_t DISPLAY_HEIGHT = 240;
constexpr uint16_t CLOCKS_PER_LINE = DISPLAY_WIDTH / 4; // 80 bytes per scanline (4 pixels per byte)
// Samples kept per line: the 80 data clocks plus a margin so edge pixels are never cut
constexpr uint16_t STORE_CLOCKS = CLOCKS_PER_LINE + 4;
constexpr uint16_t TOTAL_SCAN_LINES = DISPLAY_HEIGHT;

class STNSniffer {
public:
    static STNSniffer& instance() {
        static STNSniffer inst;
        return inst;
    }

    void init();
    void start();
    void stop();

    void handleHsync();
    void handleVsync();

    const uint8_t* getFrameBuffer() {
        reading_ = true;
        return const_cast<const uint8_t*>(const_cast<volatile uint8_t*>(front_buffer_));
    }
    void releaseFrameBuffer() { reading_ = false; }

    // Lines whose XSCL burst was exactly CLOCKS_PER_LINE clocks vs. lines dropped for a wrong count
    uint32_t goodLines() const { return good_lines_; }
    uint32_t badLines() const { return bad_lines_; }
    uint32_t clockHist(uint8_t i) const { return hist_[i]; }
    // LP pulses and accepted lines seen between the last two YD pulses
    uint32_t frameLp() const { return frame_lp_; }
    uint32_t frameGood() const { return frame_good_; }

    // Select which XSCL edge triggers the data read and how long the input filter delays it (0-3)
    void setSamplePhase(bool falling, uint8_t filter);

    // Changed lines between consecutive frames plus dropped lines; ~0 on a static screen when sampling is clean
    uint32_t measureScore(uint8_t frames);

private:
    STNSniffer() = default;
    ~STNSniffer() = default;
    STNSniffer(const STNSniffer&) = delete;
    STNSniffer& operator=(const STNSniffer&) = delete;

    uint8_t buffer_a_[STORE_CLOCKS * TOTAL_SCAN_LINES];
    uint8_t buffer_b_[STORE_CLOCKS * TOTAL_SCAN_LINES];

    // Free-running DMA target: every XSCL clock appends one sample
    static constexpr uint16_t RING_SIZE = 1024;
    volatile uint8_t ring_[RING_SIZE];

    volatile uint8_t* front_buffer_ = buffer_a_;
    volatile uint8_t* back_buffer_ = buffer_b_;
    volatile uint16_t current_line_ = 0;
    uint16_t prev_pos_ = 0;
    volatile uint32_t good_lines_ = 0;
    volatile uint32_t bad_lines_ = 0;
    volatile uint32_t hist_[7] = {};
    volatile uint32_t lp_count_ = 0, frame_lp_ = 0, frame_good_ = 0, good_at_vsync_ = 0;
    volatile uint32_t frame_seq_ = 0;
    uint32_t line_hash_[TOTAL_SCAN_LINES];
    volatile bool reading_ = false;
    volatile bool capturing_ = true;
    uint8_t frame_count_ = 0;
};

#endif // STN_SNIFFER_HPP