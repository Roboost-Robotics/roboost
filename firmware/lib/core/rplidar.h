// Minimal RPLidar A1 stream parser for the legacy SCAN command.
// Feed it raw UART bytes; it bins every revolution into BEAMS fixed 1-degree beams
// (constant beam count is what slam_toolbox / costmaps expect).
#pragma once

#include <stdint.h>
#include <limits>

class RPLidar {
public:
    static constexpr int BEAMS = 360;
    static constexpr uint8_t CMD_STOP[2] = {0xA5, 0x25};
    static constexpr uint8_t CMD_SCAN[2] = {0xA5, 0x20};

    RPLidar() {
        clear(finished_);
        reset();
    }

    // Call right after sending CMD_SCAN: waits for the 7 byte response descriptor.
    void reset() {
        state_ = State::Header;
        header_pos_ = 0;
        skipped_ = 0;
        node_pos_ = 0;
        clear(current_);
        current_points_ = 0;
    }

    // Returns true when a new revolution starts; scan()/scan_points() then hold the finished one.
    bool feed(uint8_t b) {
        if (state_ == State::Header) {
            feed_header(b);
            return false;
        }
        if (node_pos_ == 0 && !((b ^ (b >> 1)) & 0x01)) {
            return false;  // start flag and inverted start flag must differ
        }
        if (node_pos_ == 1 && !(b & 0x01)) {
            node_pos_ = 0;  // check bit missing -> resync
            return false;
        }
        node_[node_pos_++] = b;
        if (node_pos_ < 5) {
            return false;
        }
        node_pos_ = 0;
        return add_node();
    }

    // Ranges in metres, +inf where no echo was received.
    const float* scan() const { return finished_; }
    // Raw measurements in the finished revolution (use to drop partial revolutions).
    int scan_points() const { return finished_points_; }

private:
    enum class State { Header, Nodes };

    void feed_header(uint8_t b) {
        // Descriptor: A5 5A 05 00 00 40 81. Fall back to node parsing if it never shows up.
        if (header_pos_ == 0) {
            if (b == 0xA5) {
                header_pos_ = 1;
            } else if (++skipped_ > 64) {
                state_ = State::Nodes;
            }
        } else if (header_pos_ == 1) {
            header_pos_ = (b == 0x5A) ? 2 : (b == 0xA5 ? 1 : 0);
        } else if (++header_pos_ == 7) {
            state_ = State::Nodes;
        }
    }

    bool add_node() {
        const bool start = node_[0] & 0x01;
        const uint8_t quality = node_[0] >> 2;
        const uint16_t angle_q6 = (node_[1] | (node_[2] << 8)) >> 1;
        const uint16_t distance_q2 = node_[3] | (node_[4] << 8);

        if (start) {
            for (int i = 0; i < BEAMS; i++) {
                finished_[i] = current_[i];
            }
            finished_points_ = current_points_;
            clear(current_);
            current_points_ = 0;
        }
        current_points_++;
        if (quality > 0 && distance_q2 > 0) {
            const int beam = ((angle_q6 + 32) / 64) % BEAMS;  // nearest degree
            const float range = distance_q2 / 4000.0f;
            if (range < current_[beam]) {
                current_[beam] = range;
            }
        }
        return start;
    }

    static void clear(float* ranges) {
        for (int i = 0; i < BEAMS; i++) {
            ranges[i] = std::numeric_limits<float>::infinity();
        }
    }

    State state_;
    int header_pos_;
    int skipped_;
    uint8_t node_[5];
    int node_pos_;
    float current_[BEAMS];
    float finished_[BEAMS];
    int current_points_;
    int finished_points_ = 0;
};
