/*
 * LoRaLink.h — SX1262 link layer for the sensor node (spec §3, §4.6).
 * TX encrypted frames on f_in; optional closed-loop ACK RX window.
 */
#pragma once
#include <RadioLib.h>
#include "sps_proto.h"
#include "config.h"

enum class TxOutcome { ACKED, SENT_NO_ACK, RADIO_ERROR };

class LoRaLink {
public:
    /* Initialize the SX1262 with cfg RF params. Sync word derived from NET_ID. */
    bool begin(const NodeConfig &cfg);

    /*
     * Send an already-sealed frame with the spec §4.6 reliability model:
     *   - transmit up to retx_count times with randomized jitter
     *   - if ack_enable: open ack_window_ms RX window on f_in after each TX;
     *     a valid, authenticated ACK addressed to us matching `seq` stops
     *     retransmission early.
     * The node-side replay filter on ACKs prevents replayed-ACK suppression.
     */
    TxOutcome sendReliable(const uint8_t *frame, size_t len, uint32_t seq);

    /* Single-shot transmit: no retransmit, no ACK window (heartbeat path). */
    bool sendOnce(const uint8_t *frame, size_t len);

    int16_t lastRssi() const { return _lastRssi; }
    float   lastSnr() const { return _lastSnr; }

private:
    bool waitForAck(uint32_t seq, uint16_t window_ms);

    SX1262 *_radio = nullptr;
    const NodeConfig *_cfg = nullptr;
    sps_replay_t _ackReplay;
    int16_t _lastRssi = 0;
    float _lastSnr = 0;
};
