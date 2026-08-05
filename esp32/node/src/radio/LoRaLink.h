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

    /*
     * Downlink command RX (awake nodes only — bench/CSI/geophone modes).
     * startListen() puts the radio in continuous receive; receiveCommand()
     * is polled from loop() and returns true when an authenticated,
     * non-replayed SPS_MSG_CMD addressed to this node arrived. TX methods
     * automatically resume the listen state after transmitting.
     */
    bool startListen();
    void stopListen();
    bool receiveCommand(sps_cmd_t *out, uint32_t *seq_out);
    bool listening() const { return _listening; }

    /* Downlink anti-replay: commands are accepted only with SEQ strictly
     * above this floor (gateway SEQ is monotonic and persisted, so no
     * windowing needed). Seed from NVS at boot so a captured command can't
     * be replayed after a node reboot. */
    void setCmdSeqFloor(uint32_t seq) { if (seq > _cmdLastSeq) _cmdLastSeq = seq; }
    uint32_t cmdSeqFloor() const { return _cmdLastSeq; }

    int16_t lastRssi() const { return _lastRssi; }
    float   lastSnr() const { return _lastSnr; }

    /* TX guard: called with true right before each LoRa transmit and false
     * right after it returns. Used to hold ESP-NOW pings ONLY for the
     * transmit burst (peak-current precaution) — NOT for the ACK-wait RX
     * window, which is receive-only and costs nothing extra. Pausing for
     * the whole sendReliable call starved the WiFi radar's ping stream to
     * <1/s whenever alerts were frequent (bench-verified). */
    void setTxGuard(void (*guard)(bool active)) { _txGuard = guard; }

private:
    bool waitForAck(uint32_t seq, uint16_t window_ms);

    SX1262 *_radio = nullptr;
    const NodeConfig *_cfg = nullptr;
    bool _listening = false;
    sps_replay_t _ackReplay;
    uint32_t _cmdLastSeq = 0;  /* downlink strict-monotonic floor */
    void (*_txGuard)(bool) = nullptr;
    int guardedTransmit(const uint8_t *frame, size_t len);
    int16_t _lastRssi = 0;
    float _lastSnr = 0;
};
