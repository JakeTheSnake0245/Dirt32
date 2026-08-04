#include "LoRaLink.h"
#include <esp_random.h>

/* RX-done flag set by the SX1262 DIO1 interrupt. Polling getPacketLength()
 * is NOT a valid new-packet test: GET_RX_BUFFER_STATUS on SX126x is never
 * cleared by readData, so after the first packet it stays nonzero forever
 * and the ACK window re-reads the stale first ACK (replay-rejected) instead
 * of ever seeing a new one — "ACK works exactly once". Same bug was already
 * fixed in the gateway-bridge; this is the node-side fix. */
static volatile bool s_rxDone = false;
static void IRAM_ATTR onAckRxDone() { s_rxDone = true; }

bool LoRaLink::begin(const NodeConfig &cfg) {
    _cfg = &cfg;
    sps_replay_init(&_ackReplay, 8);

    static Module mod(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
    static SX1262 radio(&mod);
    _radio = &radio;

    /* Sync word seeded from NET_ID (spec §3). Avoid 0x34 (public LoRaWAN). */
    uint8_t sync = (uint8_t)(0x12 ^ cfg.net_id);
    if (sync == 0x34) sync ^= 0x01;

    int state = _radio->begin(cfg.f_in_mhz, cfg.bw_khz, cfg.sf, cfg.cr,
                              sync, cfg.tx_power_dbm, 8 /* preamble */);
    if (state != RADIOLIB_ERR_NONE) return false;
    _radio->setCRC(true);                 /* PHY CRC on (spec §3) */
    _radio->explicitHeader();
    return true;
}

bool LoRaLink::waitForAck(uint32_t seq, uint16_t window_ms) {
    uint8_t buf[SPS_MAX_FRAME];
    uint32_t deadline = millis() + window_ms;

    s_rxDone = false;
    _radio->setDio1Action(onAckRxDone);
    _radio->startReceive();
    while ((int32_t)(deadline - millis()) > 0) {
        if (!s_rxDone) { delay(5); continue; }
        s_rxDone = false;
        int len = _radio->getPacketLength();  /* fresh read right after IRQ */
        if (len <= 0 || (size_t)len > sizeof(buf)) { _radio->startReceive(); continue; }
        int state = _radio->readData(buf, len);
        if (state != RADIOLIB_ERR_NONE) { _radio->startReceive(); continue; }

        _lastRssi = (int16_t)_radio->getRSSI();
        _lastSnr = _radio->getSNR();

        sps_header_t h;
        uint8_t pl[SPS_MAX_PLEN];
        size_t plen;
        /* Only accept: our net, ACK type, addressed to us, authenticated
           under OUR key, not a replay, and acknowledging THIS seq. */
        if (sps_frame_open(_cfg->psk, buf, (size_t)len, &h, pl, &plen) != SPS_OK) {
            _radio->startReceive(); continue;
        }
        if (h.net_id != _cfg->net_id || h.msg_type != SPS_MSG_ACK ||
            h.node_id != _cfg->node_id) {
            _radio->startReceive(); continue;
        }
        if (sps_replay_check(&_ackReplay, h.seq) != SPS_OK) {
            _radio->startReceive(); continue;
        }
        sps_ack_t ack;
        sps_ack_read(pl, &ack);
        if (ack.ack_seq == (seq & 0xFFFFFFu)) {
            _radio->clearDio1Action();
            _radio->standby();
            return true;
        }
        _radio->startReceive();
    }
    _radio->clearDio1Action();
    _radio->standby();
    return false;
}

bool LoRaLink::sendOnce(const uint8_t *frame, size_t len) {
    return _radio->transmit(const_cast<uint8_t *>(frame), len) == RADIOLIB_ERR_NONE;
}

TxOutcome LoRaLink::sendReliable(const uint8_t *frame, size_t len, uint32_t seq) {
    for (uint8_t attempt = 0; attempt < _cfg->retx_count; attempt++) {
        if (attempt > 0) {
            uint32_t span = _cfg->retx_jitter_max_ms - _cfg->retx_jitter_min_ms;
            uint32_t jitter = _cfg->retx_jitter_min_ms +
                              (span ? (esp_random() % span) : 0);
            delay(jitter);
        }
        int state = _radio->transmit(const_cast<uint8_t *>(frame), len);
        if (state != RADIOLIB_ERR_NONE) {
            if (attempt + 1 == _cfg->retx_count) return TxOutcome::RADIO_ERROR;
            continue;
        }
        if (_cfg->ack_enable && waitForAck(seq, _cfg->ack_window_ms))
            return TxOutcome::ACKED;
    }
    return _cfg->ack_enable ? TxOutcome::SENT_NO_ACK : TxOutcome::ACKED;
}
