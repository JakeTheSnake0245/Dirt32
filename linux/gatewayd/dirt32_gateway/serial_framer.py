"""
serial_framer.py — robust COBS/CRC framing for the LoRa gateway serial link.

DROP-IN REPLACEMENT for the old text parser that read
    'RX <hex> <rssi> <snr>'
lines and failed on garbled/merged frames. This reads a self-synchronizing
binary frame instead, so a corrupt frame is detected (CRC + explicit length)
and contained to itself — it can never merge into the next one.

Byte-for-byte compatible with lora_link.h/.cpp on the ESP32.

WIRE FORMAT (before COBS), little-endian for multibyte fields:
    LEN     u8      payload length (0..255)
    TYPE    u8      message type (see TYPE_* constants below)
    PAYLOAD LEN     opaque bytes (the encrypted LoRa frame)
    RSSI    i16 LE  dBm  (0 for Pi→Bridge frames)
    SNR     i8      dB   (0 for Pi→Bridge frames)
    CRC16   u16 LE  CRC-16/CCITT-FALSE over LEN..SNR
On the wire: COBS(frame) + 0x00. A 0x00 byte means frame boundary, only.

No third-party dependencies.
"""
from __future__ import annotations

import logging
import struct
from dataclasses import dataclass
from typing import Iterator

log = logging.getLogger("serial")

# ---- Message types -------------------------------------------------------
# Bridge → Pi (received LoRa data)
TYPE_ALERT     = 0x01   # received LoRa ALERT frame
TYPE_HEARTBEAT = 0x02   # received LoRa HEARTBEAT frame
# Pi → Bridge (transmit / control)
TYPE_TX        = 0x03   # transmit this payload via radio (ACK to node)
TYPE_CFG       = 0x10   # radio config  (12-byte LLCfg payload)
TYPE_PING      = 0x11   # keepalive, no payload
# Bridge → Pi (control responses)
TYPE_RDY       = 0x20   # bridge ready / rebooted
TYPE_OK        = 0x21   # last command succeeded
TYPE_ERR       = 0x22   # error


# ---- CFG payload --------------------------------------------------------
# struct LLCfg (12 bytes, little-endian, packed — matches lora_link.h)
#   uint32_t freq_hz    e.g. 903000000
#   uint8_t  sf         7-12
#   uint32_t bw_hz      e.g. 125000
#   uint8_t  cr         5-8
#   uint8_t  net_id
#   int8_t   txpwr_dbm
_CFG_FMT = '<IBIBBb'   # 4+1+4+1+1+1 = 12 bytes, no padding with '<'

def encode_cfg(freq_mhz: float, sf: int, bw_khz: float,
               cr: int, net_id: int, txpwr_dbm: int) -> bytes:
    """Pack radio config into the 12-byte LLCfg binary payload."""
    return struct.pack(_CFG_FMT,
                       round(freq_mhz * 1e6),
                       sf,
                       round(bw_khz * 1e3),
                       cr, net_id, txpwr_dbm)


# ---- CRC-16/CCITT-FALSE -------------------------------------------------
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


# ---- COBS ---------------------------------------------------------------
def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])          # placeholder for first code byte
    code_idx = 0
    code = 1
    for b in data:
        if b:
            out.append(b)
            code += 1
            if code != 0xFF:
                continue
        out[code_idx] = code
        code = 1
        code_idx = len(out)
        out.append(0)             # placeholder for next code byte
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise ValueError("zero code byte in COBS body")
        i += 1
        end = i + code - 1
        if end > n:
            raise ValueError("COBS block overruns buffer")
        out += data[i:end]
        i = end
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


# ---- Frame --------------------------------------------------------------
@dataclass
class LoRaFrame:
    type: int
    payload: bytes      # raw encrypted LoRa bytes — feed straight to ingest
    rssi: int
    snr: int

    @property
    def hex(self) -> str:
        return self.payload.hex()


def _parse_raw(raw: bytes) -> LoRaFrame:
    if len(raw) < 7:
        raise ValueError("frame too short")
    plen = raw[0]
    if len(raw) != plen + 7:    # 1 LEN + 1 TYPE + plen + 2 RSSI + 1 SNR + 2 CRC
        raise ValueError(f"length mismatch: LEN={plen} total={len(raw)}")
    rx_crc = raw[-2] | (raw[-1] << 8)
    if crc16_ccitt(raw[:-2]) != rx_crc:
        raise ValueError("CRC mismatch")
    return LoRaFrame(
        type=raw[1],
        payload=raw[2:2 + plen],
        rssi=int.from_bytes(raw[2 + plen:4 + plen], "little", signed=True),
        snr=int.from_bytes(raw[4 + plen:5 + plen], "little", signed=True),
    )


class SerialFramer:
    """Accumulate raw serial bytes; yield validated LoRaFrame objects.

    Usage:
        framer = SerialFramer()
        while True:
            chunk = os.read(fd, 512)
            for frame in framer.feed(chunk):
                dispatch(frame.payload, frame.rssi, frame.snr)
    """

    def __init__(self, max_frame: int = 512):
        self._buf = bytearray()
        self._max = max_frame
        self._overflow = False
        self.dropped = 0

    def feed(self, chunk: bytes) -> Iterator[LoRaFrame]:
        for b in chunk:
            if b == 0x00:                          # frame boundary
                if self._overflow:
                    self.dropped += 1
                    log.warning("dropped oversized frame")
                    self._buf.clear()
                    self._overflow = False
                    continue
                if not self._buf:                  # empty / back-to-back delimiter
                    continue
                frame = bytes(self._buf)
                self._buf.clear()
                try:
                    yield _parse_raw(cobs_decode(frame))
                except ValueError as e:
                    self.dropped += 1
                    log.warning("dropped bad frame (%d bytes): %s", len(frame), e)
            else:
                if len(self._buf) < self._max:
                    self._buf.append(b)
                else:
                    self._overflow = True


def encode_frame(mtype: int, payload: bytes = b"",
                 rssi: int = 0, snr: int = 0) -> bytes:
    """Build a complete wire frame (COBS-encoded + 0x00 delimiter).

    Used for Pi→Bridge messages: TYPE_TX (ACK), TYPE_CFG, TYPE_PING.
    rssi / snr are 0 for Pi→Bridge direction.
    """
    if len(payload) > 255:
        raise ValueError("payload too long (max 255 bytes)")
    raw = bytearray()
    raw.append(len(payload))
    raw.append(mtype & 0xFF)
    raw += payload
    raw += int(rssi).to_bytes(2, "little", signed=True)
    raw += int(snr).to_bytes(1, "little", signed=True)
    raw += crc16_ccitt(bytes(raw)).to_bytes(2, "little")
    return cobs_encode(bytes(raw)) + b"\x00"
