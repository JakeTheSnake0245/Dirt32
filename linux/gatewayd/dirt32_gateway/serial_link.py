"""Serial link to the gateway radio bridge (Heltec on USB).  Stdlib-only —
raw termios, no pyserial.

Binary COBS/CRC-16 framing (see serial_framer.py) replaces the old text
protocol that garbled on USB-CDC packet boundaries.  A 0x00 byte is the
only frame delimiter; corruption is caught by CRC + explicit length and
contained to a single frame — it can never bleed into the next one.

Bridge line protocol (lora_link.h / serial_framer.py):
  Bridge → Pi:  COBS(LEN TYPE PAYLOAD RSSI_i16 SNR_i8 CRC16) + 0x00
  Pi → Bridge:  same format
  TYPE_ALERT / TYPE_HEARTBEAT  received LoRa frame forwarded to ingest
  TYPE_TX                      Pi asks bridge to transmit payload via radio
  TYPE_CFG                     Pi sends radio config (12-byte LLCfg)
  TYPE_PING                    Pi keepalive; bridge replies TYPE_OK
  TYPE_RDY                     bridge (re)booted; Pi re-sends CFG
"""
from __future__ import annotations
import os
import termios
import threading
import time

from .serial_framer import (
    SerialFramer, LoRaFrame, encode_frame, encode_cfg,
    TYPE_ALERT, TYPE_HEARTBEAT,
    TYPE_TX, TYPE_CFG, TYPE_PING,
    TYPE_RDY, TYPE_OK, TYPE_ERR,
)


class SerialLink:
    def __init__(self, device: str, radio_cfg: dict, on_frame, log=print):
        """on_frame(frame_bytes, rssi, snr) is called from the reader thread."""
        self.device     = device
        self.radio_cfg  = radio_cfg
        self.on_frame   = on_frame
        self.log        = log
        self._fd        = None
        self._wlock     = threading.Lock()
        self._stop      = threading.Event()
        self._last_cfg   = 0.0
        self._last_write = 0.0
        self._PING_INTERVAL = 10.0   # seconds — must be < bridge SILENT threshold (15 s)
        self._framer    = SerialFramer()

    # -- port handling ------------------------------------------------------

    def _open(self) -> bool:
        try:
            fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY)
        except OSError as e:
            self.log(f"[serial] cannot open {self.device}: {e}")
            return False
        attrs = termios.tcgetattr(fd)
        # raw 115200 8N1
        attrs[0] = 0                             # iflag
        attrs[1] = 0                             # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0                             # lflag
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN]  = 0
        attrs[6][termios.VTIME] = 5              # 0.5 s read timeout
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        self._fd = fd
        self.log(f"[serial] opened {self.device}")
        self._send_cfg()
        return True

    def _send_cfg(self) -> None:
        c = self.radio_cfg
        payload = encode_cfg(c["freq_mhz"], c["sf"], c["bw_khz"],
                             c["cr"], c["net_id"], c["tx_power_dbm"])
        self._write_frame(TYPE_CFG, payload)
        self._last_cfg = time.time()

    def _write_frame(self, mtype: int, payload: bytes = b"") -> None:
        """Build and write one COBS frame to the bridge (thread-safe)."""
        data = encode_frame(mtype, payload)
        with self._wlock:
            if self._fd is not None:
                try:
                    os.write(self._fd, data)
                    self._last_write = time.time()
                except OSError:
                    self._close()

    def send_frame(self, frame: bytes) -> None:
        """Transmit a raw LoRa frame (gateway-sealed ACK) via the bridge."""
        self._write_frame(TYPE_TX, frame)

    def _close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    # -- reader thread ------------------------------------------------------

    def run(self) -> None:
        """Blocking COBS read loop with auto-reopen (USB unplug tolerant)."""
        while not self._stop.is_set():
            if self._fd is None:
                self._framer = SerialFramer()   # reset state machine on reconnect
                if not self._open():
                    time.sleep(3)
                    continue

            # Keepalive: PING the bridge every 10 s so its HOST indicator
            # stays green even when no nodes are transmitting.
            if time.time() - self._last_write > self._PING_INTERVAL:
                self._write_frame(TYPE_PING)

            try:
                chunk = os.read(self._fd, 512)
            except OSError:
                self.log("[serial] read error — reopening")
                self._close()
                continue
            if not chunk:
                continue

            try:
                for frame in self._framer.feed(chunk):
                    self._dispatch(frame)
            except Exception as e:          # noqa: BLE001
                self.log(f"[serial] framer error: {e}")

    def _dispatch(self, frame: LoRaFrame) -> None:
        """Handle one decoded COBS frame.  Must never kill the reader thread."""
        try:
            if frame.type in (TYPE_ALERT, TYPE_HEARTBEAT):
                # Raw encrypted LoRa bytes — feed to ingest for crypto + dispatch
                self.on_frame(frame.payload, frame.rssi, frame.snr)

            elif frame.type == TYPE_RDY:
                # Bridge (re)booted — it is back on its default radio config,
                # so always reapply ours. Guard against CFG→RDY echo loops.
                self.log("[serial] bridge ready — syncing radio config")
                now = time.time()
                if now - self._last_cfg > 1.0:
                    self._last_cfg = now
                    self._send_cfg()

            elif frame.type == TYPE_ERR:
                code = frame.payload[0] if frame.payload else 0
                self.log(f"[serial] bridge ERR (code=0x{code:02x})")

            # TYPE_OK, TYPE_PING, unknowns: ignore silently

        except Exception as e:              # noqa: BLE001 — reader must survive
            self.log(f"[serial] dispatch error type=0x{frame.type:02x}: {e}")

    def start(self) -> threading.Thread:
        t = threading.Thread(target=self.run, daemon=True, name="serial")
        t.start()
        return t

    def stop(self) -> None:
        self._stop.set()
        self._close()
