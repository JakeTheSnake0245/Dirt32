"""Serial link to the gateway radio bridge (Heltec on USB). Stdlib only —
raw termios instead of pyserial.

Bridge line protocol (see esp32/gateway-bridge/src/main.cpp):
  <- RX <hex> <rssi> <snr>
  <- RDY / OK ... / ERR ...
  -> CFG <freq> <sf> <bw> <cr> <net_id> <txpower>
  -> TX <hex>
"""
from __future__ import annotations
import os
import re
import termios
import threading
import time

# Strict pattern — exactly one hex token (even length), signed RSSI, signed SNR.
_RX_RE = re.compile(
    r'^RX ([0-9a-fA-F]{2,128}) (-?\d+) (-?\d+)$'
)


class SerialLink:
    def __init__(self, device: str, radio_cfg: dict, on_frame, log=print):
        """on_frame(frame_bytes, rssi, snr) is called from the reader thread."""
        self.device = device
        self.radio_cfg = radio_cfg
        self.on_frame = on_frame
        self.log = log
        self._fd = None
        self._wlock = threading.Lock()
        self._stop = threading.Event()
        self._last_cfg = 0.0
        self._last_write = 0.0          # tracks last line sent for keepalive
        self._PING_INTERVAL = 10.0      # seconds — must be < bridge SILENT threshold (15s)

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
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 5              # 0.5 s read timeout
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        self._fd = fd
        self.log(f"[serial] opened {self.device}")
        self._send_cfg()
        return True

    def _send_cfg(self):
        c = self.radio_cfg
        self.write_line("CFG %.1f %d %.1f %d %d %d" % (
            c["freq_mhz"], c["sf"], c["bw_khz"], c["cr"],
            c["net_id"], c["tx_power_dbm"]))

    def write_line(self, line: str):
        with self._wlock:
            if self._fd is not None:
                try:
                    os.write(self._fd, (line + "\n").encode())
                    self._last_write = time.time()
                except OSError:
                    self._close()

    def send_frame(self, frame: bytes):
        self.write_line("TX " + frame.hex())

    def _close(self):
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    # -- reader thread ------------------------------------------------------

    def run(self):
        """Blocking read loop with auto-reopen (USB unplug tolerant)."""
        buf = b""
        while not self._stop.is_set():
            if self._fd is None:
                if not self._open():
                    time.sleep(3)
                    continue
            # Keepalive: PING the bridge every 10 s so its HOST indicator
            # stays green even when no nodes are transmitting.
            if time.time() - self._last_write > self._PING_INTERVAL:
                self.write_line("PING")

            try:
                chunk = os.read(self._fd, 512)
            except OSError:
                self.log("[serial] read error — reopening")
                self._close()
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                flush = self._handle(line.decode(errors="replace").strip())
                if flush:
                    buf = b""   # discard partial next line — one bad frame
                    break       # must not poison the following read

    def _handle(self, line: str) -> bool:
        """Process one bridge line.  Returns True if the caller should flush
        the accumulation buffer (garbled frame — don't let the remainder
        corrupt the next line).  A malformed line must never kill the thread."""
        if not line:
            return False
        if line.startswith("RX "):
            m = _RX_RE.match(line)
            if not m:
                self.log(f"[serial] dropped garbled RX line: {line!r}")
                return True   # flush — outside try/except so it always returns
            try:
                frame = bytes.fromhex(m.group(1))
                rssi = int(m.group(2))
                snr  = int(m.group(3)) / 10.0   # snr*10 integer from bridge
                self.on_frame(frame, rssi, snr)
            except Exception as e:        # noqa: BLE001
                self.log(f"[serial] error dispatching frame: {e}")
            return False
        try:
            if line == "RDY":
                # Bridge (re)booted — it is back on its default radio config,
                # so always reapply ours. Guard against CFG->RDY echo loops.
                self.log("[serial] bridge ready — syncing radio config")
                now = time.time()
                if now - self._last_cfg > 1.0:
                    self._last_cfg = now
                    self._send_cfg()
            elif line.startswith("ERR"):
                self.log(f"[serial] bridge: {line}")
                if "no-cfg" in line:
                    self._send_cfg()
        except Exception as e:            # noqa: BLE001 — reader must survive
            self.log(f"[serial] error handling line {line!r}: {e}")
        return False

    def start(self):
        t = threading.Thread(target=self.run, daemon=True, name="serial")
        t.start()
        return t

    def stop(self):
        self._stop.set()
        self._close()
