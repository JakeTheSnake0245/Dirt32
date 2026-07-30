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
import termios
import threading
import time


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
                self._handle(line.decode(errors="replace").strip())

    def _handle(self, line: str):
        """A malformed bridge line must never kill the reader thread."""
        try:
            if not line:
                return
            if line.startswith("RX "):
                parts = line.split()
                # Scan for the first token that is a non-empty even-length
                # hex string.  Under normal operation parts[1] is correct, but
                # if the bridge emitted a spurious "RX " prefix (USB-CDC split)
                # we may see "RX RX <hex> …" and need to skip the extra token.
                hex_idx = None
                _hex_chars = set("0123456789abcdefABCDEF")
                for i, tok in enumerate(parts[1:], start=1):
                    if tok and len(tok) % 2 == 0 and all(c in _hex_chars for c in tok):
                        hex_idx = i
                        break
                if hex_idx is None:
                    self.log(f"[serial] dropped garbled RX line: {line!r}")
                    return
                frame = bytes.fromhex(parts[hex_idx])
                def _i(s):
                    try: return int(s)
                    except (ValueError, TypeError): return None
                rssi = _i(parts[hex_idx + 1]) if len(parts) > hex_idx + 1 else None
                # SNR sent as snr*10 integer to avoid float printf bugs
                snr_raw = _i(parts[hex_idx + 2]) if len(parts) > hex_idx + 2 else None
                snr = snr_raw / 10.0 if snr_raw is not None else None
                self.on_frame(frame, rssi, snr)
            elif line == "RDY":
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

    def start(self):
        t = threading.Thread(target=self.run, daemon=True, name="serial")
        t.start()
        return t

    def stop(self):
        self._stop.set()
        self._close()
