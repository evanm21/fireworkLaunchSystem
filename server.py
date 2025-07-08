# ======================================================
#  SAFE FIRING SYSTEM – Raspberry Pi (server.py)
# ------------------------------------------------------
#!/usr/bin/env python3
"""DIY fire‑work ignition server with positive‑safety features – **ADC removed**.

Protocol  (all commands case‑insensitive, \n‑terminated):

    HB                     heartbeat – must arrive every <HEARTBEAT_TIMEOUT> s
    ARM ON|OFF             arm (live) / safe (default)
    FIRE <relay>           fire cue (only when ARMED)
    STATUS                 armed state, last fire time

Every request yields one line response:
    OK <info> / ERR <msg>

Hardware assumptions
--------------------
* Relay board driven by BCM GPIO pins (mapped in RELAY_MAP).
* Stack‑light or LED on LED_PIN lights when ARMED.
* A physical keyswitch or dead‑man switch is **still strongly recommended**.
"""
from __future__ import annotations

import signal
import socket
import sys
import threading
import time
from datetime import datetime
from typing import Final
import argparse          # ← add

# ── argument parsing (goes here) ───────────────────────────
parser = argparse.ArgumentParser(
    description="Safe firing server (use --mock when testing on non-Pi)"
)
parser.add_argument(
    "--mock",
    action="store_true",
    help="replace RPi.GPIO with a console-print mock for local testing",
)
args = parser.parse_args()
# ----------------------------------------------------------

# ── GPIO SETUP ─────────────────────────────────────────────
try:
    import RPi.GPIO as GPIO
except (RuntimeError, ModuleNotFoundError):
    # fall back automatically if we're not on a Pi
    args.mock = True

if args.mock:
    class _MockGPIO:              # ← same stub you saw earlier
        BCM = OUT = HIGH = LOW = None
        def setmode(self, *_): pass
        def setwarnings(self, *_): pass
        def setup(self, pin, mode, initial=None): print(f"[GPIO] setup {pin}")
        def output(self, pin, state):            print(f"[GPIO] {pin} -> {'HIGH' if state else 'LOW'}")
        def cleanup(self):                       print("[GPIO] cleanup")
    GPIO = _MockGPIO()          # type: ignore
else:
    import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ─── CONSTANTS & CONFIG ───────────────────────────────────────────
HOST = ""            # 0.0.0.0 – listen on all interfaces
PORT = 65432
BUFFER = 1024         # bytes per recv()
PULSE_MS = 500        # relay ON duration (ms)
HEARTBEAT_TIMEOUT = 5 # seconds – if missed → SAFE
LED_PIN = 12          # optional indicator LED (set to None to disable)

# Relay‑to‑GPIO mapping – EDIT to match your wiring
RELAY_MAP: Final[dict[int, int]] = {
    1: 2,
    2: 3,
    3: 4,
    4: 17,
    5: 27,
    6: 22,
    7: 10,
    8: 9
}

# ─── GLOBAL STATE ─────────────────────────────────────────────────
ARMED = False
_LAST_FIRE: str | None = None
_last_hb = time.time()
_state_lock = threading.Lock()
_pins_initialised: set[int] = set()

# ─── UTILITY FUNCTIONS ────────────────────────────────────────────

def _ensure_pin(pin: int):
    if pin not in _pins_initialised:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        _pins_initialised.add(pin)


def _set_all_low():
    for pin in _pins_initialised:
        GPIO.output(pin, GPIO.LOW)


def _set_led(on: bool):
    if LED_PIN is None:
        return
    GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(LED_PIN, GPIO.HIGH if on else GPIO.LOW)


def _safe_mode():
    global ARMED
    with _state_lock:
        ARMED = False
        _set_all_low()
        _set_led(False)


def _arm_mode():
    global ARMED
    with _state_lock:
        ARMED = True
        _set_led(True)


def _pulse_relay(relay: int, ms: int = PULSE_MS):
    pin = RELAY_MAP[relay]
    _ensure_pin(pin)
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(ms / 1000)
    GPIO.output(pin, GPIO.LOW)

# ─── HEARTBEAT WATCHDOG THREAD ────────────────────────────────────

def _hb_watchdog():
    while True:
        time.sleep(1)
        with _state_lock:
            if ARMED and time.time() - _last_hb > HEARTBEAT_TIMEOUT:
                print("⚠️  Heartbeat lost – reverting to SAFE mode")
                _safe_mode()

threading.Thread(target=_hb_watchdog, daemon=True).start()

# ─── COMMAND PARSER ───────────────────────────────────────────────

def _process(cmd: str, conn: socket.socket):
    global _last_hb, _LAST_FIRE

    parts = cmd.strip().split()
    if not parts:
        return

    op = parts[0].upper()

    # ── Heartbeat ──────────────────
    if op == "HB":
        _last_hb = time.time()
        # conn.sendall(b"OK HB\n")
        return

    # ── ARM / SAFE ─────────────────
    if op == "ARM" and len(parts) == 2:
        if parts[1].upper() == "ON":
            _arm_mode()
            conn.sendall(b"OK ARMED\n")
        elif parts[1].upper() == "OFF":
            _safe_mode()
            conn.sendall(b"OK SAFE\n")
        else:
            conn.sendall(b"ERR Use ARM ON|OFF\n")
        return

    # ── FIRE cue ───────────────────
    if op == "FIRE" and len(parts) == 2:
        try:
            relay = int(parts[1])
        except ValueError:
            conn.sendall(b"ERR Relay must be int\n")
            return
        if relay not in RELAY_MAP:
            conn.sendall(b"ERR Unknown relay\n")
            return
        if not ARMED:
            conn.sendall(b"ERR System not ARMED\n")
            return
        _pulse_relay(relay)
        _LAST_FIRE = datetime.now().isoformat(timespec="seconds")
        conn.sendall(f"OK FIRED {relay}\n".encode())
        return

    # ── STATUS ─────────────────────
    if op == "STATUS":
        st = "ARMED" if ARMED else "SAFE"
        last = _LAST_FIRE or "never"
        conn.sendall(f"OK {st} LAST {last}\n".encode())
        return

    conn.sendall(b"ERR Unknown/invalid command\n")

# ─── CLIENT HANDLER ───────────────────────────────────────────────

def _handler(conn: socket.socket, addr):
    print(f"🔗 Connected {addr}")
    with conn:
        buf = b""
        while True:
            data = conn.recv(BUFFER)
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line:
                    _process(line.decode(), conn)
    print(f"❌ Disconnected {addr}")

# ─── CLEANUP ──────────────────────────────────────────────────────

def _cleanup(*_):
    print("\n🛑 Shutting down – GPIO cleanup …")
    _safe_mode()
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, _cleanup)

# ─── MAIN LOOP ────────────────────────────────────────────────────
print(f"📡 Listening on {PORT} – Ctrl‑C to quit")
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    while True:
        conn, addr = s.accept()
        threading.Thread(target=_handler, args=(conn, addr), daemon=True).start()