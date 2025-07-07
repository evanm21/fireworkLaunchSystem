# ======================================================
#  SAFE FIRING SYSTEM – PC / Operator Console (client.py)
# ------------------------------------------------------
#!/usr/bin/env python3
"""Interactive client with automatic heartbeat and two‑step workflow."""
import socket
import sys
import threading
import time

PORT = 65432  # must match server
HB_INTERVAL = 1  # seconds
# DEFAULT_HOST = "raspberrypi.local"
DEFAULT_HOST = "127.0.0.1"


def _hb_loop(sock: socket.socket):
    while True:
        try:
            sock.sendall(b"HB\n")
        except OSError:
            break
        time.sleep(HB_INTERVAL)


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print(f"🔗 Connecting to {host}:{PORT} …")
        s.connect((host, PORT))
        print("✅ Connected. Type commands: ARM ON, ARM OFF, FIRE <n>, CONT, STATUS")
        threading.Thread(target=_hb_loop, args=(s,), daemon=True).start()
        try:
            while True:
                cmd = input("> ").strip()
                if not cmd:
                    continue
                s.sendall((cmd + "\n").encode())
                resp = s.recv(1024)
                if not resp:
                    print("❌ Server closed connection")
                    break
                print(resp.decode().strip())
        except KeyboardInterrupt:
            print("\n👋 Bye")


if __name__ == "__main__":
    main()
