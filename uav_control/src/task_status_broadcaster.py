#!/usr/bin/env python3
import os
import json
import socket
import time
import logging
import shutil

# === Logging Setup ===
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("broadcaster")

# === Config ===

TMP_DIR = "tmp"
if os.path.exists(TMP_DIR):
    shutil.rmtree(TMP_DIR)   # delete the folder if it exists
os.makedirs(TMP_DIR, exist_ok=True)

UGV_STATUS_FILE = os.path.join(TMP_DIR, "ugv_status.json")




STATUS_FILE = os.path.join(TMP_DIR, "uav_status.json")
BROADCAST_IP = "255.255.255.255"   # or subnet, e.g. "192.168.10.255"
UGV_LISTEN_PORT = 5005
C_LISTEN_PORT = 5055
INTERVAL = 0.5   # broadcast every 1 second

def broadcast_status():
    """Continuously read status file and broadcast contents."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    last_sent = None
    try:
        while True:
            if os.path.exists(STATUS_FILE):
                try:
                    with open(STATUS_FILE, "r") as f:
                        status = json.load(f)
                        msg = json.dumps(status).encode()
                        sock.sendto(msg, (BROADCAST_IP, UGV_LISTEN_PORT))
                        sock.sendto(msg, (BROADCAST_IP, C_LISTEN_PORT))
                        log.info(f"📡 Broadcast: {msg}")
                        last_sent = status

                except Exception as e:
                    log.error(f"❌ Failed to read/broadcast: {e}")

            time.sleep(INTERVAL)
    finally:
        sock.close()


if __name__ == "__main__":
    try:
        broadcast_status()
    except KeyboardInterrupt:
        log.warning("⏹️ Broadcaster stopped by user.")
