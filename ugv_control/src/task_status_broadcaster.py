#!/usr/bin/env python3
import os
import json
import time
import socket
import shutil

TMP_DIR = "tmp"
if os.path.exists(TMP_DIR):
    shutil.rmtree(TMP_DIR)   # delete the folder if it exists
os.makedirs(TMP_DIR, exist_ok=True)

UGV_STATUS_FILE = os.path.join(TMP_DIR, "ugv_status.json")

BROADCAST_IP = "192.168.10.255"   # subnet broadcast or 255.255.255.255
UAV_LISTEN_PORT = 5006            # port where others can listen to UGV status
C_LISTEN_PORT = 5056              # optional, for a central monitor


def load_status():
    """Read UGV status JSON from tmp/"""
    try:
        if os.path.exists(UGV_STATUS_FILE):
            with open(UGV_STATUS_FILE, "r") as f:
                return json.load(f)
    except Exception as e:
        print(f"Failed to read UGV status: {e}")
    return {"ugv_task_status": {}}


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print(f"Broadcasting UGV status from {UGV_STATUS_FILE} ...")
    try:
        while True:
            status = load_status()
            msg = json.dumps(status)
            try:
                sock.sendto(msg.encode(), (BROADCAST_IP, UAV_LISTEN_PORT))
                sock.sendto(msg.encode(), (BROADCAST_IP, C_LISTEN_PORT))
                print(f"→ Sent: {msg}")
            except Exception as e:
                print(f"Broadcast error: {e}")
            time.sleep(0.5)   # 1 Hz
    finally:
        sock.close()
        print("UGV broadcaster stopped.")


if __name__ == "__main__":
    main()
