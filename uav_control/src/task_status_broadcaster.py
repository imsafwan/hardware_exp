#!/usr/bin/env python3

''' This script continuously reads UAV and AoI status files and broadcasts the merged status via UDP. '''

import os
import json
import socket
import time
import logging
import shutil
import datetime

# === Logging Setup ===

# ensure log directory exists
LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE = os.path.join(LOG_DIR, f"uav_actions_broadcast_{timestamp}.log")

logging.basicConfig(
    filename=LOG_FILE,   # now stored inside logs/
    level=logging.INFO,  # DEBUG, INFO, WARNING, ERROR, CRITICAL
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)

logger = logging.getLogger("main")

def logger_print(msg, level="info"):
    print(msg)
    if level == "info":
        logger.info(msg)
    elif level == "warn":
        logger.warn(msg)
    elif level == "error":
        logger.error(msg)
    elif level == "debug":
        logger.debug(msg)


#=== Config ===
TMP_DIR = "tmp"
if os.path.exists(TMP_DIR):
    shutil.rmtree(TMP_DIR)   # delete the folder if it exists
os.makedirs(TMP_DIR, exist_ok=True)

UAV_STATUS_FILE = os.path.join(TMP_DIR, "uav_status.json")
AOI_STATUS_FILE = os.path.join(TMP_DIR, "aoi_status.json")

BROADCAST_IP =  "192.168.10.255" #  "255.255.255.255"   # or subnet, e.g. "192.168.10.255"
UGV_LISTEN_PORT = 5005
C_LISTEN_PORT = 5055
INTERVAL = 0.25   # broadcast every 0.5 second

def safe_load_json(filename):
    """Load JSON from file if exists, else return empty dict."""
    #print('aaaa')
    #print(filename)
    if os.path.exists(filename):
        try:
            with open(filename, "r") as f:
                return json.load(f)
        except Exception as e:
            logger_print(f"Failed to read {filename}: {e}")
    return {}

def broadcast_status():
    """Continuously read UAV + AoI status files and broadcast merged contents."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    #print('a')

    try:
        #print('aa')
        while True:
            status = {}
            #print('aaa')

            # Merge UAV status
            uav_status = safe_load_json(UAV_STATUS_FILE)
            #print(uav_status)
            if uav_status:
                status["uav_task_status"] = uav_status.get("uav_task_status", {})

            # Merge AoI status
            aoi_status = safe_load_json(AOI_STATUS_FILE)
            if aoi_status:
                status["aoi_status"] = aoi_status.get("aoi_status", {})

            if status:
                msg = json.dumps(status).encode()
                sock.sendto(msg, (BROADCAST_IP, UGV_LISTEN_PORT))
                sock.sendto(msg, (BROADCAST_IP, C_LISTEN_PORT))
                logger_print(f"Broadcast: {msg}")
                #print('b')
            else:
                logger_print("No status to broadcast.")
                #print('c')

            time.sleep(INTERVAL)
    finally:
        sock.close()


if __name__ == "__main__":
    try:
        broadcast_status()
    except KeyboardInterrupt:
        logger_print("Broadcaster stopped by user.")