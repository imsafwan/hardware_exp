#!/usr/bin/env python3
import socket
import json
import select
import subprocess
import signal
import sys

UAV_PORT = 5055
UGV_PORT = 5056
BUFFER_SIZE = 4096

# Track statuses
uav_status = {}
ugv_status = {}

def create_socket(port):
    """Helper to create and bind a UDP socket."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    return sock

def pretty_print(agent, msg):
    """Print status nicely depending on agent."""
    print(f"\n📥 Message from {agent}:")
    if "uav_task_status" in msg:
        print(f"   UAV status: {msg['uav_task_status']}")
    if "ugv_task_status" in msg:
        print(f"   UGV status: {msg['ugv_task_status']}")
    if "uav_task_status" not in msg and "ugv_task_status" not in msg:
        print("   ⚠️ No task status in message")

def all_success(status_dict):
    """Check if all actions in status_dict are SUCCESS."""
    return status_dict and all(v.get("status") == "SUCCESS" for v in status_dict.values())

def main():
    global uav_status, ugv_status

    sock_uav = create_socket(UAV_PORT)
    sock_ugv = create_socket(UGV_PORT)

    print(f"✅ Central listening on ports -> UAV:{UAV_PORT}, UGV:{UGV_PORT}")

    while True:
        readable, _, _ = select.select([sock_uav, sock_ugv], [], [])
        for s in readable:
            try:
                data, addr = s.recvfrom(BUFFER_SIZE)
                msg = json.loads(data.decode())
            except json.JSONDecodeError:
                print(f"⚠️ Invalid JSON from {addr}: {data}")
                continue

            agent = "UAV" if s is sock_uav else "UGV"
            pretty_print(agent, msg)

            # Update global statuses
            if "uav_task_status" in msg:
                uav_status = msg["uav_task_status"]
            if "ugv_task_status" in msg:
                ugv_status = msg["ugv_task_status"]

            # ✅ Check if all tasks finished successfully
            if all_success(uav_status) and all_success(ugv_status):
                print("\n🎉 All UAV + UGV tasks completed SUCCESSFULLY. Stopping system...")

                # Send termination signal to children (if you launched UAV/UGV from here)
                # subprocess.Popen(... for UAV/UGV would give you PIDs to kill here.
                sys.exit(0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Central server interrupted by user. Exiting...")
