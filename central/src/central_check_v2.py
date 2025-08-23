#!/usr/bin/env python3
import socket
import json
import select
import subprocess
import sys

UAV_PORT = 5055
UGV_PORT = 5056
BUFFER_SIZE = 4096

# SSH targets
UAV_HOST = "safwan@192.168.10.120"
UGV_HOST = "administrator@192.168.10.77"
UAV_SCRIPT = "~/uav_control/src/uav_task.sh"
UGV_SCRIPT = "~/ugv_control/src/ugv_task.sh"

# Track statuses
uav_status = {}
ugv_status = {}
procs = []  # to hold SSH subprocess handles


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


def launch_remote(host, script):
    """Launch a remote script via SSH."""
    print(f"🚀 Running {script} on {host}...")
    return subprocess.Popen(
        ["ssh", host, script],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )


def stop_all(sock_uav, sock_ugv):
    """Close sockets and terminate subprocesses."""
    print("\n🛑 Stopping all remote processes...")
    for p in procs:
        try:
            p.terminate()
        except Exception:
            pass
    sock_uav.close()
    sock_ugv.close()
    sys.exit(0)


def main():
    global uav_status, ugv_status, procs

    sock_uav = create_socket(UAV_PORT)
    sock_ugv = create_socket(UGV_PORT)

    print(f"✅ Central listening on ports -> UAV:{UAV_PORT}, UGV:{UGV_PORT}")

    # 🚀 Launch scripts remotely (they will run until finished)
    procs = [
        launch_remote(UAV_HOST, UAV_SCRIPT),
        launch_remote(UGV_HOST, UGV_SCRIPT),
    ]

    try:
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

                # Update statuses
                if "uav_task_status" in msg:
                    uav_status = msg["uav_task_status"]
                if "ugv_task_status" in msg:
                    ugv_status = msg["ugv_task_status"]

                # ✅ Stop when both finished successfully
                if all_success(uav_status) and all_success(ugv_status):
                    print("\n🎉 All UAV + UGV tasks completed SUCCESSFULLY. Stopping system...")
                    stop_all(sock_uav, sock_ugv)

    except KeyboardInterrupt:
        print("\n🛑 Central server interrupted by user.")
        stop_all(sock_uav, sock_ugv)


if __name__ == "__main__":
    main()
