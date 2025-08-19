import socket
import json
import select

UAV_PORT = 5055
UGV_PORT = 5056
BUFFER_SIZE = 4096

def create_socket(port):
    """Helper to create and bind a UDP socket."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    return sock

def safe_get(msg, key):
    """Safe dict access with fallback."""
    return msg.get(key, "❌ Not available")

def pretty_print(agent, msg):
    """Print status nicely depending on agent."""
    print(f"\n📥 Message from {agent}:")
    if "uav_task_status" in msg:
        print(f"   UAV status: {msg['uav_task_status']}")
    if "ugv_task_status" in msg:
        print(f"   UGV status: {msg['ugv_task_status']}")
    if "uav_task_status" not in msg and "ugv_task_status" not in msg:
        print("   ⚠️ No task status in message")

def main():
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

if __name__ == "__main__":
    main()
