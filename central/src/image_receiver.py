# continuous_receiver_with_metadata.py
import socket
import struct
import os
from datetime import datetime

HOST = ""
PORT = 5070
SAVE_FOLDER = "received_images"
QUEUE_FILE = "queue.txt"

os.makedirs(SAVE_FOLDER, exist_ok=True)

def receive_exact(conn, size):
    data = b""
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk:
            return None
        data += chunk
    return data

def start_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(5)

    print(f"[GS] Listening for UAV on port {PORT}...")

    while True:
        conn, addr = server.accept()
        print(f"[GS] UAV connected: {addr}")

        header = receive_exact(conn, 20)  # 8+8+4
        if not header:
            conn.close()
            continue

        target_lat, target_lon, img_size = struct.unpack("ddI", header)
        img_bytes = receive_exact(conn, img_size)
        conn.close()

        if img_bytes:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"img_{timestamp}.jpg"
            img_path = os.path.join(SAVE_FOLDER, filename)

            # save image
            with open(img_path, "wb") as f:
                f.write(img_bytes)

            print(f"[GS] Saved image: {img_path}")

            # -------- ADD TO QUEUE -------- #
            with open(QUEUE_FILE, "a") as q:
                q.write(f"{filename},{target_lat},{target_lon}\n")

            print("[GS] Added to queue")

        else:
            print("[GS] No image received!")

if __name__ == "__main__":
    start_server()