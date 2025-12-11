# llm_worker.py
import time
import os
import json
from openai import OpenAI
import base64

IMAGE_FOLDER = "received_images"
QUEUE_FILE = "queue.txt"
PROCESSED_QUEUE_FILE = "queue_done.txt"

client = OpenAI(api_key='')
TARGET_OBJECT = "human or person"

def encode_image(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")

def analyze(image_path):
    img_b64 = encode_image(image_path)

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "user", "content": [
                {"type": "text", "text": f"Is '{TARGET_OBJECT}' present in this image? Reply YES or NO."},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"}}
            ]}
        ]
    )

    return response.choices[0].message.content.strip()

def process_queue():
    if not os.path.exists(QUEUE_FILE):
        return

    # read queue lines
    with open(QUEUE_FILE, "r") as f:
        lines = f.readlines()

    if not lines:
        return

    # process ONLY the first line (FIFO queue)
    line = lines[0].strip()
    filename, lat, lon = line.split(",")
    img_path = os.path.join(IMAGE_FOLDER, filename)

    print(f"[LLM] Processing {filename} ...")

    result = analyze(img_path)
    print(f"[LLM RESULT] {filename} → {result}")

    # If YES → print a special alert message
    if "YES" in result.upper():
        print(f">>> ALERT: Target object FOUND near lat={lat}, lon={lon}")
    else:
        print(f"[INFO] No target object found in {filename}")

    # Log the result
    with open(PROCESSED_QUEUE_FILE, "a") as f:
        f.write(f"{filename},{lat},{lon},{result}\n")

    # remove first line from queue
    with open(QUEUE_FILE, "w") as f:
        f.writelines(lines[1:])

if __name__ == "__main__":
    print("[LLM] Worker running...")
    while True:
        process_queue()
        time.sleep(1)
