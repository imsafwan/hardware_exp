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
PROMPT = """
Analyze the aerial image for a simulated disaster response.

Check for ANY of the following hazards:
- human/mannequin/doll (victim)
- person lying on the ground
- orange hazard cones
- smoke or fire

Respond ONLY:
YES — if any hazard is present  
NO — if none are present
"""


def encode_image(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")


def analyze(image_path):
    img_b64 = encode_image(image_path)

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": PROMPT},
                    {"type": "image_url",
                     "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"}}
                ]
            }
        ]
    )

    # FIXED: new SDK uses .content, not ["content"]
    return response.choices[0].message.content.strip()


def process_queue():
    if not os.path.exists(QUEUE_FILE):
        return

    with open(QUEUE_FILE, "r") as f:
        lines = f.readlines()

    if not lines:
        return

    # FIFO: take first entry
    line = lines[0].strip()
    filename, lat, lon = line.split(",")

    img_path = os.path.join(IMAGE_FOLDER, filename)
    print(f"[LLM] Processing {filename} ...")

    result = analyze(img_path)
    print(f"[LLM RESULT] {filename} → {result}")

    if "YES" in result.upper():
        print(f" ALERT: Hazard detected near lat={lat}, lon={lon}")
    else:
        print(f"[INFO] No hazard detected in {filename}")

    with open(PROCESSED_QUEUE_FILE, "a") as f:
        f.write(f"{filename},{lat},{lon},{result}\n")

    # Remove first entry from queue
    with open(QUEUE_FILE, "w") as f:
        f.writelines(lines[1:])


if __name__ == "__main__":
    print("[LLM] Worker running...")
    while True:
        process_queue()
        time.sleep(1)
