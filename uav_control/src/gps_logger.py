import asyncio
import csv
import time
import os


def get_sim_time_from_file(filename="/home/safwan/hardware_exp/central/src/clock_log.txt", retry_delay=0.05, timeout=5.0):
    start = time.time()
    while True:
        try:
            with open(filename, "r") as f:
                line = f.read().strip()
                if line:
                    return float(line)
        except Exception:
            pass

        if time.time() - start > timeout:
            raise RuntimeError("Timeout: could not read sim time from file.")
        time.sleep(retry_delay)

async def record_gps_uav(drone, starting_time=None, folder="gps_logs"):
    """
    Record UAV GPS to a CSV file once every 1 second with timestamped filename.
    :param drone: Connected mavsdk.System() instance
    :param starting_time: reference time (defaults to now)
    :param folder: directory to save logs
    """
    if starting_time is None:
        starting_time = get_sim_time_from_file() #time.time()

    # Ensure log folder exists
    os.makedirs(folder, exist_ok=True)

    # Use integer seconds since epoch for filename
    timestamp = int(starting_time)
    filename = os.path.join(folder, f"{timestamp}.csv")

    with open(filename, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "latitude_deg", "longitude_deg"])  # header

        async for pos in drone.telemetry.position():
            gps = (pos.latitude_deg, pos.longitude_deg)
            elapsed = get_sim_time_from_file() - starting_time # time.time() - starting_time
            writer.writerow([elapsed, gps[0], gps[1]])
            f.flush()
            # sleep so we only log once per second
            #await asyncio.sleep(1.0)
