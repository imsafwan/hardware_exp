import asyncio
import math
import time
import cv2
import numpy as np
from simple_pid import PID
import pymap3d as pm
from mavsdk.offboard import OffboardError, VelocityNedYaw, VelocityBodyYawspeed
import os
from datetime import datetime
import logging
from mavsdk.telemetry import FlightMode



logger = logging.getLogger(__name__)   # module-level logger




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


# ------------- Utility --------------




async def anext(aiter):
    return await aiter.__anext__()



async def takeoff(drone, altitude=10.0):
    tolerance = 0.5
    await drone.action.set_takeoff_altitude(altitude)
    await drone.action.arm()
    await drone.action.takeoff()
    logger_print(f" Taking off to {altitude}m...")

    try:
        while True:
            pos = await anext(drone.telemetry.position())
            rel_alt = pos.relative_altitude_m
            #logger_print(f" Current relative altitude: {rel_alt:.2f} m")
            if rel_alt >= altitude - tolerance:
                #logger_print(f" Reached target altitude: {rel_alt:.2f} m")
                return True
    except Exception as e:
        logger_print(f" Takeoff error: {e}")
        return False



async def goto_gps_target_modi_2(
    drone,
    target_lat, target_lon, target_alt,
    ref_lat, ref_lon, ref_alt,
    speed_mps=0.8,
    arrival_thresh=0.5,
    offset_n=0.0,
    offset_e=0.0
):
    loop_hz = 20
    vz_clip = 0.65
    hover_bias = -0.075
    alt_Kp = 0.4

    # Convert GPS target to NED
    tgt_n, tgt_e, _ = pm.geodetic2ned(target_lat, target_lon, ref_alt,
                                      ref_lat, ref_lon, ref_alt)
    tgt_n += offset_n
    tgt_e += offset_e

    logger_print(f"Navigating to NED (N:{tgt_n:.2f} E:{tgt_e:.2f}) @ rel Alt:{target_alt:.2f}")

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
    try:
        await drone.offboard.start()
        logger_print("Offboard started")
    except OffboardError as e:
        logger_print(f" Offboard start failed: {e}")
        return False

    dt = 1.0 / loop_hz
    try:
        while True:
            pv  = await anext(drone.telemetry.position_velocity_ned())
            gps = await anext(drone.telemetry.position())

            cur_n = pv.position.north_m
            cur_e = pv.position.east_m
            rel_z = gps.relative_altitude_m

            # noise_n = np.random.normal(0, 0.5)   # mean=0, std=0.5m
            # noise_e = np.random.normal(0, 0.5)

            # cur_n_noisy = cur_n + noise_n
            # cur_e_noisy = cur_e + noise_e

            # dn = tgt_n - cur_n_noisy
            # de = tgt_e - cur_e_noisy

            #logger_print('GPS: lat:{:.6f} lon:{:.6f}'.format(gps.latitude_deg, gps.longitude_deg))

            dn = tgt_n - cur_n
            de = tgt_e - cur_e
            dist_xy = math.hypot(dn, de)
            #logger_print(f" Distance to target XY: {dist_xy:.2f} m")

            alt_err = target_alt - rel_z
            vd = -(alt_err * alt_Kp) + hover_bias
            vd = max(min(vd, vz_clip), -vz_clip)

            if dist_xy < arrival_thresh:
                #logger_print(" Arrived at target XY & altitude")
                break

            if dist_xy > 1e-3:
                vn = (dn / dist_xy) * speed_mps
                ve = (de / dist_xy) * speed_mps
            else:
                vn = ve = 0.0
            
            # if np.random.random() < 0.1:  # 5% chance per cycle
            #     gust_n = np.random.uniform(-1.2, 1.2)
            #     gust_e = np.random.uniform(-1.2, 1.2)
            #     vn += gust_n
            #     ve += gust_e
            #     logger_print('Wind gust applied')

            #logger_print(f"z:{rel_z:.2f}m   XY:{dist_xy:.2f}m  AltErr:{alt_err:.2f}m  vn:{vn:.2f} ve:{ve:.2f} vd:{vd:.2f}")
            await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, 0.0))
            #await asyncio.sleep(dt)
    except Exception as e:
        logger_print(f" Navigation error: {e}")
        return False

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
    logger_print(" Stopped at target")

    try:
        await drone.offboard.stop()
        logger_print("Exited Offboard mode, stopped at target")
    except Exception as e:
        logger_print(f"Failed to stop Offboard: {e}")



    try:
        logger_print(" Taking a photo at target...")
        capture_and_save_photo()
        logger_print(" Photo captured and saved")
    except Exception as e:
        logger_print(f"Failed to take photo: {e}")

    return True


async def goto_gps_target_modi_3(
    drone,
    target_lat, target_lon, target_alt,
    ref_lat, ref_lon, ref_alt,
    speed_mps=0.8,
    arrival_thresh=0.5,
    offset_n=0.0,
    offset_e=0.0
):
    loop_hz = 20
    vz_clip = 0.65
    hover_bias = -0.075
    alt_Kp = 0.4

    # Convert GPS target to NED
    tgt_n, tgt_e, _ = pm.geodetic2ned(target_lat, target_lon, ref_alt,
                                      ref_lat, ref_lon, ref_alt)
    tgt_n += offset_n
    tgt_e += offset_e

    logger_print(f"Navigating to NED (N:{tgt_n:.2f} E:{tgt_e:.2f}) @ rel Alt:{target_alt:.2f}")

    



    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
    try:
        await drone.offboard.start()
        logger_print("Offboard started")
    except OffboardError as e:
        logger_print(f" Offboard start failed: {e}")
        return False
    
    # ------------------------------------------------------
    # Start watchdog task
    # ------------------------------------------------------
    watchdog_task = asyncio.create_task(offboard_watchdog(drone))
    logger_print("[Watchdog] Started")

    dt = 1.0 / loop_hz
    try:
        while True:
            pv  = await anext(drone.telemetry.position_velocity_ned())
            gps = await anext(drone.telemetry.position())

            cur_n = pv.position.north_m
            cur_e = pv.position.east_m
            rel_z = gps.relative_altitude_m

            # noise_n = np.random.normal(0, 0.5)   # mean=0, std=0.5m
            # noise_e = np.random.normal(0, 0.5)

            # cur_n_noisy = cur_n + noise_n
            # cur_e_noisy = cur_e + noise_e

            # dn = tgt_n - cur_n_noisy
            # de = tgt_e - cur_e_noisy

            #logger_print('GPS: lat:{:.6f} lon:{:.6f}'.format(gps.latitude_deg, gps.longitude_deg))

            dn = tgt_n - cur_n
            de = tgt_e - cur_e
            dist_xy = math.hypot(dn, de)
            #logger_print(f" Distance to target XY: {dist_xy:.2f} m")

            alt_err = target_alt - rel_z
            vd = -(alt_err * alt_Kp) + hover_bias
            vd = max(min(vd, vz_clip), -vz_clip)

            if dist_xy < arrival_thresh:
                #logger_print(" Arrived at target XY & altitude")
                break

            if dist_xy > 1e-3:
                vn = (dn / dist_xy) * speed_mps
                ve = (de / dist_xy) * speed_mps
            else:
                vn = ve = 0.0
            
            # if np.random.random() < 0.1:  # 5% chance per cycle
            #     gust_n = np.random.uniform(-1.2, 1.2)
            #     gust_e = np.random.uniform(-1.2, 1.2)
            #     vn += gust_n
            #     ve += gust_e
            #     logger_print('Wind gust applied')

            #logger_print(f"z:{rel_z:.2f}m   XY:{dist_xy:.2f}m  AltErr:{alt_err:.2f}m  vn:{vn:.2f} ve:{ve:.2f} vd:{vd:.2f}")
            await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, 0.0))
            #await asyncio.sleep(dt)
    except Exception as e:
        logger_print(f"Navigation error: {e}")
        watchdog_task.cancel()
        return False

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
    logger_print(" Stopped at target")

    try:
        await drone.offboard.stop()
        logger_print("Exited Offboard mode, stopped at target")
    except Exception as e:
        logger_print(f"Failed to stop Offboard: {e}")

    # ------------------------------------------------------
    # CANCEL WATCHDOG (CRITICAL)
    # ------------------------------------------------------
    watchdog_task.cancel()
    logger_print("[Watchdog] Cancelled")


    try:
        logger_print(" Taking a photo at target...")
        capture_and_save_photo()
        logger_print(" Photo captured and saved")
    except Exception as e:
        logger_print(f"Failed to take photo: {e}")

    return True

async def offboard_watchdog(drone, restart=True, max_restarts=10):
    
    restart_count = 0

    try:
        async for state in drone.telemetry.flight_mode():
            if state != FlightMode.OFFBOARD:
                logger_print(f"[Watchdog] Offboard LOST → Now in: {state}")

                if not restart:
                    continue

                restart_count += 1
                if restart_count > max_restarts:
                    logger_print("[Watchdog] Too many restarts → stopping watchdog")
                    return

                try:
                    # Safe initial setpoint BEFORE attempting offboard start
                    await drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0, 0, 0, 0)
                    )
                    await drone.offboard.start()
                    logger_print("[Watchdog] Offboard successfully RESTARTED")

                except Exception as e:
                    logger_print(f"[Watchdog] Restart failed: {e}")

            await asyncio.sleep(0.1)

    except asyncio.CancelledError:
        logger_print("[Watchdog] Cancelled safely")
        return
    
def capture_and_save_photo(device="/dev/video0", folder="photo"):
    # Ensure folder exists
    os.makedirs(folder, exist_ok=True)
    # Generate filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = os.path.join(folder, f"photo_{timestamp}.jpg")
    # Capture image
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        logger_print(f"Failed to open camera: {device}")
        return False
    ret, frame = cap.read()
    cap.release()
    if ret:
        cv2.imwrite(save_path, frame)
        logger_print(f"Photo saved to {save_path}")
        return True
    else:
        logger_print("Failed to capture frame")
        return False

async def land(drone):
    logger_print(" Landing...")
    try:
        await drone.action.land()
        async for is_armed in drone.telemetry.armed():
            if not is_armed:
                logger_print(" Landing complete.")
                return True
    except Exception as e:
        logger_print(f" Landing error: {e}")
        return False




# ------------------- Vision-Based Landing ------------------- #

async def vision_landing(drone, camera_index="/dev/video0", target_align_altitude=3.0):
    logger_print(" Starting vision-based landing...")

    # --- Camera setup ---
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        logger_print(f" Failed to open camera: {camera_index}")
        return False
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # --- ArUco setup ---
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # --- PID setup ---
    x_pid = PID(0.0025, 0.0, 0.0001, setpoint=0.0, output_limits=(-0.5, 0.5))
    y_pid = PID(0.0025, 0.0, 0.0001, setpoint=0.0, output_limits=(-0.5, 0.5))
    z_pid = PID(0.05,   0.0, 0.0001, setpoint=target_align_altitude, output_limits=(-0.5, 0.5))

    try:
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        await drone.offboard.start()
        logger_print(" Offboard started for vision landing")
    except OffboardError as e:
        logger_print(f" Offboard start failed: {e}")
        return False

    while True:
        ret, frame = cap.read()
        
        if not ret:
            logger_print(" Failed to grab frame")
            continue

        corners, ids, _ = aruco_detector.detectMarkers(frame)
        async for pos in drone.telemetry.position():
            current_alt = pos.relative_altitude_m
            break

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids) 
            center = np.mean(corners[0][0], axis=0)
            error_x = (center[0] - 640 / 2)
            error_y = -(center[1] - 480 / 2)

            vx = y_pid(-1 * error_y)
            vy = x_pid(-1 * error_x)
            vz = z_pid(-1 * current_alt)

            logger_print(f"error_x:{error_x:.2f} error_y:{error_y:.2f} alt:{current_alt:.2f} → vx:{vx:.2f} vy:{vy:.2f} vz:{vz:.2f}")

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(forward_m_s=vx, right_m_s=vy, down_m_s=vz, yawspeed_deg_s=0.0)
            )

            if abs(error_x) < 20 and abs(error_y) < 20 and current_alt <= target_align_altitude + 0.1:
                logger_print(" Marker aligned, initiating final landing...")
                break
        else:
            logger_print("🔍 Marker not detected, holding...")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        #cv2.imshow("Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            logger_print(" Interrupted by user")
            break

    await drone.offboard.stop()
    await asyncio.sleep(1)
    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            logger_print(" Landed successfully.")
            break

    cap.release()
    cv2.destroyAllWindows()
    return True
