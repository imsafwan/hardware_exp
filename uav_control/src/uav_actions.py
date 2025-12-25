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
import logging
import cv2
import os
import socket
from datetime import datetime
import struct



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

    

    start_time = time.time()
    time_limit = 5.0  # seconds

    while True:
        try:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
            await drone.offboard.start()
            logger_print("Offboard started")
            break
        except OffboardError as e:
            if time.time() - start_time > time_limit:
                logger_print(f" Offboard start failed: {e}")
                return False
            continue
    
    try:
        while True:
            pv  = await anext(drone.telemetry.position_velocity_ned())
            gps = await anext(drone.telemetry.position())

            cur_n = pv.position.north_m
            cur_e = pv.position.east_m
            rel_z = gps.relative_altitude_m

            logger_print('GPS: lat:{:.6f} lon:{:.6f}'.format(gps.latitude_deg, gps.longitude_deg))

            dn = tgt_n - cur_n
            de = tgt_e - cur_e
            dist_xy = math.hypot(dn, de)
            logger_print(f" Distance to target XY: {dist_xy:.2f} m")

            alt_err = target_alt - rel_z
            vd = -(alt_err * alt_Kp) + hover_bias
            vd = max(min(vd, vz_clip), -vz_clip)

            if dist_xy < arrival_thresh:
                logger_print(" Arrived at target XY & altitude")
                break

            if dist_xy > 1e-3:
                vn = (dn / dist_xy) * speed_mps
                ve = (de / dist_xy) * speed_mps
            else:
                vn = ve = 0.0

            logger_print(f"z:{rel_z:.2f}m   XY:{dist_xy:.2f}m  AltErr:{alt_err:.2f}m  vn:{vn:.2f} ve:{ve:.2f} vd:{vd:.2f}")
            await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, 0.0))
            
    except Exception as e:
        logger_print(f" Navigation error: {e}")
        return False

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,-0.15,0))
    logger_print(" Stopped at target")

    try:
        await drone.offboard.stop()
        logger_print("Exited Offboard mode, stopped at target")
    except Exception as e:
        logger_print(f"Failed to stop Offboard: {e}")

    try:
         logger_print(" Taking a photo at target...")
         capture_and_send_photo(target_lat =target_lat, target_lon=target_lon)
         logger_print(" Photo captured and saved")
    except Exception as e:
         logger_print(f"Failed to take photo: {e}")

    return True



def capture_and_send_photo(
        device="/dev/video0",
        folder="photo",
        server_ip="192.168.10.233",
        server_port=5070,
        target_lat=None,
        target_lon=None):

    os.makedirs(folder, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = os.path.join(folder, f"photo_{timestamp}.jpg")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open camera")
        return False

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Failed to capture frame")
        return False

    # Save
    cv2.imwrite(save_path, frame)
    print(f"[UAV] Photo saved: {save_path}")

    # Read JPEG bytes
    with open(save_path, "rb") as f:
        img_bytes = f.read()

    img_size = len(img_bytes)

    # -------------------------------
    # BUILD HEADER: lat, lon, size
    # -------------------------------
    # struct format:
    # d = double (8 bytes)
    # d = double
    # I = unsigned int (4 bytes)
    header = struct.pack("ddI", float(target_lat), float(target_lon), img_size)

    # -------------------------------
    # SEND TO GROUND STATION
    # -------------------------------
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((server_ip, server_port))

    sock.sendall(header)     # send metadata
    sock.sendall(img_bytes)  # send image

    sock.close()
    print("[UAV] Photo + metadata sent!")

    return True

def capture_and_save_photo(device=0, folder="photo"):
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

async def vision_landing(drone, camera_index=0, target_align_altitude=3.0):
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
            error_x = -(center[0] - 640 / 2)
            error_y = (center[1] - 480 / 2)

            vx = y_pid(1 * error_y)
            vy = x_pid(1 * error_x)
            vz = z_pid(1 * current_alt)

            logger_print(f"error_x:{error_x:.2f} error_y:{error_y:.2f} alt:{current_alt:.2f} → vx:{vx:.2f} vy:{vy:.2f} vz:{vz:.2f}")

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(forward_m_s=vx, right_m_s=vy, down_m_s=0.0, yawspeed_deg_s=0.0)
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



async def vision_landing_v2(drone, camera_index=0, target_marker_size=7500, 
                        image_width=640, image_height=480, log_dir="log"):
    """
    Vision-based precision landing using ArUco marker detection with size-based Z control.
    Allows manual landing via RC after alignment is achieved.
    
    Args:
        drone: MAVSDK drone instance
        camera_index: Camera device index (default: 0)
        target_marker_size: Target marker area in pixels² for landing (default: 9500)
        image_width: Camera frame width (default: 640)
        image_height: Camera frame height (default: 480)
        log_dir: Directory for video output (default: "log")
    
    Returns:
        bool: True if landing successful (auto or manual), False otherwise
    """
    logger_print("🎯 Starting vision-based landing with marker size control...")
    
    # --- Setup video recording ---
    os.makedirs(log_dir, exist_ok=True)
    timestamp = time.strftime('%Y%m%d-%H%M%S')
    video_path = f"{log_dir}/landing_{timestamp}.avi"
    
    # --- Camera setup ---
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        logger_print(f"❌ Failed to open camera: {camera_index}")
        return False
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
    logger_print("✅ Camera initialized")
    
    # --- Video writer setup ---
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video_writer = cv2.VideoWriter(video_path, fourcc, 20.0, (image_width, image_height))
    logger_print(f"📹 Recording to: {video_path}")
    
    # --- ArUco setup ---
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    # --- PID setup ---
    x_pid = PID(0.0025, 0.0, 0.0001, setpoint=0.0, output_limits=(-0.3, 0.3))
    y_pid = PID(0.0025, 0.0, 0.0001, setpoint=0.0, output_limits=(-0.3, 0.3))
    # Z PID now controls based on marker size (larger size = closer to ground)
    z_pid = PID(0.0002, 0.0, 0.00002, setpoint=target_marker_size, output_limits=(-0.125, 0.125))
    
    aligned = False  # Track alignment status
    
    def calculate_marker_size(corners):
        """Calculate the area of the detected marker using Shoelace formula"""
        pts = corners[0][0]
        area = 0.5 * abs(
            (pts[0][0] * pts[1][1] + pts[1][0] * pts[2][1] + 
             pts[2][0] * pts[3][1] + pts[3][0] * pts[0][1]) -
            (pts[1][0] * pts[0][1] + pts[2][0] * pts[1][1] + 
             pts[3][0] * pts[2][1] + pts[0][0] * pts[3][1])
        )
        return area
    
    try:
        # --- Start offboard mode ---
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, -0.15, 0))
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, -0.15, 0))
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, -0.15, 0))
        try:
            await drone.offboard.start()
            logger_print("✅ Offboard started for vision landing")
        except OffboardError as e:
            logger_print(f"❌ Offboard start failed: {e}")
            cap.release()
            video_writer.release()
            return False
        
        # --- Main landing loop ---
        while True:
            # Check if drone has been manually landed or disarmed
            try:
                async for armed in drone.telemetry.armed():
                    if not armed:
                        logger_print("✅ Drone disarmed (manual intervention detected)")
                        if aligned:
                            logger_print("✅ Task successful - aligned and manually landed")
                        else:
                            logger_print("✅ Task successful - manually landed (alignment not achieved)")
                        cap.release()
                        video_writer.release()
                        cv2.destroyAllWindows()
                        return True  # Always successful if landed
                    break
                
                async for in_air in drone.telemetry.in_air():
                    if not in_air:
                        logger_print("✅ Drone landed (manual intervention detected)")
                        if aligned:
                            logger_print("✅ Task successful - aligned and manually landed")
                        else:
                            logger_print("✅ Task successful - manually landed (alignment not achieved)")
                        cap.release()
                        video_writer.release()
                        cv2.destroyAllWindows()
                        return True  # Always successful if landed
                    break
            except:
                pass  # Continue if telemetry temporarily unavailable
            
            ret, frame = cap.read()
            if not ret:
                logger_print("❌ Failed to grab frame")
                continue
            
            # Ensure frame is correct size
            frame = cv2.resize(frame, (image_width, image_height))
            
            # Detect ArUco markers
            corners, ids, _ = aruco_detector.detectMarkers(frame)
            
            # Draw detected markers on frame
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Write frame to video
            video_writer.write(frame)
            
            # Get current altitude for logging
            async for pos in drone.telemetry.position():
                current_alt = pos.relative_altitude_m
                break
            
            if ids is not None:
                # Calculate marker center for X-Y alignment
                center = np.mean(corners[0][0], axis=0)
                
                # Calculate errors
                error_x = -(center[0] - image_width / 2)
                error_y = (center[1] - image_height / 2)
                
                # Calculate marker size for Z control
                marker_size = calculate_marker_size(corners)
                
                # Calculate PID outputs
                vx = y_pid(1 * error_y)
                vy = x_pid(1 * error_x)
                vz = z_pid(marker_size)  # Positive output when marker too small (descend needed)
                
                logger_print(f"📐 Marker size: {marker_size:.2f} px² (target: {target_marker_size})")
                logger_print(f"📍 error_x: {error_x:.2f}, error_y: {error_y:.2f}, alt: {current_alt:.2f}m")
                logger_print(f"🔧 PID → vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")
                
                # Send velocity commands
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(forward_m_s=vx, right_m_s=vy, down_m_s=0.0, yawspeed_deg_s=0.0)
                )
                
                # Check if aligned: well-centered and marker is large enough (close to ground)
                if abs(error_x) < 20 and abs(error_y) < 20 and marker_size >= target_marker_size * 0.85:
                    if not aligned:
                        aligned = True
                        logger_print("✅ MARKER ALIGNED! (size-based alignment complete)")
                        logger_print("💡 You can now land manually using your remote control")
                    
                    # Hold position after alignment
                    await drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0, 0, -0.1, 0)
                    )
                    
            else:
                logger_print("🔍 Marker not detected, holding position with slow yaw search...")
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0, 0, -0.1, 0.0)  # Slow yaw to search for marker
                )
            
            # Allow manual interrupt
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logger_print("🛑 Interrupted by user")
                break
        
        # If loop exits normally (unlikely), stop offboard
        await drone.offboard.stop()
        logger_print("🏁 Vision landing task completed")
        
        return aligned
        
    except Exception as e:
        logger_print(f"❌ Error during vision landing: {e}")
        logger_print(traceback.format_exc())
        return False
        
    finally:
        # --- Cleanup ---
        try:
            cap.release()
            video_writer.release()
            cv2.destroyAllWindows()
            logger_print("🧹 Camera and video resources released")
        except:
            pass
