import asyncio
import cv2
import numpy as np
from simple_pid import PID
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

import sys
import time
import os
import traceback
import logging

# === Setup Logging and Video ===
os.makedirs("log", exist_ok=True)
timestamp = time.strftime('%Y%m%d-%H%M%S')
log_path = f"log/log_{timestamp}.txt"
video_path = f"log/landing_{timestamp}.avi"



# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(log_path),
        logging.StreamHandler(sys.stdout)
    ]
)
log = logging.info



class ArUcoPrecisionLanding:
    def __init__(self):
        #self.marker_length = 2.0  # meters
        self.target_altitude = 5.0  # meters
        self.target_align_altitude = 2.5  # descend while aligning
        self.image_width = 640
        self.image_height = 480

        # ArUco setup
        #self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # PID Controllers
        self.x_pid = PID(0.0025, 0.0, 0.0001, setpoint=0.0, output_limits=(-0.5, 0.5))
        self.y_pid = PID(0.0025, 0.0, 0.0001, setpoint=0.0, output_limits=(-0.5, 0.5))
        self.z_pid = PID(0.05, 0.0, 0.0001, setpoint=self.target_align_altitude, output_limits=(-0.5, 0.5))
        
        
        # Camera setup
        self.cap = cv2.VideoCapture("/dev/video10")
        if not self.cap.isOpened():
            log("❌ Failed to open /dev/video10")
            raise RuntimeError("Failed to open /dev/video10")
        
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)

        # Video writer
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(video_path, fourcc, 20.0, (self.image_width, self.image_height))
        log("✅ Camera and video writer initialized.")

    async def connect_drone(self):
        self.drone = System()
        await self.drone.connect(system_address="udp://:14540")
        log("⏳ Waiting for drone connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                log("✅ Drone connected!")
                break

    async def arm_and_takeoff(self):
        await self.drone.action.set_takeoff_altitude(self.target_altitude)
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        log("🛫 Takeoff initiated...")

        async for pos in self.drone.telemetry.position():
            if abs(pos.relative_altitude_m - self.target_altitude) < 0.5:
                log(f"📏 Altitude reached: {pos.relative_altitude_m:.2f} m")
                break

    async def start_offboard_mode(self):
        log("🚀 Starting OFFBOARD mode...")
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yawspeed_deg_s=0.0)
        )
        try:
            await self.drone.offboard.start()
            log("✅ Offboard started.")
        except OffboardError as e:
            log(f"❌ Offboard start failed: {e._result.result}. Landing...")
            await self.drone.action.land()
            raise

    async def align_and_land(self):
        log("🎯 Starting marker alignment...")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                log("❌ Failed to grab frame.")
                continue

            
            
            frame = cv2.resize(frame, (self.image_width, self.image_height))
            

            corners, ids, _ = self.aruco_detector.detectMarkers(frame)
            if ids is not None:
                  cv2.aruco.drawDetectedMarkers(frame, corners, ids)             
            self.video_writer.write(frame)

            async for pos in self.drone.telemetry.position():
                current_alt = pos.relative_altitude_m
                break  # get latest altitude

            cv2.imshow("ArUco Markers", frame)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                
                center = np.mean(corners[0][0], axis=0)
                error_x = (center[0] - self.image_width / 2)
                error_y = -(center[1] - self.image_height / 2)

                vx = self.y_pid(-1 * error_y)
                vy = self.x_pid(-1 * error_x)
                vz = self.z_pid(-1* current_alt)

                log(f"error_x: {error_x:.2f}, error_y: {error_y:.2f}, error_z: {current_alt:.2f}")
                log(f"🔧 PID velocities: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")

                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(forward_m_s=vx, right_m_s=vy, down_m_s=vz, yawspeed_deg_s=0.0)
                )

                

                if abs(error_x) < 20 and abs(error_y) < 20 and current_alt <= self.target_align_altitude + 0.1:
                    log("📍 Marker aligned at ~2.5m. Initiating final landing...")
                    break
            else:
                log("🔍 Marker not detected. Holding...")
                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(forward_m_s=0, right_m_s=0, down_m_s=0, yawspeed_deg_s=0.0)
                )
            if cv2.waitKey(1) & 0xFF == ord('q'):
                log("🛑 Interrupted by user.")
                break

        await self.drone.offboard.stop()
        await asyncio.sleep(1)
        await self.drone.action.land()

        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                log("✅ Landed successfully.")
                break

    async def run(self):
        try:
            
            await self.connect_drone()
            await self.arm_and_takeoff()
            await self.start_offboard_mode()
            await self.align_and_land()
        finally:
            self.cleanup()

    def cleanup(self):
        if self.cap.isOpened():
            self.cap.release()
        self.video_writer.release()
        cv2.destroyAllWindows()
        log("🧹 Resources cleaned up.")


if __name__ == "__main__":
    try:

        
        #time.sleep(0)  # Optional startup delay
        landing = ArUcoPrecisionLanding()
        asyncio.run(landing.run())

    except KeyboardInterrupt:
        log(" Interrupted by user.")
        try:
            landing.cleanup()
        except:
            pass
        sys.exit(0)
    except Exception as e:
        log(f"❗Unhandled Exception: {e}")
        log(traceback.format_exc())
        try:
            landing.cleanup()
        except:
            pass
        sys.exit(1)