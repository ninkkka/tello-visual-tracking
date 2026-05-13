"""
ФИНАЛЬНЫЙ ПОЛЁТНЫЙ КОД v17 — Стабильная версия
"""

import multiprocessing
import traceback
from djitellopy import Tello
import cv2
import numpy as np
import time
from simple_pid import PID
import math
import socket
import struct
import threading
from collections import deque
from ctypes import c_bool

# ================= КАМЕРА =================
camera_matrix = np.array([[517.306, 0, 318.643],
[0, 516.469, 255.314],
[0, 0, 1]], dtype=np.float64)

dist_coeffs = np.array([0.262383, -0.953104,
-0.005358, 0.002628, 1.16331], dtype=np.float64)

marker_length = 0.12

# ================= НАСТРОЙКИ МАРКЕРОВ =================
MARKER_ID_FLOOR = 24
MARKER_ID_HAND = 3

# ================= ПИД (из MATLAB) =================
pid_x = PID(0.57, 0.50, 0.15, setpoint=0.0) # LR
pid_y = PID(0.05, 0.01, 0.00, setpoint=0.0) # UD
pid_z = PID(0.09, 0.12, 0.02, setpoint=0.0) # FB

pid_x.output_limits = (-0.7, 0.7)
pid_y.output_limits = (-0.3, 0.3)
pid_z.output_limits = (-0.7, 0.7)

# ================= КОНСТАНТЫ =================
DESIRED_CX = 320
DESIRED_CY = 240
DESIRED_DIAG_PX = 100

DEADZONE = 0.07
DEADZONE_DIAG = 5
MAX_LOST_FRAMES = 20
INIT_FRAMES = 50
TAKEOFF_DELAY = 5
BRAKE_ZONE_M = 0.08
BRAKE_SPEED = 0.8
UD_BRAKE_ZONE = 0.06
UD_BRAKE_SPEED = 0.7
UD_GAIN = 18
FB_BACK_CLIP = -0.35
UD_CLIP = 0.40
FB_RAMP_STEPS = 3
PITCH_COMPENSATION = 0.0

# ================= ARUCO =================
def detect_marker_and_pose(frame):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None and len(corners) > 0:
        marker_id = int(ids[0][0])
        pts = corners[0][0]
        center_px = pts.mean(axis=0)
        diag_px = np.linalg.norm(pts[0] - pts[2])
        
        obj_points = np.array([
            [-marker_length/2,  marker_length/2, 0],
            [ marker_length/2,  marker_length/2, 0],
            [ marker_length/2, -marker_length/2, 0],
            [-marker_length/2, -marker_length/2, 0]
        ], dtype=np.float32)
        img_points = pts.astype(np.float32)
        _, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, camera_matrix, dist_coeffs)
        return True, tvec.flatten(), marker_id, center_px, diag_px
    return False, None, None, None, None


def apply_tcw_to_point(Tcw, point_cam):
    point_h = np.array([point_cam[0], point_cam[1], point_cam[2], 1.0])
    Twc = np.linalg.inv(Tcw)
    world = Twc @ point_h
    return world[:3] / world[3]


def normalize_commands(cmd_x, cmd_y, cmd_z, max_total=1.0):
    norm = abs(cmd_x) + abs(cmd_y) + abs(cmd_z)
    if norm > max_total:
        scale = max_total / norm
        cmd_x *= scale
        cmd_y *= scale
        cmd_z *= scale
    cmd_x = np.clip(cmd_x, -0.7, 0.7)
    cmd_y = np.clip(cmd_y, -UD_CLIP, UD_CLIP)
    cmd_z = np.clip(cmd_z, -0.7, 0.7)
    return cmd_x, cmd_y, cmd_z


# ================= ОСНОВНОЙ КЛАСС =================
class Video:
    def __init__(self):
        self.run = multiprocessing.Value(c_bool, True)
        self.stream_server_running = multiprocessing.Value(c_bool, True)

    def reset_all_pids(self):
        pid_x.reset()
        pid_y.reset()
        pid_z.reset()

    def stream_to_slam(self, frame_read):
        HOST = '0.0.0.0'
        PORT = 9999
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(1)
        server.settimeout(1.0)
        print("Waiting for SLAM video connection...")
        conn = None
        while self.stream_server_running.value:
            try:
                conn, addr = server.accept()
                print("SLAM video connected:", addr)
                break
            except socket.timeout:
                continue
        if conn is None:
            return
        while self.stream_server_running.value:
            try:
                frame = frame_read.frame
                if frame is None:
                    continue
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                gray = cv2.resize(gray, (640, 480))
                _, encoded = cv2.imencode('.jpg', gray)
                data = encoded.tobytes()
                conn.sendall(struct.pack('<L', len(data)))
                conn.sendall(data)
            except:
                break
        conn.close()
        server.close()

    def video_get(self):
        drone = Tello()
        lost_frames = 0
        
        hand_marker_detected = False
        time_of_takeoff = None
        
        try:
            drone.connect()
            print("Battery:", drone.get_battery(), "%")
            drone.streamon()
            frame_read = drone.get_frame_read()

            stream_thread = threading.Thread(
                target=self.stream_to_slam, args=(frame_read,), daemon=True)
            stream_thread.start()

            ctrl_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            ctrl_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            ctrl_server.bind(('0.0.0.0', 9997))
            ctrl_server.listen(1)
            print("Waiting for SLAM control...")
            sock, addr = ctrl_server.accept()
            sock.settimeout(0.01)
            print("SLAM control connected:", addr)

            target_set = False
            flight_started = False
            countdown_started = False
            countdown_time = 0
            stable_count = 0
            desired_marker = np.zeros(3)
            yaw_offset = 0
            fb_freeze_counter = 0
            fb_ramp_counter = 0
            desired_diag = DESIRED_DIAG_PX

            print("\n=== ИНИЦИАЛИЗАЦИЯ SLAM ===")

            global pid_y
            pid_y = PID(0.05, 0.01, 0.00, setpoint=0.0)
            pid_y.output_limits = (-0.3, 0.3)
            pid_y._last_error = 0.0

            while self.run.value:
                img = frame_read.frame
                if img is None:
                    continue
                gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
                gray = cv2.resize(gray, (640, 480))

                has_tcw = False
                Tcw = np.eye(4)
                try:
                    data = sock.recv(64)
                    if len(data) == 64:
                        Tcw = np.array(struct.unpack('<16f', data)).reshape(4, 4)
                        has_tcw = True
                except:
                    pass

                found, tvec, marker_id, center_px, diag_px = detect_marker_and_pose(gray)
                yaw = drone.get_yaw()
                pitch = drone.get_pitch()
                roll = drone.get_roll()

                # ===== УСТАНОВКА ЦЕЛИ =====
                if not target_set and has_tcw and found:
                    stable_count += 1
                    if stable_count == INIT_FRAMES:
                        pid_z.setpoint = 0.0
                        pid_x.setpoint = 0.0
                        pid_y.setpoint = 0.0
                        self.reset_all_pids()
                        yaw_offset = yaw
                        target_set = True
                        countdown_started = True
                        countdown_time = time.time()
                        print(f"\n>>> ЦЕЛЬ: диагональ={DESIRED_DIAG_PX}px (фиксированная)")
                        print(f">>> ВЗЛЁТ через {TAKEOFF_DELAY} секунд!")
                    elif stable_count % 10 == 0:
                        print(f"  Init... {stable_count}/{INIT_FRAMES}")
                    continue

                # ===== ОБРАТНЫЙ ОТСЧЁТ =====
                if target_set and not flight_started:
                    elapsed = time.time() - countdown_time
                    remaining = TAKEOFF_DELAY - elapsed
                    if remaining > 0:
                        if int(remaining) != int(remaining + 0.1):
                            print(f"  Взлёт через {int(remaining)}...")
                        time.sleep(0.05)
                        continue
                    else:
                        print("[TAKEOFF] Взлетаю!")
                        drone.takeoff()
                        
                        time_of_takeoff = time.time()
                        hand_marker_detected = False
                        self.reset_all_pids()
                        
                        time.sleep(2)
                        flight_started = True
                        print("[FLY] В воздухе! Покажи маркер №2!\n")
                        continue

                # ===== ПОЛЁТ =====
                if flight_started:
                    if has_tcw and found:
                        
                        elapsed = time.time() - time_of_takeoff
                        current_altitude = drone.get_height() / 100
                        
                        if elapsed < 3.0 and marker_id == MARKER_ID_FLOOR:
                            found = False
                        
                        if found and marker_id == MARKER_ID_HAND and not hand_marker_detected:
                            hand_marker_detected = True
                            pid_z.setpoint = 0.0
                            pid_z.reset()
                            pid_y.reset()
                            fb_freeze_counter = 5
                            fb_ramp_counter = 0
                            print(f"[NEW MARKER] Диагональ: {diag_px:.0f}px → цель: {DESIRED_DIAG_PX}px")
                        
                        if found and marker_id == MARKER_ID_FLOOR and current_altitude > 0.3:
                            found = False
                        
                        if not found:
                            lost_frames += 1
                            if lost_frames > MAX_LOST_FRAMES:
                                drone.send_rc_control(0, 0, 0, 0)
                                self.reset_all_pids()
                            time.sleep(0.05)
                            continue
                        
                        lost_frames = 0
                        
                        # ===== ОШИБКИ =====
                        if center_px is not None and tvec is not None:
                            fx = camera_matrix[0, 0]
                            
                            # LR — пиксели с roll-компенсацией
                            roll_offset_px = roll * (fx / 60.0)
                            corrected_cx = DESIRED_CX - roll_offset_px
                            error_x_px = corrected_cx - center_px[0]
                            ar_distance = max(tvec[2], 0.3)
                            error_x_m = error_x_px * ar_distance / fx
                            
                            # UD — напрямую из tvec
                            error_y_m = -tvec[1]
                            
                            if abs(error_x_m) < DEADZONE:
                                error_x_m = 0
                                pid_x.reset()
                            if abs(error_y_m) < DEADZONE:
                                error_y_m = 0
                                pid_y.reset()
                            
                            # Z — пиксельная логика
                            error_diag = DESIRED_DIAG_PX - diag_px
                            if abs(error_diag) < DEADZONE_DIAG:
                                error_diag = 0
                                pid_z.reset()
                            error_z_norm = error_diag / 40.0
                        else:
                            error_x_m = 0.0
                            error_y_m = 0.0
                            error_z_norm = 0.0
                        
                        # ===== ПИД =====
                        cmd_x_world = pid_x(error_x_m)
                        if abs(error_x_m) < BRAKE_ZONE_M:
                            cmd_x_world *= BRAKE_SPEED
                        
                        cmd_y_raw = pid_y(error_y_m)
                        
                        # Сброс интегратора при смене знака
                        if hasattr(pid_y, '_last_error') and pid_y._last_error is not None:
                            if (error_y_m > 0 and pid_y._last_error < 0) or \
                               (error_y_m < 0 and pid_y._last_error > 0):
                                pid_y._integral = 0
                        pid_y._last_error = error_y_m
                        
                        cmd_y_world = -cmd_y_raw * UD_GAIN

                        if abs(error_y_m) < UD_BRAKE_ZONE:
                            cmd_y_world *= UD_BRAKE_SPEED
                        
                        # FB
                        if fb_freeze_counter > 0:
                            cmd_z_world = 0
                            fb_freeze_counter -= 1
                            fb_ramp_counter = 0
                        else:
                            if fb_ramp_counter < FB_RAMP_STEPS:
                                ramp = (fb_ramp_counter + 1) / FB_RAMP_STEPS
                                cmd_z_world = -pid_z(error_z_norm) * ramp
                                fb_ramp_counter += 1
                            else:
                                cmd_z_world = -pid_z(error_z_norm)
                            
                            if abs(error_z_norm) > 0.05 and abs(cmd_z_world) < 0.03:
                                cmd_z_world = 0.03 if error_z_norm > 0 else -0.03
                            
                            if cmd_z_world < FB_BACK_CLIP:
                                cmd_z_world = FB_BACK_CLIP

                        # ===== Преобразование в систему дрона =====
                        yaw_rad = math.radians(yaw - yaw_offset)
                        pitch_rad = math.radians(pitch)
                        roll_rad = math.radians(roll)
                        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
                        cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
                        cr, sr = math.cos(roll_rad), math.sin(roll_rad)

                        cmd_x_drone = (cmd_x_world * cy * cp +
                                       cmd_y_world * (cy * sp * sr - sy * cr) +
                                       cmd_z_world * (cy * sp * cr + sy * sr))
                        cmd_y_drone = cmd_y_world
                        cmd_z_drone = (-cmd_x_world * sp +
                                       cmd_y_world * cp * sr +
                                       cmd_z_world * cp * cr)

                        cmd_x_drone, cmd_y_drone, cmd_z_drone = normalize_commands(
                            cmd_x_drone, cmd_y_drone, cmd_z_drone, max_total=1.0
                        )

                        drone.send_rc_control(
                            int(round(cmd_x_drone * 100)),   # LR
                            int(round(cmd_z_drone * 100)),   # FB
                            int(round(cmd_y_drone * 100)),   # UD
                            0
                        )

                        print(f"  UD: err={error_y_m:+.3f}м cmd={cmd_y_drone:+.2f}")
                        print(f"ID{marker_id} xy=({error_x_m:+.3f},{error_y_m:+.3f})м "
                              f"diag={diag_px:.0f}→{DESIRED_DIAG_PX} err_z={error_z_norm:+.3f} "
                              f"cmd=({cmd_x_drone:+.2f},{cmd_z_drone:+.2f},{cmd_y_drone:+.2f}) "
                              f"P={pitch:.0f} R={roll:.0f}")
                    else:
                        lost_frames += 1
                        if lost_frames > MAX_LOST_FRAMES:
                            drone.send_rc_control(0, 0, 0, 0)
                            self.reset_all_pids()

                time.sleep(0.03)

        except Exception:
            traceback.print_exc()
        finally:
            self.stream_server_running.value = False
            drone.send_rc_control(0, 0, 0, 0)
            drone.land()
            drone.streamoff()
            drone.end()
            print("Finished.")


def main():
    vr = Video()
    proc = multiprocessing.Process(target=vr.video_get)
    proc.start()
    try:
        while proc.is_alive():
            time.sleep(1)
    except KeyboardInterrupt:
        vr.run.value = False
        vr.stream_server_running.value = False
        time.sleep(3)
        if proc.is_alive():
            proc.terminate()


if __name__ == '__main__':
    main()
