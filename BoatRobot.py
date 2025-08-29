#!/usr/bin/env python3
import math, time, csv, sys, os, collections, subprocess, threading, queue, shutil, argparse
from datetime import datetime

DRY_RUN = False  # True → no GPIO/serial/camera, prints actions

# -------------------- USER SETTINGS --------------------
WAYPOINTS = [
    # (lat, lon) x12
    (41.864662, -87.953105),
    (41.864665, -87.952775),
    (41.864716, -87.952527),
    (41.864882, -87.952463),
    (41.865017, -87.95251),
    (41.865161, -87.952597),
    (41.865251, -87.952823),
    (41.865196, -87.953072),
    (41.865066, -87.95305),
    (41.864938, -87.953063),
    (41.864826, -87.953107),
]
WAYPOINT_RADIUS_M = 6.0
SLOWDOWN_RADIUS_M = 20.0
BASE_THR = 0.55

# PID for heading control
PID_KP = 0.018
PID_KI = 0.000
PID_KD = 0.08
PID_I_CLAMP = 0.6

# Motor behavior
MIN_PWM = 0.20    # overcome deadband
MAX_PWM = 0.95
REVERSE_ALLOWED = False  # set True if you want reverse maneuvers
PWM_HZ = 1000     # DRV8833 is quiet > ~20 kHz, but software PWM is happier ~1 kHz

# DRV8833 pins (BCM numbering): AIN1/AIN2 for LEFT, BIN1/BIN2 for RIGHT
LEFT_IN1  = 32
LEFT_IN2  = 36
RIGHT_IN1 = 38
RIGHT_IN2 = 40

# GPS: now on /dev/ttyS0 per your hardware
GPS_PORT = "/dev/ttyS0"
GPS_BAUD = 9600
GPS_TIMEOUT_S = 0.8

# IMU (MPU6050 on I2C)
IMU_ADDR = 0x68
IMU_RATE_HZ = 25  # log rate

# Storage
ROOT_PATH = os.path.expanduser("~/BoatFlight")
VIDEO_PATH = os.path.join(ROOT_PATH, "video")
LOG_PATH   = os.path.join(ROOT_PATH, "logs")

# Video defaults (CLI-overridable)
DEFAULT_SEGMENT_SEC = 120
DEFAULT_RES = "1920x1080"
DEFAULT_FPS = 30
# -------------------- END USER SETTINGS ----------------

# ---------- Optional GPIO/Serial/I2C imports ----------
GPIO = None
serial = None
pynmea2 = None
smbus2 = None

if not DRY_RUN:
    try:
        import RPi.GPIO as GPIO
    except Exception as e:
        print(f"[WARN] RPi.GPIO not available ({e}); forcing DRY_RUN")
        DRY_RUN = True

try:
    import serial, pynmea2
except Exception as e:
    print(f"[WARN] serial/pynmea2 not available ({e}); GPS will be mocked in DRY_RUN.")

try:
    from smbus2 import SMBus
    smbus2 = SMBus
except Exception as e:
    print(f"[WARN] smbus2 not available ({e}); IMU disabled if not installed).")

# ---------- Geo helpers ----------
def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1 = math.radians(lat1); p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dlmb/2)**2
    return 2*R*math.asin(math.sqrt(a))

def bearing_deg(lat1, lon1, lat2, lon2):
    p1 = math.radians(lat1); p2 = math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    y = math.sin(dlmb) * math.cos(p2)
    x = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dlmb)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

def ang_diff_deg(target, current):
    return (target - current + 180.0) % 360.0 - 180.0

def clamp(v, lo, hi): return max(lo, min(hi, v))

# ---------- PID ----------
class PID:
    def __init__(self, kp, ki, kd, i_clamp):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i, self.prev_e, self.prev_t = 0.0, None, None
        self.i_clamp = i_clamp
    def reset(self): self.i = 0.0; self.prev_e = None; self.prev_t = None
    def step(self, e, t):
        if self.prev_t is None:
            self.prev_t, self.prev_e = t, e
            d = 0.0; dt = 0.0
        else:
            dt = max(1e-3, t - self.prev_t)
            d = (e - self.prev_e) / dt
            self.prev_t, self.prev_e = t, e
        self.i = clamp(self.i + e*dt, -self.i_clamp, self.i_clamp)
        return self.kp*e + self.ki*self.i + self.kd*d

# ---------- Motor driver (DRV8833) ----------
class MotorDriverDRV8833:
    def __init__(self):
        self.enabled = False
        if DRY_RUN or GPIO is None:
            return
        GPIO.setmode(GPIO.BCM)
        for pin in [LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2]:
            GPIO.setup(pin, GPIO.OUT)
        # Software PWM on 4 pins
        self.pwm = {
            "L1": GPIO.PWM(LEFT_IN1, PWM_HZ),
            "L2": GPIO.PWM(LEFT_IN2, PWM_HZ),
            "R1": GPIO.PWM(RIGHT_IN1, PWM_HZ),
            "R2": GPIO.PWM(RIGHT_IN2, PWM_HZ),
        }
        for p in self.pwm.values(): p.start(0)
        self.enabled = True

    def _duty(self, x01):
        # map 0..1 to (% duty), enforcing deadband and cap
        if x01 <= 0: return 0.0
        return 100.0 * clamp(MIN_PWM + x01*(MAX_PWM - MIN_PWM), 0.0, 1.0)

    def set_signed(self, left_11, right_11):
        """
        left_11/right_11 in [-1..1], sign = direction.
        If REVERSE_ALLOWED=False, negatives become 0 (coast).
        """
        l = clamp(left_11, -1.0, 1.0)
        r = clamp(right_11, -1.0, 1.0)
        if not REVERSE_ALLOWED:
            l = max(0.0, l)
            r = max(0.0, r)

        if DRY_RUN or not self.enabled:
            print(f"[MOTORS] L={l:+.2f} R={r:+.2f}")
            return

        # LEFT: forward => L1 duty, L2 0 ; reverse => L1 0, L2 duty
        if l >= 0:
            self.pwm["L1"].ChangeDutyCycle(self._duty(l))
            self.pwm["L2"].ChangeDutyCycle(0)
        else:
            self.pwm["L1"].ChangeDutyCycle(0)
            self.pwm["L2"].ChangeDutyCycle(self._duty(-l))

        # RIGHT: forward => R1 duty, R2 0 ; reverse => R1 0, R2 duty
        if r >= 0:
            self.pwm["R1"].ChangeDutyCycle(self._duty(r))
            self.pwm["R2"].ChangeDutyCycle(0)
        else:
            self.pwm["R1"].ChangeDutyCycle(0)
            self.pwm["R2"].ChangeDutyCycle(self._duty(-r))

    def stop(self):
        if DRY_RUN or not self.enabled:
            print("[MOTORS] STOP")
            return
        for p in self.pwm.values(): p.ChangeDutyCycle(0)
        GPIO.cleanup()

# ---------- GPS ----------
class GPSReader:
    def __init__(self, port=GPS_PORT, baud=GPS_BAUD, timeout=GPS_TIMEOUT_S):
        self.port, self.baud, self.timeout = port, baud, timeout
        self.ser = None
        self.last_fix = None
        if DRY_RUN: return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        except Exception as e:
            print(f"[ERROR] Opening GPS serial ({self.port}): {e}")
            raise

    def read_fix(self):
        if DRY_RUN or self.ser is None:
            lat, lon = WAYPOINTS[0]
            t = time.time()
            self.last_fix = (lat, lon, None, None, t)
            return self.last_fix

        deadline = time.time() + self.timeout
        lat = lon = cog = sog = None
        t = time.time()
        while time.time() < deadline:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line or not line.startswith("$"): continue
                msg = pynmea2.parse(line)
                if msg.sentence_type == "RMC" and msg.status == "A":
                    lat = msg.latitude; lon = msg.longitude
                    cog = float(msg.true_course) if msg.true_course else None
                    sog = float(msg.spd_over_grnd)*0.514444 if msg.spd_over_grnd else None
                elif msg.sentence_type == "GGA":
                    lat = msg.latitude or lat
                    lon = msg.longitude or lon
            except Exception:
                continue
            if lat is not None and lon is not None:
                self.last_fix = (lat, lon, cog, sog, t)
                return self.last_fix
        return self.last_fix

# ---------- IMU (MPU6050) ----------
class MPU6050:
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H  = 0x43
    ACCEL_SCALE = 16384.0  # LSB/g @ ±2g
    GYRO_SCALE  = 131.0    # LSB/(°/s) @ ±250 dps

    def __init__(self, bus_num=1, addr=IMU_ADDR):
        self.addr = addr
        self.bus = None
        self.enabled = False
        if DRY_RUN or smbus2 is None:
            return
        try:
            self.bus = smbus2(bus_num)
            self.bus.write_byte_data(self.addr, self.PWR_MGMT_1, 0x00)  # wake
            time.sleep(0.05)
            self.enabled = True
        except Exception as e:
            print(f"[WARN] MPU6050 init failed: {e}")

    def _read_word(self, reg):
        hi = self.bus.read_byte_data(self.addr, reg)
        lo = self.bus.read_byte_data(self.addr, reg+1)
        val = (hi << 8) | lo
        if val >= 0x8000: val = -((65535 - val) + 1)
        return val

    def read(self):
        if DRY_RUN or not self.enabled:
            return {"t": time.time(), "ax": 0.0, "ay": 0.0, "az": 1.0, "gx": 0.0, "gy": 0.0, "gz": 0.0}
        try:
            ax = self._read_word(self.ACCEL_XOUT_H) / self.ACCEL_SCALE
            ay = self._read_word(self.ACCEL_XOUT_H+2) / self.ACCEL_SCALE
            az = self._read_word(self.ACCEL_XOUT_H+4) / self.ACCEL_SCALE
            gx = self._read_word(self.GYRO_XOUT_H)  / self.GYRO_SCALE
            gy = self._read_word(self.GYRO_XOUT_H+2)/ self.GYRO_SCALE
            gz = self._read_word(self.GYRO_XOUT_H+4)/ self.GYRO_SCALE
            return {"t": time.time(), "ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz}
        except Exception as e:
            print(f"[WARN] IMU read failed: {e}")
            return None

# ---------- Video Recorder ----------
class VideoRecorder:
    def __init__(self, out_dir, segment_sec=DEFAULT_SEGMENT_SEC, res=DEFAULT_RES, fps=DEFAULT_FPS, bitrate="10M"):
        self.out_dir = out_dir
        self.segment_sec = segment_sec
        self.res = res
        self.fps = fps
        self.bitrate = bitrate
        os.makedirs(self.out_dir, exist_ok=True)
        self.stop_evt = threading.Event()
        self.thread = None
        self.conv_q = queue.Queue()
        self.conv_thread = threading.Thread(target=self._converter_worker, daemon=True)

    def start(self, loop=True):
        self.stop_evt.clear()
        self.thread = threading.Thread(target=self._run, args=(loop,), daemon=True)
        self.thread.start()
        self.conv_thread.start()

    def stop(self):
        self.stop_evt.set()
        if self.thread: self.thread.join()
        self.conv_q.put(None)  # sentinel
        self.conv_thread.join()

    def _run(self, loop):
        while not self.stop_evt.is_set():
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            h264 = os.path.join(self.out_dir, f"FlightVideo_{ts}.h264")
            cmd = [
                "rpicam-vid",
                "-t", str(int(self.segment_sec*1000)),
                "-o", h264,
                "--inline",
                "--framerate", str(self.fps),
                "--width", self.res.split("x")[0],
                "--height", self.res.split("x")[1],
                "--bitrate", self.bitrate,
            ]
            print(f"[VIDEO] Recording segment → {os.path.basename(h264)}")
            if DRY_RUN:
                time.sleep(self.segment_sec)
            else:
                try:
                    subprocess.run(cmd, check=False)
                except FileNotFoundError:
                    print("[ERROR] rpicam-vid not found. Install it or set DRY_RUN=True.")
                    break

            self.conv_q.put(h264)
            if not loop: break

    def _converter_worker(self):
        mp4box = shutil.which("MP4Box")
        ffmpeg = shutil.which("ffmpeg")
        while True:
            h264 = self.conv_q.get()
            if h264 is None:
                return
            mp4 = h264.replace(".h264", ".mp4")
            if DRY_RUN:
                print(f"[VIDEO] (DRY) Convert {os.path.basename(h264)} → {os.path.basename(mp4)}")
                continue
            try:
                if mp4box:
                    subprocess.run([mp4box, "-add", h264, mp4], check=True)
                    os.remove(h264)
                elif ffmpeg:
                    # Stream-copy if possible
                    subprocess.run(["ffmpeg", "-y", "-i", h264, "-c", "copy", mp4], check=True)
                    os.remove(h264)
                else:
                    print("[VIDEO] No MP4Box/ffmpeg; keeping .h264.")
            except Exception as e:
                print(f"[VIDEO] Conversion failed: {e}")

# ---------- Navigation helpers ----------
def mix_forward_steer(forward_thr_01, steer_pm1):
    """
    Returns left/right in [-1..1] (signed speed). Forward throttle 0..1, steer -1..1.
    """
    left = clamp(forward_thr_01 - steer_pm1, -1.0, 1.0)
    right = clamp(forward_thr_01 + steer_pm1, -1.0, 1.0)
    return left, right

# ---------- Main loop ----------
def run(segment_sec, res, fps, bitrate, loop_segments):
    os.makedirs(LOG_PATH, exist_ok=True)
    os.makedirs(VIDEO_PATH, exist_ok=True)
    nav_log = os.path.join(LOG_PATH, f"nav_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    imu_log = os.path.join(LOG_PATH, f"imu_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

    nav_f = open(nav_log, "w", newline="")
    nav_w = csv.writer(nav_f)
    nav_w.writerow([
        "t_iso","wp_idx","lat","lon","cog_deg","sog_mps",
        "wp_lat","wp_lon","dist_m","brg_deg","hdg_deg","hdg_err_deg",
        "thr_cmd","steer_cmd","cmd_left","cmd_right"
    ])

    imu_f = open(imu_log, "w", newline="")
    imu_w = csv.writer(imu_f)
    imu_w.writerow(["t_iso","ax_g","ay_g","az_g","gx_dps","gy_dps","gz_dps"])

    gps = GPSReader()
    motors = MotorDriverDRV8833()
    pid = PID(PID_KP, PID_KI, PID_KD, PID_I_CLAMP)
    imu = MPU6050()
    video = VideoRecorder(VIDEO_PATH, segment_sec=segment_sec, res=res, fps=fps, bitrate=bitrate)

    imu_dt = 1.0 / IMU_RATE_HZ
    next_imu_t = time.time()

    wp_idx = 0
    prev_positions = collections.deque(maxlen=6)
    last_progress_time = time.time()
    last_dist = None

    # Start video
    video.start(loop=loop_segments)

    try:
        while wp_idx < len(WAYPOINTS):
            # --- GPS/Nav update ---
            lat = lon = cog_deg = sog_mps = None
            fix = gps.read_fix()
            if fix:
                lat, lon, cog_deg, sog_mps, t = fix
            else:
                t = time.time()

            wp_lat, wp_lon = WAYPOINTS[wp_idx]
            dist_m = haversine_m(lat, lon, wp_lat, wp_lon) if (lat and lon) else float("nan")
            brg_deg = bearing_deg(lat, lon, wp_lat, wp_lon) if (lat and lon) else float("nan")

            # Determine current heading
            hdg_deg = None
            if cog_deg is not None and (sog_mps or 0) > 0.2:
                hdg_deg = cog_deg % 360.0
            else:
                prev_positions.append((t, lat, lon))
                if len(prev_positions) >= 2:
                    _, lat0, lon0 = prev_positions[0]
                    _, lat1, lon1 = prev_positions[-1]
                    if (lat0 is not None and lon0 is not None and
                        lat1 is not None and lon1 is not None and
                        haversine_m(lat0, lon0, lat1, lon1) > 0.6):
                        hdg_deg = bearing_deg(lat0, lon0, lat1, lon1)

            # Control
            if hdg_deg is None or math.isnan(dist_m):
                thr_cmd = 0.35
                steer_cmd = 0.0
            else:
                err = ang_diff_deg(brg_deg, hdg_deg)
                steer_raw = pid.step(err, t)
                steer_cmd = clamp(steer_raw, -1.0, 1.0)

                thr_cmd = BASE_THR
                if dist_m < SLOWDOWN_RADIUS_M:
                    min_thr = 0.28
                    alpha = clamp(dist_m / SLOWDOWN_RADIUS_M, 0.0, 1.0)
                    thr_cmd = min_thr + alpha*(BASE_THR - min_thr)

            # Waypoint arrival
            if dist_m <= WAYPOINT_RADIUS_M:
                motors.set_signed(0, 0)
                pid.reset()
                print(f"[WP] Reached waypoint {wp_idx+1}/{len(WAYPOINTS)} at {dist_m:.1f} m")
                time.sleep(1.0)
                wp_idx += 1
                last_progress_time = time.time()
                last_dist = None
                continue

            # Watchdog for progress (optional)
            if last_dist is None or (not math.isnan(dist_m) and dist_m < last_dist - 1.0):
                last_progress_time = time.time()
                last_dist = dist_m
            elif time.time() - last_progress_time > 90:
                print("[WARN] No progress toward waypoint for 90s")

            # Mix + send to motors (signed)
            left_11, right_11 = mix_forward_steer(thr_cmd, steer_cmd)
            motors.set_signed(left_11, right_11)

            # Log nav
            nav_w.writerow([
                datetime.utcnow().isoformat(),
                wp_idx,
                f"{(lat if lat is not None else float('nan')):.7f}",
                f"{(lon if lon is not None else float('nan')):.7f}",
                f"{(cog_deg if cog_deg is not None else float('nan')):.2f}",
                f"{(sog_mps if sog_mps is not None else float('nan')):.2f}",
                f"{wp_lat:.7f}", f"{wp_lon:.7f}",
                f"{(dist_m if not math.isnan(dist_m) else float('nan')):.2f}",
                f"{(brg_deg if brg_deg is not None else float('nan')):.2f}",
                f"{(hdg_deg if hdg_deg is not None else float('nan')):.2f}",
                f"{(ang_diff_deg(brg_deg, hdg_deg) if hdg_deg is not None and brg_deg is not None else float('nan')):.2f}",
                f"{thr_cmd:.2f}", f"{steer_cmd:.2f}",
                f"{left_11:.2f}", f"{right_11:.2f}",
            ])

            # --- IMU logging at fixed rate ---
            now = time.time()
            if now >= next_imu_t:
                next_imu_t += imu_dt
                imu_data = imu.read()
                if imu_data is not None:
                    imu_w.writerow([
                        datetime.utcnow().isoformat(),
                        f"{imu_data['ax']:.5f}", f"{imu_data['ay']:.5f}", f"{imu_data['az']:.5f}",
                        f"{imu_data['gx']:.5f}", f"{imu_data['gy']:.5f}", f"{imu_data['gz']:.5f}",
                    ])

            time.sleep(0.2)

        print("[DONE] All waypoints reached.")
        motors.stop()
    except KeyboardInterrupt:
        print("\n[ABORT] Ctrl-C received; stopping motors.")
        motors.stop()
    except Exception as e:
        print(f"[ERROR] {e}")
        motors.stop()
    finally:
        video.stop()
        nav_f.close()
        imu_f.close()

def parse_args():
    ap = argparse.ArgumentParser(description="Boat nav + IMU + video (DRV8833 + /dev/ttyS0 GPS)")
    ap.add_argument("--segment", type=int, default=DEFAULT_SEGMENT_SEC, help="video segment length (s)")
    ap.add_argument("--res", type=str, default=DEFAULT_RES, help="video resolution WxH")
    ap.add_argument("--fps", type=int, default=DEFAULT_FPS, help="video FPS")
    ap.add_argument("--bitrate", type=str, default="10M", help="video bitrate (e.g. 10M)")
    ap.add_argument("--one-shot", action="store_true", help="record a single segment instead of looping")
    return ap.parse_args()

if __name__ == "__main__":
    args = parse_args()
    run(
        segment_sec=args.segment,
        res=args.res,
        fps=args.fps,
        bitrate=args.bitrate,
        loop_segments=not args.one_shot
    )
