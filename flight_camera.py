import os
import csv
import time
import threading
import subprocess
from datetime import datetime
from pathlib import Path
import gps
import argparse
from smbus2 import SMBus

# Constants
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
MPU_ADDR = 0x68
I2C_BUS = 1

def setup_mpu6050():
    with SMBus(I2C_BUS) as bus:
        # Wake up MPU6050
        bus.write_byte_data(MPU_ADDR, 0x6B, 0)
    print("✅ MPU6050 initialized")

def read_mpu6050():
    with SMBus(I2C_BUS) as bus:
        def read_word(reg):
            high = bus.read_byte_data(MPU_ADDR, reg)
            low = bus.read_byte_data(MPU_ADDR, reg + 1)
            val = (high << 8) + low
            if val >= 0x8000:
                val = -((65535 - val) + 1)
            return val

        accel_x = read_word(0x3B)
        accel_y = read_word(0x3D)
        accel_z = read_word(0x3F)
        gyro_x = read_word(0x43)
        gyro_y = read_word(0x45)
        gyro_z = read_word(0x47)

    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

def log_imu_data(duration_sec, imu_log_path):
    print(f"🧭 Logging IMU to: {imu_log_path}")
    setup_mpu6050()

    with open(imu_log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z'])

        start_time = time.time()
        while time.time() - start_time < duration_sec:
            try:
                a_x, a_y, a_z, g_x, g_y, g_z = read_mpu6050()
                writer.writerow([datetime.now().isoformat(), a_x, a_y, a_z, g_x, g_y, g_z])
            except Exception as e:
                print(f"IMU error: {e}")
            time.sleep(0.02)  # ~50 Hz

def record_gps_data(duration_sec, gps_log_path):
    print(f"📍 Logging GPS to: {gps_log_path}")
    session = gps.gps(mode=gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

    with open(gps_log_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude', 'fix'])

        start_time = time.time()
        while time.time() - start_time < duration_sec:
            try:
                report = session.next()
                if report['class'] == 'TPV':
                    lat = getattr(report, 'lat', None)
                    lon = getattr(report, 'lon', None)
                    alt = getattr(report, 'alt', None)
                    mode = getattr(report, 'mode', 0)
                    writer.writerow([datetime.now().isoformat(), lat, lon, alt, mode])
            except:
                continue
            time.sleep(1)

def record_video(duration_sec, h264_path):
    print(f"🎥 Recording video to: {h264_path}")
    cmd = [
        "rpicam-vid",
        "-t", str(duration_sec * 1000),
        "-o", str(h264_path),
        "--width", str(VIDEO_WIDTH),
        "--height", str(VIDEO_HEIGHT),
        "--nopreview"
    ]
    subprocess.run(cmd)

def convert_video(h264_path, mp4_path):
    print("🎬 Converting video to MP4...")
    cmd = [
        "ffmpeg", "-y",
        "-framerate", "30",
        "-i", str(h264_path),
        "-c", "copy",
        str(mp4_path)
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main():
    parser = argparse.ArgumentParser(description="Record flight video with optional GPS logging.")
    parser.add_argument('--duration', type=int, default=15, help='Duration of video (seconds)')
    parser.add_argument('--gps', action='store_true', help='Enable GPS logging')
    args = parser.parse_args()

    # Folder setup
    log_dir = Path.home() / "RPI" / "FlightLogs"
    log_dir.mkdir(parents=True, exist_ok=True)

    # File names
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    base = f"FlightVideo_{timestamp}"
    h264_path = log_dir / f"{base}.h264"
    mp4_path = log_dir / f"{base}.mp4"
    gps_log_path = log_dir / f"GPSLog_{timestamp}.csv"
    imu_log_path = log_dir / f"IMULog_{timestamp}.csv"

    print("🚀 Starting flight recorder...")

    threads = []

    # Start IMU logging (always enabled)
    imu_thread = threading.Thread(target=log_imu_data, args=(args.duration, imu_log_path))
    imu_thread.start()
    threads.append(imu_thread)

    # Optional GPS logging
    if args.gps:
        gps_thread = threading.Thread(target=record_gps_data, args=(args.duration, gps_log_path))
        gps_thread.start()
        threads.append(gps_thread)

    # Record video
    record_video(args.duration, h264_path)

    # Wait for other threads
    for t in threads:
        t.join()

    # Convert to MP4
    convert_video(h264_path, mp4_path)

    print("✅ Flight recording complete.")

if __name__ == "__main__":
    main()
