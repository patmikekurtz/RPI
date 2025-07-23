import os
import csv
import time
import threading
import subprocess
from datetime import datetime
from pathlib import Path
import gps
import argparse

# ===== Default Config =====
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
DEVICE = "/dev/ttyUSB0"  # Adjust if needed
RECORD_PREVIEW = False
# ==========================

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
                    mode = getattr(report, 'mode', 0)  # 1 = no fix, 2 = 2D, 3 = 3D

                    writer.writerow([datetime.now().isoformat(), lat, lon, alt, mode])

                    if lat and lon:
                        print(f"🛰️  Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt}, Fix Mode: {mode}")
                    else:
                        print(f"🔍 Waiting for fix... (Fix Mode: {mode})")
            except StopIteration:
                print("GPSD has terminated")
                break
            except Exception as e:
                print(f"GPS error: {e}")
                continue

            time.sleep(1)

def record_video(duration_sec, h264_path):
    cmd = [
        "rpicam-vid",
        "-t", str(duration_sec * 1000),
        "-o", str(h264_path),
        "--width", str(VIDEO_WIDTH),
        "--height", str(VIDEO_HEIGHT)
    ]
    if not RECORD_PREVIEW:
        cmd.append("--nopreview")

    print(f"🎥 Recording video to: {h264_path}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print("❌ Video recording failed.")
    else:
        print("✅ Video recording complete.")

def convert_video(h264_path, mp4_path):
    print("🎬 Converting video to MP4...")
    cmd = [
        "ffmpeg",
        "-y",
        "-framerate", "30",
        "-i", str(h264_path),
        "-c", "copy",
        str(mp4_path)
    ]
    result = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if result.returncode == 0:
        print(f"✅ MP4 saved to: {mp4_path}")
    else:
        print("❌ Video conversion failed.")

def main():
    parser = argparse.ArgumentParser(description="Record flight video with optional GPS logging.")
    parser.add_argument('--duration', type=int, default=15, help='Video duration in seconds (default: 15)')
    parser.add_argument('--gps', action='store_true', help='Enable GPS logging')
    args = parser.parse_args()

    # Setup log folder
    log_dir = Path.home() / "RPI" / "FlightLogs"
    log_dir.mkdir(parents=True, exist_ok=True)

    # Generate timestamped filenames
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    base_name = f"FlightVideo_{timestamp}"
    h264_path = log_dir / f"{base_name}.h264"
    mp4_path = log_dir / f"{base_name}.mp4"
    gps_log_path = log_dir / f"GPSLog_{timestamp}.csv"

    print("🚀 Starting flight recorder...")

    # Start GPS thread if enabled
    gps_thread = None
    if args.gps:
        gps_thread = threading.Thread(target=record_gps_data, args=(args.duration, gps_log_path))
        gps_thread.start()

    # Record video
    record_video(args.duration, h264_path)

    # Wait for GPS thread
    if gps_thread:
        gps_thread.join()

    # Convert to MP4
    convert_video(h264_path, mp4_path)

    print("✅ Flight recording complete.")

if __name__ == "__main__":
    main()
