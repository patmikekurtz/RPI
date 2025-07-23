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
DEVICE = "/dev/ttyACM0"  # Adjust if needed
RECORD_PREVIEW = False
# ==========================

def record_gps_data(duration_sec, gps_log_path):
    print(f"📍 Logging GPS to: {gps_log_path}")
    session = gps.gps(mode=gps.WATCH_ENABLE)
    with open(gps_log_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude'])

        start_time = time.time()
        while time.time() - start_time < duration_sec:
            try:
                report = session.next()
                if report['class'] == 'TPV':
                    lat = getattr(report, 'lat', None)
                    lon = getattr(report, 'lon', None)
                    alt = getattr(report, 'alt', None)
                    if lat and lon:
                        writer.writerow([datetime.now().isoformat(), lat, lon, alt])
                        print(f"Lat: {lat}, Lon: {lon}, Alt: {alt}")
            except Exception:
                pass
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
        print(f"MP4 saved to: {mp4_path}")
    else:
        print("❌ Video conversion failed.")

def main():
    parser = argparse.ArgumentParser(description="Record flight video with optional GPS logging.")
    parser.add_argument('--duration', type=int, default=15, help='Video duration in seconds (default: 15)')
    parser.add_argument('--gps', action='store_true', help='Enable GPS logging')
    args = parser.parse_args()

    # Setup file paths
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    desktop = Path.home() / "Desktop"
    video_base = f"FlightVideo_{timestamp}"
    h264_path = desktop / f"{video_base}.h264"
    mp4_path = desktop / f"{video_base}.mp4"
    gps_log_path = desktop / f"GPSLog_{timestamp}.csv"

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
