import os
import argparse
from datetime import datetime
from pathlib import Path
import subprocess

def main():
    parser = argparse.ArgumentParser(description='Record video from Raspberry Pi camera using rpicam-vid and convert to MP4.')
    parser.add_argument('-d', '--duration', type=int, default=10,
                        help='Duration of video in seconds (default: 10)')
    parser.add_argument('-w', '--width', type=int, default=1280,
                        help='Video width (default: 1280)')
    parser.add_argument('-t', '--height', type=int, default=720,
                        help='Video height (default: 720)')
    parser.add_argument('--preview', action='store_true',
                        help='Enable preview window (default: off)')
    args = parser.parse_args()

    # Prepare file paths
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_filename = f"FlightVideo_{timestamp}"
    desktop = Path.home() / 'Desktop'
    h264_path = desktop / f"{base_filename}.h264"
    mp4_path = desktop / f"{base_filename}.mp4"

    # Record video using rpicam-vid
    cmd = [
        "rpicam-vid",
        "-t", str(args.duration * 1000),
        "-o", str(h264_path),
        "--width", str(args.width),
        "--height", str(args.height),
    ]
    if not args.preview:
        cmd.append("--nopreview")

    print(f"Recording {args.duration}s video to: {h264_path}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print("Error: Failed to record video.")
        return

    # Convert to MP4 using ffmpeg
    print(f"Converting to MP4: {mp4_path}")
    convert_cmd = [
        "ffmpeg",
        "-y",  # Overwrite without asking
        "-framerate", "30",
        "-i", str(h264_path),
        "-c", "copy",
        str(mp4_path)
    ]
    result = subprocess.run(convert_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if result.returncode != 0:
        print("Error: Failed to convert to MP4.")
        return

    print(f"✅ MP4 video saved to: {mp4_path}")

if __name__ == "__main__":
    main()
