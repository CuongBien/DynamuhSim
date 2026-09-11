#!/usr/bin/env python3

import csv
import argparse
import re
import signal
import subprocess
import sys
import time

TOPIC = "/dynamic_obstacle/pose"
parser = argparse.ArgumentParser()
parser.add_argument("output_positional", nargs="?")
parser.add_argument("--output", dest="output_option")
args = parser.parse_args()
OUTPUT = args.output_option or args.output_positional or "obstacle_ground_truth.csv"

running = True


def stop_handler(signum, frame):
    global running
    running = False


signal.signal(signal.SIGINT, stop_handler)
signal.signal(signal.SIGTERM, stop_handler)

process = subprocess.Popen(
    ["gz", "topic", "-e", "-t", TOPIC],
    stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL,
    text=True,
    bufsize=1,
)

csv_file = open(OUTPUT, "w", newline="")
writer = csv.writer(csv_file)
writer.writerow(["sim_time", "obstacle_x", "obstacle_y"])

sec = None
nsec = None
obstacle_x = None
obstacle_y = 0.0
inside_position = False

try:
    for raw in process.stdout:
        if not running:
            break

        line = raw.strip()

        if line.startswith("sec:"):
            sec = int(line.split(":", 1)[1].strip())
            continue

        if line.startswith("nsec:"):
            nsec = int(line.split(":", 1)[1].strip())
            continue

        if line == "position {":
            obstacle_x = None
            obstacle_y = 0.0
            inside_position = True
            continue

        if inside_position:
            if line.startswith("x:"):
                obstacle_x = float(line.split(":", 1)[1].strip())
                continue

            if line.startswith("y:"):
                obstacle_y = float(line.split(":", 1)[1].strip())
                continue

            if line == "}":
                inside_position = False
                continue

        if line == "}" and not inside_position:
            if obstacle_x is not None:
                if sec is not None and nsec is not None:
                    sim_time = sec + nsec * 1e-9
                else:
                    sim_time = time.time()

                writer.writerow([
                    f"{sim_time:.9f}",
                    f"{obstacle_x:.9f}",
                    f"{obstacle_y:.9f}",
                ])
                csv_file.flush()

                obstacle_x = None
                obstacle_y = 0.0
                sec = None
                nsec = None

finally:
    process.terminate()
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()

    csv_file.close()

print(f"Saved: {OUTPUT}")
