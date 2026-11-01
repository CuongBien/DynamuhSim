#!/usr/bin/env python3
import argparse
import ast
import csv
import json
import signal
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--output", required=True)
parser.add_argument("--human-id", type=int, default=1)
args = parser.parse_args()

topic = "/rvo2_human/ground_truth"
process = subprocess.Popen(
    ["gz", "topic", "-e", "-t", topic],
    stdout=subprocess.PIPE,
    stderr=None,
    text=True,
    bufsize=1,
)

def stop(signum, frame):
    if process.poll() is None:
        process.terminate()

signal.signal(signal.SIGTERM, stop)

count = 0

try:
    with open(args.output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "sim_time", "obstacle_x", "obstacle_y"
        ])
        f.flush()

        for line in process.stdout:
            line = line.strip()

            if not line.startswith("data:"):
                continue

            try:
                # gz.msgs.StringMsg -> escaped JSON string
                encoded = line.split(":", 1)[1].strip()
                payload = json.loads(ast.literal_eval(encoded))

                humans = payload.get("humans", [])
                human = next(
                    h for h in humans
                    if int(h["id"]) == args.human_id
                )

                writer.writerow([
                    float(payload["timestamp"]),
                    float(human["x"]),
                    float(human["y"]),
                ])
                f.flush()
                count += 1

                if count == 1:
                    print("RVO2 ground truth received", flush=True)

            except (
                ValueError, SyntaxError, TypeError,
                KeyError, StopIteration
            ):
                continue

except KeyboardInterrupt:
    pass

finally:
    if process.poll() is None:
        process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()

print(f"Recorded {count} human poses", flush=True)
