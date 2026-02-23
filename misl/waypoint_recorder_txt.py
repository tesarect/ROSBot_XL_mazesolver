#!/usr/bin/env python3

import subprocess
import time
import argparse

OUTPUT_FILE = "waypoints.txt"

TOPICS = [
    ("/odometry/filtered", "pose.pose"),
    ("/rosbot_xl_base_controller/odom", "pose.pose"),
]

def topic_available(topic):
    result = subprocess.run(
        ["ros2", "topic", "list"],
        capture_output=True,
        text=True
    )
    return topic in result.stdout

def get_topic_output(topic, field):
    result = subprocess.run(
        ["ros2", "topic", "echo", topic, "--field", field, "--once"],
        capture_output=True,
        text=True
    )
    return result.stdout.strip()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-waypoint", required=True)
    args = parser.parse_args()

    label = f"waypoint {args.waypoint}"

    with open(OUTPUT_FILE, "a") as f:
        f.write(f"\n--- {label} ---\n")
        f.write(f"Timestamp: {time.time()}\n")

        for topic, field in TOPICS:
            for _ in range(5):
                if topic_available(topic):
                    break
                time.sleep(0.5)

            output = get_topic_output(topic, field)
            f.write(f"\nTopic: {topic}\n")
            f.write(output + "\n")

if __name__ == "__main__":
    main()
