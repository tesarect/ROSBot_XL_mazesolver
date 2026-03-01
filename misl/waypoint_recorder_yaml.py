#!/usr/bin/env python3

import subprocess
import time
import argparse
import yaml
import os

OUTPUT_FILE = "waypoints.yaml"

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
    return result.stdout


def parse_pose(raw_text):
    pose = {
        "position": {},
        "orientation": {}
    }

    lines = raw_text.splitlines()
    current_section = None

    for line in lines:
        line = line.strip()

        if line.startswith("position:"):
            current_section = "position"
        elif line.startswith("orientation:"):
            current_section = "orientation"
        elif ":" in line and current_section:
            key, value = line.split(":")
            pose[current_section][key.strip()] = round(float(value.strip()), 4)

    return pose


def load_existing_data():
    if not os.path.exists(OUTPUT_FILE):
        return []

    with open(OUTPUT_FILE, "r") as f:
        try:
            return yaml.safe_load(f) or []
        except yaml.YAMLError:
            return []


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("waypoint", type=int, help="Waypoint ID (integer)")
    args = parser.parse_args()

    waypoint_id = str(args.waypoint)

    waypoint_entry = {
        waypoint_id: {
            "timestamp": time.time(),
            "data": {}
        }
    }

    for topic, field in TOPICS:

        # Wait briefly for topic to become available
        for _ in range(5):
            if topic_available(topic):
                break
            time.sleep(0.5)

        raw_pose = get_topic_output(topic, field)
        structured_pose = parse_pose(raw_pose)

        waypoint_entry[waypoint_id]["data"][topic] = structured_pose

    existing_data = load_existing_data()
    existing_data.append(waypoint_entry)

    with open(OUTPUT_FILE, "w") as f:
        yaml.dump(existing_data, f, sort_keys=False)

    print(f"Waypoint {waypoint_id} saved to {OUTPUT_FILE}")


if __name__ == "__main__":
    main()