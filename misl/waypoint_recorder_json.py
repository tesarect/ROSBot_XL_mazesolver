# #!/usr/bin/env python3

# import subprocess
# import time
# import argparse
# import json
# import os

# OUTPUT_FILE = "waypoints.json"

# TOPICS = [
#     ("/odometry/filtered", "pose.pose"),
#     ("/rosbot_xl_base_controller/odom", "pose.pose"),
# ]

# def topic_available(topic):
#     result = subprocess.run(
#         ["ros2", "topic", "list"],
#         capture_output=True,
#         text=True
#     )
#     return topic in result.stdout

# def get_topic_output(topic, field):
#     result = subprocess.run(
#         ["ros2", "topic", "echo", topic, "--field", field, "--once"],
#         capture_output=True,
#         text=True
#     )
#     return result.stdout

# def parse_pose(raw_text):
#     pose = {
#         "position": {},
#         "orientation": {}
#     }

#     lines = raw_text.splitlines()
#     current_section = None

#     for line in lines:
#         line = line.strip()

#         if line.startswith("position:"):
#             current_section = "position"
#         elif line.startswith("orientation:"):
#             current_section = "orientation"
#         elif ":" in line and current_section:
#             key, value = line.split(":")
#             pose[current_section][key.strip()] = float(value.strip())

#     return pose

# def load_existing_data():
#     if not os.path.exists(OUTPUT_FILE):
#         return []
#     with open(OUTPUT_FILE, "r") as f:
#         try:
#             return json.load(f)
#         except json.JSONDecodeError:
#             return []

# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument("-waypoint", required=True)
#     args = parser.parse_args()

#     waypoint_entry = {
#         "waypoint": args.waypoint,
#         "timestamp": time.time(),
#         "data": {}
#     }

#     for topic, field in TOPICS:
#         for _ in range(5):
#             if topic_available(topic):
#                 break
#             time.sleep(0.5)

#         raw_pose = get_topic_output(topic, field)
#         structured_pose = parse_pose(raw_pose)

#         waypoint_entry["data"][topic] = structured_pose

#     existing_data = load_existing_data()
#     existing_data.append(waypoint_entry)

#     with open(OUTPUT_FILE, "w") as f:
#         json.dump(existing_data, f, indent=2)

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3

import subprocess
import time
import argparse
import json
import os

OUTPUT_FILE = "waypoints.json"

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
            return json.load(f)
        except json.JSONDecodeError:
            return []

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-waypoint", required=True)
    args = parser.parse_args()

    waypoint_entry = {
        "waypoint": args.waypoint,
        "timestamp": time.time(),
        "data": {}
    }

    for topic, field in TOPICS:
        for _ in range(5):
            if topic_available(topic):
                break
            time.sleep(0.5)

        raw_pose = get_topic_output(topic, field)
        structured_pose = parse_pose(raw_pose)

        waypoint_entry["data"][topic] = structured_pose

    existing_data = load_existing_data()
    existing_data.append(waypoint_entry)

    with open(OUTPUT_FILE, "w") as f:
        json.dump(existing_data, f, indent=2)

if __name__ == "__main__":
    main()
