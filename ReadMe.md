# ROSBot XL robot Maze Solver
A demonstration for solving and maneuvering through the maze in both simulation and real world.  

## Waypoint Recording
Waypoints can be recorded through the waypoint_recorder_XXX.py script which is available under `misc` directory. The below image is for reference while recording the waypoints.
usage,
```bash.sh
cd ~/ros2_ws/src/ROSBot_XL_mazesolver/misl
python3 waypoint_recorder_json.py -waypoint <waypoint number>
```
This will generate a waypoint.json file under the same directory. After inspection of that file add a `sim_` or `real_` prefix to the waypoint.json file and place it inside the `resources/waypoints` directory.

