# ROSBot XL robot Maze Solver
A demonstration for solving and maneuvering through the maze in both simulation and real world.  

## Waypoint Recording
![](/resources/images/robot_xl_maze_world_waypoints.png)
Waypoints can be recorded through the waypoint_recorder_XXX.py script which is available under `misc` directory. The below image is for reference while recording the waypoints.
usage,
```bash.sh
cd ~/ros2_ws/src/ROSBot_XL_mazesolver/misl
python3 waypoint_recorder_json.py -waypoint <waypoint number>
```
> [NOTE] 
> The below part has to be done manually. moving the waypoints.json to resources/waypoints and adding a prefix.

Once a waypoint.json file is generated under the same directory. Do a general inspection of that file, add a `sim_` or `real_` prefix to the waypoint.json file and move it inside the `resources/waypoints` directory.

## Distance Controller
Run the distance controller to achive the below motion testing in an empty world as shown in the below image
<img src="resources/images/task1_distance_controller_waypoints_sim.png" width="300" height="200">

Start the empty world
```bash.sh
source ~/ros2_ws/install/setup.bash
ros2 launch rosbot_xl_gazebo empty_simulation.launch.py mecanum:=true
```

Run distance controller. Alternatively to experiment with PID values you can pass in the values directly
```bash.sh
ros2 run distance_controller distance_controller
ros2 run distance_controller distance_controller --ros-args -p kP:=0.5 -p kI:=0.02 -p kD:=1.2
```