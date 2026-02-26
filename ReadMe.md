

# ROSBot XL robot Maze Solver
A demonstration for  manoeuvring through the maze in both simulation and real world.  

## Waypoint Recording
![waypoints](/resources/images/robot_xl_maze_world_waypoints.png)
Waypoints can be recorded through the `waypoint_recorder_XXX.py` script which is available under `misc` directory. The below image is for reference while recording the waypoints.
usage,
```bash.sh
cd ~/ros2_ws/src/ROSBot_XL_mazesolver/misl
python3 waypoint_recorder_json.py -waypoint <waypoint number>
```
As you reach the desired goal with orientation, run the above command with a designated waypoint number each time. 
This will append your current pose into the existing json file under the same location.
> [NOTE] 
> The below part has to be done manually. moving the waypoints.json to resources/waypoints and adding a prefix.

Once a waypoint.json file is generated under the same directory. Do a general inspection of that file, add a `sim_` or `real_` prefix to the waypoint.json file and move it inside the `resources/waypoints` directory.

## PID Distance Controller
### Task 1
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
```
To fine tune them or play with PID values you can pass them as arguments directly like below
```bash.sh
ros2 run distance_controller distance_controller --ros-args -p kP:=0.5 -p kI:=0.02 -p kD:=1.2
```
### Task 2
> [Important] 
> For this section if the waypoints json files are not present under the `resources/waypoints`, generate them first as mentioned [Waypoint Recording](#waypoint-recording) section

In this section, based on the selected `scene_number` (1-simulation & 2-real), waypoints are selected and passed on to distance controller.
```bash.sh
ros2 run distance_controller distance_controller 2
```
ROSBot_XL can also be fine tuned through following extra arguments as listed below and execute movement based on them.
- `num_waypoints`: Number of waypoints from the home position to be executed directly from command like. Default waypoints selected will be `3`.
- `odom_topic`: Odom topic through `odom_topic` argument. Default will be `/odometry/filtered`

Once it reaches the last waypoint(as per passed in `num_waypoints`), it returns back to its home position.
[See waypoints](#waypoint-recording)

```bash.sh
ros2 run distance_controller distance_controller 1 --ros-args -p num_waypoints:=4
ros2 run distance_controller distance_controller 1 --ros-args -p odom_topic:=/rosbot_xl_base_controller/odom
ros2 run distance_controller distance_controller 2 --ros-args -p num_waypoints:=6 -p kP:=0.5 -p kI:=0.02 -p kD:=1.2
```
## PID Turn Controller
