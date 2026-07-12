THIS ROS 2 PROJECT IS MANUALLY UPLOADED, SO THE BUILD, LOG, AND INSTALL FILES HAVE BEEN DELETED. PLEASE BUILD THE SOURCE PACKAGE ON YOUR ROS2 PC AS INSTRUCTED IN STEP 1

Here are the instructions for running the project:

1. Run a terminal, then run the "cd autonomous_mobile_robot_ws" command, then run "colcon build --symlink-install".

2. Once you complete step 1, you should run 7 terminals. First, in each terminal, you should type the "source ~/autonomous_mobile_robot_ws/install/setup.bash", then run the following on each terminal in the EXACT order as below:

# Terminal 1 — Gazebo (world with the actor)
export TURTLEBOT3_MODEL=waffle
ros2 launch navigation_manager tb3_world_humans.launch.py

# Terminal 2 — Nav2, AFTER Gazebo's window is up
ros2 launch nav2_bringup bringup_launch.py \
  map:=$(ros2 pkg prefix navigation_manager)/share/navigation_manager/maps/tb3_world.yaml \
  params_file:=$(ros2 pkg prefix navigation_manager)/share/navigation_manager/config/nav2_params.yaml \
  use_sim_time:=true

# Terminal 3 — RViz (verification/demo eyes)
ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true

# Terminal 4 - Human predictor
ros 2 run navigation_manager human_predictor --ros-args -p use_sim_time:=true

# Terminal 5 — Battery simulator (before the monitor, so /battery_state exists)
ros2 run navigation_manager battery_simulator --ros-args -p use_sim_time:=true \
  -p charger_x:=2.0 -p charger_y:=2.0

# Terminal 6 — failure monitor (same charger coords!)
ros2 run navigation_manager failure_monitor --ros-args -p use_sim_time:=true \
  -p charger_x:=2.0 -p charger_y:=2.0

Please note that this places charger coordinates at (2.0,2.0), so when the robot battery is 20% or less, the robot goes to these exact coordinates to charge.

# Terminal 7 — send the mission
ros2 run navigation_manager goal_navigator --ros-args -p use_sim_time:=true
